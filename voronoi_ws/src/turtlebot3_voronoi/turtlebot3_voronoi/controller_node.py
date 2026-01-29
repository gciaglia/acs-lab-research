#!/usr/bin/env python3
"""
Per-robot Voronoi coverage controller node.

Each TurtleBot3 runs one instance. The node subscribes to its own odometry,
shares its world-frame position with the fleet, computes the bounded Voronoi
partition, and drives toward its cell's centroid using Lloyd's algorithm.

State machine:
    explore  (0 – explore_duration seconds) → random walk to scatter robots
    voronoi  (explore_duration onwards)      → Lloyd's algorithm convergence
"""

import math
import random

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

from turtlebot3_voronoi.odometry import OdometryProcessor
from turtlebot3_voronoi.voronoi_math import get_bounded_voronoi_cells, compute_centroid, find_hotspot
from turtlebot3_voronoi.motion import compute_collision_avoidance, compute_velocity_command
from turtlebot3_voronoi.density_field import UniformDensity, GridDensity


class VoronoiController(Node):
    """Voronoi coverage controller for a single TurtleBot3."""

    def __init__(self):
        super().__init__('voronoi_controller')

        # -- Declare parameters --------------------------------------------------
        self.declare_parameter('robot_name', 'tb3_0')
        self.declare_parameter('robot_id', 0)
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('region_min_x', -3.0)
        self.declare_parameter('region_max_x', 3.0)
        self.declare_parameter('region_min_y', -3.0)
        self.declare_parameter('region_max_y', 3.0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('lloyd_gain', 0.3)
        self.declare_parameter('explore_duration', 5.0)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('centroid_samples', 500)
        self.declare_parameter('avoidance_radius', 0.4)
        self.declare_parameter('emergency_radius', 0.25)
        self.declare_parameter('density_type', 'uniform')
        self.declare_parameter('mrt_file', '')
        self.declare_parameter('mrt_center_x', 46160)
        self.declare_parameter('mrt_center_y', 29736)
        self.declare_parameter('mrt_extract_size', 50)
        self.declare_parameter('target_mode', 'hotspot')  # 'centroid' or 'hotspot'

        # -- Read parameters -----------------------------------------------------
        self.robot_name = self.get_parameter('robot_name').value
        self.robot_id = self.get_parameter('robot_id').value
        self.num_robots = self.get_parameter('num_robots').value
        spawn_x = self.get_parameter('spawn_x').value
        spawn_y = self.get_parameter('spawn_y').value
        spawn_yaw = self.get_parameter('spawn_yaw').value
        self.region_min_x = self.get_parameter('region_min_x').value
        self.region_max_x = self.get_parameter('region_max_x').value
        self.region_min_y = self.get_parameter('region_min_y').value
        self.region_max_y = self.get_parameter('region_max_y').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.lloyd_gain = self.get_parameter('lloyd_gain').value
        self.explore_duration = self.get_parameter('explore_duration').value
        control_rate = self.get_parameter('control_rate').value
        self.centroid_samples = self.get_parameter('centroid_samples').value
        self.avoidance_radius = self.get_parameter('avoidance_radius').value
        self.emergency_radius = self.get_parameter('emergency_radius').value
        density_type = self.get_parameter('density_type').value
        mrt_file = self.get_parameter('mrt_file').value
        mrt_center_x = self.get_parameter('mrt_center_x').value
        mrt_center_y = self.get_parameter('mrt_center_y').value
        mrt_extract_size = self.get_parameter('mrt_extract_size').value
        self.target_mode = self.get_parameter('target_mode').value

        # -- Derived state -------------------------------------------------------
        self.region_bounds = (
            self.region_min_x, self.region_max_x,
            self.region_min_y, self.region_max_y,
        )

        # Odometry processor (spawn offset + yaw rotation + quaternion-to-yaw)
        self.odom = OdometryProcessor(spawn_x, spawn_y, spawn_yaw)

        # Density field for weighted centroids
        self.density_fn = self._build_density_field(
            density_type, mrt_file, mrt_center_x, mrt_center_y, mrt_extract_size
        )

        # Fleet positions: robot_name -> (x, y)
        self.fleet_positions = {}

        # Robot name list for subscription (all robots in fleet)
        self.robot_names = [f'tb3_{i}' for i in range(self.num_robots)]

        # State machine
        self.state = 'explore'
        self.start_time = None

        # Random exploration target
        self.explore_target_x = random.uniform(
            self.region_min_x + 0.5, self.region_max_x - 0.5
        )
        self.explore_target_y = random.uniform(
            self.region_min_y + 0.5, self.region_max_y - 0.5
        )
        self.target_x = self.explore_target_x
        self.target_y = self.explore_target_y

        # -- Publishers ----------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'/{self.robot_name}/cmd_vel', 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, f'/{self.robot_name}/world_pose', 10
        )

        # -- Subscribers ---------------------------------------------------------
        # Own odometry
        self.create_subscription(
            Odometry, f'/{self.robot_name}/odom',
            self._odom_callback, 10
        )
        # Fleet positions (all robots including self)
        for name in self.robot_names:
            if name != self.robot_name:
                self.create_subscription(
                    PoseStamped, f'/{name}/world_pose',
                    lambda msg, n=name: self._fleet_pose_callback(msg, n),
                    10
                )

        # -- Timers --------------------------------------------------------------
        self.create_timer(1.0 / control_rate, self._control_loop)
        self.create_timer(0.1, self._broadcast_pose)

        # Logging counter
        self._log_counter = 0

        self.get_logger().info(
            f'Controller started: {self.robot_name} (ID {self.robot_id}), '
            f'explore {self.explore_duration}s then Lloyd\'s, '
            f'density={density_type}, target={self.target_mode}'
        )

    # -- Density field setup ---------------------------------------------------

    def _build_density_field(self, density_type, mrt_file,
                              mrt_center_x, mrt_center_y, mrt_extract_size):
        """Create the appropriate density field from parameters."""
        if density_type == 'uniform' or not mrt_file:
            return None  # None = geometric centroid (no weighting)

        try:
            if mrt_file.endswith('.npy'):
                field = GridDensity.from_npy(mrt_file, self.region_bounds)
            elif mrt_file.lower().endswith('.tif'):
                field = GridDensity.from_geotiff(
                    mrt_file, self.region_bounds,
                    center_x=mrt_center_x,
                    center_y=mrt_center_y,
                    extract_size=mrt_extract_size,
                )
                self.get_logger().info(
                    f'Extracting {mrt_extract_size}m region centered at '
                    f'pixel ({mrt_center_x}, {mrt_center_y})'
                )
            else:
                self.get_logger().error(f'Unsupported density file: {mrt_file}')
                return None
            self.get_logger().info(f'Loaded density field from {mrt_file}')
            return field
        except Exception as e:
            self.get_logger().error(f'Failed to load density field: {e}')
            return None

    # -- Callbacks -------------------------------------------------------------

    def _odom_callback(self, msg):
        """Process odometry from Gazebo bridge."""
        q = msg.pose.pose.orientation
        self.odom.update(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            q.x, q.y, q.z, q.w,
        )
        # Store own position in fleet dict
        self.fleet_positions[self.robot_name] = (self.odom.x, self.odom.y)

        # Initialize start time on first odom
        if self.start_time is None:
            self.start_time = self.get_clock().now()

    def _fleet_pose_callback(self, msg, robot_name):
        """Receive another robot's world-frame position."""
        self.fleet_positions[robot_name] = (
            msg.pose.position.x, msg.pose.position.y,
        )

    def _broadcast_pose(self):
        """Publish this robot's world-frame pose for fleet coordination."""
        if not self.odom.is_initialized:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'default'
        msg.pose.position.x = self.odom.x
        msg.pose.position.y = self.odom.y
        msg.pose.position.z = 0.0
        self.pose_pub.publish(msg)

    # -- Control loop ----------------------------------------------------------

    def _control_loop(self):
        """Main 10 Hz control loop: explore → Voronoi → move."""
        if not self.odom.is_initialized or self.start_time is None:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # --- State machine ---
        if self.state == 'explore':
            if elapsed >= self.explore_duration:
                self.state = 'voronoi'
                self.get_logger().info(
                    f'{self.robot_name}: switching to Lloyd\'s algorithm'
                )
            else:
                # Random walk: pick new target when close
                dist = math.sqrt(
                    (self.odom.x - self.explore_target_x) ** 2 +
                    (self.odom.y - self.explore_target_y) ** 2
                )
                if dist < 0.3:
                    self.explore_target_x = random.uniform(
                        self.region_min_x + 0.5, self.region_max_x - 0.5
                    )
                    self.explore_target_y = random.uniform(
                        self.region_min_y + 0.5, self.region_max_y - 0.5
                    )
                self.target_x = self.explore_target_x
                self.target_y = self.explore_target_y

        if self.state == 'voronoi':
            # Collect fleet positions as sorted array
            sorted_names = sorted(self.fleet_positions.keys())
            if len(sorted_names) < 2:
                self._publish_stop()
                return

            positions = np.array([
                [self.fleet_positions[n][0], self.fleet_positions[n][1]]
                for n in sorted_names
            ])

            try:
                my_idx = sorted_names.index(self.robot_name)
            except ValueError:
                self._publish_stop()
                return

            cells = get_bounded_voronoi_cells(positions, self.region_bounds)
            if cells is None:
                self._publish_stop()
                return

            # Compute target based on mode: hotspot or centroid
            if self.target_mode == 'hotspot' and self.density_fn is not None:
                # Find the hottest point in the cell
                target = find_hotspot(
                    cells[my_idx],
                    density_fn=self.density_fn,
                    n_samples=self.centroid_samples,
                )
            else:
                # Use density-weighted centroid (or geometric if no density)
                target = compute_centroid(
                    cells[my_idx],
                    density_fn=self.density_fn,
                    n_samples=self.centroid_samples,
                )
            self.target_x, self.target_y = target

        # --- Motion command ---
        other_positions = [
            pos for name, pos in self.fleet_positions.items()
            if name != self.robot_name
        ]
        avoid_vx, avoid_vy, min_dist = compute_collision_avoidance(
            (self.odom.x, self.odom.y), other_positions, self.avoidance_radius,
        )
        lin, ang = compute_velocity_command(
            self.odom.x, self.odom.y, self.odom.yaw,
            self.target_x, self.target_y,
            avoid_vx, avoid_vy, min_dist,
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
            lloyd_gain=self.lloyd_gain,
            emergency_radius=self.emergency_radius,
        )

        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self.cmd_vel_pub.publish(cmd)

        # Periodic logging
        self._log_counter += 1
        if self._log_counter % 30 == 0:
            dist = math.sqrt(
                (self.odom.x - self.target_x) ** 2 +
                (self.odom.y - self.target_y) ** 2
            )
            self.get_logger().info(
                f'{self.robot_name} [{self.state}]: '
                f'pos=({self.odom.x:.2f}, {self.odom.y:.2f}), '
                f'target=({self.target_x:.2f}, {self.target_y:.2f}), '
                f'dist={dist:.2f}'
            )

    def _publish_stop(self):
        """Publish zero-velocity command."""
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VoronoiController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
