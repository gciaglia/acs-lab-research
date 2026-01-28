#!/usr/bin/env python3
"""
Record and plot robot positions from Gazebo odom and controller world_pose.

Subscribes to:
  /<robot>/odom        - raw odometry from Gazebo (via ros_gz_bridge)
  /<robot>/world_pose  - offset-corrected pose published by controller nodes

On Ctrl+C, saves two CSVs and a comparison plot showing both sources
side-by-side so you can identify any translation mismatch.

Usage:
  # In a sourced terminal while the simulation is running:
  python3 scripts/plot_positions.py

  # Specify output directory:
  python3 scripts/plot_positions.py --output-dir /tmp/debug
"""

import argparse
import csv
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


ROBOT_COLORS = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red', 'tab:purple']


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class PositionPlotter(Node):

    def __init__(self, num_robots, output_dir):
        super().__init__('position_plotter')

        self.num_robots = num_robots
        self.output_dir = output_dir
        self.robot_names = [f'tb3_{i}' for i in range(num_robots)]

        # Storage: dict of robot_name -> list of (t, x, y)
        self.gazebo_data = {name: [] for name in self.robot_names}
        self.rviz_data = {name: [] for name in self.robot_names}

        self._gazebo_count = 0
        self._rviz_count = 0

        # Subscribe to raw Gazebo odom
        for name in self.robot_names:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda msg, n=name: self._odom_cb(msg, n),
                10,
            )

        # Subscribe to controller world_pose (what RViz sees)
        for name in self.robot_names:
            self.create_subscription(
                PoseStamped, f'/{name}/world_pose',
                lambda msg, n=name: self._world_pose_cb(msg, n),
                10,
            )

        self.get_logger().info(
            f'Recording {num_robots} robots. Press Ctrl+C to stop and plot.'
        )

    def _odom_cb(self, msg, robot_name):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.gazebo_data[robot_name].append((t, x, y))
        self._gazebo_count += 1
        if self._gazebo_count % 500 == 0:
            self.get_logger().info(
                f'Recorded {self._gazebo_count} gazebo, '
                f'{self._rviz_count} rviz samples'
            )

    def _world_pose_cb(self, msg, robot_name):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.rviz_data[robot_name].append((t, x, y))
        self._rviz_count += 1

    def save_and_plot(self):
        os.makedirs(self.output_dir, exist_ok=True)

        # --- Write CSVs ---
        for label, data in [('gazebo_odom', self.gazebo_data),
                            ('rviz_world_pose', self.rviz_data)]:
            path = os.path.join(self.output_dir, f'{label}_positions.csv')
            with open(path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp_sec', 'robot', 'x', 'y'])
                for name in self.robot_names:
                    for t, x, y in data[name]:
                        writer.writerow([f'{t:.6f}', name,
                                         f'{x:.6f}', f'{y:.6f}'])
            self.get_logger().info(f'Wrote {path}')

        # --- Plot ---
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warn(
                'matplotlib not installed, skipping plot. '
                'Install with: pip install matplotlib'
            )
            return

        fig, axes = plt.subplots(1, 3, figsize=(18, 6))

        # Panel 1: Raw Gazebo odom
        ax = axes[0]
        ax.set_title('Gazebo Odom (raw, no offset)')
        for i, name in enumerate(self.robot_names):
            pts = self.gazebo_data[name]
            if not pts:
                continue
            xs = [p[1] for p in pts]
            ys = [p[2] for p in pts]
            color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
            ax.plot(xs, ys, color=color, alpha=0.6, linewidth=1)
            ax.plot(xs[0], ys[0], 'o', color=color, markersize=8)
            ax.plot(xs[-1], ys[-1], 's', color=color, markersize=8)
            ax.annotate(name, (xs[0], ys[0]), fontsize=7)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        # Draw arena bounds
        ax.axhline(-3, color='gray', linestyle='--', alpha=0.5)
        ax.axhline(3, color='gray', linestyle='--', alpha=0.5)
        ax.axvline(-3, color='gray', linestyle='--', alpha=0.5)
        ax.axvline(3, color='gray', linestyle='--', alpha=0.5)

        # Panel 2: Controller world_pose (what RViz shows)
        ax = axes[1]
        ax.set_title('Controller world_pose (RViz)')
        for i, name in enumerate(self.robot_names):
            pts = self.rviz_data[name]
            if not pts:
                continue
            xs = [p[1] for p in pts]
            ys = [p[2] for p in pts]
            color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
            ax.plot(xs, ys, color=color, alpha=0.6, linewidth=1)
            ax.plot(xs[0], ys[0], 'o', color=color, markersize=8)
            ax.plot(xs[-1], ys[-1], 's', color=color, markersize=8)
            ax.annotate(name, (xs[0], ys[0]), fontsize=7)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.axhline(-3, color='gray', linestyle='--', alpha=0.5)
        ax.axhline(3, color='gray', linestyle='--', alpha=0.5)
        ax.axvline(-3, color='gray', linestyle='--', alpha=0.5)
        ax.axvline(3, color='gray', linestyle='--', alpha=0.5)

        # Panel 3: Overlay both on same axes
        ax = axes[2]
        ax.set_title('Overlay (solid=Gazebo, dashed=RViz)')
        for i, name in enumerate(self.robot_names):
            color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
            gz_pts = self.gazebo_data[name]
            rv_pts = self.rviz_data[name]
            if gz_pts:
                gx = [p[1] for p in gz_pts]
                gy = [p[2] for p in gz_pts]
                ax.plot(gx, gy, color=color, alpha=0.6, linewidth=1,
                        linestyle='-', label=f'{name} gazebo')
                ax.plot(gx[0], gy[0], 'o', color=color, markersize=8)
            if rv_pts:
                rx = [p[1] for p in rv_pts]
                ry = [p[2] for p in rv_pts]
                ax.plot(rx, ry, color=color, alpha=0.6, linewidth=1,
                        linestyle='--', label=f'{name} rviz')
                ax.plot(rx[0], ry[0], '^', color=color, markersize=8)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.axhline(-3, color='gray', linestyle='--', alpha=0.3)
        ax.axhline(3, color='gray', linestyle='--', alpha=0.3)
        ax.axvline(-3, color='gray', linestyle='--', alpha=0.3)
        ax.axvline(3, color='gray', linestyle='--', alpha=0.3)
        ax.legend(fontsize=6, loc='upper right', ncol=2)

        plt.tight_layout()
        plot_path = os.path.join(self.output_dir, 'position_comparison.png')
        plt.savefig(plot_path, dpi=150)
        self.get_logger().info(f'Saved plot → {plot_path}')
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Record and plot robot positions from Gazebo and RViz'
    )
    parser.add_argument('--num-robots', type=int, default=5)
    parser.add_argument('--output-dir', type=str, default='.')
    args = parser.parse_args()

    rclpy.init()
    node = PositionPlotter(args.num_robots, args.output_dir)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Stopping — generating plot...')
        node.save_and_plot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
