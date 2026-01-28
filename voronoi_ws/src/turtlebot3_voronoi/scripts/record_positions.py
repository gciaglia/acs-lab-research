#!/usr/bin/env python3
"""
Troubleshooting script: records robot positions from two sources to CSV.

Outputs:
  rviz_positions.csv   - from /<robot>/world_pose topics (what RViz displays,
                          after spawn-offset correction by the controller)
  gazebo_positions.csv - from /<robot>/odom topics (raw Gazebo odometry,
                          local frame starting at 0,0 per robot)

Usage:
  # In a sourced terminal while the simulation is running:
  python3 scripts/record_positions.py

  # Or specify output directory and robot count:
  python3 scripts/record_positions.py --output-dir /tmp --num-robots 5

  Press Ctrl+C to stop recording and flush CSVs.
"""

import argparse
import csv
import math
import os
import signal
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def quaternion_to_yaw(qx, qy, qz, qw):
    """Extract yaw from a quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class PositionRecorder(Node):
    """Subscribes to world_pose and odom topics, writes two CSVs."""

    def __init__(self, num_robots, output_dir):
        super().__init__('position_recorder')

        self.robot_names = [f'tb3_{i}' for i in range(num_robots)]

        # Open CSV files
        rviz_path = os.path.join(output_dir, 'rviz_positions.csv')
        gazebo_path = os.path.join(output_dir, 'gazebo_positions.csv')

        self._rviz_file = open(rviz_path, 'w', newline='')
        self._gazebo_file = open(gazebo_path, 'w', newline='')

        self._rviz_writer = csv.writer(self._rviz_file)
        self._gazebo_writer = csv.writer(self._gazebo_file)

        header = ['timestamp_sec', 'robot', 'x', 'y', 'yaw']
        self._rviz_writer.writerow(header)
        self._gazebo_writer.writerow(header)

        self._rviz_count = 0
        self._gazebo_count = 0

        # Subscribe to world_pose (what RViz sees)
        for name in self.robot_names:
            self.create_subscription(
                PoseStamped, f'/{name}/world_pose',
                lambda msg, n=name: self._world_pose_cb(msg, n),
                10,
            )

        # Subscribe to odom (raw Gazebo odometry)
        for name in self.robot_names:
            self.create_subscription(
                Odometry, f'/{name}/odom',
                lambda msg, n=name: self._odom_cb(msg, n),
                10,
            )

        self.get_logger().info(
            f'Recording {num_robots} robots → {rviz_path}, {gazebo_path}'
        )

    def _world_pose_cb(self, msg, robot_name):
        """Record a world_pose sample (RViz source)."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.position.x
        y = msg.pose.position.y
        # world_pose is PoseStamped with no orientation set by controller,
        # so yaw is 0; record it anyway for completeness
        q = msg.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self._rviz_writer.writerow([f'{t:.6f}', robot_name,
                                    f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}'])
        self._rviz_count += 1

    def _odom_cb(self, msg, robot_name):
        """Record an odom sample (Gazebo source)."""
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self._gazebo_writer.writerow([f'{t:.6f}', robot_name,
                                      f'{x:.6f}', f'{y:.6f}', f'{yaw:.6f}'])
        self._gazebo_count += 1

    def close(self):
        """Flush and close CSV files."""
        self._rviz_file.close()
        self._gazebo_file.close()
        self.get_logger().info(
            f'Done. Wrote {self._rviz_count} RViz rows, '
            f'{self._gazebo_count} Gazebo rows.'
        )


def main():
    parser = argparse.ArgumentParser(description='Record robot positions to CSV')
    parser.add_argument('--num-robots', type=int, default=5,
                        help='Number of robots to track (default: 5)')
    parser.add_argument('--output-dir', type=str, default='.',
                        help='Directory for CSV output (default: cwd)')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    rclpy.init()
    node = PositionRecorder(args.num_robots, args.output_dir)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
