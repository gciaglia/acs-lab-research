#!/usr/bin/env python3
"""
Voronoi visualization node for RViz2.

Runs as a single instance (not per-robot). Subscribes to all fleet
world_pose topics, computes the Voronoi partition, and publishes a
MarkerArray showing the arena boundary, robot positions, Voronoi cell
edges, and cell centroids.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from turtlebot3_voronoi.voronoi_math import get_bounded_voronoi_cells, compute_centroid


# Per-robot colors: blue, orange, green, red, purple
ROBOT_COLORS = [
    ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
    ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
    ColorRGBA(r=0.0, g=0.8, b=0.0, a=1.0),
    ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
    ColorRGBA(r=0.6, g=0.0, b=0.8, a=1.0),
]


class VoronoiVisualizer(Node):
    """Publishes MarkerArray of Voronoi cells and robot positions."""

    def __init__(self):
        super().__init__('voronoi_visualizer')

        # Parameters
        self.declare_parameter('num_robots', 5)
        self.declare_parameter('region_min_x', -3.0)
        self.declare_parameter('region_max_x', 3.0)
        self.declare_parameter('region_min_y', -3.0)
        self.declare_parameter('region_max_y', 3.0)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('frame_id', 'default')

        num_robots = self.get_parameter('num_robots').value
        self.region_min_x = self.get_parameter('region_min_x').value
        self.region_max_x = self.get_parameter('region_max_x').value
        self.region_min_y = self.get_parameter('region_min_y').value
        self.region_max_y = self.get_parameter('region_max_y').value
        rate_hz = self.get_parameter('rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value

        self.region_bounds = (
            self.region_min_x, self.region_max_x,
            self.region_min_y, self.region_max_y,
        )

        # Fleet positions: robot_name -> (x, y)
        self.fleet_positions = {}
        self.robot_names = [f'tb3_{i}' for i in range(num_robots)]

        # Subscribe to all world_pose topics
        for name in self.robot_names:
            self.create_subscription(
                PoseStamped, f'/{name}/world_pose',
                lambda msg, n=name: self._pose_callback(msg, n),
                10
            )

        # Marker publisher
        self.marker_pub = self.create_publisher(
            MarkerArray, '/voronoi/markers', 10
        )

        # Visualization timer
        self.create_timer(1.0 / rate_hz, self._publish_markers)

        self.get_logger().info(
            f'Visualizer started: {num_robots} robots, '
            f'publishing to /voronoi/markers at {rate_hz} Hz'
        )

    def _pose_callback(self, msg, robot_name):
        self.fleet_positions[robot_name] = (
            msg.pose.position.x, msg.pose.position.y,
        )

    def _publish_markers(self):
        if len(self.fleet_positions) < 2:
            return

        marker_array = MarkerArray()
        marker_id = 0
        now = self.get_clock().now().to_msg()

        sorted_names = sorted(self.fleet_positions.keys())
        positions = np.array([
            [self.fleet_positions[n][0], self.fleet_positions[n][1]]
            for n in sorted_names
        ])

        # --- Arena boundary ---
        boundary = Marker()
        boundary.header.frame_id = self.frame_id
        boundary.header.stamp = now
        boundary.ns = 'arena_boundary'
        boundary.id = marker_id
        marker_id += 1
        boundary.type = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.scale.x = 0.1
        boundary.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=1.0)
        corners = [
            (self.region_min_x, self.region_min_y),
            (self.region_max_x, self.region_min_y),
            (self.region_max_x, self.region_max_y),
            (self.region_min_x, self.region_max_y),
            (self.region_min_x, self.region_min_y),
        ]
        for cx, cy in corners:
            p = Point()
            p.x, p.y, p.z = float(cx), float(cy), 0.05
            boundary.points.append(p)
        marker_array.markers.append(boundary)

        # --- Robot markers + labels ---
        for i, name in enumerate(sorted_names):
            rx, ry = self.fleet_positions[name]
            color = ROBOT_COLORS[i % len(ROBOT_COLORS)]

            # Cylinder for robot position
            robot_m = Marker()
            robot_m.header.frame_id = self.frame_id
            robot_m.header.stamp = now
            robot_m.ns = 'robots'
            robot_m.id = marker_id
            marker_id += 1
            robot_m.type = Marker.CYLINDER
            robot_m.action = Marker.ADD
            robot_m.pose.position.x = float(rx)
            robot_m.pose.position.y = float(ry)
            robot_m.pose.position.z = 0.1
            robot_m.scale.x = 0.3
            robot_m.scale.y = 0.3
            robot_m.scale.z = 0.2
            robot_m.color = color
            marker_array.markers.append(robot_m)

            # Text label
            label_m = Marker()
            label_m.header.frame_id = self.frame_id
            label_m.header.stamp = now
            label_m.ns = 'robot_labels'
            label_m.id = marker_id
            marker_id += 1
            label_m.type = Marker.TEXT_VIEW_FACING
            label_m.action = Marker.ADD
            label_m.pose.position.x = float(rx)
            label_m.pose.position.y = float(ry)
            label_m.pose.position.z = 0.4
            label_m.scale.z = 0.3
            label_m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label_m.text = f'R{i}'
            marker_array.markers.append(label_m)

        # --- Voronoi cells + centroids ---
        cells = get_bounded_voronoi_cells(positions, self.region_bounds)
        if cells is not None:
            for i, cell in enumerate(cells):
                if len(cell) < 3:
                    continue

                color = ROBOT_COLORS[i % len(ROBOT_COLORS)]

                # Sort vertices by angle for clean polygon drawing
                center = cell.mean(axis=0)
                angles = np.arctan2(
                    cell[:, 1] - center[1], cell[:, 0] - center[0]
                )
                sorted_cell = cell[np.argsort(angles)]

                # Cell boundary line
                cell_m = Marker()
                cell_m.header.frame_id = self.frame_id
                cell_m.header.stamp = now
                cell_m.ns = 'voronoi_cells'
                cell_m.id = marker_id
                marker_id += 1
                cell_m.type = Marker.LINE_STRIP
                cell_m.action = Marker.ADD
                cell_m.scale.x = 0.05
                cell_m.color = ColorRGBA(
                    r=color.r, g=color.g, b=color.b, a=0.8
                )
                for v in sorted_cell:
                    p = Point()
                    p.x, p.y, p.z = float(v[0]), float(v[1]), 0.02
                    cell_m.points.append(p)
                # Close polygon
                p = Point()
                p.x = float(sorted_cell[0][0])
                p.y = float(sorted_cell[0][1])
                p.z = 0.02
                cell_m.points.append(p)
                marker_array.markers.append(cell_m)

                # Centroid sphere
                centroid = compute_centroid(cell)
                centroid_m = Marker()
                centroid_m.header.frame_id = self.frame_id
                centroid_m.header.stamp = now
                centroid_m.ns = 'centroids'
                centroid_m.id = marker_id
                marker_id += 1
                centroid_m.type = Marker.SPHERE
                centroid_m.action = Marker.ADD
                centroid_m.pose.position.x = float(centroid[0])
                centroid_m.pose.position.y = float(centroid[1])
                centroid_m.pose.position.z = 0.1
                centroid_m.scale.x = 0.15
                centroid_m.scale.y = 0.15
                centroid_m.scale.z = 0.15
                centroid_m.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
                marker_array.markers.append(centroid_m)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = VoronoiVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
