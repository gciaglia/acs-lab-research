"""
Odometry processing for TurtleBot3 robots in Gazebo.

Pure Python module (no ROS imports). Gazebo Harmonic's DiffDrive plugin
reports odometry in a local frame starting at (0, 0), aligned with the
robot's initial heading. This class applies a full rigid-body transform
(rotation by spawn yaw + translation by spawn position) to convert
local-frame odometry into world-frame coordinates.
"""

import math


class OdometryProcessor:
    """Processes odometry readings into world-frame pose.

    Gazebo reports each robot's odometry relative to its spawn point:
    position starts at (0, 0) and the X-axis points along the robot's
    initial heading. To get world-frame coordinates, we rotate the local
    displacement by spawn_yaw and add the spawn position.

    Args:
        spawn_x: World-frame X coordinate where the robot was spawned.
        spawn_y: World-frame Y coordinate where the robot was spawned.
        spawn_yaw: World-frame heading (radians) at spawn time.
    """

    def __init__(self, spawn_x, spawn_y, spawn_yaw=0.0):
        self._spawn_x = spawn_x
        self._spawn_y = spawn_y
        self._spawn_yaw = spawn_yaw
        self._cos_yaw = math.cos(spawn_yaw)
        self._sin_yaw = math.sin(spawn_yaw)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._initialized = False

    def update(self, odom_x, odom_y, qx, qy, qz, qw):
        """Process a new odometry reading.

        Args:
            odom_x: Position X from Odometry message (local frame).
            odom_y: Position Y from Odometry message (local frame).
            qx, qy, qz, qw: Orientation quaternion components.
        """
        # Rotate local odom displacement by spawn yaw, then translate
        self._x = (self._spawn_x
                    + odom_x * self._cos_yaw
                    - odom_y * self._sin_yaw)
        self._y = (self._spawn_y
                    + odom_x * self._sin_yaw
                    + odom_y * self._cos_yaw)

        # Extract local yaw from quaternion and add spawn yaw for world frame
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        local_yaw = math.atan2(siny_cosp, cosy_cosp)
        self._yaw = local_yaw + self._spawn_yaw

        self._initialized = True

    @property
    def x(self):
        """World-frame X position."""
        return self._x

    @property
    def y(self):
        """World-frame Y position."""
        return self._y

    @property
    def yaw(self):
        """World-frame heading angle in radians."""
        return self._yaw

    @property
    def position(self):
        """World-frame (x, y) tuple."""
        return (self._x, self._y)

    @property
    def is_initialized(self):
        """True after the first odometry reading has been processed."""
        return self._initialized
