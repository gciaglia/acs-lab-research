"""
Velocity command generation and collision avoidance.

Pure Python module (no ROS imports). Computes repulsive vectors from nearby
robots and generates (linear_x, angular_z) velocity commands for differential
drive robots navigating toward a target.
"""

import math


def compute_collision_avoidance(my_position, other_positions, avoidance_radius=0.4):
    """Compute a repulsive avoidance vector from nearby robots.

    For each robot within avoidance_radius, adds a repulsive vector pointing
    away from that robot with magnitude proportional to proximity.

    Args:
        my_position: (x, y) tuple of this robot's position.
        other_positions: List of (x, y) tuples for other robots.
        avoidance_radius: Distance threshold for repulsion (meters).

    Returns:
        (avoid_vx, avoid_vy, min_dist) — repulsive vector components and
        the distance to the nearest robot.
    """
    avoid_vx = 0.0
    avoid_vy = 0.0
    min_dist = float('inf')
    mx, my_ = my_position

    for ox, oy in other_positions:
        dx = mx - ox
        dy = my_ - oy
        dist = math.sqrt(dx * dx + dy * dy)
        min_dist = min(min_dist, dist)

        if dist < avoidance_radius and dist > 0.01:
            strength = (avoidance_radius - dist) / avoidance_radius
            avoid_vx += (dx / dist) * strength
            avoid_vy += (dy / dist) * strength

    return avoid_vx, avoid_vy, min_dist


def _normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def compute_velocity_command(
    current_x, current_y, current_yaw,
    target_x, target_y,
    avoid_vx, avoid_vy, min_dist,
    linear_speed=0.2,
    angular_speed=0.8,
    lloyd_gain=0.3,
    emergency_radius=0.25,
):
    """Generate (linear_x, angular_z) to drive toward a target.

    Incorporates collision avoidance. If a robot is within emergency_radius,
    the robot backs away. Otherwise it blends the avoidance vector into the
    target direction and uses a turn-then-drive strategy.

    Args:
        current_x, current_y, current_yaw: Robot's current pose.
        target_x, target_y: Desired position.
        avoid_vx, avoid_vy: Repulsive vector from collision avoidance.
        min_dist: Distance to nearest other robot.
        linear_speed: Maximum forward speed (m/s).
        angular_speed: Maximum turning speed (rad/s).
        lloyd_gain: Proportional gain for centroid-seeking.
        emergency_radius: Distance triggering emergency evasion (meters).

    Returns:
        (linear_x, angular_z) tuple for a Twist message.
    """
    # Emergency evasion
    if min_dist < emergency_radius:
        if abs(avoid_vx) > 0.01 or abs(avoid_vy) > 0.01:
            avoid_angle = math.atan2(avoid_vy, avoid_vx)
            angle_error = _normalize_angle(avoid_angle - current_yaw)
            sign = 1.0 if angle_error >= 0 else -1.0
            return 0.05, angular_speed * sign
        return 0.0, 0.0

    # Navigate toward target with avoidance blended in
    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.sqrt(dx * dx + dy * dy)

    dx += avoid_vx * 0.5
    dy += avoid_vy * 0.5

    target_angle = math.atan2(dy, dx)
    angle_error = _normalize_angle(target_angle - current_yaw)

    if distance < 0.1:
        # Close enough to target — stop
        return 0.0, 0.0
    elif abs(angle_error) > 0.5:
        # Turn in place to face target
        sign = 1.0 if angle_error >= 0 else -1.0
        return 0.0, angular_speed * sign
    else:
        # Drive forward with proportional steering
        lin = min(linear_speed, lloyd_gain * distance)
        ang = 2.0 * angle_error
        return lin, ang
