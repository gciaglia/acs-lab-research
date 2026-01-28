"""
Spawn 5 TurtleBot3 Burger robots into Gazebo Harmonic.

For each robot:
  1. Loads the base TurtleBot3 SDF and namespaces its topics.
  2. Spawns the model via ros_gz_sim create.
  3. Creates a ros_gz_bridge for cmd_vel (ROS→GZ) and odom (GZ→ROS).
"""

import math
import os
import random
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import Node


# Arena bounds (with margin so robots don't spawn against walls)
REGION_MIN_X = -3.0
REGION_MAX_X = 3.0
REGION_MIN_Y = -3.0
REGION_MAX_Y = 3.0
SPAWN_MARGIN = 0.5


def _create_namespaced_sdf(robot_name, base_sdf_path):
    """Create a modified SDF with namespaced topics for this robot."""
    with open(base_sdf_path, 'r') as f:
        sdf = f.read()

    # Namespace topic names so each robot gets its own cmd_vel / odom
    sdf = sdf.replace(
        '<topic>cmd_vel</topic>',
        f'<topic>/model/{robot_name}/cmd_vel</topic>',
    )
    sdf = sdf.replace(
        '<odom_topic>odom</odom_topic>',
        f'<odom_topic>/model/{robot_name}/odom</odom_topic>',
    )
    sdf = sdf.replace(
        '<tf_topic>/tf</tf_topic>',
        f'<tf_topic>/model/{robot_name}/tf</tf_topic>',
    )
    sdf = sdf.replace(
        '<topic>joint_states</topic>',
        f'<topic>/model/{robot_name}/joint_states</topic>',
    )

    tmp_path = os.path.join(tempfile.gettempdir(), f'{robot_name}.sdf')
    with open(tmp_path, 'w') as f:
        f.write(sdf)
    return tmp_path


def _spawn_one_robot(context, robot):
    """Return spawn + bridge nodes for a single robot."""
    pkg_tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    base_sdf = os.path.join(pkg_tb3_gz, 'models', 'turtlebot3_burger', 'model.sdf')

    sdf_file = _create_namespaced_sdf(robot['name'], base_sdf)

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{robot["name"]}',
        output='screen',
        arguments=[
            '-name', robot['name'],
            '-file', sdf_file,
            '-x', str(robot['x']),
            '-y', str(robot['y']),
            '-z', '0.01',
            '-Y', str(robot['yaw']),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot["name"]}_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            f'/model/{robot["name"]}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/model/{robot["name"]}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            (f'/model/{robot["name"]}/cmd_vel', f'/{robot["name"]}/cmd_vel'),
            (f'/model/{robot["name"]}/odom', f'/{robot["name"]}/odom'),
        ],
    )

    return [spawn, bridge]


def _generate_random_robots(num_robots=5):
    """Generate random spawn positions within the arena bounds."""
    robots = []
    min_x = REGION_MIN_X + SPAWN_MARGIN
    max_x = REGION_MAX_X - SPAWN_MARGIN
    min_y = REGION_MIN_Y + SPAWN_MARGIN
    max_y = REGION_MAX_Y - SPAWN_MARGIN

    for i in range(num_robots):
        robots.append({
            'name': f'tb{i}',
            'x': random.uniform(min_x, max_x),
            'y': random.uniform(min_y, max_y),
            'yaw': random.uniform(-math.pi, math.pi),
        })
    return robots


def _launch_setup(context):
    robots = _generate_random_robots()
    nodes = []
    for robot in robots:
        nodes.extend(_spawn_one_robot(context, robot))
    return nodes


def generate_launch_description():
    pkg_tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    models_path = os.path.join(pkg_tb3_gz, 'models')

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
