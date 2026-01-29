"""
Main bringup launch file for the Voronoi coverage simulation.

Orchestrates:
  t=0s   Gazebo Harmonic + static TF
  t=2s   Clock bridge
  t=3s   Spawn 5 TurtleBot3 robots + topic bridges
  t=8s   5x controller nodes + 1x visualizer node
  t=10s  RViz2 (optional)

Robots spawn at random locations within the arena each launch.

Usage:
  ros2 launch turtlebot3_voronoi bringup.launch.py
  ros2 launch turtlebot3_voronoi bringup.launch.py use_rviz:=false
  ros2 launch turtlebot3_voronoi bringup.launch.py density_type:=mrt_file mrt_file:=/path/to/data.npy
"""

import math
import os
import random
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NUM_ROBOTS = 5

# Arena bounds (with margin so robots don't spawn against walls)
REGION_MIN_X = -3.0
REGION_MAX_X = 3.0
REGION_MIN_Y = -3.0
REGION_MAX_Y = 3.0
SPAWN_MARGIN = 0.5  # stay this far from walls

# Per-robot colors matching RViz visualizer (R, G, B)
ROBOT_COLORS = [
    (0.0, 0.0, 1.0),    # blue
    (1.0, 0.5, 0.0),    # orange
    (0.0, 0.8, 0.0),    # green
    (1.0, 0.0, 0.0),    # red
    (0.6, 0.0, 0.8),    # purple
]


def _generate_random_robots(num_robots):
    """Generate random spawn positions within the arena bounds."""
    robots = []
    for i in range(num_robots):
        robots.append({
            'name': f'tb3_{i}',
            'x': random.uniform(REGION_MIN_X + SPAWN_MARGIN, REGION_MAX_X - SPAWN_MARGIN),
            'y': random.uniform(REGION_MIN_Y + SPAWN_MARGIN, REGION_MAX_Y - SPAWN_MARGIN),
            'yaw': random.uniform(-math.pi, math.pi),
            'id': i,
        })
    return robots


def _create_namespaced_sdf(robot_name, base_sdf_path, color):
    """Create a modified SDF with namespaced topics and color marker."""
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

    # Inject a colored cylinder marker on top of the robot
    r, g, b = color
    color_visual = f"""
      <visual name="color_marker">
        <pose>0 0 0.12 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.06</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>"""
    # Insert the color marker into the first <link> (base_link)
    sdf = re.sub(r'(</link>)', color_visual + r'\n      \1', sdf, count=1)

    tmp_path = os.path.join(tempfile.gettempdir(), f'{robot_name}.sdf')
    with open(tmp_path, 'w') as f:
        f.write(sdf)
    return tmp_path


def _launch_setup(context):
    """Generate all spawn, bridge, controller, and visualizer nodes."""
    pkg_voronoi = get_package_share_directory('turtlebot3_voronoi')
    pkg_tb3_gz = get_package_share_directory('turtlebot3_gazebo')
    base_sdf = os.path.join(pkg_tb3_gz, 'models', 'turtlebot3_burger', 'model.sdf')

    density_type = LaunchConfiguration('density_type').perform(context)
    mrt_file = LaunchConfiguration('mrt_file').perform(context)
    mrt_center_x = int(LaunchConfiguration('mrt_center_x').perform(context))
    mrt_center_y = int(LaunchConfiguration('mrt_center_y').perform(context))
    mrt_extract_size = int(LaunchConfiguration('mrt_extract_size').perform(context))
    target_mode = LaunchConfiguration('target_mode').perform(context)

    robots = _generate_random_robots(NUM_ROBOTS)

    region_params = {
        'region_min_x': REGION_MIN_X,
        'region_max_x': REGION_MAX_X,
        'region_min_y': REGION_MIN_Y,
        'region_max_y': REGION_MAX_Y,
    }

    # -- Spawn + bridge nodes (delayed to t=3s in the main description) --------
    spawn_nodes = []
    for robot in robots:
        color = ROBOT_COLORS[robot['id'] % len(ROBOT_COLORS)]
        sdf_file = _create_namespaced_sdf(robot['name'], base_sdf, color)

        spawn_nodes.append(Node(
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
        ))

        spawn_nodes.append(Node(
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
        ))

    # -- Controller nodes (delayed to t=8s) ------------------------------------
    controller_nodes = []
    for robot in robots:
        controller_nodes.append(Node(
            package='turtlebot3_voronoi',
            executable='voronoi_controller',
            name=f'{robot["name"]}_controller',
            output='screen',
            parameters=[{
                'robot_name': robot['name'],
                'robot_id': robot['id'],
                'num_robots': NUM_ROBOTS,
                'spawn_x': robot['x'],
                'spawn_y': robot['y'],
                'spawn_yaw': robot['yaw'],
                'use_sim_time': True,
                'linear_speed': 0.2,
                'angular_speed': 0.8,
                'lloyd_gain': 0.3,
                'explore_duration': 5.0,
                'control_rate': 10.0,
                'centroid_samples': 500,
                'avoidance_radius': 0.4,
                'emergency_radius': 0.25,
                'density_type': density_type,
                'mrt_file': mrt_file,
                'mrt_center_x': mrt_center_x,
                'mrt_center_y': mrt_center_y,
                'mrt_extract_size': mrt_extract_size,
                'target_mode': target_mode,
                **region_params,
            }],
        ))

    # -- Visualizer node -------------------------------------------------------
    visualizer_node = Node(
        package='turtlebot3_voronoi',
        executable='voronoi_visualizer',
        name='voronoi_visualizer',
        output='screen',
        parameters=[{
            'num_robots': NUM_ROBOTS,
            'use_sim_time': True,
            'rate_hz': 5.0,
            'frame_id': 'default',
            'density_type': density_type,
            'mrt_file': mrt_file,
            'mrt_center_x': mrt_center_x,
            'mrt_center_y': mrt_center_y,
            'mrt_extract_size': mrt_extract_size,
            **region_params,
        }],
    )

    return [
        # t=3s: Spawn robots + bridges
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg_tb3_gz, 'models'),
        ),
        TimerAction(period=3.0, actions=spawn_nodes),

        # t=8s: Controllers + visualizer
        TimerAction(period=8.0, actions=controller_nodes + [visualizer_node]),
    ]


def generate_launch_description():
    pkg_voronoi = get_package_share_directory('turtlebot3_voronoi')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_voronoi, 'worlds', 'bounded_arena.sdf')

    # -- Gazebo Harmonic -------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # -- Clock bridge ----------------------------------------------------------
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # -- Static TF (default -> odom) -------------------------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'default', 'odom'],
    )

    # -- RViz2 -----------------------------------------------------------------
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config = os.path.join(pkg_voronoi, 'rviz', 'voronoi.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz2 for visualization'),
        DeclareLaunchArgument('density_type', default_value='uniform',
                              description='Density field type: uniform | mrt_file'),
        DeclareLaunchArgument('mrt_file', default_value='',
                              description='Path to MRT .npy or .tif file'),
        DeclareLaunchArgument('mrt_center_x', default_value='46160',
                              description='X pixel coordinate for MRT extraction center (Gammage default)'),
        DeclareLaunchArgument('mrt_center_y', default_value='29736',
                              description='Y pixel coordinate for MRT extraction center (Gammage default)'),
        DeclareLaunchArgument('mrt_extract_size', default_value='50',
                              description='Region size in meters to extract from MRT (default 50m)'),
        DeclareLaunchArgument('target_mode', default_value='hotspot',
                              description='Target mode: hotspot (track max density) | centroid (weighted centroid)'),

        # t=0s: Gazebo + static TF
        gazebo,
        static_tf,

        # t=2s: Clock bridge
        TimerAction(period=2.0, actions=[clock_bridge]),

        # t=3s–8s: Spawn, controllers, visualizer (via OpaqueFunction)
        OpaqueFunction(function=_launch_setup),

        # t=10s: RViz2
        TimerAction(period=10.0, actions=[rviz_node]),
    ])
