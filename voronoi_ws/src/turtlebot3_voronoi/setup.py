from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_voronoi'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Multi-TurtleBot3 Voronoi coverage simulation with modular architecture',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voronoi_controller = turtlebot3_voronoi.controller_node:main',
            'voronoi_visualizer = turtlebot3_voronoi.visualizer_node:main',
        ],
    },
)
