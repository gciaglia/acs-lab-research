# TurtleBot3 Voronoi Coverage Simulation

Multi-robot Voronoi tessellation coverage control simulation using 5 TurtleBot3 Burger robots in Gazebo Harmonic with ROS2.

## Overview

This package implements distributed Voronoi coverage control where robots autonomously partition a bounded arena and navigate toward their Voronoi cell centroids. The simulation includes:

- 5 TurtleBot3 Burger robots with color-coded markers
- Lloyd's algorithm for coverage control
- Collision avoidance between robots
- Real-time Voronoi partition visualization in RViz2
- Support for custom density fields (uniform or MRT data)

## Prerequisites

- **ROS2 Humble** (or later)
- **Gazebo Harmonic** (gz-sim)
- **TurtleBot3 packages**:
  ```bash
  sudo apt install ros-humble-turtlebot3-gazebo
  ```
- **ROS-Gazebo bridge packages**:
  ```bash
  sudo apt install ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
  ```
- **Python dependencies**: numpy, scipy (typically included with ROS2)

## Building

```bash
cd ~/path/to/voronoi_ws
colcon build --packages-select turtlebot3_voronoi
source install/setup.bash
```

## Running the Simulation

### Basic Launch

```bash
ros2 launch turtlebot3_voronoi bringup.launch.py
```

This will:
1. Start Gazebo Harmonic with a bounded 6m x 6m arena
2. Spawn 5 TurtleBot3 robots at random positions
3. Launch controller nodes for each robot
4. Open RViz2 with Voronoi partition visualization

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `true` | Launch RViz2 for visualization |
| `density_type` | `uniform` | Density field: `uniform` or `mrt_file` |
| `mrt_file` | `""` | Path to `.npy` or `.tif` density file |
| `mrt_center_x` | `46160` | X pixel coordinate for MRT extraction (Gammage default) |
| `mrt_center_y` | `29736` | Y pixel coordinate for MRT extraction (Gammage default) |
| `mrt_extract_size` | `50` | Region size in meters to extract (default: 50m x 50m) |
| `target_mode` | `hotspot` | Target mode: `hotspot` (track max density) or `centroid` (weighted centroid) |

### Examples

Run without RViz:
```bash
ros2 launch turtlebot3_voronoi bringup.launch.py use_rviz:=false
```

Run with MRT data from Gammage Center (default 50m region):
```bash
ros2 launch turtlebot3_voronoi bringup.launch.py \
  density_type:=mrt_file \
  mrt_file:=/path/to/DataSimMRT/mrt_20120627_1200.tif
```

Run with custom MRT region (100m around different location):
```bash
ros2 launch turtlebot3_voronoi bringup.launch.py \
  density_type:=mrt_file \
  mrt_file:=/path/to/mrt_20120627_1200.tif \
  mrt_center_x:=46200 \
  mrt_center_y:=29800 \
  mrt_extract_size:=100
```

## Using MRT Data

The simulation supports Mean Radiant Temperature (MRT) data from GeoTIFF files. The MRT data acts as a density field that biases robot movement toward high-temperature (hotspot) locations.

### Preview MRT Regions

Before running the simulation, use the preview script to explore the MRT data and find good extraction coordinates:

```bash
# Preview default Gammage region (50m)
python3 scripts/preview_mrt_region.py /path/to/mrt_20120627_1200.tif

# Preview custom region
python3 scripts/preview_mrt_region.py /path/to/mrt_20120627_1200.tif \
  --center 46200 29800 --size 100
```

The preview shows:
- Left: Context view with extraction box
- Right: Normalized view as the simulation will see it
- Hotspot location in simulation coordinates
- Ready-to-use launch command

### MRT Data Coordinate System

- MRT TIF files: 1 pixel = 1 meter resolution
- Gammage Center: pixel (46160, 29736)
- Default extraction: 50m x 50m region mapped to 6m x 6m simulation arena
- Scale: ~8:1 (each simulation meter ≈ 8 real meters)

## Project Structure

```
turtlebot3_voronoi/
├── config/
│   └── params.yaml          # Tunable parameters
├── launch/
│   ├── bringup.launch.py    # Main launch file
│   └── spawn_robots.launch.py
├── rviz/
│   └── voronoi.rviz         # RViz configuration
├── scripts/
│   ├── record_positions.py  # Position logging utility
│   └── plot_positions.py    # Position plotting utility
├── turtlebot3_voronoi/
│   ├── controller_node.py   # Robot controller
│   ├── visualizer_node.py   # Voronoi visualization
│   ├── voronoi_math.py      # Voronoi computation
│   ├── density_field.py     # Density field handling
│   ├── motion.py            # Motion control
│   └── odometry.py          # Odometry processing
└── worlds/
    └── bounded_arena.sdf    # Gazebo world file
```

## Parameters

Key parameters can be adjusted in `config/params.yaml`:

- **Arena bounds**: 6m x 6m region (-3 to +3 on each axis)
- **Linear speed**: 0.2 m/s
- **Angular speed**: 0.8 rad/s
- **Lloyd gain**: 0.3 (centroid-seeking proportional gain)
- **Avoidance radius**: 0.4m (collision avoidance activation)
- **Explore duration**: 5s (random walk before Voronoi control starts)

## Topics

Each robot (`tb3_0` through `tb3_4`) publishes/subscribes to:
- `/<robot_name>/cmd_vel` - Velocity commands
- `/<robot_name>/odom` - Odometry data

Visualization:
- `/voronoi_markers` - Voronoi partition visualization markers
