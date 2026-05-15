# Setup

The workspace is arranged as a standard ROS 2 colcon workspace:

```text
ros2_ws/
└── src/
    ├── ugv_rover
    ├── ugv_navigation2
    ├── roarm_m2_s
    └── ghfvjkvxcjvydfxczvzxv_description
```

## Requirements

- Ubuntu 22.04 style environment
- ROS 2 Humble or a compatible ROS 2 distribution
- Nav2 packages
- RViz2
- Python 3.10
- `cv_bridge`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `nav2_msgs`
- Isaac Sim for the USD scene and OmniGraph workflow

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch Navigation

```bash
ros2 launch ugv_navigation2 bringup_real.launch.py
```

The default map is packaged at:

```text
ros2_ws/src/ugv_navigation2/map/my_map.yaml
```

To override the map:

```bash
ros2 launch ugv_navigation2 bringup_real.launch.py map:=/absolute/path/to/map.yaml
```

## Launch Rover Control

```bash
ros2 launch ugv_rover ugv_all.launch.py
```

Useful launch arguments:

```bash
ros2 launch ugv_rover ugv_all.launch.py \
  cmd_in_topic:=/cmd_vel_sim \
  cmd_real_topic:=/cmd_vel \
  odom_sim_topic:=/odom_sim \
  odom_real_topic:=/odom
```

## Launch RoArm

```bash
ros2 launch roarm_m2_s roarm_all.launch.py
```

## View Robot Description

```bash
ros2 launch ghfvjkvxcjvydfxczvzxv_description display.launch.py
```

## Clean Generated Files

```bash
cd ros2_ws
rm -rf build install log
```

Those folders are ignored by Git and should be regenerated locally.
