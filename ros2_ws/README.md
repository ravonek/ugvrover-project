# ROS 2 Workspace

This directory contains the source workspace for the UGV rover project.

## Packages

| Package | Type | Role |
| --- | --- | --- |
| `ugv_rover` | `ament_python` | Color-goal navigation, sim/real odometry sync, rover helpers |
| `ugv_navigation2` | `ament_cmake` | Nav2 launch, map, parameters, RViz views |
| `roarm_m2_s` | `ament_python` | RoArm control, camera/color input, pick-and-place sequencing |
| `ghfvjkvxcjvydfxczvzxv_description` | `ament_python` | URDF/USD/STL robot description and visualization launch files |

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Common Launches

```bash
ros2 launch ugv_navigation2 bringup_real.launch.py
ros2 launch ugv_rover ugv_all.launch.py
ros2 launch roarm_m2_s roarm_all.launch.py
```

Generated `build/`, `install/`, and `log/` folders are intentionally not committed.
