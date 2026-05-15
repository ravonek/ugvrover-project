# Robot Description Package

This package contains the exported robot description used by the project.

## Contents

- `urdf/`: URDF, Gazebo, Xacro, and USD configuration files.
- `meshes/`: STL meshes for the rover body and wheels.
- `launch/display.launch.py`: RViz visualization launch.
- `launch/gazebo.launch.py`: Gazebo visualization launch.
- `config/display.rviz`: RViz display configuration.

The package name is kept as exported so existing launch files and metadata continue to work.

## Launch

```bash
ros2 launch ghfvjkvxcjvydfxczvzxv_description display.launch.py
```
