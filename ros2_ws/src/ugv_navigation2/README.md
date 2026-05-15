# ugv_navigation2

Navigation package for the UGV rover.

## Contents

- `launch/bringup_launch.py`: Nav2 bringup for simulation-style runs.
- `launch/bringup_real.launch.py`: Nav2 bringup for the real rover map.
- `param/ugv_rover_sim.yaml`: Nav2 parameters for simulation.
- `param/ugv_rover_real.yaml`: Nav2 parameters for real robot use.
- `map/my_map.yaml`: packaged occupancy map.
- `rviz/`: RViz configurations for navigation monitoring.

## Launch

```bash
ros2 launch ugv_navigation2 bringup_real.launch.py
```

Override the map if needed:

```bash
ros2 launch ugv_navigation2 bringup_real.launch.py map:=/absolute/path/to/map.yaml
```
