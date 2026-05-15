# Simulation

This directory contains the Isaac Sim scene used for the UGV rover and RoArm M2-S workflow.

## Main Scene

| File | Purpose |
| --- | --- |
| `navigation.usd` | Main Isaac Sim scene with the rover, manipulator, maze-style environment, and target cube setup |

## Action Graph Role

The project uses Isaac Sim OmniGraph nodes to bridge the simulated scene into ROS 2:

- publish simulation clock to ROS 2
- publish odometry and TF / transform tree data
- convert simulated LiDAR beams into ROS 2 `LaserScan`
- subscribe to ROS 2 `Twist` commands for rover movement
- drive wheel and arm articulation controllers

The related screenshots are stored in:

```text
docs/assets/photos/omnigraph-odometry.jpeg
docs/assets/photos/omnigraph-lidar.jpeg
docs/assets/photos/omnigraph-control.jpeg
```

## Typical Workflow

1. Start ROS 2 and build/source `ros2_ws`.
2. Open Isaac Sim.
3. Load `simulation/navigation.usd`.
4. Enable the ROS 2 bridge and run the Action Graphs.
5. Launch the navigation and manipulation nodes from the ROS workspace.
