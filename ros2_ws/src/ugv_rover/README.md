# ugv_rover

ROS 2 Python package for rover-side mission logic.

## What It Does

- Receives cube/color labels from `/colors`.
- Sends route goals to Nav2 through `/navigate_to_pose`.
- Publishes mission state on `/goal_done` and `/cube_sent_done`.
- Converts simulated velocity/odometry behavior into corrected real rover `/cmd_vel`.
- Provides helper control for the rover/cube joint workflow.

## Console Scripts

| Command | Source | Purpose |
| --- | --- | --- |
| `nav_goal` | `ugv_rover/nav2.py` | Color-based route planner and Nav2 action client |
| `sim_real_sync_guard` | `ugv_rover/sim_real_sync_guard.py` | Sim-real odometry follower and command limiter |
| `cube` | `ugv_rover/cube_joint_controller.py` | Cube/joint helper node |

## Launch

```bash
ros2 launch ugv_rover ugv_all.launch.py
```

The launch file starts `nav_goal`, `sim_real_sync_guard`, and `cube` together.
