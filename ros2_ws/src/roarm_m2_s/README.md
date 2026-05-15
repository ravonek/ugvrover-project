# roarm_m2_s

ROS 2 Python package for RoArm M2-S control and pick-and-place sequencing.

## What It Does

- Reads color decisions from `/colors`.
- Waits for rover navigation completion on `/goal_done`.
- Publishes arm joint targets on `/joint_command`.
- Publishes complete motion rows on `/motion_matrix`.
- Runs red/green placement sequences and returns the arm to home.

## Console Scripts

| Command | Source | Purpose |
| --- | --- | --- |
| `roarm` | `roarm_m2_s/roarm_.py` | Main RoArm control node |
| `camera2` | `roarm_m2_s/camera2.py` | Camera/color input node |
| `pick_place` | `roarm_m2_s/pick_place_script.py` | Pick-and-place state machine |

## Launch

```bash
ros2 launch roarm_m2_s roarm_all.launch.py
```
