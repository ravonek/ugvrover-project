# Architecture

This project combines a ROS 2 rover stack, Nav2 localization/navigation, Isaac Sim topic bridges, and a RoArm pick-and-place workflow.

## High-Level Data Flow

```mermaid
flowchart LR
    subgraph Simulation["Isaac Sim"]
        USD["navigation.usd"]
        Graphs["OmniGraph ROS2 publishers/subscribers"]
        USD --> Graphs
    end

    subgraph ROS["ROS 2 workspace"]
        Nav2["ugv_navigation2"]
        Rover["ugv_rover"]
        Arm["roarm_m2_s"]
        Desc["robot description"]
    end

    Graphs -->|/scan /tf /odom_sim /cmd_vel_sim| Nav2
    Nav2 -->|/navigate_to_pose| Rover
    Rover -->|/cmd_vel| Base["UGV base"]
    Rover -->|/goal_done| Arm
    Arm -->|/joint_command| RoArm["RoArm M2-S"]
    Desc --> RViz["RViz / Gazebo visualization"]
```

## Navigation and Manipulation Sequence

```mermaid
sequenceDiagram
    participant Vision as Camera / color detector
    participant Nav as ugv_rover/nav_goal
    participant Nav2 as Nav2 action server
    participant Sync as sim_real_sync_guard
    participant Base as UGV base
    participant Arm as roarm_m2_s/pick_place

    Vision->>Nav: publish /colors
    Nav->>Nav2: send first navigation sequence
    Nav2->>Sync: publish /cmd_vel_sim
    Sync->>Base: publish corrected /cmd_vel
    Nav->>Arm: publish /goal_done
    Arm->>Arm: execute first pick/place phase
    Nav->>Nav2: send color-specific drop route
    Nav->>Arm: publish /cube_sent_done
    Arm->>Arm: return to home pose
```

## Package Responsibilities

```mermaid
flowchart TB
    subgraph ugv_rover
        A["nav2.py: color route planner"]
        B["sim_real_sync_guard.py: odom follower"]
        C["cube_joint_controller.py: cube/joint helper"]
    end

    subgraph ugv_navigation2
        D["Nav2 launch files"]
        E["real/sim params"]
        F["map + RViz configs"]
    end

    subgraph roarm_m2_s
        G["camera2.py"]
        H["roarm_.py"]
        I["pick_place_script.py"]
    end

    subgraph description
        J["URDF"]
        K["USD"]
        L["STL meshes"]
    end

    G --> A
    A --> D
    D --> B
    A --> I
    I --> H
    J --> F
```

## Important Runtime Topics

| Topic | Direction | Used by |
| --- | --- | --- |
| `/colors` | perception to planner | `ugv_rover.nav2`, `roarm_m2_s.pick_place_script` |
| `/navigate_to_pose` | planner to Nav2 | `ugv_rover.nav2` |
| `/goal_done` | rover to arm | `ugv_rover.nav2`, `roarm_m2_s.pick_place_script` |
| `/cube_sent_done` | rover status | `ugv_rover.nav2` |
| `/cmd_vel_sim` | simulation command | `sim_real_sync_guard` |
| `/cmd_vel` | real rover command | base controller |
| `/odom_sim` | simulation odometry | `sim_real_sync_guard` |
| `/odom` | real odometry | Nav2 and `sim_real_sync_guard` |
| `/joint_command` | arm command | RoArm / Isaac bridge |
| `/motion_matrix` | arm motion plan | Isaac/visualization integration |
