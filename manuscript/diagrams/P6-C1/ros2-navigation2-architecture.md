# Diagram: ROS2 Navigation2 Architecture

**Type**: Architecture Diagram  
**Chapter**: P6-C1  
**Purpose**: Illustrate ROS2 Navigation2 stack components and data flow

## Diagram Specification

```mermaid
graph TB
    subgraph "Inputs"
        Goal[Goal Pose<br/>/goal_pose]
        Map[Static Map<br/>/map]
        Odom[Odometry<br/>/odom]
        Laser[LiDAR Data<br/>/scan]
    end
    
    subgraph "Navigation2 Stack"
        Planner[Global Planner<br/>A*, RRT*]
        Controller[Local Controller<br/>DWB, TEB]
        Costmap[Costmap<br/>Static + Dynamic]
        Recovery[Recovery Behaviors<br/>Rotate, Clear]
    end
    
    subgraph "Outputs"
        CmdVel[Velocity Command<br/>/cmd_vel]
        Path[Global Path<br/>/plan]
        LocalPath[Local Trajectory<br/>/local_plan]
    end
    
    subgraph "Robot Hardware"
        Motors[Motors]
        Sensors[Sensors]
    end
    
    Goal --> Planner
    Map --> Planner
    Map --> Costmap
    Odom --> Costmap
    Laser --> Costmap
    
    Planner --> Path
    Path --> Controller
    Costmap --> Controller
    Controller --> LocalPath
    LocalPath --> CmdVel
    
    Controller --> Recovery
    Recovery --> CmdVel
    
    CmdVel --> Motors
    Sensors --> Odom
    Sensors --> Laser
    
    style Planner fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style Controller fill:#7ED321,stroke:#5A9A15,color:#fff
    style Costmap fill:#F5A623,stroke:#D68910,color:#fff
    style Recovery fill:#BD10E0,stroke:#9012FE,color:#fff
    style CmdVel fill:#D0021B,stroke:#A00115,color:#fff
```

## Description

This diagram illustrates the ROS2 Navigation2 stack architecture for autonomous mobile robot navigation. The system processes sensor data and goal commands to generate velocity commands for the robot motors.

**Key Components**:
- **Global Planner**: Computes optimal path from start to goal using static map
- **Local Controller**: Follows global path while avoiding dynamic obstacles
- **Costmap**: Grid-based representation of obstacles (static map + dynamic sensor data)
- **Recovery Behaviors**: Handles stuck situations (rotate in place, clear costmap)

**Data Flow**:
1. Goal pose and static map → Global Planner → Global Path
2. Global Path + Costmap → Local Controller → Local Trajectory
3. Local Trajectory → Velocity Commands → Motors
4. Sensors → Odometry + LiDAR → Costmap (updates)


