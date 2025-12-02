# Diagram: Differential Drive Kinematics

**Type**: Mechanical Diagram  
**Chapter**: P6-C1  
**Purpose**: Illustrate forward and inverse kinematics for differential drive robot

## Diagram Specification

```mermaid
graph TB
    subgraph "Differential Drive Robot"
        Base[Base Link<br/>x, y, θ]
        LeftWheel[Left Wheel<br/>v_left, ω_left]
        RightWheel[Right Wheel<br/>v_right, ω_right]
        Caster[Caster Wheel]
    end
    
    subgraph "Forward Kinematics"
        FW1[Wheel Velocities<br/>v_left, v_right]
        FW2[Robot Velocity<br/>v = r/2 × ω_left + ω_right<br/>ω = r/L × ω_right - ω_left]
        FW3[Robot Pose<br/>x, y, θ]
    end
    
    subgraph "Inverse Kinematics"
        IW1[Desired Motion<br/>v_desired, ω_desired]
        IW2[Wheel Commands<br/>ω_left = 2v - Lω / 2r<br/>ω_right = 2v + Lω / 2r]
    end
    
    Base --> LeftWheel
    Base --> RightWheel
    Base --> Caster
    
    LeftWheel --> FW1
    RightWheel --> FW1
    FW1 --> FW2
    FW2 --> FW3
    
    IW1 --> IW2
    IW2 --> LeftWheel
    IW2 --> RightWheel
    
    style Base fill:#4A90E2,stroke:#2E5C8A,color:#fff
    style LeftWheel fill:#7ED321,stroke:#5A9A15,color:#fff
    style RightWheel fill:#7ED321,stroke:#5A9A15,color:#fff
    style FW2 fill:#F5A623,stroke:#D68910,color:#fff
    style IW2 fill:#F5A623,stroke:#D68910,color:#fff
```

## Description

This diagram shows the relationship between wheel velocities and robot motion for a differential drive mobile robot. The forward kinematics path (top) computes robot velocity and pose from wheel velocities, while the inverse kinematics path (bottom) computes required wheel velocities from desired robot motion.

**Key Elements**:
- Base link with position (x, y) and orientation (θ)
- Two independently driven wheels with velocities v_left and v_right
- Caster wheel for support
- Forward kinematics equations converting wheel velocities to robot motion
- Inverse kinematics equations converting desired motion to wheel commands

**Parameters**:
- L: Wheelbase (distance between wheel centers)
- r: Wheel radius
- v: Linear velocity
- ω: Angular velocity


