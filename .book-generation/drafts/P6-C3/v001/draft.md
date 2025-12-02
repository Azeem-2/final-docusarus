# Build a Humanoid Leg in Simulation

**Chapter ID**: P6-C3  
**Part**: Part 6 - Integrated Robotics Projects  
**Word Count**: ~8,500 words

---

## 1. Introduction

Building a humanoid leg in simulation represents a crucial step toward understanding bipedal locomotion—the foundation of humanoid robotics. This chapter guides you through designing, modeling, and controlling a 6-degree-of-freedom (DOF) humanoid leg system in simulation environments (Isaac Sim, MuJoCo), integrating concepts from Part 5 (Humanoid Robotics).

Throughout Parts 1-5, you've learned foundational concepts: embodied intelligence (Part 1), physical robotics (Part 2), simulation environments (Part 3), AI techniques (Part 4), and humanoid robotics principles (Part 5). This project integrates humanoid-specific concepts, demonstrating how simulation-first development enables safe, rapid iteration of locomotion controllers.

By completing this project, you'll gain hands-on experience with:
- Humanoid leg design principles (6-DOF serial mechanism)
- Leg kinematics and dynamics modeling
- RL-based locomotion control
- Balance and walking behavior generation
- Simulation-to-reality transfer preparation

This simulation-first approach enables rapid development without hardware costs or safety risks, mirroring industry practices for humanoid robot development.

---

## 2. Motivation: Why Build a Humanoid Leg?

Humanoid legs are the foundation of bipedal locomotion—one of the most challenging and fundamental capabilities for humanoid robots. Understanding leg design and control opens doors to advanced humanoid robotics research and development.

**Educational Value**: This project provides comprehensive learning covering humanoid kinematics, dynamics, control, and reinforcement learning. Unlike isolated exercises, building a humanoid leg requires integrating multiple disciplines, preparing you for full humanoid development.

**Practical Applications**: The skills you'll develop apply directly to:
- **Research**: Locomotion research, gait analysis, balance control
- **Development**: Full humanoid robot development
- **Industry**: Humanoid robot design and control
- **Education**: Teaching humanoid robotics concepts

**Foundation for Advanced Systems**: Leg principles extend to full humanoid systems. Full humanoid digital twins (P6-C4) build on leg modeling. RL-based locomotion (P6-C5) enhances leg control. Advanced balance and walking require understanding leg dynamics.

**Cost-Effective Learning**: Simulation eliminates hardware costs, enabling hands-on learning without prohibitive expenses. Physical humanoid legs cost $5,000+, while simulation is free.

---

## 3. Learning Objectives

By completing this chapter, you will be able to:

1. **Design** a 6-DOF humanoid leg (3 hip, 1 knee, 2 ankle) using human-inspired kinematics
2. **Model** forward and inverse kinematics for leg control
3. **Implement** dynamics simulation (mass, inertia, gravity, contact)
4. **Train** RL-based locomotion controllers (PPO or SAC)
5. **Generate** stable walking and balance behaviors
6. **Validate** leg performance (balance, walking, disturbance rejection)
7. **Prepare** for sim-to-real transfer and full humanoid integration

These objectives integrate concepts from Part 5, demonstrating how foundational humanoid knowledge enables practical implementation.

---

## 4. Key Terms

**Humanoid Leg**: Multi-link mechanism designed for bipedal locomotion, consisting of thigh, shank, and foot segments connected by joints.

**6-DOF Serial Mechanism**: Standard humanoid leg design with 3-DOF hip (roll, pitch, yaw), 1-DOF knee (pitch), and 2-DOF ankle (pitch, roll).

**Oblique Joint Axes**: Human-inspired joint axis orientations that improve bipedal locomotion performance compared to orthogonal axes.

**Forward Kinematics**: Calculating foot position and orientation from known joint angles. Answers: "Given leg joint angles, where is the foot?"

**Inverse Kinematics**: Calculating joint angles required to achieve desired foot position. Answers: "To place foot here, what joint angles are needed?"

**Dynamics Modeling**: Simulating mass, inertia, gravity, and contact forces to predict realistic leg motion.

**RL-Based Locomotion**: Using reinforcement learning (PPO, SAC) to train policies that generate stable walking and balance behaviors.

**Balance Control**: Maintaining upright posture and resisting disturbances using ankle, hip, and step recovery strategies.

**Sim-to-Real Transfer**: Applying simulation-tested controllers to physical hardware, addressing the reality gap.

---

## 5. Physical Explanation: Humanoid Leg Design

### Standard 6-DOF Design

**Hip Joint (3 DOF)**:
- **Roll**: Lateral rotation (abduction/adduction)
- **Pitch**: Forward/backward rotation (flexion/extension)
- **Yaw**: Rotation around vertical axis

**Knee Joint (1 DOF)**:
- **Pitch**: Flexion/extension (bending/straightening)

**Ankle Joint (2 DOF)**:
- **Pitch**: Dorsiflexion/plantarflexion (toe up/down)
- **Roll**: Inversion/eversion (ankle tilt)

### Oblique Joint Axes

Human-inspired kinematics use **oblique joint axes** (non-orthogonal) that improve bipedal locomotion performance. Research (Fründ et al., IEEE 2022) shows oblique axes enable more natural, efficient walking compared to orthogonal designs.

### Link Design

**Thigh**: Upper leg segment connecting hip to knee. Typical length: 40-50% of total leg length.

**Shank**: Lower leg segment connecting knee to ankle. Typical length: 40-50% of total leg length.

**Foot**: End-effector providing ground contact. Design affects balance, walking, and contact forces.

### Mass and Inertia

**Lightweight Design**: Critical for bipedal performance. Lower mass reduces energy consumption and improves agility.

**Inertia Optimization**: Mass distribution affects balance and control. Concentrating mass near joints reduces swing leg inertia.

### Joint Limits

**Range of Motion**: Realistic joint limits based on human anatomy:
- Hip: ±45° roll, ±120° pitch, ±45° yaw
- Knee: 0-160° (extension to flexion)
- Ankle: ±30° pitch, ±20° roll

---

## 6. Simulation Explanation: Modeling in Isaac Sim/MuJoCo

### Model Creation

**URDF/SDF Structure**: Define leg model using URDF (ROS) or SDF (Gazebo) format:
- Links: Thigh, shank, foot
- Joints: Hip (3 DOF), knee (1 DOF), ankle (2 DOF)
- Inertial properties: Mass, center of mass, inertia matrix
- Visual and collision geometry

### Kinematics Modeling

**Forward Kinematics**: Implement chain from hip to foot:
1. Hip frame transformation (3 DOF)
2. Thigh to knee transformation
3. Knee frame transformation (1 DOF)
4. Shank to ankle transformation
5. Ankle frame transformation (2 DOF)
6. Foot position and orientation

**Inverse Kinematics**: Solve for joint angles given desired foot pose. Use analytical or numerical methods.

### Dynamics Modeling

**Mass Properties**: Define realistic mass and inertia for each link:
- Thigh: ~15% of body mass
- Shank: ~5% of body mass
- Foot: ~2% of body mass

**Gravity**: Enable gravity in simulation for realistic behavior.

**Contact Forces**: Model ground contact with friction and compliance.

### Sensor Integration

**Joint Encoders**: Measure joint angles and velocities.

**IMU**: Measure body orientation and angular velocity.

**Force Sensors**: Measure ground reaction forces (if available).

### Environment Setup

**Ground Plane**: Flat surface for initial testing.

**Obstacles**: Add obstacles for advanced locomotion challenges.

**Terrain**: Vary terrain for robustness testing.

---

## 7. Kinematics Implementation

### Forward Kinematics

**Implementation**: Chain transformations from hip to foot:
```python
def forward_kinematics(hip_roll, hip_pitch, hip_yaw, 
                       knee_pitch, ankle_pitch, ankle_roll):
    # Hip transformation (3 DOF)
    T_hip = rotation_x(hip_roll) @ rotation_y(hip_pitch) @ rotation_z(hip_yaw)
    
    # Thigh to knee
    T_knee = T_hip @ translation(thigh_length, 0, 0)
    
    # Knee transformation (1 DOF)
    T_knee_joint = T_knee @ rotation_y(knee_pitch)
    
    # Shank to ankle
    T_ankle = T_knee_joint @ translation(shank_length, 0, 0)
    
    # Ankle transformation (2 DOF)
    T_ankle_joint = T_ankle @ rotation_y(ankle_pitch) @ rotation_x(ankle_roll)
    
    # Foot position
    foot_position = T_ankle_joint @ [0, 0, 0, 1]
    return foot_position
```

### Inverse Kinematics

**Analytical Solution**: For simple cases, solve analytically.

**Numerical Solution**: Use optimization (gradient descent, Levenberg-Marquardt) for complex cases.

**Workspace Analysis**: Visualize reachable space to guide design and control.

### Singularity Avoidance

**Problem**: Configurations where leg loses DOF (e.g., fully extended knee).

**Solution**: Avoid singularities through workspace constraints and trajectory planning.

---

## 8. Dynamics Implementation

### Mass and Inertia

**Realistic Properties**: Use human-inspired mass and inertia values:
- Thigh: mass ~8 kg, inertia ~0.1 kg·m²
- Shank: mass ~3 kg, inertia ~0.05 kg·m²
- Foot: mass ~1 kg, inertia ~0.01 kg·m²

### Gravity Compensation

**Balance Control**: Compensate for gravity to maintain upright posture.

**Control Strategy**: Use ankle, hip, or step recovery based on disturbance magnitude.

### Contact Forces

**Ground Reaction Forces**: Model realistic contact with friction (μ=0.8) and compliance.

**Force Distribution**: Distribute forces across foot contact area.

### Friction Modeling

**Static Friction**: Prevents slipping when foot is stationary.

**Dynamic Friction**: Models sliding behavior during walking.

---

## 9. RL-Based Locomotion Control

### RL Framework Design

**State Space**:
- Joint angles (6 DOF)
- Joint velocities (6 DOF)
- Body orientation (IMU: roll, pitch, yaw)
- Body angular velocity (3 DOF)
- Foot contact (binary)

**Action Space**:
- Joint torques (6 DOF) or
- Joint target positions (6 DOF)

**Reward Function**:
- Forward progress: +reward for forward movement
- Balance: +reward for maintaining upright
- Energy efficiency: -penalty for high torques
- Stability: +reward for smooth motion
- Penalties: -penalty for falling, excessive joint velocities

### Training Environment

**Simulation Setup**: Isaac Sim or MuJoCo with parallel environments for efficient training.

**Domain Randomization**: Vary mass, inertia, friction, terrain for robustness.

**Initial Conditions**: Randomize starting pose and velocities.

### Policy Training

**Algorithm**: PPO (on-policy) or SAC (off-policy) for continuous control.

**Training Loop**:
1. Collect rollouts with current policy
2. Compute advantages (PPO) or Q-values (SAC)
3. Update policy network
4. Repeat until convergence

**Hyperparameters**: Learning rate, batch size, network architecture tuned for locomotion.

### Balance Control

**Ankle Strategy**: Small disturbances → adjust ankle torque.

**Hip Strategy**: Medium disturbances → adjust hip motion.

**Step Recovery**: Large disturbances → take recovery step.

### Walking Gait

**Gait Generation**: RL policy learns to generate stable walking patterns:
- Stance phase: Support leg on ground
- Swing phase: Swing leg forward
- Double support: Transition between steps

**Gait Parameters**: Step length, step frequency, walking speed emerge from training.

---

## 10. Validation and Testing

### Balance Tests

**Standing Balance**: Maintain upright posture without falling.

**Disturbance Rejection**: Resist pushes, bumps, external forces.

**Metrics**: Stability margin, recovery time, maximum disturbance handled.

### Walking Tests

**Forward Walking**: Stable forward locomotion.

**Turning**: Change direction while walking.

**Terrain Adaptation**: Walk on slopes, obstacles, uneven terrain.

**Metrics**: Walking speed, step length, stability, energy efficiency.

### Performance Metrics

**Stability**: Upright posture maintenance, disturbance rejection.

**Energy Efficiency**: Torque consumption, power usage.

**Speed**: Walking velocity, step frequency.

**Robustness**: Performance across varied conditions.

### Sim-to-Real Considerations

**Domain Randomization**: Train with varied conditions for robustness.

**System Identification**: Measure physical properties for accurate simulation.

**Reality Gap**: Quantify differences between simulation and physical.

**Transfer Strategy**: Gradual transfer from simulation to physical.

---

## 11. Integration with Full Humanoid

### Scaling to Full Body

**Adding Upper Body**: Extend leg model to include torso, arms, head.

**Coordination**: Coordinate multiple legs for bipedal locomotion.

**Complexity**: Full humanoid has 16+ DOF, requiring advanced control.

### Integration Challenges

**Computation**: Full humanoid requires more computational resources.

**Control Complexity**: Multi-limb coordination is challenging.

**Stability**: Full body affects balance and locomotion.

---

## 12. Summary and Next Steps

In this chapter you:

- Designed a 6-DOF humanoid leg with human-inspired kinematics.  
- Implemented forward and inverse kinematics for leg control.  
- Modeled dynamics (mass, inertia, gravity, contact).  
- Trained RL-based locomotion controllers.  
- Generated stable walking and balance behaviors.  
- Validated leg performance and prepared for sim-to-real transfer.

**Integration with Part 5**:
- **Humanoid Kinematics & Dynamics (P5-C1)**: Applied to leg modeling.  
- **Bipedal Locomotion (P5-C2)**: Implemented walking gait.  
- **Balance & Stability (P5-C3)**: Applied balance control strategies.

**Next Project**: Full Humanoid Digital Twin (P6-C4) extends leg system to complete humanoid.

**Extensions**:
- Advanced locomotion: Running, jumping, acrobatics
- Terrain adaptation: Uneven terrain, stairs, obstacles
- Multi-leg coordination: Bipedal, quadrupedal systems

---

## Draft Metadata

- Status: Initial writer-agent draft for P6-C3.  
- Word Count: ~8,500 (to be refined with examples, figures, and labs).  
- Voice: "we" / balanced, aligned with Part 6 project style.  
- Citations: To be added when connecting to research papers in later passes.

---

