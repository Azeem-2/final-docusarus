# Chapter: Bipedal Locomotion (P5-C2)

---
title: Bipedal Locomotion
slug: /P5-C2-bipedal-locomotion
sidebar_label: Bipedal Locomotion
sidebar_position: 2
---

## 1. Introduction – Walking on Two Legs

Humanoid robots must walk, run, and navigate like humans. **Bipedal locomotion**—walking on two legs—is one of the most challenging and fundamental capabilities for humanoid robots.

In this chapter, you will learn:

- **Walking gait fundamentals**: Stance phase, swing phase, gait cycles.  
- **ZMP (Zero Moment Point) control**: Maintaining balance during walking.  
- **Capture point control**: Predictive balance recovery.  
- **Model Predictive Control (MPC)**: Optimized walking trajectories.  
- **Terrain adaptation**: Walking on slopes, obstacles, uneven terrain.  
- **Implementation**: Simulation and physical deployment.

The goal is to understand how humanoid robots achieve stable, efficient bipedal locomotion.

---

## 2. Walking Gait Fundamentals

**Walking** involves a cyclic pattern of leg movements.

### Gait Cycle

A **gait cycle** consists of:
- **Stance phase**: Foot is on ground, supporting body weight.  
- **Swing phase**: Foot is in air, moving forward.  
- **Double support**: Both feet on ground (transition between steps).  
- **Single support**: One foot on ground (most of walking).

### Step Timing

- **Step frequency**: Steps per second.  
- **Step length**: Distance between foot placements.  
- **Walking speed**: Step frequency × step length.

### Human-Like Walking

Efficient human-like walking:
- Straight legs during stance (energy efficient).  
- Smooth CoM motion (minimizes energy).  
- Natural arm swing (balance and efficiency).

---

## 3. Zero Moment Point (ZMP) Control

**Zero Moment Point (ZMP)** is a key concept for balance during walking.

### ZMP Definition

The **ZMP** is the point where the net moment (torque) is zero:
- If ZMP is within support polygon: Robot is stable.  
- If ZMP leaves support polygon: Robot will fall.

### Support Polygon

The **support polygon** is the area of contact between foot and ground:
- Single support: Area of one foot.  
- Double support: Convex hull of both feet.

### ZMP-Based Walking

ZMP-based control:
1. **Plan ZMP trajectory**: Keep ZMP within support polygon.  
2. **Generate CoM trajectory**: Compute center of mass motion from ZMP.  
3. **Execute joint trajectories**: Achieve CoM motion through joint control.

### Advantages

- **Stability**: Ensures balance during walking.  
- **Predictable**: Well-understood control approach.  
- **Proven**: Used in many humanoid robots.

---

## 4. Capture Point Control

**Capture point** enables predictive balance recovery.

### Capture Point Definition

The **capture point** is the point on the ground where the robot can come to rest:
- Depends on CoM position and velocity.  
- If robot steps on capture point: Can stop and balance.

### Capture Point Control

- **Balance recovery**: Step toward capture point to recover balance.  
- **Step placement**: Place foot at capture point for stability.  
- **Predictive**: Anticipates balance needs.

### Advantages

- **Predictive**: Anticipates balance recovery needs.  
- **Robust**: Handles larger disturbances.  
- **Natural**: Similar to human balance recovery.

---

## 5. Model Predictive Control (MPC) for Walking

**Model Predictive Control (MPC)** optimizes walking trajectories.

### MPC Overview

MPC:
1. **Predict**: Predict future states over a horizon.  
2. **Optimize**: Optimize control inputs to minimize cost.  
3. **Execute**: Apply first control input, repeat.

### MPC for Walking

- **Constraints**: Joint limits, balance (ZMP), contact forces.  
- **Cost function**: Energy, smoothness, tracking error.  
- **Real-time**: Fast solvers for real-time control.

### Advantages

- **Optimal**: Optimizes walking trajectories.  
- **Constraint handling**: Naturally handles limits.  
- **Robust**: Handles disturbances and uncertainties.

---

## 6. Gait Generation

**Gait generation** creates walking patterns.

### Walking Pattern Generation

- **Step planning**: Plan foot placements.  
- **CoM trajectory**: Generate center of mass motion.  
- **Joint trajectories**: Compute joint angles from CoM motion.

### Trajectory Smoothing

- **Smooth motions**: Avoid jerky movements.  
- **Energy efficiency**: Minimize energy consumption.  
- **Natural appearance**: Human-like walking.

---

## 7. Terrain Adaptation

Real-world walking requires **terrain adaptation**.

### Slope Walking

- **Uphill**: Adjust step length, lean forward.  
- **Downhill**: Adjust step length, lean back.  
- **Side slopes**: Adjust lateral balance.

### Obstacle Avoidance

- **Step over**: Lift foot higher, longer step.  
- **Step around**: Adjust step placement.  
- **Planning**: Plan foot placements to avoid obstacles.

### Uneven Terrain

- **Adaptive stepping**: Adjust to surface height.  
- **Robust control**: Handle unexpected terrain.  
- **Sensor feedback**: Use vision, force sensors.

---

## 8. Energy Efficiency

**Energy efficiency** is important for battery-powered robots.

### Minimizing Energy

- **Straight legs**: Reduce knee torque during stance.  
- **Smooth CoM motion**: Minimize accelerations.  
- **Efficient gaits**: Optimize step frequency and length.

### Trade-offs

- **Speed vs energy**: Faster walking uses more energy.  
- **Stability vs efficiency**: More stable may be less efficient.  
- **Terrain vs efficiency**: Rough terrain requires more energy.

---

## 9. Implementation: Simulation and Physical

### Simulation

- **Physics engines**: MuJoCo, Gazebo, Isaac Sim.  
- **Controller testing**: Test walking controllers safely.  
- **Parameter tuning**: Optimize controller parameters.

### Physical Deployment

- **Hardware**: Real humanoid robots.  
- **Sensors**: IMU, force sensors, joint encoders.  
- **Real-time control**: Fast control loops.

### Sim-to-Real Transfer

- **Reality gap**: Differences between simulation and reality.  
- **Robust controllers**: Controllers that work in both.  
- **Domain randomization**: Train in diverse simulations.

---

## 10. Summary and Bridge to Balance & Stability

In this chapter you:

- Learned that walking involves cyclic stance and swing phases.  
- Explored ZMP control for maintaining balance during walking.  
- Understood capture point for predictive balance recovery.  
- Recognized MPC for optimized walking trajectories.  
- Saw terrain adaptation and energy efficiency considerations.

**Integration with Part 5**:
- **Humanoid kinematics & dynamics (P5-C1)**: Foundation for walking control.  
- **Bipedal locomotion (P5-C2)**: Walking control and gaits.  
- **Balance & stability (P5-C3)**: Maintaining balance during walking.

In the next chapter (P5-C3: Balance & Stability), you'll see how humanoid robots maintain balance and recover from disturbances, building on the walking control concepts from this chapter.

---

