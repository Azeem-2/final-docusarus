# Chapter: Balance & Stability (P5-C3)

---
title: Balance & Stability
slug: /P5-C3-balance-stability
sidebar_label: Balance & Stability
sidebar_position: 3
---

## 1. Introduction – Maintaining Upright Posture

Humanoid robots must maintain stable upright posture to walk, manipulate, and interact. **Balance and stability** are fundamental to all humanoid capabilities.

In this chapter, you will learn:

- **Balance metrics**: ZMP, CoP, capture point, stability margins.  
- **Balance control strategies**: Ankle strategy, hip strategy, step recovery.  
- **Disturbance rejection**: Handling pushes, bumps, external forces.  
- **Implementation**: Balance controllers, sensor integration, real-time control.

The goal is to understand how humanoid robots maintain balance and recover from disturbances.

---

## 2. Balance Metrics: ZMP and CoP

**Balance metrics** assess the robot's balance state.

### Zero Moment Point (ZMP)

The **ZMP** is the point where net moment is zero:
- **Within support polygon**: Robot is stable.  
- **Outside support polygon**: Robot will fall.  
- **Calculation**: From forces and moments.

### Center of Pressure (CoP)

The **CoP** is the point where ground reaction force acts:
- **Measured**: By force sensors in feet.  
- **Related to ZMP**: CoP ≈ ZMP when robot is balanced.  
- **Real-time**: Can be measured in real-time.

### Relationship

- **Balanced robot**: ZMP and CoP are within support polygon.  
- **Unbalanced robot**: ZMP or CoP approaches support polygon edge.  
- **Falling**: ZMP or CoP leaves support polygon.

---

## 3. Capture Point

**Capture point** predicts balance recovery.

### Capture Point Definition

The **capture point** is the point on the ground where the robot can come to rest:
- **Calculation**: Based on CoM position and velocity.  
- **Interpretation**: If robot steps here, can stop and balance.

### Balance Recovery

- **Step toward capture point**: Recover balance.  
- **Step placement**: Place foot at capture point.  
- **Predictive**: Anticipates balance needs.

### Advantages

- **Predictive**: Anticipates recovery needs.  
- **Robust**: Handles larger disturbances.  
- **Natural**: Similar to human recovery.

---

## 4. Stability Margins

**Stability margins** provide safety buffers.

### ZMP Margin

- **Definition**: Distance from ZMP to support polygon edge.  
- **Interpretation**: Larger margin = more stable.  
- **Minimum margin**: Safety threshold for stability.

### CoM Margin

- **Definition**: Distance from CoM projection to support polygon edge.  
- **Interpretation**: Larger margin = more stable.  
- **Use**: Assess balance state.

### Safety Margins

- **Additional buffers**: Extra margins for robustness.  
- **Disturbance handling**: Margins help handle unexpected forces.  
- **Design**: Choose margins based on expected disturbances.

---

## 5. Balance Control Strategies: Ankle Strategy

**Ankle strategy** adjusts ankle torque for balance.

### How It Works

- **Small disturbances**: Adjust ankle torque.  
- **Fast response**: Quick torque adjustments.  
- **Limited range**: Ankle has limited motion range.

### When to Use

- **Small pushes**: Disturbances that can be handled by ankle.  
- **Fast response needed**: Quick balance corrections.  
- **Standing balance**: Maintaining upright posture.

### Implementation

- **Ankle torque control**: Control ankle joint torque.  
- **Force sensors**: Use CoP feedback.  
- **Real-time**: Fast control loop.

---

## 6. Balance Control Strategies: Hip Strategy

**Hip strategy** adjusts hip motion to shift CoM.

### How It Works

- **Larger disturbances**: Move hips to shift CoM.  
- **Slower response**: Hip motion takes time.  
- **More range**: Hip has larger motion range.

### When to Use

- **Medium pushes**: Disturbances too large for ankle.  
- **CoM shifting**: Need to shift center of mass.  
- **Recovery**: Part of recovery strategy.

### Implementation

- **Hip motion control**: Control hip joint motion.  
- **CoM control**: Shift center of mass.  
- **Coordination**: Coordinate with ankle strategy.

---

## 7. Step Recovery

**Step recovery** takes a step to recover balance.

### How It Works

- **Large disturbances**: Take step to recover.  
- **Step planning**: Plan optimal step placement.  
- **Execution**: Execute step quickly.

### When to Use

- **Large pushes**: Disturbances too large for ankle/hip.  
- **Balance loss**: When balance is about to be lost.  
- **Recovery**: Last resort for balance recovery.

### Implementation

- **Step planning**: Plan step placement (e.g., at capture point).  
- **Trajectory generation**: Generate step trajectory.  
- **Execution**: Execute step quickly.

---

## 8. Disturbance Rejection

**Disturbance rejection** maintains balance under external forces.

### Handling Pushes

- **Detect**: Sense external force (force sensors, IMU).  
- **Assess**: Determine disturbance magnitude.  
- **Recover**: Apply appropriate recovery strategy.

### Robustness

- **Multiple strategies**: Combine ankle, hip, step recovery.  
- **Adaptive**: Choose strategy based on disturbance.  
- **Recovery**: Recover to stable state.

### Multi-Strategy Coordination

- **Small disturbance**: Ankle strategy.  
- **Medium disturbance**: Hip strategy.  
- **Large disturbance**: Step recovery.  
- **Combination**: Use multiple strategies together.

---

## 9. Implementation: Balance Controllers

### Balance Controller Design

- **Sensor integration**: IMU, force sensors, joint encoders.  
- **Real-time control**: Fast control loops (1kHz+).  
- **Multi-strategy**: Coordinate ankle, hip, step recovery.

### Sensor Integration

- **IMU**: Measure orientation and angular velocity.  
- **Force sensors**: Measure ground reaction forces, CoP.  
- **Joint encoders**: Measure joint positions and velocities.

### Real-Time Balance Control

- **Fast loops**: High-frequency control (1kHz+).  
- **Low latency**: Quick response to disturbances.  
- **Stability**: Ensure control loop stability.

---

## 10. Summary and Integration

In this chapter you:

- Learned that balance requires metrics (ZMP, CoP, capture point) and stability margins.  
- Explored balance control strategies: ankle (small disturbances), hip (medium), step recovery (large).  
- Understood disturbance rejection through multi-strategy coordination.  
- Recognized implementation considerations: sensors, real-time control, robustness.

**Integration with Part 5**:
- **Humanoid kinematics & dynamics (P5-C1)**: Foundation for balance analysis.  
- **Bipedal locomotion (P5-C2)**: Balance during walking.  
- **Balance & stability (P5-C3)**: Maintaining balance in all scenarios.

Balance and stability are fundamental to all humanoid capabilities, enabling walking, manipulation, and interaction.

---

## Draft Metadata

- Status: Initial writer-agent draft for P5-C3.  
- Word Count: ~1,600 (to be refined with examples, figures, and labs).  
- Voice: "we" / balanced, aligned with Part 5 style.  
- Citations: To be added when connecting to standard balance and stability references and research papers in later passes.

