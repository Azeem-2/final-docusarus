---
title: Trajectory Optimization
slug: /part4/chapter5-trajectory-optimization
sidebar_label: Trajectory Optimization
sidebar_position: 5
---

# Chapter: Trajectory Optimization (P4-C5)
## 1. Introduction – Optimal Motion Trajectories

Motion planning (P3-C5) finds a path through configuration space, but doesn't specify timing or smoothness. **Trajectory optimization** finds optimal motion trajectories that are smooth, time-efficient, and energy-efficient.

In this chapter, you will learn:

- **Path vs trajectory**: Geometric path vs trajectory with timing.  
- **Cost functions**: Time-optimal, smoothness, energy-optimal.  
- **Optimization methods**: Quadratic programming, nonlinear optimization, direct collocation.  
- **Constraint handling**: Joint limits, obstacles, dynamic constraints.  
- **Real-time optimization**: Fast solvers, warm starts.

The goal is to understand how to optimize robot trajectories for smooth, efficient motion that respects physical constraints.

---

## 2. Path vs Trajectory

### Path

A **path** is a geometric sequence of configurations:
- **No timing**: Just positions in configuration space.  
- **Geometric**: Where to go, not when or how fast.  
- **Example**: Sequence of joint angles for a manipulation task.

### Trajectory

A **trajectory** is a path with timing information:
- **With timing**: Positions, velocities, accelerations over time.  
- **Complete motion**: How to move, not just where.  
- **Example**: Joint angles, velocities, and accelerations as functions of time.

### Why Timing Matters

- **Smoothness**: Jerk and acceleration affect motion quality.  
- **Energy**: Torque and power consumption depend on velocities and accelerations.  
- **Time**: Execution time affects task efficiency.  
- **Constraints**: Joint limits, velocity limits, torque limits.

---

## 3. Cost Functions

The **cost function** defines what makes a trajectory "good."

### Time-Optimal

Minimize **execution time**:
- Fastest possible motion.  
- May require high accelerations.  
- Useful when speed is critical.

### Smoothness

Minimize **jerk or acceleration**:
- Smooth, comfortable motion.  
- Reduces wear on robot.  
- Better for human-robot interaction.

### Energy-Optimal

Minimize **torque or power consumption**:
- Efficient energy use.  
- Important for battery-powered robots.  
- Extends operating time.

### Multi-Objective

**Weighted combination** of objectives:
- Balance competing goals (speed vs smoothness).  
- Adjust weights based on task requirements.  
- Example: 0.7 × time + 0.3 × smoothness.

---

## 4. Quadratic Programming (QP)

**Quadratic programming** solves optimization problems with:
- **Quadratic cost**: Cost function is quadratic in variables.  
- **Linear constraints**: Constraints are linear.  
- **Fast solvers**: Efficient algorithms available.

### When to Use QP

- Simple cost functions (quadratic).  
- Linear dynamics.  
- Real-time optimization needs.  
- Simple constraints.

### Example

Optimize trajectory with quadratic cost on accelerations and linear joint limits.

---

## 5. Nonlinear Optimization

**Nonlinear optimization** handles general cost functions and constraints:
- **General costs**: Any cost function.  
- **Nonlinear constraints**: Complex constraint relationships.  
- **Iterative solvers**: Gradient-based methods.

### When to Use Nonlinear Optimization

- Complex cost functions.  
- Nonlinear dynamics.  
- Complex constraints (obstacles, non-convex).  
- When QP is insufficient.

### Solvers

- **Gradient descent**: First-order methods.  
- **Newton's method**: Second-order methods.  
- **Interior point**: Constraint handling.

---

## 6. Direct Collocation

**Direct collocation** discretizes the trajectory and optimizes waypoints directly:
- **Discretization**: Break trajectory into waypoints.  
- **Optimize waypoints**: Find optimal waypoint positions.  
- **Constraint handling**: Natural integration of limits.

### Advantages

- **Natural constraints**: Easy to add joint limits, obstacles.  
- **Flexible**: Can handle complex constraints.  
- **Stable**: Well-conditioned optimization problems.

### Disadvantages

- **Many variables**: One waypoint per time step.  
- **Computational cost**: Larger optimization problems.

---

## 7. Shooting Methods

**Shooting methods** optimize control inputs and simulate forward:
- **Control inputs**: Optimize torques or forces.  
- **Forward simulation**: Get trajectory from controls.  
- **Sensitivity**: May require good initial guess.

### Advantages

- **Fewer variables**: Only control inputs, not all waypoints.  
- **Physics-based**: Respects dynamics naturally.

### Disadvantages

- **Sensitivity**: Sensitive to initial guess.  
- **Simulation cost**: Requires forward simulation.

---

## 8. Constraint Handling

Real robots have **constraints** that must be respected.

### Joint Limits

- **Position limits**: Joint angles within valid range.  
- **Velocity limits**: Maximum joint velocities.  
- **Acceleration limits**: Maximum joint accelerations.

### Obstacles

- **Collision avoidance**: Trajectory must avoid obstacles.  
- **Safety margins**: Maintain distance from obstacles.  
- **Dynamic obstacles**: Moving obstacles require replanning.

### Dynamic Constraints

- **Torque limits**: Actuator torque constraints.  
- **Stability**: Maintain balance, prevent tipping.  
- **Power limits**: Maximum power consumption.

---

## 9. Real-Time Trajectory Optimization

For reactive control, optimization must be **fast**.

### Fast Solvers

- **Efficient algorithms**: QP solvers, specialized methods.  
- **GPU acceleration**: Parallel computation.  
- **Approximate methods**: Trade accuracy for speed.

### Warm Starts

- **Reuse solutions**: Initialize with previous solution.  
- **Incremental updates**: Update trajectory as conditions change.  
- **Reduces computation**: Faster convergence.

### Reactive Control

- **Sensor feedback**: Adapt to changing conditions.  
- **Replanning**: Update trajectory when obstacles appear.  
- **Real-time loop**: Optimize → execute → sense → repeat.

---

## 10. Summary and Integration

In this chapter you:

- Learned that trajectories include timing, paths don't.  
- Explored cost functions: time, smoothness, energy.  
- Understood optimization methods: QP, nonlinear, direct collocation.  
- Recognized constraint handling: limits, obstacles, dynamics.  
- Saw real-time optimization techniques.

**Integration with Part 4**:
- **Motion planning (P3-C5)**: Provides initial path.  
- **Trajectory optimization (P4-C5)**: Optimizes path into trajectory.  
- **Control policies (P4-C3)**: Can use optimized trajectories or learn directly.

Trajectory optimization bridges motion planning and control, enabling smooth, efficient robot motion.

