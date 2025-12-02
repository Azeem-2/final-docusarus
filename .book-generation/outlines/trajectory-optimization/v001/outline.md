# Chapter Outline – Trajectory Optimization (P4-C5)

---
chapter_id: P4-C5
title: Trajectory Optimization
version: v001
created: 2025-12-01
---

## 1. Introduction – Optimal Motion Trajectories

- Why trajectory optimization: Path planning gives geometry, optimization gives timing and smoothness.  
- Real-world motivation: Smooth, time-optimal, energy-efficient robot motion.  
- Key methods: Quadratic programming, nonlinear optimization, direct collocation.

## 2. Path vs Trajectory

- Path: Geometric sequence of configurations.  
- Trajectory: Path with timing information (velocities, accelerations).  
- Why timing matters: Smoothness, energy, time constraints.

## 3. Cost Functions

- Time-optimal: Minimizing execution time.  
- Smoothness: Minimizing jerk, acceleration.  
- Energy-optimal: Minimizing torque, power consumption.  
- Multi-objective: Weighted combinations.

## 4. Quadratic Programming (QP)

- Linear dynamics, quadratic cost.  
- Fast solvers: Efficient computation.  
- Applications: Simple cost functions, real-time optimization.

## 5. Nonlinear Optimization

- General cost functions and constraints.  
- Iterative solvers: Gradient-based methods.  
- Applications: Complex objectives, non-convex problems.

## 6. Direct Collocation

- Discretize trajectory into waypoints.  
- Optimize waypoints directly.  
- Constraint handling: Natural integration of limits.

## 7. Shooting Methods

- Optimize control inputs.  
- Forward simulation: Get trajectory from controls.  
- Sensitivity: May require good initial guess.

## 8. Constraint Handling

- Joint limits: Position, velocity, acceleration bounds.  
- Obstacles: Collision avoidance constraints.  
- Dynamic constraints: Torque limits, stability.

## 9. Real-Time Trajectory Optimization

- Fast solvers: Efficient algorithms.  
- Warm starts: Reusing previous solutions.  
- Reactive control: Adapting to sensor input.

## 10. Summary and Integration

- Key takeaways: Trajectory optimization enables smooth, optimal motion.  
- Integration: Connects motion planning (P3-C5) to control policies (P4-C3).  
- Bridge: How optimized trajectories integrate with learned policies.

---

