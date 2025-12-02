# Chapter Outline – Bipedal Locomotion (P5-C2)

---
chapter_id: P5-C2
title: Bipedal Locomotion
version: v001
created: 2025-12-01
---

## 1. Introduction – Walking on Two Legs

- Why bipedal locomotion: Human-like form factor, energy efficiency, manipulation advantages.  
- Real-world motivation: Humanoid robots need to walk, run, navigate.  
- Key challenges: Balance, terrain adaptation, energy efficiency.

## 2. Walking Gait Fundamentals

- Gait cycle: Stance phase (foot on ground) and swing phase (foot in air).  
- Double support vs single support: Transition phases.  
- Step timing: When to lift and place feet.

## 3. Zero Moment Point (ZMP) Control

- ZMP definition: Point where net moment is zero.  
- Support polygon: Area of foot contact with ground.  
- ZMP constraint: ZMP must stay within support polygon for stability.  
- ZMP-based walking: Plan ZMP trajectory, generate CoM motion.

## 4. Capture Point Control

- Capture point: Point where robot can come to rest.  
- Capture point control: Use capture point for balance recovery.  
- Step placement: Adjust step placement based on capture point.

## 5. Model Predictive Control (MPC) for Walking

- MPC overview: Predict future states, optimize control.  
- MPC for locomotion: Handle constraints, optimize walking trajectories.  
- Real-time optimization: Fast solvers for walking control.

## 6. Gait Generation

- Walking pattern generation: Create walking trajectories.  
- Step planning: Plan foot placements.  
- Trajectory smoothing: Smooth walking motions.

## 7. Terrain Adaptation

- Slope walking: Walking on inclines and declines.  
- Obstacle avoidance: Stepping over obstacles.  
- Uneven terrain: Adapting to rough surfaces.

## 8. Energy Efficiency

- Minimizing energy consumption.  
- Efficient walking patterns.  
- Trade-offs: Speed vs energy.

## 9. Implementation: Simulation and Physical

- Simulation: Testing walking controllers in simulation.  
- Physical deployment: Deploying on real humanoid robots.  
- Challenges: Sim-to-real transfer for walking.

## 10. Summary and Bridge to Balance & Stability

- Key takeaways: Bipedal locomotion requires balance control and gait planning.  
- Integration: Connects kinematics/dynamics (P5-C1) to balance (P5-C3).  
- Bridge to P5-C3: How balance control enables stable walking.

---

