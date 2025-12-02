# Lessons Blueprint: P5-C2 Bipedal Locomotion

**Chapter ID**: P5-C2  
**Version**: v001  
**Created**: 2025-12-01

---

## Lesson 1: Walking Gait Fundamentals and ZMP Control

- **Pedagogical Layer**: Layer 1–2 – Intuition & Concepts  
- **Estimated Time**: 90–120 minutes  
- **Learning Outcomes**:  
  1. Explain walking gait cycle (stance, swing, double support).  
  2. Understand Zero Moment Point (ZMP) and its role in balance.  
  3. Describe ZMP-based walking control.

### Parts 1–6

- **Hook**: A humanoid robot needs to walk, but how do we control two legs to maintain balance while moving forward?  
- **Theory**:  
  - **Walking gait cycle**:  
    - Stance phase: Foot on ground, supporting body weight.  
    - Swing phase: Foot in air, moving forward.  
    - Double support: Both feet on ground (transition).  
    - Single support: One foot on ground.  
  - **Zero Moment Point (ZMP)**: Point where net moment is zero.  
    - Support polygon: Area of foot contact with ground.  
    - ZMP constraint: ZMP must stay within support polygon for stability.  
  - **ZMP-based walking**:  
    - Plan ZMP trajectory within support polygon.  
    - Generate center of mass (CoM) trajectory.  
    - Execute joint trajectories to achieve CoM motion.  
- **Walkthrough**:  
  - Show walking gait cycle: stance → swing → double support.  
  - Demonstrate ZMP calculation and support polygon.  
  - Walk through ZMP-based control: plan ZMP → generate CoM → execute.  
- **Challenge**:  
  - Students design a simple walking controller:  
    1. Identify gait phases.  
    2. Plan ZMP trajectory for one step.  
    3. Design CoM trajectory generation.  
- **Takeaways**:  
  - Walking involves cyclic stance and swing phases.  
  - ZMP must stay within support polygon for stability.  
  - ZMP-based control enables stable walking.  
- **Learn with AI**:  
  - `walking_gait_designer`: RI component that helps students design walking gaits and ZMP trajectories.

---

## Lesson 2: Capture Point and MPC for Walking

- **Pedagogical Layer**: Layer 2–3 – Concepts & Application  
- **Estimated Time**: 90–120 minutes  
- **Learning Outcomes**:  
  1. Understand capture point and its use in balance recovery.  
  2. Explain Model Predictive Control (MPC) for walking.  
  3. Compare ZMP, capture point, and MPC approaches.

### Parts 1–6

- **Hook**: ZMP control works, but can we predict and plan better? Capture point and MPC offer more predictive control.  
- **Theory**:  
  - **Capture point**: Point where robot can come to rest.  
    - Depends on CoM position and velocity.  
    - Used for balance recovery and step placement.  
  - **Model Predictive Control (MPC)**:  
    - Predict future states and optimize control.  
    - Handle constraints (joint limits, balance).  
    - Real-time optimization for walking trajectories.  
  - **Comparison**:  
    - ZMP: Reactive, within support polygon.  
    - Capture point: Predictive, for recovery.  
    - MPC: Optimized, handles constraints.  
- **Walkthrough**:  
  - Show capture point calculation and use for step placement.  
  - Demonstrate MPC: predict → optimize → execute.  
  - Compare approaches: when to use each.  
- **Challenge**:  
  - Students choose control approach:  
    1. Identify task requirements (reactive vs predictive).  
    2. Choose ZMP, capture point, or MPC.  
    3. Justify choice.  
- **Takeaways**:  
  - Capture point enables predictive balance recovery.  
  - MPC optimizes walking trajectories with constraints.  
  - Different approaches suit different scenarios.  
- **Learn with AI**:  
  - `walking_control_selector`: RI component that helps students choose appropriate walking control approaches.

---

## Lesson 3: Gait Generation, Terrain Adaptation, and Implementation

- **Pedagogical Layer**: Layer 3–4 – Application & Integration  
- **Estimated Time**: 90–120 minutes  
- **Learning Outcomes**:  
  1. Understand gait generation and step planning.  
  2. Explain terrain adaptation (slopes, obstacles, uneven terrain).  
  3. Describe implementation in simulation and on physical robots.

### Parts 1–6

- **Hook**: Walking on flat ground is one thing, but real robots need to handle slopes, obstacles, and rough terrain. How do we adapt?  
- **Theory**:  
  - **Gait generation**:  
    - Walking pattern generation: Create walking trajectories.  
    - Step planning: Plan foot placements.  
    - Trajectory smoothing: Smooth walking motions.  
  - **Terrain adaptation**:  
    - Slope walking: Adjust for inclines and declines.  
    - Obstacle avoidance: Step over obstacles.  
    - Uneven terrain: Adapt to rough surfaces.  
  - **Energy efficiency**:  
    - Minimize energy consumption.  
    - Efficient walking patterns.  
    - Speed vs energy trade-offs.  
  - **Implementation**:  
    - Simulation: Test controllers in simulation.  
    - Physical deployment: Deploy on real robots.  
    - Sim-to-real transfer: Handle reality gap.  
- **Walkthrough**:  
  - Show gait generation: pattern → step plan → trajectory.  
  - Demonstrate terrain adaptation: slope detection → gait adjustment.  
  - Walk through implementation: simulation → physical deployment.  
- **Challenge**:  
  - Students design terrain-adaptive walking:  
    1. Identify terrain type (slope, obstacle, rough).  
    2. Design adaptation strategy.  
    3. Plan implementation (simulation first).  
- **Takeaways**:  
  - Gait generation creates walking patterns.  
  - Terrain adaptation enables robust walking.  
  - Implementation requires careful sim-to-real transfer.  
- **Learn with AI**:  
  - `terrain_adaptive_walking_designer`: RI component that helps students design terrain-adaptive walking controllers.

---

