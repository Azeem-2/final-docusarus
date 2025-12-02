# Lessons Blueprint: P5-C3 Balance & Stability

**Chapter ID**: P5-C3  
**Version**: v001  
**Created**: 2025-12-01

---

## Lesson 1: Balance Metrics: ZMP, CoP, and Capture Point

- **Pedagogical Layer**: Layer 1–2 – Intuition & Concepts  
- **Estimated Time**: 90–120 minutes  
- **Learning Outcomes**:  
  1. Explain balance metrics: ZMP, CoP, capture point.  
  2. Understand stability margins.  
  3. Describe how these metrics assess balance.

### Parts 1–6

- **Hook**: A humanoid robot is pushed. How do we know if it will fall? Balance metrics tell us.  
- **Theory**:  
  - **Zero Moment Point (ZMP)**: Point where net moment is zero.  
    - Must stay within support polygon for stability.  
    - Used for balance assessment and control.  
  - **Center of Pressure (CoP)**: Point where ground reaction force acts.  
    - Measured by force sensors.  
    - Related to ZMP.  
  - **Capture point**: Point where robot can come to rest.  
    - Depends on CoM position and velocity.  
    - Used for balance recovery.  
  - **Stability margins**:  
    - ZMP margin: Distance from ZMP to support polygon edge.  
    - CoM margin: Distance from CoM to support polygon edge.  
    - Safety margins: Additional margins for robustness.  
- **Walkthrough**:  
  - Show ZMP calculation and support polygon.  
  - Demonstrate CoP measurement from force sensors.  
  - Calculate capture point from CoM state.  
  - Show stability margins and their interpretation.  
- **Challenge**:  
  - Students assess balance state:  
    1. Calculate ZMP from robot state.  
    2. Determine if robot is stable (ZMP in support polygon).  
    3. Compute stability margin.  
- **Takeaways**:  
  - ZMP and CoP assess current balance state.  
  - Capture point predicts balance recovery.  
  - Stability margins provide safety buffers.  
- **Learn with AI**:  
  - `balance_metric_calculator`: RI component that helps students calculate and interpret balance metrics.

---

## Lesson 2: Balance Control Strategies: Ankle, Hip, and Step Recovery

- **Pedagogical Layer**: Layer 2–3 – Concepts & Application  
- **Estimated Time**: 90–120 minutes  
- **Learning Outcomes**:  
  1. Understand ankle strategy for balance control.  
  2. Explain hip strategy and step recovery.  
  3. Compare different balance control strategies.

### Parts 1–6

- **Hook**: A robot is pushed. Should it adjust its ankles, move its hips, or take a step? Different strategies for different disturbances.  
- **Theory**:  
  - **Ankle strategy**: Adjust ankle torque to maintain balance.  
    - Small disturbances: Fast response, limited range.  
    - Implementation: Ankle torque control.  
  - **Hip strategy**: Adjust hip motion to shift CoM.  
    - Larger disturbances: Slower response, more range.  
    - Implementation: Hip motion control.  
  - **Step recovery**: Take step to recover balance.  
    - Large disturbances: Requires planning and execution.  
    - Step placement: Optimal placement for recovery.  
  - **Strategy selection**:  
    - Small disturbance → ankle strategy.  
    - Medium disturbance → hip strategy.  
    - Large disturbance → step recovery.  
- **Walkthrough**:  
  - Show ankle strategy: small push → ankle torque adjustment.  
  - Demonstrate hip strategy: medium push → hip motion.  
  - Walk through step recovery: large push → step planning → execution.  
- **Challenge**:  
  - Students design balance recovery:  
    1. Identify disturbance magnitude.  
    2. Choose appropriate strategy (ankle, hip, step).  
    3. Design recovery action.  
- **Takeaways**:  
  - Ankle strategy handles small disturbances quickly.  
  - Hip strategy handles medium disturbances.  
  - Step recovery handles large disturbances.  
- **Learn with AI**:  
  - `balance_recovery_planner`: RI component that helps students design balance recovery strategies.

---

## Lesson 3: Disturbance Rejection and Implementation

- **Pedagogical Layer**: Layer 3–4 – Application & Integration  
- **Estimated Time**: 90–120 minutes  
- **Learning Outcomes**:  
  1. Understand disturbance rejection strategies.  
  2. Explain robustness and recovery from pushes.  
  3. Describe balance controller implementation.

### Parts 1–6

- **Hook**: Real robots face unexpected pushes, bumps, and disturbances. How do we make balance controllers robust?  
- **Theory**:  
  - **Disturbance rejection**:  
    - Handling pushes: Recovering from external forces.  
    - Robustness: Maintaining balance under disturbances.  
    - Recovery strategies: Combining ankle, hip, and step recovery.  
  - **Balance controller design**:  
    - Sensor integration: IMU, force sensors, joint encoders.  
    - Real-time balance control: Fast response to disturbances.  
    - Multi-strategy coordination: Ankle + hip + step.  
  - **Implementation**:  
    - Simulation: Test balance controllers.  
    - Physical deployment: Deploy on real robots.  
    - Evaluation: Test robustness to disturbances.  
- **Walkthrough**:  
  - Show disturbance rejection: push → detect → recover.  
  - Demonstrate multi-strategy coordination.  
  - Walk through implementation: sensors → controller → actuators.  
- **Challenge**:  
  - Students design robust balance controller:  
    1. Identify disturbance types (pushes, bumps, terrain).  
    2. Design multi-strategy recovery.  
    3. Plan implementation and testing.  
- **Takeaways**:  
  - Robust balance requires multiple recovery strategies.  
  - Real-time sensing and control enable fast recovery.  
  - Implementation requires careful sensor integration.  
- **Learn with AI**:  
  - `robust_balance_controller_designer`: RI component that helps students design robust balance controllers.

---

