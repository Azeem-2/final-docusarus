# Chapter Outline – Balance & Stability (P5-C3)

---
chapter_id: P5-C3
title: Balance & Stability
version: v001
created: 2025-12-01
---

## 1. Introduction – Maintaining Upright Posture

- Why balance matters: Humanoid robots must maintain stable upright posture.  
- Real-world motivation: Balance enables walking, manipulation, interaction.  
- Key concepts: ZMP, CoP, capture point, stability margins.

## 2. Balance Metrics: ZMP and CoP

- Zero Moment Point (ZMP): Point where net moment is zero.  
- Center of Pressure (CoP): Point where ground reaction force acts.  
- Relationship: ZMP and CoP for balance assessment.

## 3. Capture Point

- Capture point definition: Point where robot can come to rest.  
- Capture point calculation: Based on CoM position and velocity.  
- Balance recovery: Using capture point for recovery.

## 4. Stability Margins

- ZMP margin: Distance from ZMP to support polygon edge.  
- CoM margin: Distance from CoM to support polygon edge.  
- Safety margins: Additional margins for robustness.

## 5. Balance Control Strategies: Ankle Strategy

- Ankle strategy: Adjust ankle torque to maintain balance.  
- Small disturbances: Fast response, limited range.  
- Implementation: Ankle torque control.

## 6. Balance Control Strategies: Hip Strategy

- Hip strategy: Adjust hip motion to shift CoM.  
- Larger disturbances: Slower response, more range.  
- Implementation: Hip motion control.

## 7. Step Recovery

- Step recovery: Take step to recover balance.  
- Large disturbances: Requires planning and execution.  
- Step placement: Optimal step placement for recovery.

## 8. Disturbance Rejection

- Handling pushes: Recovering from external forces.  
- Robustness: Maintaining balance under disturbances.  
- Recovery strategies: Combining ankle, hip, and step recovery.

## 9. Implementation: Balance Controllers

- Balance controller design.  
- Sensor integration: Using IMU, force sensors.  
- Real-time balance control.

## 10. Summary and Integration

- Key takeaways: Balance requires multiple strategies and metrics.  
- Integration: Connects locomotion (P5-C2) to manipulation (P5-C4).  
- Bridge: Balance enables all humanoid capabilities.

---

