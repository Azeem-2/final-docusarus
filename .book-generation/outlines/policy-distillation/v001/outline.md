# Chapter Outline – Policy Distillation (P4-C6)

---
chapter_id: P4-C6
title: Policy Distillation
version: v001
created: 2025-12-01
---

## 1. Introduction – Compressing Policies for Deployment

- Why policy distillation: Large policies are slow, need smaller models for real-time deployment.  
- Real-world motivation: Transfer knowledge from powerful teacher to efficient student.  
- Key concepts: Teacher-student framework, knowledge distillation, compression.

## 2. What Is Policy Distillation?

- Definition: Compressing large teacher policies into smaller student policies.  
- Teacher-student framework: Large teacher teaches small student.  
- Motivation: Deployment efficiency, transfer learning, privileged information handling.

## 3. Distillation Methods: Behavioral Cloning

- Student learns to mimic teacher actions.  
- Advantages: Simple, effective.  
- Limitations: May not capture teacher's reasoning.

## 4. Distillation Methods: Feature Matching

- Match intermediate representations between teacher and student.  
- Advantages: Preserves internal structure.  
- Implementation: More complex than behavioral cloning.

## 5. Distillation Methods: Logit Matching

- Match output distributions between teacher and student.  
- Advantages: Preserves decision-making behavior.  
- Applications: Works well for discrete and continuous actions.

## 6. Teacher-Student Framework

- Teacher: Large, powerful policy (may use privileged information).  
- Student: Small, fast policy (only real-world observations).  
- Distillation process: Transfer knowledge from teacher to student.

## 7. Privileged Information

- What is privileged information: Information available in simulation but not real world.  
- Teacher uses privileged info: True velocities, contact forces, object properties.  
- Student without privileged info: Only real-world observable sensors.

## 8. Progressive Distillation

- Iterative compression: Multiple distillation steps.  
- Advantages: Better compression, maintains performance.  
- Applications: Very large policy compression.

## 9. Practical Considerations

- Model size vs performance trade-offs.  
- Deployment: Real-time inference requirements.  
- Evaluation: Comparing teacher vs student performance.

## 10. Summary and Integration

- Key takeaways: Policy distillation enables efficient deployment.  
- Integration: Connects RL advanced (P4-C4) to practical deployment.  
- Bridge: Completes Part 4's policy learning theme.

---

