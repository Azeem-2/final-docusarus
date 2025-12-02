# Research: Policy Distillation (P4-C6)

**Chapter**: P4-C6  
**Topic**: Policy Distillation  
**Version**: v001  
**Created**: 2025-12-01  
**Agent**: research-agent  
**Skill**: research-methodology

---

## Research Questions

1. What is policy distillation and why is it useful for robotics?
2. How do we compress large teacher policies into smaller student policies?
3. What are the key distillation techniques (behavioral cloning, feature matching, logit matching)?
4. How do we handle privileged information in teacher-student distillation?
5. What are the practical considerations for deploying distilled policies?

---

## Key Concepts Identified

### Core Concepts
- **Policy distillation**: Compressing large teacher policies into smaller student policies
- **Teacher-student framework**: Large teacher policy teaches small student policy
- **Knowledge distillation**: Transferring knowledge from teacher to student
- **Privileged information**: Information available to teacher but not student
- **Compression**: Reducing model size while maintaining performance

### Techniques
- **Behavioral cloning**: Student learns to mimic teacher actions
- **Feature matching**: Match intermediate representations
- **Logit matching**: Match output distributions
- **Progressive distillation**: Iterative compression
- **Multi-teacher distillation**: Learn from multiple teachers

### Workflows
- Train teacher policy (large, powerful)
- Distill to student policy (small, fast)
- Deploy student policy on robot

---

## Source Material (MCP Context7)

### RSL-RL (/leggedrobotics/rsl_rl)
- Fast and simple RL library for robotics
- Supports Student-Teacher Distillation
- Used for legged robot control

### Key Features
- **Student-Teacher Distillation**: Transfer knowledge from teacher to student
- **Policy compression**: Reduce model size for deployment
- **Robotics applications**: Legged locomotion, manipulation

---

## Research Notes

### Policy Distillation Motivation

**Why distill?**:
- **Deployment**: Large policies are slow, need smaller models for real-time
- **Efficiency**: Smaller models use less memory and compute
- **Transfer**: Transfer knowledge from simulation to real robot
- **Privileged information**: Teacher can use privileged info, student cannot

### Distillation Methods

**Behavioral Cloning**:
- Student learns to predict teacher actions
- Simple, effective
- May not capture teacher's reasoning

**Feature Matching**:
- Match intermediate representations
- Preserves internal structure
- More complex to implement

**Logit Matching**:
- Match output distributions
- Preserves decision-making behavior
- Works well for discrete actions

### Teacher-Student Framework

- **Teacher**: Large, powerful policy (may use privileged information)
- **Student**: Small, fast policy (only real-world observations)
- **Distillation**: Transfer knowledge from teacher to student

---

## Prerequisites from Previous Chapters

- **P4-C3**: Control policies (neural network policies)
- **P4-C4**: RL advanced (training policies, PPO, SAC, TD3)
- **P3-C7**: Sim-to-real transfer (teacher-student distillation mentioned)

---

## Research Status

**Status**: Scaffold complete  
**Next Step**: Proceed to outliner-agent for chapter outline creation

**Note**: This research scaffold provides conceptual foundation. Full research with Tier 1/2 citations will be completed during the writing phase when specific claims need verification.

---

