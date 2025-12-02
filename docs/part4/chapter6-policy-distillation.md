---
title: Policy Distillation
slug: /part4/chapter6-policy-distillation
sidebar_label: Policy Distillation
sidebar_position: 6
---

# Chapter: Policy Distillation (P4-C6)
## 1. Introduction – Compressing Policies for Deployment

Advanced RL algorithms (P4-C4) can train powerful policies, but these policies are often too large and slow for real-time robot deployment. **Policy distillation** compresses large teacher policies into smaller, faster student policies while maintaining performance.

In this chapter, you will learn:

- **What policy distillation is**: Compressing large teacher policies into smaller student policies.  
- **Teacher-student framework**: How large teachers teach small students.  
- **Distillation methods**: Behavioral cloning, feature matching, logit matching.  
- **Privileged information**: Handling information available to teacher but not student.  
- **Deployment**: Practical considerations for deploying distilled policies.

The goal is to understand how to compress and deploy learned policies efficiently, bridging advanced RL training to practical robot deployment.

---

## 2. What Is Policy Distillation?

**Policy distillation** is the process of compressing a large, powerful teacher policy into a smaller, faster student policy.

### Teacher-Student Framework

- **Teacher**: Large, powerful policy (may use privileged information).  
- **Student**: Small, fast policy (only real-world observations).  
- **Distillation**: Transfer knowledge from teacher to student.

### Motivation

**Why distill?**:
- **Deployment efficiency**: Smaller models run faster, use less memory.  
- **Transfer learning**: Transfer knowledge from simulation to real robot.  
- **Privileged information**: Teacher can use privileged info, student cannot.  
- **Model compression**: Reduce model size for edge deployment.

---

## 3. Distillation Methods: Behavioral Cloning

**Behavioral cloning** for distillation: student learns to predict teacher actions.

### Process

1. **Collect teacher actions**: Run teacher policy, collect state-action pairs.  
2. **Train student**: Learn to predict teacher actions given states.  
3. **Deploy student**: Use smaller, faster student policy.

### Advantages

- **Simple**: Standard supervised learning.  
- **Effective**: Works well for many tasks.  
- **Fast training**: Quick to train student.

### Limitations

- **May not capture reasoning**: Student mimics actions, not decision process.  
- **Distribution shift**: Student may fail on states teacher didn't visit.

---

## 4. Distillation Methods: Feature Matching

**Feature matching**: Match intermediate representations between teacher and student.

### Process

1. **Extract features**: Get intermediate representations from teacher and student.  
2. **Match features**: Minimize distance between teacher and student features.  
3. **Train student**: Learn to produce similar features to teacher.

### Advantages

- **Preserves structure**: Maintains internal knowledge representation.  
- **Better transfer**: More complete knowledge transfer.  
- **Interpretable**: Can inspect what student learned.

### Implementation

- Match features at multiple layers.  
- Use L2 loss or cosine similarity.  
- Balance feature matching with action prediction.

---

## 5. Distillation Methods: Logit Matching

**Logit matching**: Match output distributions between teacher and student.

### Process

1. **Get teacher outputs**: Collect teacher's action distributions.  
2. **Match distributions**: Student learns to produce similar distributions.  
3. **Train student**: Minimize KL divergence between distributions.

### Advantages

- **Preserves behavior**: Maintains decision-making characteristics.  
- **Works for continuous actions**: Can match continuous distributions.  
- **Robust**: Less sensitive to exact action values.

### Applications

- Discrete action spaces: Match probability distributions.  
- Continuous actions: Match Gaussian or other distributions.  
- Multi-modal policies: Preserve multiple action modes.

---

## 6. Teacher-Student Framework

### Teacher Policy

- **Large architecture**: Many parameters, powerful representation.  
- **Privileged information**: May use information not available to student.  
- **Training**: Trained with RL (PPO, SAC, TD3) or imitation learning.

### Student Policy

- **Small architecture**: Fewer parameters, efficient inference.  
- **Real-world observations**: Only uses sensors available on robot.  
- **Training**: Trained via distillation from teacher.

### Distillation Process

1. **Train teacher**: Use advanced RL to train powerful teacher.  
2. **Collect teacher data**: Run teacher, collect state-action pairs or features.  
3. **Train student**: Distill knowledge to smaller student.  
4. **Deploy student**: Use student for real-time robot control.

---

## 7. Privileged Information

**Privileged information** is information available in simulation but not in the real world.

### Examples

- **True velocities**: Exact joint velocities (may not be directly measurable).  
- **Contact forces**: Ground truth contact forces.  
- **Object properties**: Mass, friction, material properties.  
- **Future information**: Knowledge of future states.

### Teacher Uses Privileged Information

- Teacher policy can use privileged information for better decisions.  
- Enables more powerful teacher policies.  
- Common in sim-to-real transfer (P3-C7).

### Student Without Privileged Information

- Student only has real-world observable sensors.  
- Must learn to make decisions without privileged info.  
- Distillation transfers teacher's knowledge despite information gap.

---

## 8. Progressive Distillation

**Progressive distillation** compresses policies through multiple iterative steps.

### Process

1. **Step 1**: Distill large teacher → medium student.  
2. **Step 2**: Distill medium student → small student.  
3. **Repeat**: Continue until desired size.

### Advantages

- **Better compression**: Can compress very large policies.  
- **Maintains performance**: Gradual compression preserves knowledge.  
- **Flexible**: Can stop at any compression level.

### Applications

- Very large policies (millions of parameters).  
- Extreme compression (10x or more).  
- When single-step distillation fails.

---

## 9. Practical Considerations

### Model Size vs Performance

- **Trade-off**: Smaller models are faster but may have lower performance.  
- **Target size**: Choose based on deployment constraints.  
- **Evaluation**: Compare teacher vs student performance.

### Deployment

- **Real-time inference**: Student must run fast enough for control.  
- **Memory**: Smaller models use less memory.  
- **Hardware**: Can deploy on edge devices.

### Evaluation

- **Performance comparison**: Teacher vs student on same tasks.  
- **Efficiency metrics**: Inference time, memory usage.  
- **Robustness**: Test student on diverse scenarios.

---

## 10. Summary and Integration

In this chapter you:

- Learned that policy distillation compresses large teacher policies into smaller student policies.  
- Explored distillation methods: behavioral cloning, feature matching, logit matching.  
- Understood teacher-student framework and privileged information.  
- Recognized progressive distillation for very large policies.  
- Saw practical deployment considerations.

**Integration with Part 4**:
- **Control policies (P4-C3)**: Policies that can be distilled.  
- **RL advanced (P4-C4)**: Training powerful teacher policies.  
- **Policy distillation (P4-C6)**: Compressing policies for deployment.

Policy distillation bridges advanced RL training to practical robot deployment, enabling efficient use of learned policies in real-world applications.

---

