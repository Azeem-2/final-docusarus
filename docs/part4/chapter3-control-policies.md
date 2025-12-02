---
title: Control Policies
slug: /part4/chapter3-control-policies
sidebar_label: Control Policies
sidebar_position: 3
---

# Chapter: Control Policies (P4-C3)
## 1. Introduction – Learned Control for Robotics

Traditional control requires hand-tuning parameters for each task. **Learned control policies** adapt from data, enabling robots to perform complex behaviors that are difficult to program by hand.

In this chapter, you will learn:

- **What control policies are**: Functions that map observations to actions.  
- **Policy architectures**: MLP, CNN, Transformer, Diffusion models.  
- **Training methods**: Imitation learning, reinforcement learning, offline RL.  
- **Vision-based control**: Policies that use image observations.  
- **Multi-modal policies**: Combining vision, proprioception, and language.  
- **Deployment**: Real-time inference, safety, integration.

The goal is to understand how learned control policies enable robots to perform complex manipulation and navigation tasks through data-driven learning rather than hand-programming.

---

## 2. What Are Control Policies?

A **control policy** is a function that maps observations (what the robot senses) to actions (what the robot should do).

### Traditional vs Learned Control

**Traditional control** (e.g., PID controllers):
- Hand-designed: Engineers write control laws based on physics.  
- Fixed behavior: Same inputs always produce same outputs.  
- Requires tuning: Parameters must be adjusted for each task.

**Learned control policies**:
- Data-driven: Learn from demonstrations or experience.  
- Adaptive: Can handle variations and new situations.  
- Complex behaviors: Can learn behaviors hard to program manually.

### Policy Representation

Learned policies are typically implemented as **neural networks**:
- Universal function approximators: Can represent complex mappings.  
- Trainable: Parameters learned from data.  
- Flexible: Can handle high-dimensional observations and actions.

The policy takes observations (images, joint angles, sensor data) and outputs actions (joint torques, end-effector poses, velocities).

---

## 3. Policy Architectures: MLP and CNN

Different policy architectures suit different observation types and task complexities.

### MLP (Multi-Layer Perceptron)

**MLP** is a simple feedforward neural network:
- **Structure**: Multiple layers of fully connected neurons.  
- **Input**: Low-dimensional observations (joint angles, velocities, target positions).  
- **Output**: Actions (joint torques, end-effector poses).

**When to use**:
- Low-dimensional observations (proprioception only).  
- Fast inference required.  
- Simple tasks that don't require spatial reasoning.

**Example**: A robot arm policy that takes joint angles and target position, outputs joint torques.

### CNN (Convolutional Neural Network)

**CNN** processes image observations:
- **Structure**: Convolutional layers extract spatial features, followed by fully connected layers.  
- **Input**: Images (camera frames).  
- **Output**: Actions.

**When to use**:
- Image-based observations.  
- Tasks requiring spatial understanding.  
- Vision-based manipulation or navigation.

**Example**: A manipulation policy that takes camera images and outputs end-effector poses for grasping.

---

## 4. Policy Architectures: Transformer and Diffusion

For more complex tasks, advanced architectures provide additional capabilities.

### Transformer

**Transformer** policies use attention mechanisms:
- **Structure**: Self-attention and cross-attention layers.  
- **Input**: Sequences (multi-modal inputs, temporal sequences).  
- **Output**: Actions or action sequences.

**When to use**:
- Multi-modal inputs (vision + language + proprioception).  
- Sequential reasoning required.  
- Tasks requiring attention to different parts of the input.

**Example**: A policy that processes images, language commands, and joint states, using attention to focus on relevant information.

### Diffusion Policy

**Diffusion Policy** uses diffusion models for action generation:
- **Structure**: Denoising diffusion process generates actions.  
- **Input**: Observations (images, proprioception).  
- **Output**: Action sequences (smooth trajectories).

**Key advantages**:
- **Multimodal distributions**: Handles multiple valid action solutions.  
- **Smooth trajectories**: Naturally generates smooth action sequences.  
- **Action chunking**: Predicts sequences of actions for smooth motion.

**When to use**:
- Tasks with multiple valid solutions (symmetries, redundancies).  
- Manipulation tasks requiring smooth trajectories.  
- When action chunking improves performance.

**Example**: A manipulation policy that generates smooth end-effector trajectories for picking and placing objects.

---

## 5. Training Control Policies: Imitation Learning

**Imitation learning** trains policies by learning from expert demonstrations.

### Behavioral Cloning

The simplest approach: treat demonstrations as supervised learning:
1. **Collect demonstrations**: Expert performs task, recording state-action pairs.  
2. **Train policy**: Learn to predict expert actions given states.  
3. **Deploy**: Use learned policy to perform task.

**Advantages**:
- Data-efficient: May need only tens or hundreds of demonstrations.  
- Simple: Standard supervised learning.

**Challenges**:
- Distribution shift: Policy may encounter states not in training data.  
- Error compounding: Small errors can lead to failure.

### Dataset Aggregation (DAgger)

Iterative improvement with expert feedback:
1. Train policy on initial demonstrations.  
2. Deploy policy, collect failure cases.  
3. Expert provides corrections for failures.  
4. Retrain with augmented dataset.  
5. Repeat.

This reduces distribution shift by collecting data from states the policy actually visits.

---

## 6. Training Control Policies: Reinforcement Learning

**Reinforcement learning** trains policies through trial and error.

### RL for Control

The agent interacts with the environment:
1. Observe state.  
2. Take action.  
3. Receive reward.  
4. Update policy to maximize future rewards.

**Advantages**:
- Can discover novel behaviors.  
- Handles exploration of state space.

**Challenges**:
- Sample inefficient: May need millions of episodes.  
- Exploration: Must balance exploration and exploitation.  
- Reward design: Critical for success.

### Policy Gradients

Update policy parameters to increase expected reward:
- **Gradient ascent**: Adjust parameters in direction that increases reward.  
- **Exploration**: Policy must explore to find good actions.  
- **Stability**: Training can be unstable, requires careful tuning.

---

## 7. Training Control Policies: Offline RL

**Offline RL** learns from fixed datasets without environment interaction.

### When to Use Offline RL

- **Safety-critical tasks**: Cannot explore freely in real environment.  
- **Limited data collection**: Expensive or time-consuming to collect new data.  
- **Historical data**: Have existing datasets from previous experiments.

### Challenges

- **Distribution shift**: Dataset may not cover all states policy will encounter.  
- **Conservative learning**: Must avoid overestimating value of out-of-distribution actions.  
- **Data quality**: Performance depends on dataset quality and coverage.

### Methods

- **Conservative Q-learning**: Penalize out-of-distribution actions.  
- **Behavior cloning with RL fine-tuning**: Start with imitation, improve with RL.  
- **Dataset augmentation**: Use simulation to augment real data.

---

## 8. Vision-Based Control Policies

**Vision-based control** policies map images directly to actions, without explicit object detection or scene understanding.

### Architecture

- **Vision encoder**: CNN or Vision Transformer processes images.  
- **Policy head**: Maps visual features to actions.  
- **End-to-end**: Learns visual features relevant for control.

### Advantages

- **No explicit perception**: Doesn't require separate object detection.  
- **Robust features**: Learns features robust to variations.  
- **End-to-end learning**: Optimizes entire pipeline for task.

### Example

A manipulation policy:
- **Input**: Camera image of scene with objects.  
- **Output**: End-effector pose for grasping.  
- **Training**: Learn from demonstrations or RL.

The policy learns to extract relevant visual information (object locations, orientations) implicitly through training.

---

## 9. Multi-modal Control Policies

**Multi-modal policies** combine multiple input modalities for richer understanding.

### Input Modalities

- **Vision**: Camera images (RGB, depth).  
- **Proprioception**: Joint angles, velocities, forces.  
- **Language**: Natural language commands or descriptions.

### Fusion Strategies

**Early fusion**: Concatenate all inputs before network.
- Simple, but may not learn optimal interactions.

**Late fusion**: Process each modality separately, combine at end.
- Preserves modality-specific features, but may miss interactions.

**Attention-based fusion**: Learn which modalities to attend to.
- Most flexible, can adapt to task requirements.

### Example

A manipulation policy with:
- **Vision**: Camera image of scene.  
- **Proprioception**: Current joint angles.  
- **Language**: "Pick up the red cup".

The policy uses attention to focus on visual features relevant to "red cup" while considering current robot state.

---

## 10. Deployment and Practical Considerations

### Real-Time Inference

Policies must run fast enough for real-time control:
- **Latency requirements**: 10-100ms typical for manipulation.  
- **Optimization**: Model quantization, pruning, efficient architectures.  
- **Hardware**: GPU acceleration, edge deployment options.

### Safety Mechanisms

Critical for physical deployment:
- **Torque limits**: Prevent excessive forces.  
- **Collision avoidance**: Monitor and prevent collisions.  
- **Fail-safes**: Emergency stops, recovery behaviors.  
- **Action smoothing**: Prevent jerky motions.

### Robustness

Handle real-world variations:
- **Distribution shift**: Policy encounters situations not in training.  
- **Sensor noise**: Real sensors have noise not in simulation.  
- **Unexpected situations**: Objects, obstacles, disturbances.

### Integration

Connect policies to robot systems:
- **Perception**: Sensors → policy inputs.  
- **Planning**: Policy outputs → motion planning (if needed).  
- **Control**: Policy or planning → actuators.

---

## 11. Summary and Integration

In this chapter you:

- Learned that learned control policies map observations to actions using neural networks.  
- Explored policy architectures: MLP (simple), CNN (vision), Transformer (multi-modal), Diffusion (smooth trajectories).  
- Understood training methods: imitation learning (demonstrations), RL (trial and error), offline RL (fixed datasets).  
- Recognized vision-based and multi-modal policies for complex tasks.  
- Understood deployment considerations: real-time, safety, robustness, integration.

**Integration with Part 4**:
- **Vision models (P4-C1)**: Provide visual features for vision-based policies.  
- **Multi-modal models (P4-C2)**: Enable language understanding for multi-modal policies.  
- **Control policies (P4-C3)**: Generate actions from multi-modal understanding.

In the next chapter (P4-C4: Reinforcement Learning Advanced), you'll explore advanced RL techniques for training more sophisticated policies.

---

