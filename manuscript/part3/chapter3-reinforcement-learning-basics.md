# Chapter: Reinforcement Learning Basics (P3-C3)

---
title: Reinforcement Learning Basics
slug: /P3-C3-reinforcement-learning-basics
sidebar_label: Reinforcement Learning Basics
sidebar_position: 3
---

## 1. Introduction – Learning from Experience

Robots often operate in environments that are too complex to program by hand. Instead of specifying every rule, we can let a robot **learn from trial and error**. This is the idea behind **reinforcement learning (RL)**: an agent interacts with an environment, receives feedback in the form of rewards, and gradually improves its behavior.

In this chapter, you will build an intuition for:

- The **building blocks** of an RL problem (state, action, reward, episode).  
- How rewards shape behavior—both in good and bad ways.  
- What it means to talk about **value** and **policy**, and why those ideas are central to RL.  
- How RL is used in robotics simulation, and why we must think carefully about exploration and safety.

The goal is not to cover every algorithm, but to give you a vocabulary and mental model that will make later chapters on advanced RL and sim-to-real easier to understand.

---

## 2. The RL Loop: Agent, Environment, and Reward

An RL setup usually includes:

- An **agent**: the learning system (our robot controller).  
- An **environment**: everything the agent interacts with (simulated world + robot dynamics).  
- A **state**: information the agent receives about the environment at each step.  
- An **action**: a choice the agent makes that affects what happens next.  
- A **reward**: a numeric signal that tells the agent how good the recent outcome was.  
- An **episode**: a sequence of states, actions, and rewards, from a starting point to some end condition.

The basic loop is:

1. The agent observes the current state.  
2. It chooses an action.  
3. The environment responds: it changes state, and the agent receives a reward.  
4. The agent updates its internal knowledge or policy based on that feedback.  
5. Repeat.

Over many episodes, the agent aims to choose actions that lead to **higher total reward**, often called return. In robotics, this might mean a robot arm that learns to reach a target efficiently, or a mobile base that learns to navigate to a goal without collisions.

---

## 3. Designing Rewards for Robotics Tasks

Reward design is one of the most important—and delicate—parts of RL. A reward is a **signal**, not a full specification of what you want. If you choose rewards poorly, the agent may find loopholes that technically maximize reward while behaving in ways you did not intend.

For example:

- If you reward a robot only for moving forward quickly, it may ignore safety and crash into obstacles.  
- If you reward it only at the goal, it might take a long time to learn which sequences of actions are good.

Common reward ingredients in robotics tasks include:

- Progress toward a goal (e.g., reduction in distance to target).  
- Penalties for collisions or unsafe motions.  
- Small penalties for actions or energy use to encourage efficiency.  
- Bonuses for successfully completing the task.

You can think of reward design as shaping the **landscape** the agent is climbing over. Smooth, informative rewards help learning proceed steadily; sparse or misleading rewards make learning slow or push the policy toward unwanted behavior.

---

## 4. Value and Policy – Two Ways of Thinking About Behavior

Two ideas show up again and again in RL:

- A **value function** tells you how good it is, on average, to be in a particular state (or to take a particular state–action pair), assuming you behave in a certain way afterward.  
- A **policy** tells you how the agent chooses actions given the state—essentially, its "strategy."

You can think of value as a **score** for states or state–action pairs, and policy as a **rulebook** that maps observations to actions.

Many RL algorithms use these ideas in different combinations:

- Some learn value estimates and then choose actions that look good according to those values.  
- Others directly adjust a parameterized policy to make better actions more likely over time.  
- Actor–critic methods do both, using a value estimate (critic) to help train a policy (actor).

For this chapter, it is enough to understand that:

- Value captures expectations about future rewards.  
- Policy captures the behavior that generates those rewards.

---

## 5. Simple Examples: Learning Better Actions Over Time

Imagine a tiny environment where a robot must move along a 3-state line to reach a goal at one end. Initially, it chooses actions at random. Sometimes it reaches the goal quickly; other times it wanders or steps away.

As the agent interacts:

- It observes which sequences of actions tend to lead to higher total reward.  
- It updates either a value estimate (e.g., how good it is to be in each state) or a policy (which actions to favor).  
- Over time, it becomes more likely to choose actions that move it steadily toward the goal.

Even without equations, you can see the pattern:

- **Exploration**: the agent must try different actions to discover what works.  
- **Exploitation**: once it has a sense of what works, it should use that knowledge more often.  

In more complex robotics tasks, the state might be high-dimensional (joint angles, velocities, sensor readings) and the actions might be continuous (torques, velocities). But the same basic idea applies: learn from feedback which actions lead to good long-term outcomes.

---

## 6. RL in Robotics Simulation

Reinforcement learning for robotics is usually done in **simulation** first, for several reasons:

- Learning often requires many episodes, which can be time-consuming and wear out hardware if done on real robots.  
- Exploration can be risky; random or poorly tuned policies might drive robots into unsafe states.  
- Simulation allows faster-than-real-time experiments and easier resets.

Common RL tasks in robotics simulation include:

- **Balancing**: keeping an inverted pendulum or robot link upright.  
- **Locomotion**: learning to walk, trot, or roll across uneven terrain.  
- **Manipulation**: reaching and grasping objects.  
- **Navigation**: moving a mobile robot to a goal while avoiding obstacles.

The environment modeling choices from P3-C2 directly affect these tasks. Friction, contact behavior, and scene layout all influence what the agent experiences and what it can learn.

---

## 7. Exploration and Safety Considerations

Exploration is necessary for learning, but in robotics we must treat it carefully:

- In simulation, we can allow more aggressive exploration, but we still want to avoid unstable simulations or unrealistic behaviors.  
- In the real world, random or poorly constrained actions can damage hardware or create unsafe situations.

Simple exploration strategies include:

- Starting with more randomness in action selection early in training, then reducing it as the policy improves.  
- Encouraging the agent to occasionally try less common actions to discover better paths.  

From a safety perspective:

- Policies trained in simulation should **not** be deployed blindly to physical robots.  
- Before transferring, you should check that the environment and reward design reflect important real-world constraints, and you should add safeguards (like action limits and safety monitors) on the physical system.

Later chapters on advanced RL and sim-to-real will delve deeper into techniques for safe exploration and robust transfer. For now, the key message is that exploration is powerful but must be handled with respect for hardware and human safety.

---

## 8. Summary and Bridge to Advanced RL

In this chapter you:

- Learned the building blocks of reinforcement learning: agent, environment, state, action, reward, and episode.  
- Saw how reward design strongly influences learned behavior, especially in robotics tasks.  
- Built an intuition for value functions and policies as two complementary ways to reason about behavior.  
- Explored how RL fits into robotics simulation and why exploration and safety must be considered together.  

These ideas form the conceptual foundation for more advanced RL techniques in later parts of the book, where you will see specific algorithms and case studies. With this base, you can better understand how policies are trained in simulation and why environment modeling and reward design are so critical for successful real-world deployment.

---

