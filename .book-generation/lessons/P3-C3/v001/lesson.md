# Lessons Blueprint: P3-C3 Reinforcement Learning Basics

**Chapter ID**: P3-C3  
**Version**: v001  
**Created**: 2025-12-01  

---

## Lesson 1: RL Building Blocks and Rewards

- **Pedagogical Layer**: Layer 1–2 – Intuition & Concepts  
- **Estimated Time**: 75–90 minutes  
- **Learning Outcomes**:  
  1. Define, in simple terms, agent, environment, state, action, and reward.  
  2. Explain what it means to learn from trial and error in an RL setting.  
  3. Recognize good vs problematic reward designs for simple robotics tasks.  

### Parts 1–6

- **Hook**: A simulated robot learns to move quickly toward a goal but discovers a loophole in the reward function and gets stuck spinning in place.  
- **Theory**:  
  - Informal definition of an RL loop: observe state, choose action, get reward, update behavior.  
  - Agent vs environment; state vs observation; episodes and returns (qualitative).  
  - Examples of reward design for reaching, navigation, and balancing tasks.  
- **Walkthrough**:  
  - Step through a toy gridworld or 1D line task, writing down states, actions, and rewards.  
  - Show how slightly different rewards (e.g., penalty for steps, bonus at goal) change behavior.  
- **Challenge**:  
  - Students are given 2–3 short robotics task descriptions and must sketch sensible reward definitions, calling out potential pitfalls (e.g., rewarding speed without safety).  
- **Takeaways**:  
  - RL is about learning behavior from **feedback signals**, not explicit instructions.  
  - Reward design strongly shapes what policies will do.  
- **Learn with AI**:  
  - `reward_design_reviewer`: RI component that critiques student-written rewards for clarity and unintended side effects.  

---

## Lesson 2: Value and Policy Intuition with Simple Examples

- **Pedagogical Layer**: Layer 2–3 – Concepts & Application  
- **Estimated Time**: 75–90 minutes  
- **Learning Outcomes**:  
  1. Describe, in words, what a value function and a policy represent.  
  2. Explain the high-level idea behind Q-learning and policy gradients without equations.  
  3. Connect these ideas to how a robot learns to choose better actions over time.  

### Parts 1–6

- **Hook**: Comparing a “greedy” robot that always picks the best-looking move right now vs a robot that learns long-term consequences.  
- **Theory**:  
  - Value as “how good it is to be in a state” (or take a state–action pair).  
  - Policy as “how the agent chooses actions” given the state.  
  - High-level sketches of Q-learning and policy gradient approaches.  
- **Walkthrough**:  
  - Use a tiny example (e.g., 3-state chain) to illustrate value estimates improving over episodes.  
  - Show how a policy might start random and then become more focused on good actions.  
- **Challenge**:  
  - Students label a few states/actions as likely “high value” or “low value” given a task description, and propose how a policy should behave.  
- **Takeaways**:  
  - Value and policy are two complementary ways of thinking about behavior.  
  - Many practical RL algorithms in robotics are variations on these basic ideas.  
- **Learn with AI**:  
  - `rl_concept_checker`: RI component that quizzes students on short RL scenarios and gives feedback on their explanations of value and policy.  

---

## Lesson 3: RL for Robotics Simulation – Tasks, Exploration, and Safety Preview

- **Pedagogical Layer**: Layer 3–4 – Integration & Design Intuition  
- **Estimated Time**: 75–90 minutes  
- **Learning Outcomes**:  
  1. Identify which kinds of robotics tasks are good candidates for RL in simulation.  
  2. Describe simple exploration strategies and why they are needed.  
  3. Appreciate why extra care is needed before deploying RL policies to physical robots.  

### Parts 1–6

- **Hook**: A locomotion policy that looks great in simulation but causes unstable, unsafe behavior when run directly on a real robot.  
- **Theory**:  
  - Typical robotics RL tasks in simulation: balancing, locomotion, manipulation, navigation.  
  - Intuitive exploration ideas (e.g., try new actions sometimes to learn more).  
  - Preview of safety concerns and the “reality gap.”  
- **Walkthrough**:  
  - Outline one or two example training setups (e.g., cart-pole balancing, differential-drive navigation) at a conceptual level.  
  - Discuss simple exploration settings (like “more random early, more focused later”) and what could go wrong.  
- **Challenge**:  
  - Students pick a simple robotics task and outline: state, actions, rewards, and a basic plan for exploration, plus at least one safety precaution for eventual real-world tests.  
- **Takeaways**:  
  - RL in robotics almost always starts in **simulation** for safety and speed.  
  - Good task design, exploration, and safety thinking from the start make later sim-to-real transfer more plausible.  
- **Learn with AI**:  
  - `rl_task_planner`: RI component that reviews a student’s proposed RL robotics task spec and highlights missing pieces or safety concerns.  


