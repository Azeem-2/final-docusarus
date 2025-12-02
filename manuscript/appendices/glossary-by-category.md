# Glossary by Category

**Physical AI, Simulation AI & Humanoid Robotics Book**

**Total Terms**: 130
**Last Updated**: 2025-12-02

---

## Physical / Hardware

Terms related to physical robots, hardware components, sensors, actuators, and mechanical systems.

- **Actuator**: A mechanical or electromechanical device that converts energy into motion or force to move a robot joint, link, or mechanism.
- **Humanoid Robot**: A robot with a body plan inspired by the human form, typically including a torso, two arms, two legs, and a head, designed to operate in environments and use tools built for people.
- **Robot**: A physical system with a body, sensors, actuators, and a controller that can act autonomously or semi-autonomously in the physical world.
- **Sensor**: A device that measures some aspect of the robot or its environment and converts it into a signal that a controller can use.
- **Proprioceptive Sensor**: A sensor that measures internal quantities of the robot itself, such as joint angle, wheel rotation, or body acceleration.
- **Exteroceptive Sensor**: A sensor that measures properties of the environment outside the robot, such as distance to obstacles, images of the scene, or contact with surfaces.

*See also: Hardware, Mechanical Systems, Sensors & Actuators chapters*

---

## Simulation

Terms related to virtual environments, physics engines, digital twins, and simulation-based training.

- **Digital Twin**: A living digital representation of a specific physical asset, system, or process that stays in sync with the real world through continuous data exchange and is used to understand, predict, and optimize behavior.
- **Reality Gap**: The discrepancy between how a robot or policy behaves in simulation and how it behaves in the real world under nominally equivalent conditions, caused by modeling errors, unmodeled dynamics, and sensor differences.
- **Sim-to-Real Transfer**: The process of training or designing behaviors in simulation and then deploying them successfully to physical robots while managing the effects of the reality gap.
- **Domain Randomization**: A technique for improving sim-to-real transfer by training policies on many randomized variations of the simulation environment, making them more robust to reality gap effects.
- **Physics Engine**: Software that simulates physical laws (forces, collisions, friction, gravity) to create realistic virtual environments for robot training and testing.

*See also: Simulation, Physics Engines, Digital Twins chapters*

---

## AI / Machine Learning

Terms related to artificial intelligence, machine learning, reinforcement learning, neural networks, and AI policies.

- **Artificial Intelligence (AI)**: A field of study focused on algorithms and models that produce behaviors we consider intelligent, such as perception, reasoning, learning, and planning.
- **Physical AI**: AI systems that perceive, understand, and perform complex actions in the physical world using embodied robots or devices, combining sensing, control, learning, and interaction with real environments.
- **Foundation Model**: A large-scale AI model trained on diverse data that can be adapted to many downstream tasks, such as physical reasoning or robot control, often serving as a general-purpose building block.
- **Reinforcement Learning (RL)**: A machine learning approach where an agent learns to make decisions by interacting with an environment, receiving rewards for good actions, and gradually improving its behavior through trial and error.
- **Policy**: In reinforcement learning, a policy is a function or strategy that maps states to actions, determining what the agent should do in each situation.
- **Reward**: In reinforcement learning, a numeric signal that indicates how good or bad the outcome of an action was, used to guide the agent's learning.
- **Agent**: In reinforcement learning, the learning system (typically a robot controller) that makes decisions and learns from interactions with the environment.
- **Episode**: In reinforcement learning, a complete sequence of states, actions, and rewards from a starting point to some end condition (e.g., task completion or failure).
- **Value Function**: In reinforcement learning, a function that estimates the expected cumulative reward from a given state or state-action pair, used to guide policy learning.
- **Bias**: Systematic unfairness in systems. Bias can arise from training data, algorithms, or deployment contexts. Ethical development requires identifying and mitigating bias.

*See also: AI for Robotics, Reinforcement Learning, Machine Learning chapters*

---

## General

Terms related to general concepts, professional development, research, and foundational knowledge.

- **Embodied Intelligence**: Intelligence that arises from the tight coupling of a body, controller, and environment, where physical form and sensorimotor interaction shape what can be learned and how a system behaves.
- **Robotics**: The field concerned with designing, modeling, building, and controlling embodied systems that sense, decide, and act in the physical world.
- **PhD Program**: A doctoral degree requiring original research contributions, comprehensive qualifying exams, and a thesis defense. Typically takes 4-6 years and provides deep specialization in a research area.
- **Research Assistantship (RA)**: Funding through faculty research projects. You work on faculty research while completing your degree. This provides research experience and financial support.
- **Teaching Assistantship (TA)**: Funding through course support. You assist professors with teaching, grading, and student support. This develops teaching skills and provides income.
- **Fellowship**: Competitive funding that does not require teaching or research work. Fellowships provide financial support and recognition for academic excellence.
- **Postdoctoral Position**: A temporary research position after completing a PhD. Postdocs conduct advanced research, publish papers, and prepare for faculty or industry positions.
- **Ethics**: Moral principles governing behavior. In robotics, ethics guide how we design, develop, and deploy systems. Ethical principles ensure systems benefit humanity and respect human rights.
- **Human Agency**: Human capacity to act independently and make choices. Ethical robotics supports human agency rather than replacing it. Humans should retain control over robotic systems.
- **Transparency**: Openness about how systems work. Transparency includes explainability (understanding decisions), disclosure (revealing capabilities and limitations), and accountability (identifying responsible parties).
- **Accountability**: Responsibility for actions and decisions. In robotics, accountability means clear chains of responsibility—knowing who is responsible when things go wrong.

*See also: Foundations, Professional Path, Research Pathways chapters*

---

## Safety

Terms related to safety standards, risk assessment, and safety practices in robotics.

- **Safety**: Protection from harm. In robotics, safety includes physical safety (preventing injuries), cybersecurity (preventing malicious attacks), and operational safety (ensuring reliable operation).
- **Risk Assessment**: Systematic evaluation of potential hazards and their likelihood and severity. Risk assessment identifies safety concerns before deployment.
- **Safety Standards**: Industry standards (e.g., ISO 10218, ISO 13482) that define safety requirements for robots. Compliance ensures safe operation.
- **Emergency Stop**: A safety mechanism that immediately stops robot motion when activated. Emergency stops are required for all physical robots.
- **Fail-Safe Design**: Design approach where system failures default to safe states. Fail-safe systems prevent harm even when components fail.
- **Human-in-the-Loop**: System design where humans monitor and can intervene in robot operations. Human-in-the-loop systems provide safety oversight.
- **Safety Interlock**: A safety mechanism that prevents robot operation when safety conditions are not met (e.g., door open, guard removed).

*See also: Safety Guidelines, Physical Labs, Ethical & Safety Guidelines chapters*

---

## Control Systems

Terms related to robot control, feedback control, and control algorithms.

- **Feedback Control**: A control strategy where the robot continuously measures what is happening, compares it to what is desired, and adjusts its commands based on the difference.
- **PID Controller**: A widely used feedback controller that combines proportional, integral, and derivative actions on the error signal to shape how a system responds.
- **Control Loop**: The continuous cycle of sensing, comparing, computing, and acting that enables a robot to achieve desired behaviors.
- **Error Signal**: The difference between what is desired and what is measured, used by feedback controllers to compute corrections.

*See also: Control Systems, Feedback Control chapters*

---

## Kinematics & Dynamics

Terms related to robot motion, kinematics, and dynamics.

- **Kinematics**: The study of how a robot's joints and links move relative to each other and to the environment, without considering the forces that cause the motion.
- **Dynamics**: The study of how forces and torques acting on a robot produce motion, taking into account mass, inertia, gravity, friction, and other physical effects.
- **Forward Kinematics**: Computing the position and orientation of a robot's end-effector from its joint angles.
- **Inverse Kinematics**: Computing the joint angles needed to achieve a desired end-effector position and orientation.
- **Workspace**: The set of positions and orientations that a robot's end-effector can reach.
- **Joint Space**: The space of all possible joint configurations for a robot.
- **Task Space**: The space of all possible positions and orientations for a robot's end-effector.

*See also: Kinematics, Dynamics chapters*

---

## Perception

Terms related to robot perception, sensing, and understanding the environment.

- **Perception**: The process of interpreting sensor data to understand the robot's state and environment.
- **Computer Vision**: The field of AI focused on extracting meaningful information from images and video.
- **SLAM (Simultaneous Localization and Mapping)**: The process of building a map of an unknown environment while simultaneously tracking the robot's location within that map.
- **Object Recognition**: Identifying and classifying objects in the environment from sensor data.
- **Depth Estimation**: Determining the distance to objects in the environment, typically using stereo vision, structured light, or time-of-flight sensors.

*See also: Sensors & Perception, Computer Vision chapters*

---

**Note**: This glossary is organized by category for easy reference. For alphabetical listing, see the master glossary in `.book-generation/glossary/terms.yaml`.

