# Chapter: Kinematics (P2-C5)

---
title: Kinematics
slug: /P2-C5-kinematics
sidebar_label: Kinematics
sidebar_position: 5
---

## 1. Introduction – From Joints to Motion

When you watch a robot arm move, you usually care about what the **end-effector**—the hand or tool—is doing in space: where it is, which way it points, and how it moves along a path. Motors, however, do not think in those terms. They receive commands in **joint space**: individual angles or displacements at each joint.

The branch of robotics that relates **joint space** to **task space** is called **kinematics**. It is about **geometry and motion**, not forces. In this chapter you will build an intuition for:

- How joint angles map to positions and orientations of the end-effector.  
- How to reason about what parts of space a robot can reach.  
- Why some configurations are “comfortable” and others are problematic or ambiguous.  

You will not derive full matrices or implement solvers here. Instead, you will use pictures, simple examples, and light notation to develop a mental model that will make later, more formal kinematics chapters feel much less intimidating.

---

## 2. Frames, Joints, and Workspace

Every robot is made of **links** (rigid bodies) connected by **joints** (revolute, prismatic, etc.). To describe motion precisely, we attach **coordinate frames** to key parts of the robot:

- A **base frame** fixed to the robot’s base.  
- One frame per link or important point (e.g., the gripper).  

Positions, orientations, and motions are always expressed **relative to some frame**. Changing the frame changes the numbers, but not the physical situation.

Two spaces are especially important:

- **Joint space**: the vector of all joint variables (angles or displacements). For a simple 2‑joint planar arm, this might be \((\theta_1, \theta_2)\).  
- **Task space**: the space of end-effector positions (and, in full 3D, orientations). For the same planar arm, this might be \((x, y)\) in the plane.  

The set of all task-space points that the end-effector can reach (subject to joint limits and mechanical constraints) is called the **workspace**. Visualizing the workspace for simple robots is an excellent way to build geometric intuition.

For a 2‑link planar arm with link lengths \(L_1\) and \(L_2\), the reachable points form a shape like a thick ring around the base: too close and the arm cannot fold inward far enough; too far and even fully stretched links cannot reach.

---

## 3. Forward Kinematics for a Simple Planar Arm

**Forward kinematics (FK)** answers the question:

> Given the joint configuration, where is the end-effector?

For a simple 2‑link planar arm, you can imagine the process in steps:

1. Start at the base frame origin.  
2. Rotate by \(\theta_1\); move out along the first link of length \(L_1\).  
3. From that point, rotate by \(\theta_2\) relative to the first link; move out along the second link of length \(L_2\).  

Geometrically, the end-effector position \((x, y)\) can be written as:

- \(x = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2)\)  
- \(y = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)\)  

You do not need to memorize the formulas; the important part is the **process**:

- Apply rotations and translations in sequence.  
- Keep track of how each joint angle affects downstream links.  

In 3D and for more complex robots, this composition is often handled with standard conventions (like Denavit–Hartenberg parameters) and matrix multiplication. In this chapter, we keep the math light and focus on understanding what is happening when “the FK code runs.”

---

## 4. Joint Space vs Task Space – Many-to-One

Forward kinematics defines a mapping:

\[
  f: \text{joint space} \rightarrow \text{task space}
\]

For many robots, this mapping is **many-to-one**:

- Different joint configurations can place the end-effector at the **same** position and orientation.  

In the 2‑link planar arm, for example, a point in front of the robot might be reachable with an “elbow‑up” configuration and an “elbow‑down” configuration. Both yield the same \((x, y)\), but the intermediate joint angles differ.

This has two important consequences:

1. When you plan a motion in task space (e.g., “move the hand along this line”), you must eventually choose **which joint-space path** to follow.  
2. Some points might **not** be reachable at all because of joint limits, mechanical stops, or collisions, even if the workspace shape suggests they might be.

Building a habit of thinking in both spaces—joint and task—is essential for understanding later chapters on motion planning, dynamics, and control.

---

## 5. Inverse Kinematics – The Harder Direction

If forward kinematics asks “Where is the hand given the joints?”, **inverse kinematics (IK)** asks:

> Given a desired end-effector pose, what joint configuration(s) achieve it?

Even in simple 2D cases, IK is often:

- **Ambiguous**: multiple joint configurations may reach the same task-space point.  
- **Constrained**: some desired poses may be out of reach or violate joint limits.  
- **Nonlinear**: small changes in the target can cause large changes in joint angles near certain configurations.  

Because of these properties, IK is typically solved with:

- Closed-form solutions for simple robots (where formulas can be derived).  
- Numerical methods for more complex systems (iterative algorithms that adjust joint angles to reduce error).  

In this introductory chapter, you do not need to implement solvers. The key idea is that **IK is a search problem in joint space** constrained by geometry, limits, and sometimes additional preferences (comfort, clearance, symmetry).

---

## 6. Redundancy and Singularities (Conceptual)

When a robot has **more joints than strictly necessary** to achieve a task-space goal, it is called **redundant**. Redundancy is powerful:

- The robot can choose among many joint-space solutions to avoid obstacles, respect joint limits, or maintain a comfortable posture.  

However, redundancy also introduces complexity:

- The IK problem has infinitely many solutions along certain directions.  
- Choosing between them requires additional criteria or optimization.

At the other extreme, some configurations are **singular**. Intuitively, a singularity is a posture where:

- Small joint motions fail to produce meaningful end-effector motion in some direction.  
- Or the robot becomes locally “stiff” in certain directions, making motion or control difficult.

For a planar arm, a classic example is when the arm is fully stretched out in a straight line: small changes in elbow angle barely move the hand in some directions, and the arm has lost some effective degrees of freedom at that point.

Understanding redundancy and singularities at an intuitive level helps you interpret solver behavior later: why solutions jump, why some poses feel “uncomfortable,” and why planners avoid certain configurations.

---

## 7. How Kinematics Feeds Planning and Control

Kinematics rarely lives alone. It is a building block used by:

- **Motion planning**: algorithms that search for paths in joint space or task space while respecting constraints.  
- **Control**: joint controllers need target joint positions and velocities; task-space controllers need consistent conversions between spaces.  
- **Simulation and visualization**: physics engines and visualization tools use kinematics to update link poses as joints move.  

When you ask a robot to “move the gripper here,” a typical pipeline might:

1. Use IK to find an appropriate joint configuration (or a trajectory of configurations).  
2. Use dynamics and control models to compute the torques or motor commands needed.  
3. Use kinematics repeatedly to keep track of where the robot is as it moves.

This chapter lays the **geometric** foundation for that pipeline. Later chapters add forces, dynamics, and feedback control on top.

---

## 8. Summary and Bridge to Dynamics

In this chapter you:

- Learned the basic language of frames, links, joints, joint space, and task space.  
- Saw how forward kinematics maps joint configurations to end-effector poses in simple planar arms.  
- Developed an intuition for many-to-one mappings, redundancy, and singularities.  
- Understood, conceptually, why inverse kinematics is harder and often ambiguous.  
- Connected kinematics to later topics in planning, dynamics, and control.

In the next chapter, you will move from **geometry** to **forces and motion**—from kinematics to **dynamics**. You will see how torques, inertia, and gravity interact with the kinematic structures introduced here, and why a solid grasp of kinematics makes dynamic reasoning much easier.

---

## Draft Metadata

- Status: Initial writer-agent draft for P2-C5.  
- Word Count: ~1,700 (to be refined with examples and figures).  
- Voice: “we” / balanced, aligned with Part 2.  
- Citations: None yet; to be added when a dedicated kinematics research file is created.  


