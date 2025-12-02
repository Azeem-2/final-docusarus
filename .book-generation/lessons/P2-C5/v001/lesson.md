# Lessons Blueprint: P2-C5 Kinematics

**Chapter ID**: P2-C5  
**Version**: v001  
**Created**: 2025-12-01  

---

## Lesson 1: Frames, Joints, and Workspace

- **Pedagogical Layer**: Layer 1–2 – Intuition & Concepts  
- **Estimated Time**: 75–90 minutes  
- **Learning Outcomes**:  
  1. Explain what is meant by joint space vs task space.  
  2. Describe coordinate frames for a simple planar arm.  
  3. Sketch and reason about a robot’s reachable workspace.  

### Parts 1–6

- Hook: Short story of a robot arm that “can’t reach” an object even though all joints are within limits.  
- Theory: Coordinate frames, joints, links, joint space vs task space.  
- Walkthrough: Visualizing the workspace of a 2‑link planar arm.  
- Challenge: Students sketch approximate workspace for a given link length pair.  
- Takeaways: Frames and workspace as building blocks for all later kinematics.  
- Learn with AI: `workspace_visualizer` RI component to sanity‑check sketches and descriptions.  

---

## Lesson 2: Forward Kinematics for Simple Arms

- **Pedagogical Layer**: Layer 2–3 – Concepts & Application  
- **Estimated Time**: 75–90 minutes  
- **Learning Outcomes**:  
  1. Intuitively understand how joint angles determine end‑effector position.  
  2. Work through forward kinematics for a 2‑link planar arm step by step.  
  3. Connect the idea of a kinematic chain to URDF descriptions from earlier chapters.  

### Parts 1–6

- Hook: Students see a simulation where joint sliders move an arm; question: *“How is the end‑effector position computed?”*  
- Theory: Kinematic chains, simple formulas for 2‑link planar arm (no heavy matrix notation).  
- Walkthrough: Guided computation from joint angles to x,y position with pictures.  
- Challenge: Students compute one or two positions given angles and link lengths.  
- Takeaways: Forward kinematics as a mapping from joint space to task space.  
- Learn with AI: `fk_step_explainer` RI component that walks students through each step of the FK calculation.  

---

## Lesson 3: Inverse Kinematics, Redundancy, and Singularities (Conceptual)

- **Pedagogical Layer**: Layer 3–4 – Analysis & Integration  
- **Estimated Time**: 75–90 minutes  
- **Learning Outcomes**:  
  1. Explain, at a high level, what inverse kinematics (IK) is and why it is harder than FK.  
  2. Recognize redundancy (multiple joint solutions) and singularities (problematic configurations) conceptually.  
  3. Understand how IK solutions feed motion planning and control.  

### Parts 1–6

- Hook: Show a robot arm trying to reach around an obstacle; multiple joint poses reach the same point. Ask: *“Which one should we choose?”*  
- Theory: Conceptual description of IK, redundancy, and singularities using pictures and analogies, not full Jacobian math.  
- Walkthrough: Simple planar examples: two different joint configurations producing the same end‑effector position; intuitive “stretched‑out” singularity.  
- Challenge: Students identify ambiguous poses and “difficult” poses in given sketches.  
- Takeaways: IK is many‑to‑one and sometimes ill‑conditioned; later chapters will introduce more formal tools.  
- Learn with AI: `ik_scenario_reviewer` RI component that comments on student sketches and explanations of redundancy and singularities.  


