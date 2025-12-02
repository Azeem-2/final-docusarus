# Lessons Blueprint: P3-C2 Environment Modeling

**Chapter ID**: P3-C2  
**Version**: v001  
**Created**: 2025-12-01  

---

## Lesson 1: Geometry, Materials, and Contacts

- **Pedagogical Layer**: Layer 1–2 – Intuition & Concepts  
- **Estimated Time**: 75–90 minutes  
- **Learning Outcomes**:  
  1. Describe, in plain language, what it means to “model an environment” in a simulator.  
  2. Distinguish between geometry, collision shapes, and physical materials.  
  3. Predict qualitatively how changing basic material parameters (e.g., friction) affects robot behavior.  

### Parts 1–6

- **Hook**: A mobile robot behaves perfectly in a simple “box world” but fails in a cluttered, slightly sloped lab scene—same controller, different environment.  
- **Theory**:  
  - Coordinate frames, units, and scale; why a 1 m cube vs a 0.01 m cube matters.  
  - Scene geometry (visual meshes) vs collision geometry (simplified shapes).  
  - Material parameters: friction, restitution, density; static vs dynamic objects.  
- **Walkthrough**:  
  - Start from an empty plane and progressively add walls, obstacles, and floor materials.  
  - Show how adjusting friction and restitution changes sliding, stopping distance, and bounciness.  
  - Simple decision rules for choosing collision shapes (box vs capsule vs mesh).  
- **Challenge**:  
  - Students receive two short environment descriptions and must propose reasonable geometry and material settings (e.g., “rubber mat”, “polished concrete”).  
  - Optional extension: identify likely failure modes if materials are chosen badly.  
- **Takeaways**:  
  - Environment modeling is about **useful approximations**, not photorealism.  
  - Geometry, collision, and materials together determine how robots move and interact with the world.  
- **Learn with AI**:  
  - `env_geometry_coach`: RI component that reviews a sketched environment description and suggests geometry and material choices, flagging obvious mismatches (e.g., “too slippery for this task”).  

---

## Lesson 2: Environments for Perception and Sensing

- **Pedagogical Layer**: Layer 2–3 – Concepts & Application  
- **Estimated Time**: 75–90 minutes  
- **Learning Outcomes**:  
  1. Explain how lighting, textures, and object layout affect simulated sensor data.  
  2. Recognize common perception pitfalls caused by unrealistic or inconsistent environments.  
  3. Propose simple environment adjustments that make perception tasks more realistic or robust.  

### Parts 1–6

- **Hook**: A vision-based line follower that works in bright, high-contrast simulation images but fails instantly in a dimmer, slightly noisy real lab.  
- **Theory**:  
  - How simulated cameras, depth sensors, and lidar “see” the environment.  
  - The role of textures, lighting, and object placement in generating realistic sensor data.  
  - Scale and alignment issues (e.g., misaligned frames, wrong sensor height).  
- **Walkthrough**:  
  - Compare two scenes: one overly clean and uniform, one more realistic with varied textures and lighting.  
  - Show qualitative sensor outputs (images or depth slices) from both scenes.  
  - Demonstrate how small changes (e.g., adding a reflective surface) can break naive perception pipelines.  
- **Challenge**:  
  - Students are given a simple perception task (e.g., obstacle detection) and an “unhelpful” environment description.  
  - They must propose environment edits that make the task more realistic but still learnable (e.g., remove extreme lighting, add modest clutter).  
- **Takeaways**:  
  - Perception performance depends as much on **environment design** as on model architecture.  
  - Good simulation environments strike a balance between clarity and realistic complexity.  
- **Learn with AI**:  
  - `perception_env_reviewer`: RI component that critiques a student’s proposed environment for a perception task, highlighting missing variation or unrealistic choices.  

---

## Lesson 3: Robust Environments and Domain Randomization (Conceptual)

- **Pedagogical Layer**: Layer 3–4 – Integration & Design Intuition  
- **Estimated Time**: 75–90 minutes  
- **Learning Outcomes**:  
  1. Describe, conceptually, what domain randomization is and why it helps sim-to-real.  
  2. Identify which environment aspects are good candidates for simple randomization.  
  3. Recognize the trade-off between training difficulty and robustness when randomizing environments.  

### Parts 1–6

- **Hook**: Two robots trained in simulation: one on a single “perfect” environment and one across many slightly different environments. Only the second works reliably in the real world.  
- **Theory**:  
  - Definition of domain randomization in plain language.  
  - Examples of environment parameters that can be randomized: textures, lighting, object placement, small geometry variations.  
  - Intuitive link to robustness: policies that succeed across many worlds are less brittle.  
- **Walkthrough**:  
  - Start from a baseline scene and introduce controlled randomness (e.g., small shifts in obstacles, random wall colors, minor lighting changes).  
  - Discuss how too much randomness can make learning unnecessarily hard.  
- **Challenge**:  
  - Students design a minimal domain-randomized environment spec for a simple navigation task, choosing 3–4 parameters to vary and justifying their choices.  
- **Takeaways**:  
  - Environment modeling is a lever for building **robust** policies, not just pretty demos.  
  - Simple, well-chosen randomizations often provide large benefits for sim-to-real transfer.  
- **Learn with AI**:  
  - `randomization_planner`: RI component that reviews a student’s randomization plan and suggests tweaks to maintain balance between difficulty and realism.  


