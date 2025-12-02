# Lessons Blueprint: P2-C3 Actuators & Motors

**Chapter ID**: P2-C3  
**Version**: v001  
**Created**: 2025-12-01  

---

## Lesson 1: Actuator Fundamentals & Electric Motors

- **Pedagogical Layer**: Layer 1–2 – Foundation & Application  
- **Estimated Time**: 75–90 minutes  
- **Prerequisites**: P2‑C1 (Mechanical Structures), basic physics (force, torque, energy).  
- **Learning Outcomes**:  
  1. Explain what an actuator is and why robots need different kinds of actuators.  
  2. Define torque, speed, power, efficiency, and duty cycle in simple terms.  
  3. Describe the main characteristics of brushed DC, brushless DC, and stepper motors.  

### Part 1: Hook

- Present two contrasting robots: a tiny desktop arm and a heavy industrial manipulator. Ask: *“Why don’t we just put the same motor everywhere?”*  

**AI Touchpoint – Diagnostic Hook**  
- Prompt: *“Given three tasks (precise positioning, fast scanning, heavy lifting), which motor type would you pick and why?”*  

### Part 2: Theory

- Introduce actuators as the “muscles” of robots.  
- Explain torque, speed, and power with simple numeric examples (e.g., turning a wrench, lifting a weight).  
- Walk through brushed DC, BLDC, and stepper motors: how they generate torque and how they are controlled.  

### Part 3: Walkthrough

- Read a simplified motor datasheet and interpret the torque–speed curve.  
- Show how to estimate whether a given motor can move a link or wheel under a load.  

**AI Touchpoint – Tutor**  
- Students ask: *“Explain the trade-off between torque and speed on a motor curve in 3–4 sentences.”*  

### Part 4: Challenge

- Students choose a simple mechanism (e.g., a small arm joint or a wheel) and sketch which motor type and rough size they might use, justifying the choice qualitatively.  

**AI Touchpoint – SDD‑RI Challenge**  
- AI evaluates the choice for plausibility and suggests one improvement (e.g., higher torque margin, different motor class).  

### Part 5: Takeaways

- Key messages:  
  - Actuators convert electrical power into mechanical work.  
  - Different motor types suit different tasks.  
  - Torque, speed, and power must be matched to the mechanism and job.  

### Part 6: Learn with AI

- RI component: `motor_selector`, which, given a brief task description and constraints, proposes a candidate motor type and explains why.

---

## Lesson 2: Gearing, Compliance, and High-Power Actuation

- **Pedagogical Layer**: Layer 2–3 – Application & Analysis  
- **Estimated Time**: 75–90 minutes  
- **Prerequisites**: Lesson 1; understanding of mechanical advantage from basic physics.  
- **Learning Outcomes**:  
  1. Explain why gearboxes are used and how gear ratio affects torque and speed.  
  2. Describe series elastic and compliant actuators and their role in safety and force control.  
  3. Recognize when hydraulic and pneumatic actuators are appropriate.  

### Part 1: Hook

- Show a video or story: a robot arm that stalls when lifting a heavy object vs. an arm that moves smoothly after a gearbox change. Ask: *“What changed?”*  

**AI Touchpoint – Diagnostic Hook**  
- AI prompts learners to label whether the limitation was motor torque, gearing, or control.  

### Part 2: Theory

- Introduce gear ratio and mechanical advantage with simple examples (e.g., bicycles, screw jacks).  
- Explain common transmission types: spur/planetary gearboxes, harmonic drives, belts, and cable/tendon systems.  
- Introduce series elastic actuators: motor + gearbox + spring → safer, more controllable force.  
- Provide a high‑level overview of hydraulic and pneumatic actuators and where they appear (e.g., heavy industrial robots, legged machines, grippers).  

### Part 3: Walkthrough

- Compute simple gear ratio examples and show how they shift a motor’s torque–speed curve.  
- Analyze a series elastic actuator diagram and discuss how the spring changes behavior during impact or contact tasks.  

**AI Touchpoint – Tutor**  
- Students ask: *“Explain why adding a gearbox can make a small motor lift a heavier load, and what we trade away.”*  

### Part 4: Challenge

- Students design a transmission concept for either a robot shoulder joint or a drive wheel, picking a rough gear ratio and explaining how it affects performance.  

**AI Touchpoint – SDD‑RI Challenge**  
- AI reviews the concept and suggests adjustments (e.g., different ratio, added compliance, or a different actuator type).  

### Part 5: Takeaways

- Core ideas:  
  - Motors are usually paired with transmissions to meet torque and speed requirements.  
  - Compliance can improve safety and performance in contact‑rich tasks.  
  - Hydraulic/pneumatic systems trade simplicity and cleanliness for very high power density and complexity.  

### Part 6: Learn with AI

- RI component: `geartrain_explorer`, which helps students visualize how changing gear ratios affects torque, speed, and reflected inertia.

---

## Lesson 3: Sensing, Safety, and System-Level Actuator Choices

- **Pedagogical Layer**: Layer 3–4 – Analysis & Synthesis  
- **Estimated Time**: 75–90 minutes  
- **Prerequisites**: Lessons 1–2; P2‑C2 (Sensors & Perception Hardware).  
- **Learning Outcomes**:  
  1. Describe how encoders, current sensors, and torque sensors support safe and precise actuation.  
  2. Identify common actuator failure modes and corresponding safety mechanisms.  
  3. Propose actuator choices for different robot types (arm, mobile base, humanoid leg) and justify them.  

### Part 1: Hook

- Scenario: a robot joint overheats and shuts down unexpectedly during a demo. Ask: *“Was this a control bug, a sensing problem, or an actuator sizing mistake?”*  

**AI Touchpoint – Diagnostic Hook**  
- AI asks learners to classify the likely cause and suggest additional information they would want (sensor logs, thermal data, current traces).  

### Part 2: Theory

- Describe sensing inside actuators:  
  - Position (encoders, resolvers).  
  - Velocity and current sensing.  
  - Torque sensing and joint force estimation.  
- Outline failure modes: overheating, stall, runaway motion, mechanical breakage, loss of power.  
- Introduce safety mechanisms: brakes, current limits, soft limits, over‑temperature shutdown, redundancy.  

### Part 3: Walkthrough

|- Example 1: small collaborative arm – modest gear ratios, torque sensing, backdrivable joints, safety‑rated brakes.  
|- Example 2: warehouse mobile base – geared DC motors with wheel encoders, current sensing, bump stops, and emergency stop chain.  
|- Example 3: humanoid leg – high‑torque actuators with gearboxes, series elasticity, rich sensing, and strict thermal management.  

**AI Touchpoint – Collaborator**  
- AI helps students fill in a comparison table of actuator choices and safety features for each example.

### Part 4: Challenge

- Students draft an actuation and safety concept for a simple 2‑DOF arm or a small mobile base, specifying:  
  - Motor type and rough size.  
  - Transmission approach.  
  - Key sensors and safety features.  

**AI Touchpoint – SDD‑RI Challenge**  
- AI checks the concept against a checklist (feasibility, safety, maintainability) and suggests one or two improvements.

### Part 5: Takeaways

- Emphasize that actuator choices are **system decisions**, not just component swaps.  
- Reinforce the link between actuation, sensing, safety, and later control and project chapters.  

### Part 6: Learn with AI

- RI component: `actuator_stack_designer`, which guides students through selecting actuators, transmissions, and sensors for a small project given constraints on cost, size, and safety.


