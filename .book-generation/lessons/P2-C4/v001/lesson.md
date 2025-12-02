# Lessons Blueprint: P2-C4 Power Systems & Batteries

**Chapter ID**: P2-C4  
**Version**: v001  
**Created**: 2025-12-01  

---

## Lesson 1: Energy & Power Fundamentals

- **Pedagogical Layer**: Layer 1–2 – Foundation & Concepts  
- **Estimated Time**: 75–90 minutes  
- **Prerequisites**: Basic physics (force, energy), P2‑C1–C3 intuition about loads (structures, sensors, actuators).  
- **Learning Outcomes**:  
  1. Define voltage, current, power, and energy in intuitive terms.  
  2. Compute simple power and energy examples for common robot loads.  
  3. Explain duty cycle and why it matters for sizing power systems.  

### Part 1: Hook

- Story: A small mobile robot runs perfectly in the lab but dies halfway through a demo because the “biggest” battery the team found was not actually sized correctly. Ask: *“What did they miss?”*  

**AI Touchpoint – Diagnostic Hook**  
- Prompt: *“List three possible mistakes in how the team thought about power and energy for their robot.”*  

### Part 2: Theory

- Introduce voltage, current, power (P = V × I), and energy (E = P × t) using everyday examples (phone charging, lamps, small DC motors).  
- Explain efficiency and loss qualitatively (where power “goes” in wires, converters, motors).  
- Define duty cycle with simple plots (on/off patterns and average power).  

### Part 3: Walkthrough

- Work through a simple power budget for a small robot: compute approximate power draw for electronics, sensors, and motors.  
- Show how changing duty cycles (e.g., motors not at full load all the time) affects average power.  

**AI Touchpoint – Tutor**  
- Students ask: *“Explain the difference between power and energy in 3–4 sentences, with one robot example.”*  

### Part 4: Challenge

- Students build a rough power budget for a chosen example robot (e.g., a line-following robot or small arm), listing main loads and estimated power draws.  

**AI Touchpoint – SDD‑RI Challenge**  
- AI checks the budget for missing loads or unrealistic numbers and suggests one improvement.  

### Part 5: Takeaways

- Reinforce that power systems design starts with **loads** and **duty cycles**, not just picking a random battery.  
- Emphasize that simple calculations are enough to avoid many common mistakes.  

### Part 6: Learn with AI

- RI component: `power_budget_assistant`, which helps students refine a rough power budget and estimate daily energy needs for a project robot.  

---

## Lesson 2: Batteries & Power Electronics

- **Pedagogical Layer**: Layer 2–3 – Application & Analysis  
- **Estimated Time**: 75–90 minutes  
- **Prerequisites**: Lesson 1 (power/energy basics).  
- **Learning Outcomes**:  
  1. Compare common battery chemistries and choose appropriate ones for typical robots.  
  2. Interpret basic battery and converter specs (voltage, capacity, C‑rate, efficiency).  
  3. Describe the roles of BMS, fuses, switches, and DC/DC converters in a safe power system.  

### Part 1: Hook

- Scenario: A robot runs fine on the bench but resets when it starts driving because voltage dips under load. Ask: *“Is the problem the battery, the wiring, or something else?”*  

**AI Touchpoint – Diagnostic Hook**  
- AI prompts students to hypothesize reasons for brownouts and what information they would need (battery datasheet, current logs, wiring diagram).  

### Part 2: Theory

- Overview of battery chemistries used in robots: Li-ion, LiFePO₄, NiMH, lead‑acid; trade‑offs in energy density, cycle life, safety, and cost.  
- Introduce capacity (Ah), C‑rate, and how to reason about runtime at a conceptual level.  
- Explain battery management systems (BMS): monitoring, balancing, protection.  
- Introduce power electronics: DC/DC converters, voltage rails, simple distribution topologies.  

### Part 3: Walkthrough

- Read a simplified battery datasheet: note capacity, nominal voltage, continuous/peak currents.  
- Walk through a simple distribution diagram: battery → BMS → main switch → DC/DC converters → loads.  

**AI Touchpoint – Tutor**  
- Students ask: *“Explain what a C‑rate is and why exceeding it can be dangerous.”*  

### Part 4: Challenge

- Students choose a battery chemistry and sketch a simple power tree for their robot (including BMS and at least one DC/DC converter), with a short justification.  

**AI Touchpoint – SDD‑RI Challenge**  
- AI reviews the proposed power tree for missing protection elements and suggests safety improvements.  

### Part 5: Takeaways

- Key messages:  
  - Battery chemistry and capacity strongly influence runtime, safety, and weight.  
  - Power electronics and protection circuits are essential, not optional extras.  

### Part 6: Learn with AI

- RI component: `battery_selector`, which helps students compare chemistries and capacities for a target power budget and constraints (weight, cost, safety).  

---

## Lesson 3: Charging, Safety, and Power Architectures

- **Pedagogical Layer**: Layer 3–4 – Integration & Design  
- **Estimated Time**: 75–90 minutes  
- **Prerequisites**: Lessons 1–2; Part 2 context on sensors and actuators.  
- **Learning Outcomes**:  
  1. Outline safe charging practices and basic charge profiles for common chemistries.  
  2. Estimate runtime for a robot given a battery pack and power budget.  
  3. Propose a safe power architecture for a small robot, including fuses, switches, and isolation points.  

### Part 1: Hook

- Story: A team damages a battery pack by charging it incorrectly between lab sessions. Ask: *“What went wrong, and how could they have designed their system to prevent this?”*  

**AI Touchpoint – Diagnostic Hook**  
- AI asks learners to identify at least two mistakes and two safer alternatives.  

### Part 2: Theory

- Conceptual charging basics: CC/CV charging for Li‑ion, safe charge rates, and why you should respect manufacturer guidelines.  
- Runtime estimation: converting capacity and power budget into approximate run time, and why conservative margins are important.  
- Safety and protection: fuses, circuit breakers, emergency stops, isolation switches, labeling, and basic enclosure practices.  

### Part 3: Walkthrough

- Step-by-step runtime estimate for a representative mobile robot or arm, including a safety margin.  
- Example power architecture diagrams for:  
  - A small mobile robot.  
  - A stationary arm with off-board supply.  

**AI Touchpoint – Collaborator**  
- AI helps annotate the diagrams with where protection and measurement points should be added (fuses, current sensors, emergency stop).  

### Part 4: Challenge

- Design challenge: students propose a complete power architecture for a small educational robot, including:  
  - Battery choice and capacity.  
  - Basic distribution and conversion.  
  - Charging strategy.  
  - Key safety features.  

**AI Touchpoint – SDD‑RI Challenge**  
- AI reviews the design against a checklist (runtime target, safety, maintainability) and suggests one or two changes.  

### Part 5: Takeaways

- Emphasize that power systems are **core engineering**, not an afterthought.  
- Connect back to mechanical, sensing, and actuation chapters and forward to control and safety chapters.  

### Part 6: Learn with AI

- RI component: `power_architecture_designer`, which guides students through iterating on a power system design and flags common mistakes (undersized packs, missing fuses, unsafe charging assumptions).  

---


