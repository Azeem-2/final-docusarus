---
title: Power Systems and Batteries
slug: /part2/chapter4-power-systems-and-batteries
sidebar_label: Power Systems and Batteries
sidebar_position: 4
---

# Chapter 4: Power Systems & Batteries (P2-C4)
<!-- This manuscript chapter mirrors the current working draft at:
     .book-generation/drafts/P2-C4/v001/draft.md
     Keep these in sync during later editorial and QA passes. -->

## 1. Introduction – Powering Robots Safely

In previous chapters you saw how **mechanical structures**, **sensors**, and **actuators** define what a robot can do and what it can know. Underneath all of that sits a quieter but equally critical layer: the **power system**. If the power system is undersized, unstable, or unsafe, even the best hardware and software will fail in unpredictable ways.

Many beginner projects underestimate this layer: they pick a battery that “looks big enough”, wire things together in an ad‑hoc way, and hope for the best. The result is often disappointing: robots that brown out when they start moving, batteries that wear out quickly, or, in the worst case, dangerous failures such as overheated packs or melted wires.

In this chapter, you will build an intuitive and practical understanding of how to power robots safely and effectively. You will:

- Learn the basics of voltage, current, power, energy, and duty cycle.  
- See how different battery chemistries trade off energy density, lifetime, cost, and safety.  
- Understand the role of battery management systems (BMS), DC/DC converters, and protection elements.  
- Practice estimating runtime and designing simple power architectures for common robot types.  

By the end, you should be able to look at a robot concept and sketch a credible power system that meets its needs without relying on guesswork.

---

## 2. Energy & Power Fundamentals

At the heart of every power system are a few core electrical quantities:

- **Voltage (V)** is like electrical “pressure” that pushes charge through a circuit.  
- **Current (I)** is the rate at which charge flows, measured in amperes (A).  
- **Power (P)** is the rate of doing work: \(P = V \times I\).  
- **Energy (E)** is power accumulated over time: \(E = P \times t\).  

For robots, these ideas show up everywhere. A motor may draw 3 A at 12 V when under moderate load, corresponding to about 36 W of power. A single‑board computer might draw 10 W continuously. If the robot needs to run for an hour, the **energy** requirement is the sum of all loads over that time.

Another important idea is **duty cycle**—how much of the time a given load is active and at what level. A motor that occasionally spikes to 50 W but spends most of its time at 10 W has a very different impact on energy consumption and thermal behavior than one that runs at 50 W continuously.

For a first‑pass design, you can treat each significant load as having:

- An approximate **average power** over the mission (e.g., 5 W for sensors, 15 W for compute, 30 W for drive motors).  
- A **peak power** that informs instantaneous current and protection sizing.  

These approximations are enough to build a basic power budget and select battery capacity with safety margins.

---

## 3. Battery Technologies

Robots commonly use rechargeable batteries. Several chemistries appear frequently:

- **Lithium‑ion (Li‑ion)**: High energy density and good cycle life, widely used in laptops and drones. Requires careful protection and charging.  
- **Lithium iron phosphate (LiFePO₄)**: Lower energy density than Li‑ion but improved thermal stability and cycle life; popular in robotics and energy storage.  
- **Nickel‑metal hydride (NiMH)**: Robust and relatively simple to handle, with lower energy density; sometimes used in educational robots.  
- **Lead‑acid**: Heavy but inexpensive and tolerant of some abuse; still used for stationary or cart‑mounted systems.  

Key parameters you will encounter:

- **Nominal voltage** (e.g., 12 V, 24 V).  
- **Capacity** in ampere‑hours (Ah), describing how much charge the pack can deliver.  
- **C‑rate**, describing how quickly the pack can be safely charged or discharged relative to its capacity.  
- **Energy density**, indicating how much energy is stored per unit mass or volume.  

Selecting a chemistry involves trade‑offs:

- Small mobile robots and drones often favor Li‑ion for its high energy density.  
- Larger mobile bases and educational platforms may choose LiFePO₄ for safety and longevity.  
- Stationary robots or hobby projects with relaxed weight constraints might use lead‑acid for simplicity and cost.  

While details vary, the overarching pattern is consistent: **safer, longer‑life chemistries often trade some energy density and cost**, and all chemistries demand respect for their limits.

---

## 4. Power Electronics & Distribution

Between the battery and the robot’s loads lies a network of **power electronics** and wiring:

- A **Battery Management System (BMS)** monitors cell voltages, currents, and temperatures, enforcing safe operating limits.  
- **Fuses** or circuit breakers protect wiring and devices from excessive currents.  
- **Switches** and **contactors** provide clear on/off control and isolation.  
- **DC/DC converters** step voltages up or down to provide stable rails (e.g., 5 V for logic, 12 V for motors).  

A typical small mobile robot might use:

- A single battery pack (e.g., 4‑cell LiFePO₄).  
- A BMS integrated into the pack or added externally.  
- A main power switch.  
- One or more DC/DC converters to generate logic and sensor rails from the pack voltage.  

The goals of this layer are to:

- Deliver stable voltages and currents to each load.  
- Protect against shorts and overloads.  
- Provide convenient control and diagnostic points (e.g., current sensing, voltage monitoring).  

You will see more formal circuit design in later chapters and projects; here the focus is on understanding the **roles** of these components and how they fit into a clean, readable power architecture.

---

## 5. Charging & Runtime Estimation

Charging is where many safety‑critical mistakes happen. Each chemistry has recommended **charge profiles** and limits:

- Li‑ion and LiFePO₄ commonly use **constant‑current/constant‑voltage (CC/CV)** charging with strict upper voltage limits.  
- Over‑charging or charging at too high a current can damage cells or create hazards.  
- Many educational platforms rely on manufacturer‑supplied chargers and packs to avoid low‑level mistakes.  

From a robotics perspective, you should at least:

- Understand the **maximum charge current** and voltage for your pack.  
- Plan for safe connection/disconnection procedures.  
- Avoid improvised chargers that do not match the chemistry and pack design.  

For **runtime estimation**, you can start with a simple model:

1. Compute an approximate **average power** `\(P_{\text{avg}}\)` from your power budget.  
2. Convert pack capacity (e.g., 10 Ah at 24 V) into **stored energy** `\(E = V \times \text{Ah}\)`.  
3. Estimate runtime `\(t \approx \frac{E \times \eta}{P_{\text{avg}}}\)`, where `\(\eta\)` accounts for inefficiencies and safety margins.  

Real systems are more complex because loads change over time and batteries do not deliver full nameplate capacity under all conditions, but this simple calculation is an essential first check.

---

## 6. Safety & Protection

Power systems carry enough energy to cause damage if mishandled. Safe design includes:

- **Overcurrent protection**: fuses or breakers sized for the wiring and expected loads.  
- **Over/under‑voltage protection**: BMS logic or converters that shut down or limit operation outside safe ranges.  
- **Thermal management**: avoiding enclosed battery packs without ventilation or monitoring.  
- **Isolation and labeling**: clear separation between high‑power and low‑power sections; obvious disconnects and warning labels.  

Many robotics safety incidents trace back to assumptions like “the battery will take care of itself” or “these wires look thick enough”. A structured approach—starting from datasheets, using proper protection, and planning for failure modes—reduces risk significantly.

Later chapters on safety and standards will expand on regulatory frameworks and formal requirements. In this chapter, we lay the practical groundwork: **treat power like any other engineering subsystem, with clear requirements, design, and verification**.

---

## 7. Example Power Architectures

To connect these ideas, consider three high‑level examples:

- **Small mobile robot**:  
  - Single battery pack, BMS, main switch, DC/DC converters for logic and sensors, direct or converter‑fed drive motors.  
  - Emphasis on runtime and compactness; many components may be integrated.  

- **Bench‑top arm**:  
  - External DC supply or battery pack with clear connector, per‑axis drivers, and local low‑voltage logic rails.  
  - Emphasis on safe operation in a lab environment and easy emergency shutdown.  

- **Stationary work cell**:  
  - Mains power feeding industrial supplies, with clear isolation transformers, breakers, and lock‑out/tag‑out procedures.  
  - Emphasis on compliance with electrical codes and robust long‑term operation.  

These examples illustrate that while the **details** of power hardware differ, the **patterns** repeat: central energy source → protection → distribution → loads, all governed by safety and performance requirements.

---

## 8. Summary and Bridge to Kinematics & Control

In this chapter you:

- Built an intuitive understanding of voltage, current, power, energy, and duty cycle.  
- Learned how different battery chemistries serve different robotic use cases.  
- Saw how BMSs, DC/DC converters, and protection elements create safe, stable power distributions.  
- Practiced thinking about charging, runtime, and failure modes from an engineering perspective.  
- Explored example power architectures for common robot classes.  

In upcoming chapters on **kinematics**, **dynamics**, and **control**, you will design motion in more mathematical detail. Throughout that work, remember: every torque, speed, and control signal relies on a power system that must be sized and built with equal care. A well‑designed power system turns theoretical capability into reliable, safe behavior in the real world.


