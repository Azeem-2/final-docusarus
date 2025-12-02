# Part 2 Safety Audit – Physical Labs (T149)

**Scope**: Review all Part 2 (Physical Robotics Foundations) chapters for explicit hazard warnings and safety guidance in physical labs and hands-on activities.  
**Chapters**: P2-C1 (Mechanical Structures), P2-C2 (Sensors & Perception Hardware), P2-C3 (Actuators & Motors), P2-C4 (Power Systems & Batteries), P2-C5 (Kinematics), P2-C6 (Dynamics), P2-C7 (Control Systems).  
**Date**: 2025-12-01  
**Auditor**: internal book QA (simulated safety-auditor role; final external safety sign-off deferred to Phase 10 tasks T222–T225).

---

## Checklist

- Mechanical hazards (pinch points, sharp edges, crush zones) called out where relevant.  
- Electrical hazards (battery handling, exposed wiring, short-circuits) called out where relevant.  
- Motion hazards (fast-moving joints, runaway bases, unstable setups) called out where relevant.  
- PPE and environment guidance (eye protection, clear workspace, supervision) included where appropriate.  
- Emergency stop / power cut guidance mentioned for more energetic systems.  

---

## Chapter-by-Chapter Notes

### P2-C1 Mechanical Structures

- Labs focus on basic chassis/frame assembly and static load experiments.  
- Mechanical hazards (pinch points, dropped components) are mentioned with guidance on careful handling and stable support.  
- No high-voltage or high-speed motion in this chapter’s labs; electrical hazards are minimal and covered in later chapters.  
- **Status**: Safety guidance is adequate for beginner mechanical activities; to be cross-referenced with Part 6 project labs in Phase 10.

### P2-C2 Sensors & Perception Hardware

- Labs involve mounting sensors, wiring basic circuits, and performing calibration.  
- Electrical cautions (avoid shorting battery terminals, double-check polarity, secure wiring) are present in lab descriptions.  
- Mechanical hazards limited to mounting on moving platforms; notes remind learners to power off before adjusting hardware.  
- **Status**: Adequate base-level electrical and mechanical warnings; will be revisited when integrated with Part 6 mobile robot projects.

### P2-C3 Actuators & Motors

- Labs introduce DC motors, servos, and simple gearing; motion hazards are explicitly mentioned (keep fingers away from gears, avoid loose clothing near spinning shafts).  
- Recommendations to start with low voltages/currents and to secure test rigs to a stable surface are included.  
- Encourages the use of current-limited supplies or fuses where possible.  
- **Status**: Safety emphasis appropriate; recommend adding an explicit note on disconnecting power before mechanical adjustments in future revisions.

### P2-C4 Power Systems & Batteries

- Strong focus on battery chemistry, charge/discharge limits, and basic protection concepts.  
- Labs include runtime estimation and controlled load tests, with clear guidance on:
  - Never shorting battery terminals.  
  - Using appropriate chargers.  
  - Monitoring temperature and stopping tests if packs become hot or swollen.  
- **Status**: Safety coverage is good for introductory work; future iterations should reference any institution-specific battery handling policies when applicable.

### P2-C5 Kinematics

- Labs are primarily geometric/kinematic, often using low-power arms or simulation equivalents.  
- For physical setups, notes remind learners to keep hands clear of joints and to move slowly when testing new configurations.  
- No high-energy experiments; primary risks are pinch points and minor collisions.  
- **Status**: Safety comments are present and sufficient for the intended low-energy setups.

### P2-C6 Dynamics

- Conceptual focus on forces, torques, inertia, and friction; labs may involve weighted arms or small mobile bases.  
- Guidance emphasizes:
  - Starting with small masses and low speeds.  
  - Securing test rigs to prevent tipping.  
  - Keeping observers clear of motion paths.  
- **Status**: Safety guidance is reasonable; recommend adding a short “if in doubt, reduce speed and mass” sidebar in a future draft.

### P2-C7 Control Systems

- Labs emphasize tuning and qualitative response observation rather than pushing systems to their limits.  
- Explicit cautions appear around:
  - Avoiding aggressive gains on powerful hardware.  
  - Testing new controllers at low speed/torque first.  
  - Providing an easy way to cut power or stop the robot if behavior becomes unstable.  
- **Status**: Adequate for introductory control labs; can be further strengthened by cross-linking to Part 5/6 safety material in Phase 10.

---

## Overall Assessment

- All Part 2 chapters include at least **baseline safety guidance** for their physical labs, covering mechanical, electrical, and motion risks at an introductory level.  
- No critical omissions were found for the scope of Part 2; however, higher-energy humanoid and project labs in Parts 5–6 will require stricter safety treatment (covered by later tasks T180–T182 and T222–T225).  
- **Recommendation**: In Phase 10, perform an integrated safety pass across Parts 2, 5, and 6 to harmonize warnings, PPE guidance, and emergency procedures.


