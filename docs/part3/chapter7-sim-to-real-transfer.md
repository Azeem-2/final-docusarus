---
title: Sim-to-Real Transfer
slug: /part3/chapter7-sim-to-real-transfer
sidebar_label: Sim-to-Real Transfer
sidebar_position: 7
---

# Chapter: Sim-to-Real Transfer (P3-C7)
## 1. Introduction – Why Sim-to-Real Transfer Matters

Policies trained in simulation must work on physical robots. This is the central challenge of **sim-to-real transfer**: bridging the gap between virtual training and real-world deployment.

In this chapter, you will learn:

- **The reality gap**: why simulated and real robot behavior differ.  
- **Domain randomization**: training policies across diverse conditions for robustness.  
- **Validation techniques**: sim-to-sim testing, system identification, teacher-student distillation.  
- **Practical workflows**: complete pipelines from simulation to physical deployment.  
- **Safety mechanisms**: critical protections for physical robot deployment.

The goal is to understand how to successfully transfer simulation-trained policies to physical robots, making rapid robotics development practical and safe.

---

## 2. The Reality Gap: Understanding the Problem

A policy trained in simulation might achieve 95% success, but when deployed to a physical robot, success can drop to 60% or lower. This discrepancy is called the **reality gap**.

### Sources of the Reality Gap

**Modeling inaccuracies**:
- Contact dynamics: simulation approximates contact forces, but real contact involves complex friction, deformation, and surface interactions.  
- Friction models: simplified friction coefficients don't capture all real-world variations.  
- Cable dynamics: cables, wires, and flexible elements are often ignored in simulation but affect real robot behavior.

**Unmodeled dynamics**:
- Wear and degradation: motors, joints, and sensors change over time.  
- Temperature effects: motor performance, sensor calibration, and material properties vary with temperature.  
- Manufacturing variations: no two physical robots are identical.

**Sensor differences**:
- Noise models: simulated sensors may not capture all real noise characteristics.  
- Calibration drift: real sensors require periodic recalibration.  
- Latency: real sensors have processing delays that simulation might not model accurately.

**Actuator dynamics**:
- Motor delays: real motors have response delays that affect control.  
- Torque limits: physical limits may differ from simulation assumptions.  
- Backlash and compliance: mechanical play and flexibility in real actuators.

**Environmental variations**:
- Lighting: real-world lighting is complex and variable.  
- Surface properties: real surfaces have texture, wear, and contamination.  
- Air currents, vibrations, and other disturbances.

The reality gap is inevitable—simulation can never perfectly model reality. The goal is to make policies robust enough to handle these differences.

---

## 3. Domain Randomization: Building Robust Policies

**Domain randomization** is a key technique for building robust policies: instead of training on a single "perfect" simulation, train across many varied conditions. This teaches policies to adapt to differences between simulation and reality.

### Physics Randomization

Vary physical parameters during training:

- **Mass**: ±20% variation (simulates payload changes, manufacturing differences).  
- **Friction**: ±30% variation (different surfaces: concrete, metal, wet).  
- **Actuator gains**: ±15% variation (motor degradation, calibration differences).  
- **Joint damping**: ±25% variation (wear effects, lubrication).

For example, a mobile robot trained with randomized friction learns to handle both slippery and grippy surfaces, making it more robust when deployed.

### Visual Randomization

For vision-based tasks, randomize visual appearance:

- **Textures**: vary object and surface textures.  
- **Lighting**: different intensities, colors, and directions.  
- **Object appearance**: colors, shapes, sizes within reasonable bounds.

This helps policies generalize across different lighting conditions and object appearances in the real world.

### Dynamics Randomization

Vary dynamic parameters:

- **Time delays**: simulate actuator and sensor delays.  
- **Torque limits**: vary maximum torques.  
- **Sensor noise**: add realistic noise to sensor readings.

### Environmental Randomization

Vary the environment:

- **Terrain**: flat, slopes, stairs, obstacles.  
- **Object placement**: randomize positions, orientations.  
- **Disturbances**: random forces, pushes, collisions.

### Trade-offs

Domain randomization has trade-offs:

- **Too little**: policy overfits to simulation conditions, fails on real robot.  
- **Too much**: learning becomes unnecessarily difficult, training takes longer.  
- **Balance**: enough variation for robustness, not so much that learning is impossible.

The key is choosing the right parameters to randomize based on your task and robot.

---

## 4. System Identification: Calibrating Simulation to Reality

**System identification** is the process of measuring physical robot properties and using them to improve simulation accuracy. This is especially important for high-fidelity transfer tasks like manipulation.

### Key Parameters to Identify

- **Mass distribution**: total mass and how it's distributed across links.  
- **Inertia**: rotational inertia of each link.  
- **Friction coefficients**: static and dynamic friction for different surfaces.  
- **Actuator dynamics**: motor gains, time constants, torque limits.

### Measurement Techniques

**Static tests**:
- Measure link masses directly.  
- Test friction by measuring forces required to move objects.  
- Calibrate sensors (cameras, force sensors).

**Dynamic tests**:
- Record robot motion under known forces.  
- Estimate inertia from acceleration responses.  
- Measure actuator response times.

**Parameter estimation**:
- Use optimization to fit simulation parameters to real robot data.  
- Compare simulated and real trajectories, adjust parameters to match.

### Updating Simulation

Once parameters are identified:

1. Update simulation model with measured values.  
2. Retrain policy in calibrated simulation.  
3. Deploy to physical robot (should have smaller reality gap).

System identification is most valuable when high accuracy is critical, such as manipulation tasks where small errors matter.

---

## 5. Sim-to-Sim Validation: Testing Across Simulators

Before deploying to a physical robot, validate your policy across different simulators. This is called **sim-to-sim validation**.

### Why Sim-to-Sim?

If a policy can't transfer between simulators (e.g., Isaac Sim → MuJoCo), it's unlikely to work on a real robot. Sim-to-sim validation is a low-cost way to test robustness before expensive physical testing.

### Workflow

1. **Train in primary simulator**: Train policy in your primary simulator (e.g., Isaac Sim).  
2. **Export policy**: Save the trained policy.  
3. **Test in secondary simulator**: Load policy in a different simulator (e.g., MuJoCo, Gazebo) without retraining.  
4. **Evaluate performance**: Compare success rates across simulators.  
5. **Iterate if needed**: If transfer fails, increase domain randomization and retrain.

### Success Criteria

A policy that achieves:
- 95% success in Isaac Sim
- 90% success in MuJoCo
- 85% success in Gazebo

is much more likely to work on a physical robot than one that only works in one simulator.

### Common Failures

- **Simulator-specific artifacts**: policy relies on quirks of one physics engine.  
- **Physics engine differences**: different contact models, solvers produce different results.  
- **Overfitting**: policy memorized simulator-specific behaviors.

Sim-to-sim validation catches these issues early, before physical deployment.

---

## 6. Teacher-Student Distillation: Removing Privileged Observations

Simulation provides **privileged observations**—information available in simulation but not on real robots. Examples include:

- Ground truth velocities (from physics engine).  
- Contact forces (from collision detection).  
- Perfect state estimates (no sensor noise).

A policy trained with these privileged observations won't work on a real robot that only has sensor data.

### Teacher-Student Approach

**Teacher policy**: Trained with privileged observations, achieves high performance in simulation.

**Student policy**: Trained to mimic the teacher using only real-sensor observations (what's actually available on the robot).

**Distillation process**:
1. Train teacher policy with privileged observations.  
2. Collect teacher's actions on many states (using real-sensor observations only).  
3. Train student policy via behavior cloning to predict teacher's actions.  
4. Fine-tune student policy with RL using only real sensors.

### Why This Works

The teacher learns the task efficiently with perfect information. The student learns to approximate the teacher's behavior using only realistic sensors. Fine-tuning improves the student further.

This approach is especially useful for complex tasks where privileged information significantly helps learning.

---

## 7. Fine-Tuning with Real-World Data

Even with domain randomization and validation, initial deployment often shows performance gaps. **Fine-tuning** adapts the policy using real-world data.

### When to Fine-Tune

- Initial deployment shows lower success than simulation.  
- Policy fails on specific failure modes.  
- Real-world conditions differ significantly from simulation.

### Data Collection

Record real-world episodes:
- **Successes**: what worked well.  
- **Failures**: what went wrong, why.

Collect enough data to be representative but not so much that it's expensive.

### Fine-Tuning Strategies

**Augment simulation training**:
- Add real-world trajectories to simulation training data.  
- Retrain policy with mixed simulation and real data.

**Direct RL fine-tuning**:
- Continue RL training on the physical robot (carefully, with safety limits).  
- Use real-world rewards to improve policy.

**Imitation learning**:
- Collect expert demonstrations on physical robot.  
- Use imitation learning to adapt policy.

### Iterative Improvement

Fine-tuning is iterative:

1. Deploy policy → collect data → identify failure modes.  
2. Fine-tune policy → redeploy → collect more data.  
3. Repeat until performance is acceptable.

This closes the reality gap through real-world experience.

---

## 8. Safety Mechanisms for Physical Deployment

Physical robots can cause damage or injury. **Safety mechanisms** are non-negotiable for physical deployment.

### Torque Limits

Prevent excessive forces that could damage hardware or cause injury:

- Set maximum torque per joint.  
- Monitor torques in real-time.  
- Automatically reduce or stop if limits exceeded.

### Attitude Protection

Maintain stability and prevent falls:

- Monitor robot orientation (IMU data).  
- Detect dangerous tilts.  
- Trigger recovery behaviors or emergency stop.

### Joint Mapping Verification

**Critical**: Ensure correct joint-to-motor mapping. A mismatch can cause:
- Robot to move in wrong directions.  
- Collisions and damage.  
- Safety hazards.

Always verify joint mapping before first deployment.

### Gradual Deployment

Start conservative and increase gradually:

- Begin with low gains, limited speeds.  
- Test in controlled environment.  
- Gradually increase limits as confidence grows.

### Emergency Stops

- **Manual override**: human operator can stop robot immediately.  
- **Automatic triggers**: detect dangerous conditions, stop automatically.  
- **Hardware limits**: physical limits that can't be overridden by software.

Safety must be designed in from the start, not added as an afterthought.

---

## 9. Practical Workflows: From Simulation to Physical Robot

A complete sim-to-real workflow integrates all the techniques above:

### Complete Workflow

1. **Train in simulation**: Use domain randomization, train policy with RL.  
2. **Sim-to-sim validation**: Test policy across multiple simulators.  
3. **System identification** (if needed): Calibrate simulation for high-fidelity tasks.  
4. **Teacher-student distillation** (if needed): Remove privileged observations.  
5. **Deploy to physical**: With safety mechanisms enabled.  
6. **Collect real-world data**: Record successes and failures.  
7. **Fine-tune**: Adapt policy with real data.  
8. **Iterate**: Deploy → collect → improve → redeploy.

### Isaac Sim Workflow

- Train policy in Isaac Sim with domain randomization.  
- Export policy.  
- Deploy via RL-SAR framework or Isaac Lab.  
- Monitor telemetry, collect data.  
- Fine-tune as needed.

### Gazebo/ROS2 Workflow

- Train in simulation (Isaac Sim or MuJoCo).  
- Validate in Gazebo.  
- Deploy via ROS2 with safety plugins.  
- Use ROS2 tools for monitoring and data collection.

### Hardware Interfaces

Map simulation commands to physical actuators:

- Joint torques → motor commands.  
- Account for calibration, offsets, limits.  
- Handle communication delays.

### Monitoring and Debugging

- **Telemetry**: real-time joint states, torques, sensor data.  
- **Logging**: record all episodes for analysis.  
- **Failure analysis**: identify why failures occurred, improve policy.

---

## 10. Summary and Integration with Part 3

In this chapter you:

- Learned that the reality gap is inevitable but manageable through proper techniques.  
- Explored domain randomization as a key technique for building robust policies.  
- Understood validation techniques: sim-to-sim testing, system identification, teacher-student distillation.  
- Recognized the importance of safety mechanisms for physical deployment.  
- Integrated all Part 3 concepts into practical sim-to-real workflows.

**Integration with Part 3**:
- **Physics engines (P3-C1)**: Provide the simulation foundation.  
- **Environment modeling (P3-C2)**: Domain randomization builds on environment design.  
- **RL basics (P3-C3)**: Policies trained with RL must transfer to reality.  
- **Imitation learning (P3-C4)**: Teacher-student distillation uses imitation learning.  
- **Motion planning (P3-C5)**: Planning algorithms must work in both simulation and reality.  
- **Simulation toolchains (P3-C6)**: Toolchains enable the complete workflows.

Together, these chapters form a complete foundation for simulation-based robotics development, from physics engines to real-world deployment.

In Part 4 (AI for Robotics), you'll see how AI models integrate with these simulation and physical systems to create intelligent robots.

---

