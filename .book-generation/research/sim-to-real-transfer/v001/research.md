# Research: Sim-to-Real Transfer (P3-C7)

**Chapter**: P3-C7  
**Topic**: Sim-to-Real Transfer  
**Version**: v001  
**Created**: 2025-12-01  
**Agent**: research-agent  
**Skill**: research-methodology

---

## Research Questions

1. What is the reality gap and why does it exist?
2. What are the main techniques for sim-to-real transfer (domain randomization, system identification, fine-tuning)?
3. How do you validate policies before physical deployment (sim-to-sim transfer)?
4. What are the practical workflows for deploying simulation-trained policies to physical robots?
5. How do you measure and improve sim-to-real transfer success rates?

---

## Key Concepts Identified

### Core Concepts
- **Reality Gap**: Discrepancy between simulated and real robot behavior
- **Domain Randomization**: Varying simulation parameters to improve robustness
- **System Identification**: Calibrating simulation to match physical robot
- **Sim-to-Sim Transfer**: Validating policies across different simulators
- **Teacher-Student Distillation**: Training student policy without privileged observations
- **Fine-tuning**: Adapting policies with real-world data

### Techniques
- Domain randomization (physics, visual, dynamics)
- System identification and calibration
- Sim-to-sim validation
- Teacher-student policy distillation
- Fine-tuning with real data
- Safety mechanisms (torque limits, attitude protection)

### Workflows
- Train in simulation → Validate sim-to-sim → Deploy to physical → Fine-tune
- Teacher policy (privileged) → Student policy (real sensors) → Fine-tune

---

## Source Material (MCP Context7)

### RL-SAR Framework (/fan-ziqi/rl_sar)
- **Purpose**: Framework for simulation verification and physical deployment of RL policies
- **Key Features**:
  - Unified interfaces for simulation (Gazebo, MuJoCo) and physical robots
  - Supports quadruped, wheeled, and humanoid robots
  - Safety features: torque limits, attitude protection
  - Sim-to-sim validation before real deployment
- **Use Cases**:
  1. Validate trained RL policies from IsaacGym/IsaacSim in Gazebo or MuJoCo before hardware deployment
  2. Deploy locomotion and manipulation policies on commercial robots
  3. Rapid prototyping with pre-trained models
- **Deployment Platforms**: Unitree A1, Go2, B2, G1 robots

### Isaac Lab (/websites/isaac-sim_github_io_isaaclab_main)
- **Sim-to-Real Workflow**:
  1. Train teacher policy with privileged observations
  2. Distill student policy (remove privileged terms) via behavior cloning
  3. Fine-tune student policy with RL using only real-sensor observations
- **Sim-to-Sim Transfer**: Essential step before real robot deployment
- **Domain Randomization**: Configurable via EventTermCfg variables
  - Actuator gains, dof limits, gravity, tendon parameters
  - Gaussian sampling support

---

## Research Notes

### Reality Gap Sources
- Modeling inaccuracies (contact, friction, cable dynamics)
- Unmodeled dynamics (wear, temperature effects)
- Sensor noise differences
- Actuator dynamics (motor delays, torque limits)
- Environmental variations (lighting, surface properties)

### Domain Randomization Strategies
- **Physics**: Mass ±20%, friction ±30%, actuator gains ±15%
- **Visual**: Textures, lighting, object appearance
- **Dynamics**: Joint damping, time delays, torque limits
- **Environmental**: Terrain variation, object placement

### Validation Pipeline
1. **Sim-to-Sim**: Test policy across different physics engines (Isaac Sim → MuJoCo → Gazebo)
2. **Sim-to-Real**: Deploy to physical robot with safety mechanisms
3. **Fine-tuning**: Collect real-world data, adapt policy

### Safety Considerations
- Torque limits to prevent damage
- Attitude protection for stability
- Joint mapping verification (critical for safety)
- Gradual deployment (start with low gains, increase gradually)

---

## Prerequisites from Previous Chapters

- **P3-C1**: Physics engines (MuJoCo, Isaac Sim, Bullet)
- **P3-C2**: Environment modeling (geometry, materials, domain randomization basics)
- **P3-C3**: RL basics (policies, rewards, value functions)
- **P3-C4**: Imitation learning (behavioral cloning, teacher-student)
- **P3-C5**: Motion planning (collision checking, trajectories)
- **P3-C6**: Simulation toolchains (Isaac Sim, Webots, Gazebo workflows)

---

## Research Status

**Status**: Scaffold complete  
**Next Step**: Proceed to outliner-agent for chapter outline creation

**Note**: This research scaffold provides conceptual foundation. Full research with Tier 1/2 citations will be completed during the writing phase when specific claims need verification.

---

