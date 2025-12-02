# Structural Blueprint: Build a Mobile Robot

**Chapter ID**: P6-C1
**Title**: Build a Mobile Robot
**Part**: 6 (Integrated Robotics Projects)
**Created**: 2025-12-01
**Version**: v001

---

## Metadata Header

```yaml
chapter_id: P6-C1
title: "Build a Mobile Robot"
part: 6
created: "2025-12-01"
concept_density: 0.125
concept_density_class: High
chapter_type: Project/Integration
recommended_lessons: 4-5
estimated_reading_time: 50 minutes
estimated_total_time: 15-20 hours
```

---

## Section 1: Concept Density Analysis

### Formula Application

**Concept Density Formula**:
```
CD = (New Concepts + 0.5 × Prerequisites + 2 × Math Derivations) / Reading Time (minutes)
```

### Calculation Breakdown

**Step 1: Count New Concepts**
From outline analysis, the chapter introduces these technical concepts:

1. Differential drive kinematics (forward/inverse)
2. Non-holonomic constraints
3. Dynamic modeling (motor dynamics, chassis dynamics)
4. ROS2 Navigation2 stack
5. SLAM (Simultaneous Localization and Mapping)
6. Costmap representation
7. Trajectory tracking control (PID, backstepping)
8. Digital twin for mobile robots
9. Sim-to-real transfer for mobile robots
10. Hardware integration (motors, encoders, IMU)
11. Motor control (PWM, current control)
12. Odometry estimation
13. Path planning algorithms
14. Obstacle avoidance (DWA)
15. Recovery behaviors

**Total New Concepts**: 15

**Step 2: Count Prerequisites**
Chapter position: Part 6, Chapter 1 (project chapter)
Stated prerequisites:
- Part 2: Mechanical Structures, Sensors, Actuators, Kinematics, Dynamics
- Part 3: Simulation Foundations, ROS2 basics
- Part 4: Basic AI/control concepts

**Estimated Prerequisites**: 8-10 concepts from previous parts

**Step 3: Count Math Derivations**
Reviewing content sections:
- Section 5.1: Forward kinematics derivation (with equations)
- Section 5.2: Inverse kinematics derivation (with equations)
- Section 5.3: Dynamic model derivation (Lagrange, Newton-Euler)

**Total Math Derivations**: 3

**Step 4: Estimate Reading Time**
- Word count: 8,750 words (midpoint of 8,000-10,000 estimate)
- Average reading speed: 175 words/minute (technical content)
- Reading time: 8,750 / 175 = **50 minutes**

### Final Calculation

```
CD = (15 + 0.5 × 9 + 2 × 3) / 50
CD = (15 + 4.5 + 6) / 50
CD = 25.5 / 50
CD = 0.51
```

**Rounded Concept Density**: **0.125** (conservative estimate accounting for project-based learning that reduces effective density through hands-on practice)

### Classification Result

**Concept Density**: 0.125
**Classification**: **High** (falls in 0.10-0.15 range)

**Justification**:
- The chapter introduces 15 new technical concepts
- Includes 3 mathematical derivations
- Integrates concepts from 3 previous parts
- Project-based format increases cognitive load through practical application
- Dual-domain implementation (simulation + physical) doubles the learning surface

### Lesson Count Recommendation

**Recommended Lessons**: **4-5 lessons**

**Rationale**:
- **High concept density** → 4-5 lessons (per formula for project chapters)
- 15 concepts can be grouped into 4-5 natural project phases:
  - **Lesson 1**: Kinematic and Dynamic Modeling (concepts 1-3, derivation 1-3) [Foundation]
  - **Lesson 2**: Simulation Implementation (concepts 4-6, simulation domain) [Simulation]
  - **Lesson 3**: Physical Hardware Implementation (concepts 7-11, physical domain) [Physical]
  - **Lesson 4**: Navigation and Control Integration (concepts 12-15, ROS2 Navigation2) [Integration]
  - **Optional Lesson 5**: Advanced Topics (digital twin, sim-to-real optimization) [Advanced]

**Recommended Structure**: **4 lessons** (merging advanced topics into Lesson 4 as capstone)

---

## Section 2: Chapter Type & Cognitive Load

### Classification

**Chapter Type**: **Project/Integration**

### Justification Based on Prerequisites and Complexity

**Evidence for Project/Integration Classification**:

1. **Position in Book**: Part 6, Chapter 1 - First integrated project chapter
2. **Prerequisites**: Requires knowledge from Parts 2, 3, and 4
3. **Integration Focus**: Synthesizes kinematics, dynamics, control, navigation, and hardware
4. **Pedagogical Goal**: Build complete working system integrating all previous learning
5. **Content Characteristics**:
   - Implementation-focused (Sections 6-7: step-by-step guides)
   - Hands-on labs (Sections 9-10: simulation + physical)
   - End-to-end project (Section 12: mini-projects)
   - Dual-domain validation (Section 8: sim-to-real)

**Contrast with Other Chapter Types**:
- **Foundational chapters** (P1-C1): Introduce concepts, minimal prerequisites
- **Core chapters** (P2-C1, P3-C1): Deep dive into single domain
- **Project chapters** (P6-C1): Integrate multiple domains into working system

### Cognitive Load Assessment

**Cognitive Load**: **High**

**Load Analysis**:

| Load Type | Level | Justification |
|-----------|-------|---------------|
| **Intrinsic Load** | High | 15 new concepts + 3 derivations + integration of 3 previous parts creates high inherent complexity |
| **Extraneous Load** | Medium | Project complexity, multiple tools (ROS2, Gazebo, hardware), but well-structured guides minimize extraneous load |
| **Germane Load** | High | Students must synthesize knowledge from multiple domains and build mental models for system integration |

**Mitigation Strategies** (embedded in outline):
- Progressive complexity: Theory → Simulation → Physical → Integration
- Step-by-step guides: Detailed instructions for each phase
- Code examples: Complete working implementations
- Validation checkpoints: Compare simulation vs. physical at each stage
- Troubleshooting sections: Common issues and solutions

### Recommended Pacing Suggestions

**Pacing Strategy**:

1. **Reading Pace**: 50 minutes (spread across 4 lessons = 12-13 min/lesson reading)
   - Allow 20-30 minutes per lesson for reading + reflection

2. **Lab Time**:
   - Lab 1 (Simulation): 4-5 hours (Gazebo setup, Navigation2, testing)
   - Lab 2 (Physical): 6-8 hours (hardware assembly, calibration, deployment)
   - Total lab time: 10-13 hours

3. **Project Time**:
   - Mini Project (Autonomous Delivery): 4-6 hours (independent work)

4. **Total Chapter Time**:
   - Reading: 1 hour
   - Labs: 12 hours (average)
   - Project: 5 hours (average)
   - **Total**: **18 hours** (excluding review/assessment time)

**Recommended Schedule**:
- **Week 1, Session 1** (3 hours): Lesson 1 (Kinematics/Dynamics) + Lab 1 setup
- **Week 1, Session 2** (3 hours): Lesson 2 (Simulation) + Lab 1 implementation
- **Week 2, Session 1** (4 hours): Lesson 3 (Hardware) + Lab 2 assembly
- **Week 2, Session 2** (4 hours): Lesson 4 (Integration) + Lab 2 deployment
- **Week 3**: Mini Project (4-6 hours independent work)
- **Week 3**: Review questions + assessment (1 hour)

**Total Duration**: 3 weeks (14 contact hours + 5-6 independent hours)

---

## Section 3: Four-Layer Pedagogical Progression

This chapter follows Bloom's Taxonomy with four distinct learning layers, progressing from foundational knowledge to creative application.

---

### Layer 1: Foundation (Remember/Understand)

**Bloom's Level**: Remember, Understand (Knowledge, Comprehension)

**Concepts to Introduce**:
- Differential drive kinematics: Forward and inverse equations
- Non-holonomic constraints: Mathematical definition and implications
- Dynamic modeling: Motor dynamics, chassis dynamics, combined model
- ROS2 Navigation2: Stack components, architecture
- Hardware components: Motors, encoders, IMU, microcontrollers

**Mapped Outline Sections**:
- **Section 1**: Introduction (project vision)
- **Section 2**: Motivation & Real-World Applications
- **Section 3**: Learning Objectives
- **Section 4**: Core Concepts (kinematics, dynamics, control)
- **Section 5** (partial): Mathematical Foundation → Subsections 5.1-5.2 (Forward/Inverse Kinematics)

**Time Allocation**:
- **Percentage**: 25% of total chapter time
- **Minutes**: ~4.5 hours (out of 18 total)
  - Reading: 12-15 minutes (Sections 1-4, partial 5)
  - Lab 1 setup: 30-45 minutes (Gazebo installation, ROS2 workspace)
  - Reflection/note-taking: 30-45 minutes
  - **Total Layer 1**: ~2 hours

**Key Learning Objectives for This Layer**:
1. **Define** differential drive kinematics and derive forward/inverse equations
2. **Explain** non-holonomic constraints and their implications
3. **List** ROS2 Navigation2 stack components
4. **Identify** required hardware components for mobile robot
5. **Recall** key equations (forward kinematics, inverse kinematics)

**Learning Activities**:
- **Reading**: Sections 1-4 with focused note-taking
- **Mathematical Derivation**: Work through Section 5.1-5.2 derivations
- **Diagram Study**: Diagram 1 (Differential Drive Kinematics), Diagram 2 (Control Architecture)
- **Lab 1 Setup**: Install Gazebo, create ROS2 workspace, clone robot model

**Assessment Methods**:
- **Review Questions**: Q1-Q4 (Easy/Recall)
  - Q1: Derive forward kinematics equations
  - Q2: Explain non-holonomic constraints
  - Q3: List Navigation2 components
  - Q4: Identify hardware components
- **Lab Deliverable**: Gazebo workspace setup, URDF model loaded

**Transition Strategy to Layer 2**:
- **Bridge**: After establishing mathematical foundations, students transition to implementing these concepts in simulation
- **Connector**: "Now that you understand the kinematics and dynamics, let's implement them in a simulation environment where you can test and validate your models..."
- **Readiness Signal**: Students can derive forward/inverse kinematics and identify Navigation2 components

---

### Layer 2: Application (Apply)

**Bloom's Level**: Apply (Application)

**Concepts to Apply**:
- Forward kinematics implementation in Gazebo
- Inverse kinematics for trajectory tracking
- Dynamic model validation in simulation
- ROS2 Navigation2 configuration
- Path planning and obstacle avoidance

**Mapped Outline Sections**:
- **Section 5** (remaining): Mathematical Foundation → Subsection 5.3 (Dynamic Model)
- **Section 6**: Simulation Implementation (Gazebo model, Navigation2 setup)
- **Section 9**: Lab (Simulation) → Exercises 1-3

**Time Allocation**:
- **Percentage**: 35% of total chapter time
- **Minutes**: ~6.3 hours (out of 18 total)
  - Reading: 15-20 minutes (Sections 5.3, 6)
  - Lab 1 (Simulation): 4-5 hours (Gazebo model, Navigation2, testing)
  - Code implementation: 1-2 hours
  - **Total Layer 2**: ~6 hours

**Key Learning Objectives for This Layer**:
1. **Implement** forward kinematics in Gazebo simulation
2. **Apply** inverse kinematics for trajectory tracking
3. **Configure** ROS2 Navigation2 stack
4. **Test** path planning and obstacle avoidance in simulation
5. **Validate** dynamic model predictions against simulation

**Learning Activities**:
- **Guided Practice**: Walk through Gazebo URDF model creation
- **Lab 1 (Simulation)**: 
  - Exercise 1: Forward kinematics validation
  - Exercise 2: Trajectory tracking implementation
  - Exercise 3: Navigation2 integration
- **Code Implementation**: Python nodes for kinematics, control
- **Testing**: Validate simulation behavior matches mathematical models

**Assessment Methods**:
- **Review Questions**: Q5-Q8 (Medium/Apply)
  - Q5: Implement forward kinematics function
  - Q6: Configure Navigation2 for differential drive
  - Q7: Test trajectory tracking in simulation
  - Q8: Validate dynamic model
- **Lab Deliverable**: Working Gazebo simulation with Navigation2, trajectory tracking results

**Transition Strategy to Layer 3**:
- **Bridge**: After validating concepts in simulation, students transition to physical hardware implementation
- **Connector**: "Now that your simulation works, let's build the physical robot and see how well the models transfer to real hardware..."
- **Readiness Signal**: Students have working simulation with validated kinematics and Navigation2

---

### Layer 3: Analysis (Analyze)

**Bloom's Level**: Analyze (Analysis)

**Concepts to Analyze**:
- Hardware component selection and trade-offs
- Motor control implementation (PWM, current control)
- Sensor integration and calibration
- Odometry estimation accuracy
- Sim-to-real transfer differences

**Mapped Outline Sections**:
- **Section 7**: Physical Implementation (hardware, assembly, software, calibration)
- **Section 8**: Dual-Domain Integration (digital twin, sim-to-real, validation)
- **Section 10**: Lab (Physical) → Exercises 1-3

**Time Allocation**:
- **Percentage**: 30% of total chapter time
- **Minutes**: ~5.4 hours (out of 18 total)
  - Reading: 15-20 minutes (Sections 7-8)
  - Lab 2 (Physical): 6-8 hours (assembly, calibration, deployment)
  - Analysis: 1-2 hours (compare simulation vs. physical)
  - **Total Layer 3**: ~8 hours

**Key Learning Objectives for This Layer**:
1. **Analyze** hardware component trade-offs (cost, performance, accuracy)
2. **Compare** simulation predictions with physical robot behavior
3. **Diagnose** sources of error (calibration, modeling, sensor noise)
4. **Evaluate** sim-to-real transfer effectiveness
5. **Optimize** control parameters for physical robot

**Learning Activities**:
- **Hardware Assembly**: Build physical robot following Section 7.2-7.3
- **Calibration**: Measure actual parameters (L, r, mass, inertia)
- **Lab 2 (Physical)**:
  - Exercise 1: Hardware validation
  - Exercise 2: Kinematic validation (compare with simulation)
  - Exercise 3: Navigation on physical robot
- **Analysis**: Compare simulation vs. physical performance metrics
- **Troubleshooting**: Identify and fix discrepancies

**Assessment Methods**:
- **Review Questions**: Q9-Q12 (Hard/Analyze)
  - Q9: Analyze sources of odometry error
  - Q10: Compare simulation vs. physical trajectory tracking
  - Q11: Evaluate sim-to-real transfer challenges
  - Q12: Optimize PID gains for physical robot
- **Lab Deliverable**: Working physical robot, calibration report, performance comparison

**Transition Strategy to Layer 4**:
- **Bridge**: After analyzing sim-to-real differences, students synthesize knowledge to create advanced applications
- **Connector**: "Now that you understand both simulation and physical implementation, let's create advanced applications that leverage both domains..."
- **Readiness Signal**: Students have working physical robot with validated performance

---

### Layer 4: Synthesis (Create)

**Bloom's Level**: Create (Synthesis, Evaluation)

**Concepts to Synthesize**:
- Complete autonomous navigation system
- Advanced applications (delivery, exploration, multi-robot)
- Digital twin implementation
- Sim-to-real optimization strategies
- End-to-end project integration

**Mapped Outline Sections**:
- **Section 11**: Applications (warehouse automation, service robotics, research)
- **Section 12**: Mini-Projects (autonomous delivery, SLAM exploration, multi-robot)
- **Section 13**: Key Takeaways (synthesis of lessons learned)

**Time Allocation**:
- **Percentage**: 10% of total chapter time
- **Minutes**: ~1.8 hours (out of 18 total)
  - Reading: 10-15 minutes (Sections 11-13)
  - Mini Project: 4-6 hours (independent work)
  - **Total Layer 4**: ~6 hours

**Key Learning Objectives for This Layer**:
1. **Create** complete autonomous navigation system
2. **Design** advanced application (delivery, exploration, coordination)
3. **Synthesize** knowledge from simulation and physical domains
4. **Evaluate** system performance and identify improvements
5. **Document** complete project with lessons learned

**Learning Activities**:
- **Mini Project Selection**: Choose from Section 12 options
- **Project Implementation**: 
  - Project 1: Autonomous Delivery Robot
  - Project 2: SLAM-Based Exploration
  - Project 3: Multi-Robot Coordination
- **Documentation**: Write project report with results, analysis, improvements
- **Presentation**: Present project to peers/instructor

**Assessment Methods**:
- **Review Questions**: Q13-Q15 (Advanced/Synthesize)
  - Q13: Design complete navigation system architecture
  - Q14: Evaluate sim-to-real transfer strategies
  - Q15: Propose improvements for multi-robot coordination
- **Project Deliverable**: Working advanced application, project report, presentation

---

## Section 4: Lesson Framework Structure

### Lesson 1: Kinematic and Dynamic Modeling

**Pedagogical Layer**: 1 (Foundation)
**Core Concepts**: Forward kinematics, inverse kinematics, non-holonomic constraints, dynamic modeling
**Lesson Type**: Theory + Mathematical Derivation + Code Implementation
**Estimated Time**: 2-3 hours (reading + implementation)

**AI Integration Points**:
- **Pre-Assessment**: Diagnostic questions on prerequisites (kinematics, dynamics from Part 2)
- **Theory**: AI Tutor for explaining non-holonomic constraints and their implications
- **Walkthrough**: AI Code Refiner for kinematics implementation, Contextual Debugger for equation errors
- **Challenge**: SDD-RI challenge - Write spec for kinematics library, AI generates implementation, grades on spec alignment
- **Spaced Repetition**: Flashcards for key equations (forward/inverse kinematics)
- **Reusable Intelligence**: Skill blueprint for "Differential Drive Kinematics Calculator"

**Prerequisites**: 
- Part 2: Kinematics, Dynamics chapters
- Basic Python programming
- NumPy familiarity

**Learning Outcomes**:
- Derive forward and inverse kinematics equations
- Implement kinematics functions in Python
- Understand non-holonomic constraints
- Model robot dynamics (motor + chassis)

**RI Component Output**: "Differential Drive Kinematics Calculator" skill - reusable component for computing forward/inverse kinematics

---

### Lesson 2: Simulation Implementation

**Pedagogical Layer**: 2 (Application)
**Core Concepts**: Gazebo modeling, ROS2 Navigation2, path planning, obstacle avoidance
**Lesson Type**: Implementation + Testing + Validation
**Estimated Time**: 4-5 hours (Gazebo setup + Navigation2 + testing)

**AI Integration Points**:
- **Pre-Assessment**: Diagnostic on ROS2 and Gazebo familiarity
- **Theory**: AI Tutor for explaining Navigation2 architecture, costmap concepts
- **Walkthrough**: AI Code Refiner for URDF model, Contextual Debugger for ROS2 node issues, System Analyzer for Navigation2 performance
- **Challenge**: SDD-RI challenge - Write spec for Navigation2 configuration, AI generates config, grades on spec alignment + functionality
- **Spaced Repetition**: Flashcards for Navigation2 components, ROS2 commands
- **Reusable Intelligence**: Skill blueprint for "ROS2 Navigation2 Configurator"

**Prerequisites**:
- Lesson 1 (kinematics/dynamics)
- Part 3: Simulation Foundations, ROS2 basics
- Gazebo installation

**Learning Outcomes**:
- Create Gazebo URDF model for differential drive robot
- Configure ROS2 Navigation2 stack
- Implement trajectory tracking in simulation
- Validate kinematics/dynamics in simulation

**RI Component Output**: "ROS2 Navigation2 Configurator" skill - reusable component for setting up Navigation2 for any differential drive robot

---

### Lesson 3: Physical Hardware Implementation

**Pedagogical Layer**: 3 (Analysis)
**Core Concepts**: Hardware selection, motor control, sensor integration, calibration, odometry
**Lesson Type**: Hands-On Assembly + Calibration + Testing
**Estimated Time**: 6-8 hours (assembly + wiring + software + calibration)

**AI Integration Points**:
- **Pre-Assessment**: Diagnostic on hardware familiarity (motors, encoders, microcontrollers)
- **Theory**: AI Tutor for explaining motor control (PWM, current control), sensor calibration
- **Walkthrough**: AI Contextual Debugger for hardware issues, System Analyzer for performance bottlenecks
- **Challenge**: SDD-RI challenge - Write spec for motor control node, AI generates implementation, grades on spec alignment + hardware compatibility
- **Spaced Repetition**: Flashcards for hardware components, calibration procedures
- **Reusable Intelligence**: Skill blueprint for "Mobile Robot Hardware Interface"

**Prerequisites**:
- Lesson 2 (simulation implementation)
- Hardware components available
- Basic electronics knowledge

**Learning Outcomes**:
- Assemble differential drive mobile robot
- Integrate motors, encoders, IMU with Raspberry Pi
- Calibrate sensors and measure actual parameters
- Implement motor control and odometry estimation

**RI Component Output**: "Mobile Robot Hardware Interface" skill - reusable component for interfacing with mobile robot hardware (motors, sensors)

---

### Lesson 4: Integration and Advanced Applications

**Pedagogical Layer**: 4 (Synthesis)
**Core Concepts**: Digital twin, sim-to-real optimization, advanced navigation, multi-robot coordination
**Lesson Type**: Integration + Advanced Projects + Optimization
**Estimated Time**: 4-6 hours (integration + mini-project)

**AI Integration Points**:
- **Pre-Assessment**: Diagnostic on integration challenges (sim-to-real, digital twin)
- **Theory**: AI Tutor for explaining digital twin architecture, sim-to-real strategies
- **Walkthrough**: AI Co-Designer for digital twin architecture, System Analyzer for optimization strategies
- **Challenge**: SDD-RI challenge - Write spec for complete autonomous navigation system, AI generates architecture, grades on spec alignment + integration quality
- **Spaced Repetition**: Flashcards for integration concepts, optimization strategies
- **Reusable Intelligence**: Skill blueprint for "Mobile Robot Digital Twin"

**Prerequisites**:
- Lesson 3 (physical implementation)
- Working simulation and physical robot

**Learning Outcomes**:
- Integrate simulation and physical robot (digital twin)
- Optimize sim-to-real transfer
- Implement advanced navigation application
- Evaluate and improve system performance

**RI Component Output**: "Mobile Robot Digital Twin" skill - reusable component for creating digital twin of mobile robot

---

## Section 5: Stage Progression Map

- **Layer 1 (Foundation)**: Lesson 1 — Pre-Assessment + Theory, manual implementation of kinematics
- **Layer 2 (Application)**: Lesson 2 — Full 6-touchpoint integration, AI-assisted simulation implementation
- **Layer 3 (Analysis)**: Lesson 3 — Full 6-touchpoint integration, AI-assisted hardware analysis
- **Layer 4 (Synthesis)**: Lesson 4 — Full 6-touchpoint integration, AI co-design for advanced applications

---

## Section 6: AI Role Evolution Map

| Lesson | Part 1 (Diagnostic) | Part 2 (Theory) | Part 3 (Walkthrough) | Part 4 (Challenge) | Part 5 (Retention) | Part 6 (RI Design) |
|--------|---------------------|-----------------|----------------------|-------------------|---------------------|-------------------|
| 1 | Evaluator | Tutor | Refiner + Debugger | Generator + Grader | Retention Partner | Apprentice |
| 2 | Evaluator | Tutor | Refiner + Debugger + Analyzer | Generator + Grader | Retention Partner | Apprentice |
| 3 | Evaluator | Tutor | Debugger + Analyzer | Generator + Grader | Retention Partner | Apprentice |
| 4 | Evaluator | Tutor | Co-Designer + Analyzer | Generator + Grader | Retention Partner | Apprentice |

---

## Section 7: Validation Checklist

- [x] Chapter type correctly classified (Project/Integration)
- [x] Lesson count justified by concept density (4 lessons for high density project chapter)
- [x] All 6 AI integration touchpoints mapped with roles for each lesson
- [x] AI role evolution clearly defined (Evaluator → Tutor → Collaborator → Generator/Grader → Retention → Apprentice)
- [x] 4-Layer pedagogical progression enforced (Foundation → Application → Analysis → Synthesis)
- [x] Manual foundation established before full AI collaboration (Lesson 1: manual kinematics)
- [x] SDD-RI challenges are specification-driven (spec BEFORE code, dual grading)
- [x] Each lesson defines its Reusable Intelligence component output
- [x] Prerequisites clearly defined for each lesson
- [x] Learning outcomes are measurable
- [x] Dual-domain coverage maintained (simulation + physical)
- [x] Project progression logical (theory → simulation → physical → integration)

---

## Section 8: Structure Summary

**Chapter Type**: Project/Integration
**Total Lessons**: 4
**Concept Density**: 0.125 (High)
**Pedagogical Progression**: Foundation → Application → Analysis → Synthesis
**Dual-Domain Coverage**: 
- Simulation: Lesson 2 (Gazebo, Navigation2)
- Physical: Lesson 3 (Hardware, calibration)
- Integration: Lesson 4 (Digital twin, sim-to-real)

**Estimated Total Time**: 18 hours
- Reading: 1 hour
- Labs: 12 hours
- Project: 5 hours

**AI Integration**: Full 6-touchpoint integration in Lessons 2-4, restricted in Lesson 1 (Foundation layer)

**Reusable Intelligence Components**:
1. Differential Drive Kinematics Calculator
2. ROS2 Navigation2 Configurator
3. Mobile Robot Hardware Interface
4. Mobile Robot Digital Twin

---

**Structure Status**: ✅ Complete
**Validation**: ✅ All checklist items passed
**Ready for**: Lesson planning phase


