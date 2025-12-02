# Quickstart: Generating a Book Chapter

**Purpose**: Step-by-step guide for generating a chapter using the 6-agent pipeline.

**Prerequisites**:
- Claude Code environment configured
- Access to all 6 agents: research-agent, outliner-agent, chapter-structure-architect, lesson-planner, writer-agent, book-editor
- Constitution at `.specify/memory/constitution.md`
- Specification at `specs/1-robotics-book-spec/spec.md`
- Research decisions at `specs/1-robotics-book-spec/research.md`

---

## Workflow: Complete Chapter Generation

### Step 1: Identify Chapter

Choose a chapter from the specification scope (see spec.md Section: Scope).

**Example**: Part 2, Chapter 5 — Kinematics

```bash
# Set variables
CHAPTER_ID="P2-C5"
CHAPTER_TOPIC="Kinematics"
PART_NUMBER=2
CHAPTER_NUMBER=5
```

---

### Step 2: Research Phase (Agent 1)

**Agent**: `research-agent`
**Skill**: `research-methodology`
**Output**: `.book-generation/research/kinematics/v001/research.md`

**Invocation**:
```yaml
Task: Launch research-agent
Parameters:
  topic: "Robot Kinematics - Forward and Inverse Kinematics for Manipulators"
  source_requirements:
    tier1_minimum: 10
    tier2_maximum: 5
    exclude_wikipedia: true
  domains:
    physical_robotics: true
    simulation: true
    ai_ml: false
```

**Expected Duration**: 3-4 hours (per research-methodology)

**Validation Checks**:
- ✅ Minimum 10 Tier 1 citations (peer-reviewed journals, conferences, textbooks)
- ✅ No Wikipedia or excluded sources
- ✅ All URLs verified accessible (HTTP 200)
- ✅ Citations in IEEE format

**Output Structure**:
```markdown
# Research: Kinematics

## Key Findings
- Forward kinematics: Joint angles → End-effector position
- Inverse kinematics: End-effector position → Joint angles
- Denavit-Hartenberg (DH) parameters: Standard representation
- Jacobian: Velocity mapping, singularity analysis

## Sources (15 total)
### Tier 1 (12)
[1] J.J. Craig, "Introduction to Robotics: Mechanics and Control," Pearson, 2005.
[2] B. Siciliano et al., "Robotics: Modelling, Planning and Control," Springer, 2009.
...

### Tier 2 (3)
[13] "Isaac Sim Kinematics Tutorial," NVIDIA, 2024. [Online]. ...
...

## Recommended Outline
- Introduction to kinematics
- Forward kinematics (DH parameters, transformation matrices)
- Inverse kinematics (analytical, numerical methods)
- Jacobian (velocity, singularities)
- Physical robot examples (industrial arms)
- Simulation examples (Isaac Sim, MuJoCo)
```

---

### Step 3: Outline Phase (Agent 2)

**Agent**: `outliner-agent`
**Input**: Research output from Step 2
**Output**: `.book-generation/outlines/kinematics/v001/outline.md`

**Invocation**:
```yaml
Task: Launch outliner-agent
Parameters:
  research_file: ".book-generation/research/kinematics/v001/research.md"
  chapter_specification:
    chapter_id: "P2-C5"
    title: "Kinematics"
    part_number: 2
    chapter_number: 5
```

**Validation Checks**:
- ✅ All 14 mandatory sections present
- ✅ Physical coverage ≥ 0.7 (70% keyword match)
- ✅ Simulation coverage ≥ 0.7
- ✅ Dual-domain integrated: true

**Output Structure**:
```markdown
# Chapter Outline: P2-C5 Kinematics

## 1. Introduction
- Definition of kinematics (motion without forces)
- Importance in robotics (path planning, control)

## 2. Motivation & Real-World Relevance
- Industrial robot arm picking and placing objects
- Surgical robot precision positioning
- Humanoid robot reaching and grasping

## 3. Learning Objectives
- Derive forward kinematics using DH parameters
- Solve inverse kinematics for 2-DOF and 3-DOF arms
- Compute Jacobian for velocity mapping
- Identify singularities in robot workspace

## 4. Key Terms (12 terms)
- Forward Kinematics, Inverse Kinematics, Denavit-Hartenberg (DH) Parameters, ...

## 5. Physical Explanation
- Real robot joints (revolute, prismatic)
- DH parameter measurement on physical arm
- Hardware limitations (joint limits, backlash, sensor accuracy)

## 6. Simulation Explanation
- Isaac Sim articulation API for kinematics
- MuJoCo forward kinematics functions
- Simulation accuracy vs real-world differences

## 7. Integrated Understanding
- Compare analytical IK (physical) vs numerical IK (simulation)
- Sim-to-real transfer: Joint angle errors, calibration needs
- When to use simulation (testing), when to use physical (validation)

## 8-14. [Remaining sections with similar detail]

## Coverage Validation
- Physical coverage: 0.85 (85% of content includes physical robotics keywords)
- Simulation coverage: 0.82 (82% includes simulation keywords)
- Dual-domain integrated: true
```

---

### Step 4: Structure Phase (Agent 3)

**Agent**: `chapter-structure-architect`
**Input**: Outline from Step 3
**Output**: `.book-generation/structures/P2-C5/v001/structure.md`

**Invocation**:
```yaml
Task: Launch chapter-structure-architect
Parameters:
  outline_file: ".book-generation/outlines/kinematics/v001/outline.md"
  chapter_id: "P2-C5"
```

**Validation Checks**:
- ✅ Concept density calculated correctly
- ✅ All 4 pedagogical layers defined
- ✅ All 5 AI integration touchpoints defined
- ✅ Lesson count justified by density

**Output Structure**:
```markdown
# Chapter Structure: P2-C5 Kinematics

## Concept Density Analysis
- New Concepts: 12 (forward kinematics, inverse kinematics, DH parameters, Jacobian, ...)
- Prerequisites: 8 (vectors, matrices, trigonometry, coordinate frames, ...)
- Mathematical Derivations: 4 (FK derivation, Jacobian derivation, IK solution, workspace analysis)
- Target Reading Time: 180 minutes (3 hours)

**Density Score**: (12 + 0.5 × 8 + 2 × 4) / 180 = 0.133

**Classification**: High Density (0.133 > 0.10)

**Recommendation**: Split into 3 lessons OR divide into 2 chapters (Kinematics I: Forward, Kinematics II: Inverse)

## Pedagogical Progression (4 Layers)

### Layer 1: Introduction/Motivation (Activate Prior Knowledge)
- Review coordinate frames and transformations from prior chapters
- Real-world examples: Industrial robot arm, surgical robot
- Why kinematics matters: Enables motion planning and control

### Layer 2: Theory/Concepts (Conceptual Understanding)
- Physical Explanation: Real robot joints, DH parameters
- Simulation Explanation: Isaac Sim articulation, MuJoCo FK functions
- Mathematical Foundation: Transformation matrices, homogeneous coordinates
- Intuition first: "FK is like following a chain of joints to find hand position"
- Then formalism: T = A1 × A2 × ... × An

### Layer 3: Hands-On Practice (Application and Analysis)
- Simulation Lab: Implement FK for 3-DOF arm in Isaac Sim
- Physical Lab: Measure DH parameters on educational robot arm, validate FK
- Guided examples: 2-DOF arm (simple), 3-DOF arm (intermediate)

### Layer 4: Projects/Integration (Synthesis and Evaluation)
- Mini-Project: Build simple 2-DOF arm, derive and validate FK equations
- Extension: Implement IK for pick-and-place task
- Evaluation: Compare simulated vs physical end-effector positions

## AI Integration Touchpoints (5)

### 1. Pre-Assessment
**Questions**:
- "Do you know what a coordinate frame is?"
- "Can you multiply matrices?"
- "Have you worked with trigonometric functions?"
**Action**: If gaps → recommend review of linear algebra appendix

### 2. AI Tutor (during Layer 2)
**Trigger**: User clicks "Explain DH parameters differently"
**AI Response**: Alternative explanations (visual analogies, step-by-step, worked examples)

### 3. Contextual Help (during Layer 3)
**Trigger**: User stuck in lab (e.g., transformation matrix incorrect)
**AI Response**: Debugging assistance, common error checks, hints

### 4. AI-Graded Challenge (after Layer 4)
**Task**: Submit mini-project code and results
**AI Evaluation**: Checks FK implementation, compares output to expected values, provides feedback

### 5. Spaced Repetition
**Schedule**: Review kinematics concepts after 1 day, 1 week, 1 month
**AI Prompts**: "Quick quiz: What are DH parameters? Why do we use them?"

## Lesson Count
**Decision**: 3 lessons (High Density justifies splitting)
- Lesson 1: Forward Kinematics
- Lesson 2: Inverse Kinematics
- Lesson 3: Jacobian and Singularities
```

---

### Step 5: Lesson Planning Phase (Agent 4)

**Agent**: `lesson-planner`
**Input**: Structure from Step 4
**Output**: `.book-generation/lessons/P2-C5/v001/lesson.md`

**Invocation**:
```yaml
Task: Launch lesson-planner
Parameters:
  structure_file: ".book-generation/structures/P2-C5/v001/structure.md"
  lesson_number: 1  # If splitting into 3 lessons, run agent 3 times with lesson_number 1, 2, 3
```

**Validation Checks**:
- ✅ All 6 parts present (Hook, Theory, Walkthrough, Challenge, Takeaways, Learn with AI)
- ✅ Theory includes both physical and simulation content
- ✅ Diagrams required: minimum 4 (Architecture, Flow, Mechanical, SimulationPipeline)
- ✅ Labs required: exactly 2 (Simulation, Physical)

**Output Structure** (Lesson 1: Forward Kinematics):
```markdown
# Lesson 1: Forward Kinematics

## Part 1: Hook (Engagement)
Imagine you're programming a robot arm to pick up a coffee mug. You know the arm has 3 joints, each at specific angles. But where is the robot's hand in space? Can it reach the mug?

This is the **forward kinematics problem**: Given joint angles, find the end-effector position.

Today, you'll learn how robots answer this question—both in the real world and in simulation.

## Part 2: Theory (Conceptual Content)

### Physical Explanation (800-1500 words)
- Real robot joints: Revolute (rotational), Prismatic (linear)
- Denavit-Hartenberg (DH) parameters: a, α, d, θ
- Measuring DH parameters on physical robot arm (diagrams + photos)
- Transformation matrices: How joint rotations compose

### Simulation Explanation (800-1500 words)
- Isaac Sim: Articulation API, joint properties, forward kinematics functions
- MuJoCo: mj_forward(), accessing body positions
- Digital twin accuracy: Simulation matches physical arm when DH parameters calibrated

### Diagrams (minimum 4)
1. **Architecture Diagram**: Robot arm with coordinate frames at each joint
2. **Flow Diagram**: FK algorithm (joint angles → transformations → end-effector position)
3. **Mechanical Diagram**: Physical robot arm with DH parameter annotations
4. **Simulation Pipeline**: Isaac Sim environment setup → joint control → position query

## Part 3: Walkthrough (Guided Practice)
**Example 1 (Physical)**: 2-DOF planar arm
- Given: θ1 = 30°, θ2 = 45°, link lengths L1 = 0.3m, L2 = 0.25m
- Calculate: End-effector position (x, y)
- Step-by-step derivation with diagrams

**Example 2 (Simulation)**: 3-DOF arm in Isaac Sim
- Setup: Load robotic arm URDF
- Code: Set joint angles, query end-effector pose
- Expected output: Position [x, y, z], orientation (quaternion)

## Part 4: Challenge (Independent Work)
**Simulation Lab**:
- Implement FK for 3-DOF arm in Isaac Sim
- Test with 5 different joint angle configurations
- Compare your calculations to Isaac Sim's built-in FK

**Physical Lab** (if hardware available):
- Measure DH parameters on educational robot arm
- Set joint angles manually, measure end-effector position
- Validate your FK equations against real measurements

## Part 5: Takeaways (Summary)
**Key Points** (10-15):
1. Forward kinematics maps joint angles → end-effector position
2. DH parameters (a, α, d, θ) standardize kinematic representation
3. Transformation matrices compose to give final pose
4. Simulation FK is exact (no sensor noise), physical FK has measurement errors
5. Calibration bridges sim-to-real gap
... (10-15 total)

**Common Mistakes**:
- Forgetting to convert degrees to radians
- Multiplying transformation matrices in wrong order
- Not accounting for base frame offset

**Practical Tips**:
- Always verify FK with simple cases (all joints at 0°)
- Use simulation to test FK before deploying to physical robot
- Calibrate DH parameters from real robot measurements

## Part 6: Learn with AI (Deeper Exploration)
**AI Prompts for Further Learning**:
1. "Explain the intuition behind homogeneous transformation matrices."
2. "Show me a step-by-step derivation of FK for a 4-DOF SCARA robot."
3. "How do singularities affect forward kinematics?"
4. "Compare DH parameters to other kinematic representations (e.g., Exponential Coordinates)."
5. "Generate a Python script to visualize FK for any N-DOF arm."

**AI-Guided Project**:
"Ask AI to help you design a custom robot arm (choose DOFs, link lengths) and derive its forward kinematics from scratch."
```

---

### Step 6: Prose Generation Phase (Agent 5)

**Agent**: `writer-agent`
**Skill**: `prose-generation`
**Input**: Lesson from Step 5
**Output**: `.book-generation/drafts/P2-C5/v001/draft.md`

**Invocation**:
```yaml
Task: Launch writer-agent
Parameters:
  lesson_file: ".book-generation/lessons/P2-C5/v001/lesson.md"
  skill: prose-generation
```

**Validation Checks**:
- ✅ Word count within 2000-4000 range
- ✅ Flesch-Kincaid Grade 12-14 (university level)
- ✅ Dual-domain balance ratio 0.8-1.2 (balanced physical/simulation coverage)
- ✅ All lesson sections transformed into polished prose

**Output Structure**:
```markdown
# Chapter 5: Kinematics

## Introduction

When you watch a robot arm pick up an object, you're witnessing the elegant solution to a fundamental problem: how does the robot know where its hand is in space?

The answer lies in **kinematics**—the mathematical framework that describes robot motion. In this chapter, you'll learn how robots translate joint angles into positions, both in physical systems and in simulation.

Whether you're programming an industrial robot to assemble cars or training a humanoid to grasp objects in a virtual environment, kinematics is your essential tool.

## Motivation & Real-World Relevance

**Industrial Manufacturing**: A robotic arm welds car frames with millimeter precision. The controller sets three joint angles—90°, 45°, 60°—and the arm's welding torch positions itself exactly at the weld point. How does the controller know this will work? Forward kinematics.

**Surgical Robotics**: A surgeon controls a robotic instrument during minimally invasive surgery. The robot's tip must reach a tumor with sub-millimeter accuracy. Every motion is calculated using kinematics to ensure precision.

**Humanoid Simulation**: Before deploying a humanoid robot to pick up objects in a warehouse, engineers train it in Isaac Sim. They simulate thousands of pick-and-place motions, each one calculated using forward kinematics to ensure the robot's hand reaches the target.

These scenarios share a common challenge: Given joint positions, where is the robot's end-effector (hand, tool, or sensor)?

This chapter teaches you how to answer that question.

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Define** forward kinematics and explain its role in robot control
2. **Derive** forward kinematics equations using Denavit-Hartenberg (DH) parameters
3. **Implement** forward kinematics in both simulation (Isaac Sim, MuJoCo) and physical systems
4. **Analyze** the differences between simulated and real-world kinematics
5. **Apply** transformation matrices to compute end-effector position and orientation
6. **Validate** kinematic models by comparing calculated positions to measured positions

## Key Terms

- **Forward Kinematics (FK)**: The process of computing the end-effector position and orientation given joint angles.
- **Denavit-Hartenberg (DH) Parameters**: A standardized set of four parameters (a, α, d, θ) that describe the geometry of each robot link and joint.
- **Transformation Matrix**: A 4×4 matrix representing both rotation and translation between coordinate frames.
... (12-20 terms total with definitions)

## Physical Explanation

### Real Robot Joints

Robot arms are composed of **links** (rigid segments) connected by **joints** (movable connections). Two types of joints dominate robotics:

1. **Revolute Joints**: Rotate around an axis (like your elbow or shoulder)
2. **Prismatic Joints**: Slide along an axis (like a telescope extending)

[Diagram: Physical robot arm with labeled joints and links]

When you set a revolute joint to 45°, the link attached to that joint rotates 45° around its axis. The question is: how does this rotation affect the position of the robot's hand?

### Denavit-Hartenberg Parameters

In 1955, Jacques Denavit and Richard Hartenberg invented a clever system to describe any robot's geometry using just **four parameters per joint**:

- **a (link length)**: Distance between consecutive joint axes along the common normal
- **α (link twist)**: Angle between consecutive joint axes around the common normal
- **d (link offset)**: Distance along the previous joint axis to the common normal
- **θ (joint angle)**: Angle of rotation around the joint axis (variable for revolute joints)

[Diagram: DH parameter visualization on physical arm]

... (Continue with detailed physical robotics explanation, 800-1500 words)

## Simulation Explanation

### Forward Kinematics in Isaac Sim

NVIDIA Isaac Sim provides powerful tools for kinematic simulation. The **Articulation API** lets you control robot joints and query their positions.

Here's how forward kinematics works in Isaac Sim:

```python
# Load robot arm (URDF or USD)
from omni.isaac.core import SimulationContext
from omni.isaac.core.articulations import Articulation

# Initialize simulation
simulation_context = SimulationContext()
robot = Articulation("/World/Robot")

# Set joint positions (radians)
robot.set_joint_positions([0.5, -0.3, 0.8])  # 3 joints

# Query end-effector position (forward kinematics calculated internally)
ee_position, ee_orientation = robot.get_world_pose()
print(f"End-effector position: {ee_position}")
```

Isaac Sim's physics engine uses the same DH parameters and transformation matrices you derive by hand, but computes them at 60+ frames per second.

[Diagram: Simulation pipeline showing joint angles → FK algorithm → end-effector pose]

... (Continue with detailed simulation explanation, 800-1500 words)

## Integrated Understanding

### Comparing Physical and Simulated Kinematics

**Question**: If both use the same math (DH parameters, transformation matrices), why do physical and simulated results differ?

**Answer**: Real-world imperfections.

| Factor | Simulation | Physical Reality |
|--------|-----------|------------------|
| **Joint Accuracy** | Perfect (θ = 45.0000°) | ±0.5° error (encoder resolution, backlash) |
| **Link Lengths** | Exact (a = 0.300m) | ±1mm manufacturing tolerance |
| **Rigidity** | Perfectly rigid links | Links flex under load |
| **Sensor Noise** | None (exact positions) | ±0.1mm IMU/encoder noise |

These differences create a **domain gap** between simulation and reality.

### Sim-to-Real Transfer for Kinematics

**Strategy 1: Calibration**
- Measure real robot's DH parameters (not nominal values from CAD)
- Update simulation model with calibrated parameters
- Reduces domain gap from ±5mm to ±1mm

**Strategy 2: Sensor Fusion**
- Don't trust FK alone—combine with vision or force feedback
- Example: FK predicts hand at (0.5, 0.3, 0.2), camera confirms object grasped

**Strategy 3: Robust Control**
- Design controllers that tolerate small FK errors
- Use closed-loop feedback instead of open-loop FK commands

... (Continue with integrated understanding, 500-800 words)

[Remaining sections: Diagrams, Examples, Labs, Projects, Summary, Review Questions]

---

**End of Draft** (Total: ~3500 words, balanced physical/simulation, FK Grade 13.2)
```

---

### Step 7: Editorial Review Phase (Agent 6)

**Agent**: `book-editor`
**Skill**: `content-editing`
**Input**: Draft from Step 6
**Output**: `.book-generation/reviews/P2-C5/v001/review.md`

**Invocation**:
```yaml
Task: Launch book-editor
Parameters:
  draft_file: ".book-generation/drafts/P2-C5/v001/draft.md"
  skill: content-editing
  review_passes: [Structural, Content, Citation, Consistency, Accuracy]  # All 5 passes
```

**Validation Checks**:
- ✅ Zero constitutional violations
- ✅ All 5 passes completed
- ✅ Approval status = Approved (or MinorRevisions/MajorRevisions if issues found)
- ✅ All critical issues resolved

**Output Structure**:
```markdown
# Editorial Review: P2-C5 Kinematics (v001)

**Reviewer**: book-editor
**Draft Version**: drafts/P2-C5/v001/draft.md
**Review Date**: 2025-11-30
**Review Passes**: 5 (Structural, Content, Citation, Consistency, Accuracy)

---

## Pass 1: Structural Analysis ✅ PASSED

**Checks**:
- ✅ All 14 mandatory sections present
- ✅ Section order correct (Introduction → ... → Review Questions)
- ✅ Logical flow within sections
- ✅ No orphaned headings or incomplete sections

**Issues**: None

---

## Pass 2: Content Quality ✅ PASSED

**Checks**:
- ✅ Clarity: Beginner-friendly language, no unexplained jargon
- ✅ Dual-Domain Integration: Physical (40% of content) + Simulation (42%) = Balanced
- ✅ Accuracy: Technical claims verified against sources
- ✅ Safety: Physical lab includes safety warnings (LiPo battery, rotating servos)

**Issues Found**: 2 minor

### Issue 2.1 (Minor): Equation Explanation
**Location**: Section "Physical Explanation", line 85
**Problem**: Transformation matrix equation introduced without sufficient intuition
**Suggested Fix**: Add 1-2 sentences explaining what each element represents before showing matrix

### Issue 2.2 (Minor): Simulation Code Comment
**Location**: Section "Simulation Explanation", code block line 3
**Problem**: Code comment assumes user knows what USD format is
**Suggested Fix**: Add brief explanation: "USD (Universal Scene Description) is the 3D format used by Isaac Sim"

---

## Pass 3: Citation Verification ✅ PASSED

**Checks**:
- ✅ 12 Tier 1 citations, 3 Tier 2 citations (meets minimum 10 Tier 1)
- ✅ All citations in IEEE format
- ✅ No Wikipedia or excluded sources
- ✅ All 15 URLs verified accessible (HTTP 200)

**Issues**: None

**Citations Summary**:
- [1] Craig, "Introduction to Robotics" ✓
- [2] Siciliano et al., "Robotics: Modelling, Planning and Control" ✓
- [3-12] Peer-reviewed papers from ICRA, IROS, IEEE Trans Robotics ✓
- [13-15] Isaac Sim docs, MuJoCo docs, ROS2 docs ✓

---

## Pass 4: Consistency Checking ✅ PASSED

**Checks**:
- ✅ Terminology consistent with glossary ("forward kinematics" not "FK" on first use)
- ✅ Tone: Expert yet friendly, simple, motivational (matches Article 6)
- ✅ Diagrams: Style compliant (color palette, notation, labels present)
- ✅ Math notation: Standard symbols (θ for joint angle, T for transformation)

**Issues**: None

---

## Pass 5: Factual Accuracy ✅ PASSED

**Checks**:
- ✅ Physics equations correct (DH transformation matrices verified)
- ✅ Robotics principles accurate (FK algorithm matches literature)
- ✅ AI/ML claims (N/A for this chapter)
- ✅ No pseudoscience or unverified techniques

**Issues**: None

**Verification**:
- DH parameter formulas match Craig textbook ✓
- Transformation matrix multiplication order correct ✓
- Isaac Sim API usage accurate (verified against official docs) ✓

---

## Constitutional Compliance ✅ NO VIOLATIONS

**Articles Checked**: All 20

**Violations**: None

**Highlights**:
- Article 2 (Scope): Dual-domain integration present (Physical 40%, Simulation 42%, Integrated 18%)
- Article 7 (Chapter Format): All 14 sections present and complete
- Article 13 (Safety): Physical lab includes battery and servo safety warnings
- Article 19 (Academic Integrity): 12 Tier 1 citations, all sources verified

---

## Approval Status: ✅ APPROVED (Minor Revisions Recommended)

**Decision**: Chapter is publication-ready with two minor improvements suggested (Issues 2.1, 2.2).

**Recommended Action**:
- **Option 1**: Publish as-is (minor issues do not affect quality)
- **Option 2**: Address Issues 2.1 and 2.2 in next revision (estimated time: 30 minutes)

**Next Steps**:
1. If revisions needed: writer-agent re-runs with feedback on Issues 2.1, 2.2
2. If approved as-is: Move draft to final manuscript directory
3. Generate diagrams using diagram-generator agent (if not already created)
4. Run final safety audit on physical lab instructions

---

**End of Review**
```

---

### Step 8: Finalization

**If Approved** (approval_status = Approved):
```bash
# 1. Copy draft to final manuscript
cp .book-generation/drafts/P2-C5/v001/draft.md manuscript/part2/chapter5-kinematics.md

# 2. Update chapter status
# (In chapter metadata JSON/YAML)
status: Published
version: 1
published_date: 2025-11-30

# 3. Generate diagrams (if not already done)
Task: Launch diagram-generator
Parameters:
  diagram_specs: [from lesson.md diagrams_required]
  output_directory: manuscript/diagrams/P2-C5/

# 4. Final safety audit (if physical labs present)
Task: Human safety auditor reviews physical lab
Checklist: LiPo warnings, emergency stop, pinch hazards

# 5. Add to book index and glossary
# (Update index with new terms from KeyTerms section)
```

**If Revisions Needed** (approval_status = MinorRevisions or MajorRevisions):
```bash
# 1. Re-run writer-agent with editorial feedback
Task: Launch writer-agent
Parameters:
  lesson_file: ".book-generation/lessons/P2-C5/v001/lesson.md"
  editorial_feedback: ".book-generation/reviews/P2-C5/v001/review.md"
  issues_to_address: [2.1, 2.2]  # Focus on specific issues

# 2. Output new version
# → .book-generation/drafts/P2-C5/v002/draft.md

# 3. Re-run book-editor for validation
Task: Launch book-editor
Parameters:
  draft_file: ".book-generation/drafts/P2-C5/v002/draft.md"
  review_passes: [Content]  # Fast-track: only re-check affected pass

# 4. Repeat until approved
```

---

## Parallel Chapter Generation

For independent chapters (no sequential dependencies):

```bash
# Example: Part 1 chapters (all can run in parallel)
CHAPTERS=("P1-C1" "P1-C2" "P1-C3" "P1-C4" "P1-C5")

# Launch all research-agents in parallel
for CHAPTER_ID in "${CHAPTERS[@]}"; do
  Task: research-agent (in parallel)
  Parameters: topic from spec.md for $CHAPTER_ID
done

# Wait for all to complete
# → 5 research.md files generated simultaneously

# Launch all outliner-agents in parallel
for CHAPTER_ID in "${CHAPTERS[@]}"; do
  Task: outliner-agent (in parallel)
  Parameters: research_file for $CHAPTER_ID
done

# Continue pipeline in parallel for all chapters
```

**Note**: Part 2+ chapters may have dependencies on Part 1 concepts, so check prerequisites before parallel execution.

---

## Quality Gates Summary

At each stage, validators check:

| Validator | Stage | Blocking? | What It Checks |
|-----------|-------|-----------|----------------|
| Constitutional Compliance | All | Yes | 20 articles, zero violations required |
| Dual-Domain | Outline, Lesson, Draft | Yes | Physical + Simulation coverage ≥ 70% each |
| Safety | Lesson, Draft | Yes | Physical labs have warnings |
| Citation | Research, Draft, Review | Yes | Tier 1/2, no Wikipedia, IEEE format |
| Readability | Draft | No (Warning) | Flesch-Kincaid 12-14 |
| Diagram Style | Review | No (Warning) | Color palette, notation, labels |

**Pipeline Halts On**:
- Constitutional violations (Critical)
- Dual-domain balance < 0.7
- Missing safety warnings
- Wikipedia citations
- Broken URLs

**Pipeline Continues With Warnings**:
- Readability slightly outside target
- Diagram style inconsistencies (manual review OK)

---

## Troubleshooting

**Problem**: "Dual-domain validator fails (balance score 0.5)"
**Solution**: Outliner-agent needs to add more simulation content to sections. Re-run outliner with explicit instruction to balance physical and simulation coverage.

**Problem**: "Citation validator fails (only 8 Tier 1 citations)"
**Solution**: Research-agent needs to find 2+ more peer-reviewed sources. Re-run research-agent with increased tier1_minimum parameter.

**Problem**: "Book-editor returns MajorRevisions (structure issues)"
**Solution**: Lesson-planner may have skipped sections. Review structure.md and lesson.md, ensure all 14 sections outlined. Re-run lesson-planner.

**Problem**: "Readability score 16.5 (too complex)"
**Solution**: Writer-agent needs to simplify prose. Re-run with instruction to reduce sentence length and use simpler vocabulary. Target FK Grade 12-14.

---

## Version History Example

```
.book-generation/drafts/P2-C5/
  v001/
    draft.md (Initial generation, 2025-11-30)
    metadata.yaml (agent: writer-agent, input: lessons/v001)
  v002/
    draft.md (After addressing Issues 2.1, 2.2, 2025-12-01)
    metadata.yaml (agent: writer-agent, input: lessons/v001, feedback: reviews/v001)
  v003/
    draft.md (Final approved version, 2025-12-02)
    metadata.yaml (approval_status: Approved)
```

**Always reference specific version** in subsequent stages to maintain traceability.

---

**End of Quickstart Guide**

For questions or issues, consult:
- `plan.md` — Full implementation plan
- `research.md` — Research decisions and standards
- `data-model.md` — Entity definitions and validation rules
- `contracts/` — Agent invocation specifications and validation gates
