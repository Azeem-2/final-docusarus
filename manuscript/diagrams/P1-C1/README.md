# Chapter P1-C1 Diagrams: What is Physical AI

**Chapter**: Part 1, Chapter 1 - What is Physical AI
**Created**: 2025-11-30
**Mermaid Version**: 10.6+
**Style Guide Compliance**: v1.0.0

---

## Diagram Index

### Figure 1: The Six Fundamentals of Physical AI

**File**: `figure-1-six-fundamentals.mmd`
**Type**: Flowchart (Top-Down)
**Purpose**: Illustrate the closed control loop of the six fundamental principles

**Description**: This diagram shows how the six fundamentals (Embodiment, Perception, Action, Learning, Autonomy, Context) form a continuous closed loop. Each principle feeds into the next, with Context looping back to Embodiment to demonstrate the cyclical nature of Physical AI systems.

**Color Coding**:
- **Blue (#0066CC)**: Embodiment, Perception, Action (physical components)
- **Orange (#FF9900)**: Learning, Autonomy (AI/ML components)
- **Green (#00CC66)**: Context (simulation/environment adaptation)

**Caption**: "Figure 1: The Six Fundamentals of Physical AI form a closed control loop"

---

### Figure 2: Physical vs Simulation Comparison

**File**: `figure-2-physical-vs-sim.mmd`
**Type**: Flowchart (Left-Right)
**Purpose**: Compare and contrast physical robotics with simulation approaches

**Description**: Two-column layout showing Physical Robotics (left) and Simulation (right) perspectives. The center shows integration points (Digital Twin, Sim-to-Real Transfer) that bridge both domains. Solid arrows indicate data flow, dashed arrows indicate control/calibration flow.

**Color Coding**:
- **Blue (#0066CC)**: Physical robotics components (sensors, actuators, environment)
- **Green (#00CC66)**: Simulation components (virtual sensors, physics engine)
- **Orange (#FF9900)**: Integration layer (digital twin, transfer)

**Caption**: "Figure 2: Physical and Simulation perspectives complement each other"

---

### Figure 3: Sim-to-Real Transfer Pipeline

**File**: `figure-3-sim-to-real.mmd`
**Type**: Flowchart (Left-Right)
**Purpose**: Show the complete workflow from simulation training to physical deployment

**Description**: Step-by-step pipeline showing: Train in Sim → Sim-to-Sim Validation → Transfer Analysis → Physical Deploy → Fine-tune → Update Simulation. Includes feedback loops showing continuous improvement cycle. Domain Randomization feeds into training. Real-world data feeds back to simulation updates.

**Color Coding**:
- **Green (#00CC66)**: Simulation steps (training, validation, randomization)
- **Orange (#FF9900)**: Transfer and learning steps
- **Blue (#0066CC)**: Physical deployment
- **Gray (#666666)**: Feedback/framework updates

**Caption**: "Figure 3: The Sim-to-Real transfer workflow"

---

### Figure 4: Boston Dynamics Spot Architecture

**File**: `figure-4-spot-architecture.mmd`
**Type**: Flowchart (Top-Bottom)
**Purpose**: Demonstrate a real-world Physical AI system architecture

**Description**: Four-layer architecture showing: Sensor Layer (cameras, IMU, force sensors) → Perception Layer (sensor fusion, state estimation) → Control Layer (RL policy, planner) → Actuation Layer (motors) → Physical Environment. Feedback loops show sensor data flowing back from environment.

**Color Coding**:
- **Blue (#0066CC)**: Hardware components (sensors, motors)
- **Orange (#FF9900)**: AI/processing components (fusion, policy)
- **Gray (#666666)**: Environment

**Caption**: "Figure 4: Spot's sensing and control architecture"

---

### Figure 5: Humanoid-Gym Training Pipeline

**File**: `figure-5-humanoid-gym.mmd`
**Type**: Sequence Diagram
**Purpose**: Illustrate multi-stage training approach with sim-to-sim-to-real transfer

**Description**: Sequential interaction between Isaac Gym (primary training), MuJoCo (sim-to-sim validation), Real Unitree H1 robot (deployment), and Human Operator (analysis). Shows five phases: (1) Simulation Training, (2) Sim-to-Sim Validation, (3) Physical Deployment, (4) Fine-tuning, (5) Simulation Update. Performance metrics shown at each stage (95% → 90% → 85% → 95%).

**Color Coding**:
- **Green (#00CC66)**: Simulation actors (Isaac Gym, MuJoCo)
- **Blue (#0066CC)**: Physical robot
- **Gray (#666666)**: Human operator
- **Orange (#FF9900)**: Notes showing learning/analysis steps

**Caption**: "Figure 5: Humanoid-Gym's multi-stage training approach"

---

## Usage Instructions

### Rendering Diagrams

**Option 1: Mermaid CLI**
```bash
npm install -g @mermaid-js/mermaid-cli
mmdc -i figure-1-six-fundamentals.mmd -o figure-1-six-fundamentals.png -b white -w 800
```

**Option 2: Mermaid Live Editor**
1. Visit https://mermaid.live/
2. Copy diagram code from `.mmd` file
3. Export as PNG or SVG

**Option 3: Markdown with Mermaid Plugin**
```markdown
```mermaid
[paste diagram code here]
```
```

### Quality Checks

Before publishing, verify:
- [ ] All diagrams render without errors
- [ ] Color palette matches style guide (#0066CC, #00CC66, #FF9900, #666666)
- [ ] Font size ≥12pt (minimum readability)
- [ ] Diagrams are distinguishable in grayscale
- [ ] All components have clear labels
- [ ] Figure captions are present and numbered correctly
- [ ] Width is 600-1200px (optimal viewing)

---

## File Manifest

| File | Size | Nodes | Complexity |
|------|------|-------|------------|
| `figure-1-six-fundamentals.mmd` | 1.2 KB | 6 | Low |
| `figure-2-physical-vs-sim.mmd` | 1.8 KB | 9 | Medium |
| `figure-3-sim-to-real.mmd` | 2.1 KB | 7 | Medium |
| `figure-4-spot-architecture.mmd` | 2.3 KB | 11 | High |
| `figure-5-humanoid-gym.mmd` | 2.5 KB | 4 actors, 15 steps | High |

**Total Diagrams**: 5
**Total Size**: ~10 KB (source)

---

## Maintenance

### Updating Diagrams

When modifying diagrams:
1. Edit the `.mmd` source file
2. Verify syntax using Mermaid Live Editor
3. Check color compliance with style guide
4. Update this README if diagram purpose changes
5. Regenerate exported images (PNG/SVG)

### Version Control

Track changes in git commit messages:
```bash
git add manuscript/diagrams/P1-C1/
git commit -m "Update Figure 3: Add feedback loop to sim-to-real pipeline"
```

---

## References

- **Chapter Content**: `.book-generation/drafts/P1-C1/v002/draft.md`
- **Style Guide**: `.book-generation/style-guide/diagrams.md`
- **Theme Config**: `.book-generation/style-guide/mermaid-theme.json`
- **Mermaid Docs**: https://mermaid.js.org/

---

## Contact

**Diagram Creator**: Claude Code (Diagram Generator Agent)
**Review Status**: Pending editorial review
**Last Updated**: 2025-11-30
