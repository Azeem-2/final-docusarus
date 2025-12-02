---
id: 003
title: Create Implementation Plan
stage: plan
date: 2025-11-30
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-robotics-book-spec
branch: 1-robotics-book-spec
user: a
command: /sp.plan
labels: ["planning", "architecture", "agent-orchestration", "book-generation", "workflows"]
links:
  spec: specs/1-robotics-book-spec/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-robotics-book-spec/plan.md
  - specs/1-robotics-book-spec/research.md
  - specs/1-robotics-book-spec/data-model.md
  - specs/1-robotics-book-spec/contracts/agent-invocation.yaml
  - specs/1-robotics-book-spec/contracts/validation-gates.yaml
  - specs/1-robotics-book-spec/quickstart.md
tests: []
---

## Prompt

/sp.plan

(No additional arguments provided)

## Response

Created comprehensive implementation plan for Physical AI, Simulation AI & Humanoid Robotics book generation system.

**Files Generated**:

1. **`specs/1-robotics-book-spec/plan.md`** (27KB) — Complete architectural design
2. **`specs/1-robotics-book-spec/research.md`** (16KB) — Research findings with 6 resolved technical decisions
3. **`specs/1-robotics-book-spec/data-model.md`** (15KB) — Entity definitions and validation rules
4. **`specs/1-robotics-book-spec/contracts/agent-invocation.yaml`** (9KB) — Agent interface specifications
5. **`specs/1-robotics-book-spec/contracts/validation-gates.yaml`** (11KB) — Quality validation contracts
6. **`specs/1-robotics-book-spec/quickstart.md`** (18KB) — Step-by-step chapter generation guide

---

### **Implementation Plan Highlights**

#### **Technical Architecture**

**Agent-Orchestrated Workflow**:
- 6 specialized agents in mandatory sequential pipeline
- research-agent → outliner-agent → chapter-structure-architect → lesson-planner → writer-agent → book-editor
- **NO AGENT MAY BE SKIPPED** (constitutional requirement from CLAUDE.md workflows)

**Versioned Output System**:
```
.book-generation/
  research/{topic}/v{NNN}/research.md
  outlines/{book}/v{NNN}/outline.md
  structures/{chapter}/v{NNN}/structure.md
  lessons/{chapter}/v{NNN}/lesson.md
  drafts/{chapter}/v{NNN}/draft.md
  reviews/{chapter}/v{NNN}/review.md
```

**Skill-Based Composition**:
- `research-agent` uses `research-methodology` skill
- `writer-agent` uses `prose-generation` skill
- `book-editor` uses `content-editing` skill
- Skills encode constitutional principles (Article 6 tone, Article 19 citation standards)

**Validation Gates** (6 automated validators):
1. Constitutional Compliance (all 20 articles)
2. Dual-Domain Integration (physical + simulation ≥70% each)
3. Safety Warnings (physical labs)
4. Citation Standards (Tier 1/2, no Wikipedia, IEEE format)
5. Readability (Flesch-Kincaid 12-14)
6. Diagram Style (consistent color palette, notation, labels)

#### **Constitutional Compliance Matrix**

Validated plan against all 20 constitutional articles:

| Article | Implementation Approach | Status |
|---------|------------------------|--------|
| Article 1 (Purpose) | Agent pipeline ensures systematic coverage | ✅ Aligned |
| Article 2 (Scope) | Dual-domain validator enforces 100% coverage | ✅ Aligned |
| Article 7 (Chapter Format) | Template validation ensures 14 sections | ✅ Aligned |
| Article 13 (Safety) | Safety validator checks all physical labs | ✅ Aligned |
| Article 19 (Academic Integrity) | Citation validator enforces Tier 1/2, IEEE format | ✅ Aligned |
| ... | ... | All 20 ✅ |

**Overall**: ✅ **FULLY ALIGNED** (100% constitutional compliance)

---

### **Research Decisions** (Phase 0)

**RT1: Simulation Platforms**:
- **Primary**: Isaac Sim, MuJoCo (detailed chapters, all examples)
- **Secondary**: Gazebo, Webots (comparative examples)
- **Tertiary**: Unity Robotics (mentions, specialized use cases)

**RT2: Physical Lab Hardware** (Total: $496):
- Raspberry Pi 4 (4GB): $55
- Arduino Mega 2560: $40
- Sensors: IMU ($8), Ultrasonic ($10), LIDAR ($100), Camera ($30), Encoders ($24)
- Actuators: Servos ($48), DC Motors ($30), Motor Drivers ($16)
- Power: LiPo Battery ($25), Power Bank ($20), Charger ($15)
- Mechanical: Chassis ($30), Brackets ($20), Electronics ($25)

**RT3: Citation Standards**:
- **Tier 1** (≥10 per chapter): Peer-reviewed journals, conferences, textbooks, official docs
- **Tier 2** (≤5 per chapter): Industry white papers, expert blogs, open-source docs
- **Excluded**: Wikipedia, user-editable platforms, non-peer-reviewed preprints

**RT4: Safety Protocols**:
- Mechanical: Pinch points, rotating parts, sharp edges, emergency stop
- Electrical: <12V DC limit, short circuit prevention, LiPo battery safety
- Motion: Unexpected movement warnings, HRI zones, phased testing (bench → tethered → free)

**RT5: Pedagogical Framework** (validated against learning science):
- 4-Layer Progression: Introduction (Bloom: Remember/Understand) → Theory (Understand/Apply) → Practice (Apply/Analyze) → Projects (Evaluate/Create)
- 5 AI Touchpoints: Pre-assessment, AI tutor, contextual help, AI-graded challenge, spaced repetition
- Evidence-based: Freeman (2014) active learning meta-analysis, Bloom's 2-sigma problem, Sweller's cognitive load theory

**RT6: Concept Density Formula**:
```
CD = (New Concepts + 0.5 × Prerequisites + 2 × Math Derivations) / Reading Time (minutes)

Classification:
- Low (<0.05): 1-2 lessons, Beginner
- Medium (0.05-0.10): 2-3 lessons, Intermediate
- High (>0.10): 3-4 lessons OR split chapter, Advanced
```

---

### **Data Model** (Phase 1)

**Core Entities** (6 defined):

1. **Chapter**: 40+ instances (all book chapters)
   - 14 mandatory sections (Article 7)
   - Status: Planned → Research → Outline → Structure → Lesson → Draft → Review → Published
   - Validation: Dual-domain coverage, constitutional compliance, all sections present

2. **Section**: 14 per chapter
   - Types: Introduction, Motivation, LearningObjectives, KeyTerms, PhysicalExplanation, SimulationExplanation, IntegratedUnderstanding, Diagrams, Examples, Labs, MiniProjects, Applications, Summary, ReviewQuestions
   - Validation: Word count ranges, keyword detection (physical/simulation)

3. **Diagram**: Minimum 4 per chapter
   - Types: Architecture, Flow, Mechanical, SimulationPipeline
   - Formats: Mermaid (automated), SVG/PNG (manual)
   - Validation: Style compliance (color palette, notation, labels)

4. **Lab**: Exactly 2 per chapter (Simulation + Physical)
   - Simulation: Specifies platform (Isaac Sim, MuJoCo, Gazebo, Webots)
   - Physical: Equipment list (<$500), safety warnings, testing procedures
   - Validation: Safety validator for physical labs

5. **Project**: 1 mini-project per chapter + 6 integrated projects (Part 6)
   - Types: SimulationOnly, PhysicalOnly, Hybrid (sim-to-real)
   - Validation: Measurable evaluation criteria, completable time estimates

6. **TechnicalTerm**: 5-20 per chapter KeyTerms section
   - Attributes: Definition, first_introduced chapter, category, complexity_level
   - Validation: No circular definitions, beginner-friendly language

7. **Platform**: Predefined list (Isaac Sim, MuJoCo, Gazebo, Webots, Unity, ROS2)
   - Attributes: Version coverage, licensing, official docs URL, supported OS
   - Validation: URL accessibility (HTTP 200), educational neutrality

---

### **Agent Contracts** (Phase 1)

**General Interface**:
```yaml
Input:
  agent_type: String
  input_context: Object
  output_directory: String (versioned path)
  constitutional_compliance: Boolean (default: true)

Output:
  status: Success | Failed | NeedsReview
  output_path: String (absolute path to artifact)
  version: Integer
  validation_results: Object
  next_agent: String | null
```

**Agent 1: research-agent** (uses research-methodology skill):
- Gathers ≥10 Tier 1 citations, no Wikipedia
- Output: `.book-generation/research/{topic}/v{NNN}/research.md`

**Agent 2: outliner-agent**:
- Creates hierarchical chapter structure with 14 sections
- Dual-domain coverage ≥70% physical, ≥70% simulation
- Output: `.book-generation/outlines/{book}/v{NNN}/outline.md`

**Agent 3: chapter-structure-architect**:
- Calculates concept density, classifies chapter (Low/Medium/High)
- Maps 4-layer pedagogical progression, defines 5 AI touchpoints
- Output: `.book-generation/structures/{chapter}/v{NNN}/structure.md`

**Agent 4: lesson-planner**:
- Creates 6-part lesson: Hook, Theory, Walkthrough, Challenge, Takeaways, Learn with AI
- Specifies diagrams and labs
- Output: `.book-generation/lessons/{chapter}/v{NNN}/lesson.md`

**Agent 5: writer-agent** (uses prose-generation skill):
- Transforms lesson → polished prose
- Target: 2000-4000 words, FK Grade 12-14, balance ratio 0.8-1.2
- Output: `.book-generation/drafts/{chapter}/v{NNN}/draft.md`

**Agent 6: book-editor** (uses content-editing skill):
- 5-pass review: Structural, Content, Citation, Consistency, Accuracy
- Approval status: Approved | MinorRevisions | MajorRevisions | Rejected
- Output: `.book-generation/reviews/{chapter}/v{NNN}/review.md`

---

### **Validation Gates**

**Gate 1: Constitutional Compliance** (all 20 articles):
- Critical violations BLOCK publication
- Warnings flagged for review

**Gate 2: Dual-Domain** (Article 2 enforcement):
- Detects physical keywords (hardware, sensor, motor, ...)
- Detects simulation keywords (simulator, RL, digital twin, ...)
- Balance score ≥0.7 required (min(physical, simulation) / max(physical, simulation))

**Gate 3: Safety** (Article 13 enforcement):
- Physical labs MUST have safety warnings (mechanical, electrical, motion)
- Emergency stop mentioned for motorized systems
- High-risk procedures (LiPo, AC voltage) require safety professional review

**Gate 4: Citation** (Article 19 enforcement):
- ≥10 Tier 1 citations per chapter
- No Wikipedia or excluded sources
- IEEE format, all URLs accessible (HTTP 200)

**Gate 5: Readability** (Article 6 guidance):
- Flesch-Kincaid Grade 12-14 (university level)
- Flesch Reading Ease 50-60 (fairly difficult)
- Passive voice <15%

**Gate 6: Diagram Style** (Article 10 enforcement):
- Color palette compliance
- Notation standards (solid arrows = data flow, dashed = control flow)
- All components labeled, minimum font size 12pt
- Black & white readable (patterns, not just colors)

---

### **Implementation Strategy**

**Timeline** (30 weeks total):
- **Weeks 1-2**: Setup and research completion
- **Weeks 3-4**: Pilot chapters (7 chapters, 1 per part)
- **Weeks 5-7**: Part 1 production (5 chapters)
- **Weeks 8-14**: Parts 2-3 production (14 chapters, parallel)
- **Weeks 15-21**: Parts 4-5 production (14 chapters)
- **Weeks 22-26**: Part 6 production (6 integrated projects)
- **Weeks 27-30**: Part 7 + finalization (4 chapters + book-wide review)

**Resource Requirements**:
- Technical reviewers (3): Physical robotics, Simulation/AI, Humanoid robotics
- Safety auditor (1): Certified professional for physical labs
- Educational advisor (1): Pedagogical framework validation
- Technical illustrator (1): Manual diagrams (complex mechanical assemblies)
- Beta testers (20+): Students/educators for lab completion

**Quality Assurance** (5 layers):
1. Automated gates (constitutional, dual-domain, safety, citation, readability, diagram)
2. Agent self-review (book-editor 5 passes)
3. Human expert review (technical accuracy, safety)
4. Beta testing (lab completion rate ≥90%)
5. Final audit (cross-chapter consistency, glossary, index)

---

### **Risk Mitigation**

**Risk 1: Agent Output Quality Variability**
- Mitigation: Pilot chapter phase, editorial review, iterative refinement with version control

**Risk 2: Constitutional Violations**
- Mitigation: Automated validators, zero-tolerance enforcement, pipeline halts on critical violations

**Risk 3: Dual-Domain Imbalance**
- Mitigation: Automated dual-domain validator (keyword counts, balance score), book-editor Pass 2

**Risk 4: Safety Incident During Beta Testing**
- Mitigation: Certified safety professional reviews all physical labs, liability waivers, adult supervision, simulation-first testing

**Risk 5: Platform Obsolescence**
- Mitigation: Version-pin references, maintain errata document, multi-platform examples, concept-focused content

**Risk 6: Timeline Delays**
- Mitigation: Parallel chapter generation for independent chapters, buffer weeks, pilot phase identifies bottlenecks

---

### **Key Design Decisions**

**Decision 1: Agent-Orchestrated Workflow vs. Manual Authoring**
- **Chosen**: Agent-orchestrated (6-agent pipeline)
- **Rationale**: Perfect consistency, automated validation, faster production, scalable
- **Trade-offs**: Requires agent infrastructure, potential generic prose (mitigated by prose-generation skill)

**Decision 2: Versioned Output Directories**
- **Chosen**: `.book-generation/{stage}/{topic}/v{NNN}/`
- **Rationale**: Complete audit trail, safe iteration, parallel workflows, rollback capability
- **Trade-offs**: More disk space (acceptable for text content)

**Decision 3: Skill-Based Agent Composition**
- **Chosen**: Agents invoke reusable skills (research-methodology, prose-generation, content-editing)
- **Rationale**: Modular, maintainable, constitutional alignment built-in
- **Trade-offs**: Requires skill infrastructure (already exists per CLAUDE.md)

**Decision 4: Dual-Domain Validation Gates**
- **Chosen**: Automated validator with manual review backup
- **Rationale**: 100% enforcement, fast feedback, objective measurement, secondary validation by book-editor
- **Trade-offs**: False positives possible (mitigated by manual override with justification)

**Decision 5: Diagram Generation Approach**
- **Chosen**: Hybrid (Mermaid.js for flows/architecture, manual for complex mechanical)
- **Rationale**: Fast automated diagrams, consistent styling, editability (text-based), flexibility for complex cases
- **Trade-offs**: Mermaid limitations for 3D mechanical diagrams (acceptable)

**Decision 6: Editorial Review Workflow**
- **Chosen**: 5-pass multi-pass review (Structural, Content, Citation, Consistency, Accuracy)
- **Rationale**: Focused attention per pass, thoroughness, efficiency with automated tools, traceability
- **Trade-offs**: More time per chapter (acceptable for educational quality)

---

## Outcome

Comprehensive architectural plan successfully created for Physical AI, Simulation AI & Humanoid Robotics book generation system.

**Deliverables**:
- ✅ plan.md (27KB): Complete architecture, timeline, resources, risks
- ✅ research.md (16KB): 6 technical decisions resolved (platforms, hardware, citations, safety, pedagogy, concept density)
- ✅ data-model.md (15KB): 7 core entities with validation rules
- ✅ contracts/agent-invocation.yaml (9KB): 6 agent specifications
- ✅ contracts/validation-gates.yaml (11KB): 6 automated validators
- ✅ quickstart.md (18KB): Step-by-step chapter generation guide

**Constitutional Compliance**: ✅ **FULLY ALIGNED** (all 20 articles)

**Phase 0 Complete**: All research tasks (RT1-RT6) resolved, no blocking clarifications

**Phase 1 Complete**: Data model, contracts, quickstart guide finalized

**Ready for**: `/sp.tasks` — Generate actionable task breakdown for implementation

## Evaluation

**Success Criteria Met**:
- ✅ Technical architecture defined (agent pipeline, versioned outputs, skills, validators)
- ✅ Constitutional compliance validated (all 20 articles addressed)
- ✅ All research questions resolved (platforms, hardware, citations, safety, pedagogy, concept density)
- ✅ Data model complete (7 entities, validation rules, relationships)
- ✅ Agent contracts specified (6 agents with inputs/outputs/validation)
- ✅ Validation gates defined (6 automated checkers with blocking/warning rules)
- ✅ Implementation strategy documented (timeline, resources, QA, risks)
- ✅ Quickstart guide created (complete chapter generation workflow)

**Design Decision Quality**:
- 6 major architectural decisions documented with rationale, alternatives, trade-offs
- Each decision traces to constitutional requirements or specification constraints
- Risk mitigation strategies defined for 6 identified risks

**Next Steps**:
- Execute `/sp.tasks` to break down plan into actionable tasks
- Begin Phase 1: Pilot chapter generation (7 chapters, 1 per part)
- Validate agent pipeline with real content generation
