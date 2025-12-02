# Primary Workflow: Universal Pedagogical Engine

**Version**: 1.0.0  
**Last Updated**: 2025-11-26  
**Purpose**: Define the complete agent orchestration flow for book generation and educational content creation

## Overview

This workflow defines when and how each agent is invoked, ensuring proper sequencing and skill utilization throughout the content creation pipeline. All agents must follow the Specify Kit Plus conventions and adhere to the constitution defined in `.specify/memory/constitution.md`.

**MANDATORY 6-AGENT PIPELINE**: All book generation MUST follow this exact sequence:

```
research-agent → outliner-agent → chapter-structure-architect → lesson-planner → writer-agent → book-editor
```

**No agent may be skipped.** Each agent's output is required input for the next agent in the pipeline.

## Workflow Phases

### Phase 1: Research & Discovery

**Trigger**: User requests research on a topic, or research is needed before outlining/writing

**Agent**: `research-agent`  
**Skills**: `research-methodology`

**Invocation Criteria**:
- User explicitly requests research
- Outlining phase identifies research gaps
- Writing phase encounters unverified claims
- Editing phase flags missing citations

**Process**:
1. Research agent analyzes research requirements
2. Applies `research-methodology` skill for source gathering
3. Follows Tier 1/2 source standards (NO Wikipedia)
4. Completes within 3-4 hour target per chapter section
5. Outputs versioned research to `.book-generation/research/[topic-slug]/v[NNN]/`

**Output**: Versioned research notes with authenticated sources

**Next Phase**: Proceed to Phase 2A (Outlining) - MANDATORY

---

### Phase 2: Structural Design

**Trigger**: Research complete, or user requests book/chapter structure

#### 2A: Book-Level Outlining

**Agent**: `outliner-agent`  
**Skills**: None (structural design specialist)

**Invocation Criteria**:
- User has research materials and needs book structure
- User requests organization of existing content
- Research phase complete and ready for structuring

**Process**:
1. Outliner agent analyzes research materials
2. Identifies concept clusters and dependencies
3. Creates hierarchical book structure (chapters → sections)
4. Ensures logical flow and balanced content distribution
5. Outputs versioned outline to `.book-generation/outlines/[book-slug]/v[NNN]/`

**Output**: Complete book outline with chapter hierarchy

**Next Phase**: Proceed to Phase 2B (Chapter Structure) - MANDATORY

#### 2B: Chapter Structure Architecture

**Agent**: `chapter-structure-architect`
**Skills**: None (pedagogical structure specialist)

**Invocation Criteria**:
- Outline complete from Phase 2A (MANDATORY prerequisite)
- ALWAYS invoked after outliner-agent completes

**Process**:
1. Chapter structure architect reads outline from `.book-generation/outlines/[book-slug]/`
2. For EACH chapter in outline:
   - Analyzes concept density
   - Applies concept density formula (8-9 lessons for standard chapters)
   - Maps 4-layer pedagogical progression (Manual → AI Collab → Design → Integration)
   - Defines 5 AI integration touchpoints per lesson
3. Outputs versioned structural blueprint to `.book-generation/structures/[chapter-slug]/v[NNN]/`

**Output**: Chapter structure blueprint with lesson framework for each chapter

**Next Phase**: Proceed to Phase 3A (Lesson Planning) - MANDATORY

---

### Phase 3: Content Creation

**Trigger**: Chapter structure complete from Phase 2B (MANDATORY prerequisite)

#### 3A: Lesson Content Planning

**Agent**: `lesson-planner`
**Skills**: None (content creation specialist)

**Invocation Criteria**:
- Chapter structure blueprint complete from Phase 2B (MANDATORY prerequisite)
- ALWAYS invoked after chapter-structure-architect completes

**Process**:
1. Lesson planner reads structure from `.book-generation/structures/[chapter-slug]/`
2. For EACH lesson in structure:
   - Implements 5-part adaptive content template:
     - Diagnostic Hook (Pre-Assessment)
     - Concept Theory (AI Tutor integration)
     - Walkthrough (Contextual Help integration)
     - Iterative Challenge (AI Grader integration)
     - Spaced-Repetition Checkpoint
   - Applies master methodologies (Visual Intuition First, Stepwise Scaffolding, Pattern-Based Chunking)
   - Integrates all 5 AI touchpoints at defined positions
3. Outputs versioned lesson content to `.book-generation/lessons/[chapter-slug]/v[NNN]/`

**Output**: Full lesson content with all AI touchpoints integrated

**Next Phase**: Proceed to Phase 3B (Prose Writing) - MANDATORY

#### 3B: Prose Generation

**Agent**: `writer-agent`
**Skills**: `prose-generation`

**Invocation Criteria**:
- Lesson content complete from Phase 3A (MANDATORY prerequisite)
- ALWAYS invoked after lesson-planner completes

**Process**:
1. Writer agent reads lessons from `.book-generation/lessons/[chapter-slug]/`
2. Reads structure from `.book-generation/structures/[chapter-slug]/`
3. Reads outline from `.book-generation/outlines/[book-slug]/`
4. Reviews research notes from `.book-generation/research/`
5. Applies `prose-generation` skill for:
   - Voice and tone configuration
   - Paragraph structure (4-part pattern)
   - Transition techniques
   - Chapter flow patterns
6. Transforms lesson content into publication-ready prose
7. Outputs versioned draft to `.book-generation/drafts/[chapter-slug]/v[NNN]/`

**Output**: Complete chapter draft with consistent voice and proper structure

**Next Phase**: Proceed to Phase 4 (Editing) - MANDATORY

---

### Phase 4: Quality Assurance

**Trigger**: Content draft complete, ready for editorial review

**Agent**: `book-editor`  
**Skills**: `content-editing`

**Invocation Criteria**:
- Writer agent completes chapter draft
- User requests editorial review
- Pre-publication quality check needed
- Citation verification required

**Process**:
1. Book editor reads draft from `.book-generation/drafts/[chapter-slug]/`
2. Applies `content-editing` skill's five-pass system:
   - **Pass 1**: Structural review (organization, flow)
   - **Pass 2**: Content quality (grammar, clarity, readability)
   - **Pass 3**: Citation verification (coverage, format, quality)
   - **Pass 4**: Consistency audit (terminology, style, voice)
   - **Pass 5**: Factual accuracy (truth verification against sources)
3. Cross-references with research files when available
4. Generates tracked changes and edit summary
5. Outputs versioned review to `.book-generation/reviews/[chapter-slug]/v[NNN]/`

**Output**: Comprehensive editorial review with tracked changes and quality metrics

**Next Phase**: Return to Phase 3 for revisions, or proceed to publication

---

## Agent Invocation Rules

### Rule 1: Sequential Dependencies (MANDATORY PIPELINE)

**ALL 6 agents must be invoked in this exact sequence - NO EXCEPTIONS:**

```
research-agent → outliner-agent → chapter-structure-architect → lesson-planner → writer-agent → book-editor
```

Each agent's output is the REQUIRED input for the next agent:
- `research-agent` output → required by `outliner-agent`
- `outliner-agent` output → required by `chapter-structure-architect`
- `chapter-structure-architect` output → required by `lesson-planner`
- `lesson-planner` output → required by `writer-agent`
- `writer-agent` output → required by `book-editor`

**Exception**: Research can be invoked AGAIN at any time if gaps are identified during later phases.

### Rule 2: Skill Application

Each agent MUST use its assigned skills:
- `research-agent` → `research-methodology` (MANDATORY)
- `writer-agent` → `prose-generation` (MANDATORY)
- `book-editor` → `content-editing` (MANDATORY)
- Structural agents (outliner, chapter-structure-architect, lesson-planner) → No skills (specialized design)

### Rule 3: Versioning Protocol

**ALL agents MUST version their outputs**:
- Format: `v001`, `v002`, `v003` (3-digit padded)
- Structure: `[output-type]/[slug]/v[NNN]/[file].md` + `version.json` + `_current.json`
- Increment on: Rewrites, major revisions, feedback incorporation

### Rule 4: Constitution Compliance

All agents must:
- Follow `.specify/memory/constitution.md` principles
- Create PHR (Prompt History Records) after completion
- Suggest ADRs for architecturally significant decisions
- Use MCP tools and CLI commands for information gathering

### Rule 5: Proactive Agent Invocation

Agents should proactively suggest next steps:
- Writer agent → Suggest book-editor after draft completion
- Research agent → Suggest outliner-agent after research complete
- Book editor → Suggest writer-agent for revisions if needed

---

## Workflow Decision Tree (MANDATORY SEQUENTIAL PIPELINE)

```
User Request: Generate Book Content
    │
    ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 1: RESEARCH                                           │
│ research-agent (research-methodology)                       │
│ Output: .book-generation/research/[topic]/v[NNN]/           │
└─────────────────────────────────────────────────────────────┘
    │ MANDATORY
    ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 2A: OUTLINING                                         │
│ outliner-agent                                              │
│ Output: .book-generation/outlines/[book]/v[NNN]/            │
└─────────────────────────────────────────────────────────────┘
    │ MANDATORY
    ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 2B: CHAPTER STRUCTURE                                 │
│ chapter-structure-architect                                 │
│ Output: .book-generation/structures/[chapter]/v[NNN]/       │
└─────────────────────────────────────────────────────────────┘
    │ MANDATORY
    ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 3A: LESSON PLANNING                                   │
│ lesson-planner                                              │
│ Output: .book-generation/lessons/[chapter]/v[NNN]/          │
└─────────────────────────────────────────────────────────────┘
    │ MANDATORY
    ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 3B: PROSE WRITING                                     │
│ writer-agent (prose-generation)                             │
│ Output: .book-generation/drafts/[chapter]/v[NNN]/           │
└─────────────────────────────────────────────────────────────┘
    │ MANDATORY
    ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 4: EDITORIAL REVIEW                                   │
│ book-editor (content-editing)                               │
│ Output: .book-generation/reviews/ & docs/[chapter].md       │
└─────────────────────────────────────────────────────────────┘
    │
    ▼
Publication Ready (or revision loop back to Phase 3B)
```

**CRITICAL**: No phase may be skipped. Each agent validates that ALL previous agent outputs exist before proceeding.

---

## Error Handling

### Missing Prerequisites

**If agent invoked without required inputs**:
- Report missing prerequisites clearly
- Suggest which previous agent needs to run first
- Do NOT proceed with incomplete inputs

**Example**: Writer agent invoked without outline → Report: "Outline required. Run outliner-agent first."

### Skill Not Applied

**If agent fails to use assigned skill**:
- Agent must reference skill file explicitly
- Agent must follow skill procedures exactly
- Do NOT duplicate skill content in agent file

### Versioning Violations

**If output not versioned**:
- Reject output
- Request agent to create proper version structure
- Ensure `_current.json` pointer exists

---

## Quality Gates

### Before Proceeding to Next Phase

**Research → Outlining**:
- [ ] Minimum 10 Tier 1/2 sources per major section
- [ ] 60%+ sources are Tier 1
- [ ] Research notes versioned and saved

**Outlining → Writing**:
- [ ] Complete book/chapter structure defined
- [ ] All sections have purpose statements
- [ ] Outline versioned and saved

**Writing → Editing**:
- [ ] Complete draft (no placeholders)
- [ ] Voice consistent throughout
- [ ] Citations integrated
- [ ] Draft versioned and saved

**Editing → Publication**:
- [ ] All five editorial passes complete
- [ ] Quality metrics meet targets (Flesch >60, Passive <20%, Citations 100%)
- [ ] No blocking issues remain
- [ ] Review versioned and saved

---

## Integration with Specify Kit Plus

This workflow integrates with Specify Kit Plus commands:

- `/sp.specify` → Feature specification (not directly agent-related, but may trigger research)
- `/sp.plan` → Implementation planning (may identify need for research-agent)
- `/sp.tasks` → Task breakdown (may identify agent invocation needs)
- `/sp.implement` → Implementation (may trigger writer-agent or lesson-planner)

**PHR Creation**: All agents MUST create Prompt History Records after completion (see CLAUDE.md for PHR protocol).

**ADR Suggestions**: Agents must suggest ADRs for architecturally significant decisions (see CLAUDE.md for ADR protocol).

---

## Version History

- **1.0.0** (2025-11-26): Initial workflow definition with all 6 agents and 3 skills mapped

---

**Next Steps**: See `.claude/workflows/orchestration-protocol.md` for detailed agent coordination rules.

