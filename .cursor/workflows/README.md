# Workflows Directory

This directory contains workflow definitions for the Universal Pedagogical Engine project, ensuring proper agent orchestration and skill application.

## Files

### primary-workflow.md

**Purpose**: Define the complete agent invocation sequence and phase definitions

**Contents**:
- Phase 1: Research & Discovery (research-agent)
- Phase 2: Structural Design (outliner-agent, chapter-structure-architect)
- Phase 3: Content Creation (lesson-planner, writer-agent)
- Phase 4: Quality Assurance (book-editor)
- Agent invocation rules and decision tree
- Quality gates between phases

**When to Reference**: 
- Determining which agent to invoke next
- Understanding the complete workflow sequence
- Checking phase prerequisites

### orchestration-protocol.md

**Purpose**: Define how agents coordinate, communicate, and hand off work

**Contents**:
- Agent handoff protocols (research → outline → writing → editing)
- Cross-agent validation rules
- Skill application verification
- Error recovery procedures
- Proactive agent suggestions

**When to Reference**:
- Understanding how agents coordinate
- Implementing agent handoffs
- Validating prerequisites
- Handling errors and missing inputs

### development-rules.md

**Purpose**: Define code quality standards and development practices

**Contents**:
- Constitution compliance requirements
- Agent output standards (versioning, naming, metadata)
- Agent orchestration rules
- Pre-commit procedures (PHR creation, ADR suggestions)
- Quality gates for each phase
- Error handling guidelines

**When to Reference**:
- Setting up agent outputs
- Validating quality before proceeding
- Understanding versioning requirements
- Following development standards

## Quick Reference

### Agent Invocation Sequence (MANDATORY PIPELINE)

**ALL 6 agents MUST be invoked in this exact sequence - NO EXCEPTIONS:**

```
User Request: Generate Book Content
    ↓
Phase 1:  research-agent (research-methodology)
    ↓     Output: .book-generation/research/[topic]/v[NNN]/
Phase 2A: outliner-agent
    ↓     Output: .book-generation/outlines/[book]/v[NNN]/
Phase 2B: chapter-structure-architect
    ↓     Output: .book-generation/structures/[chapter]/v[NNN]/
Phase 3A: lesson-planner
    ↓     Output: .book-generation/lessons/[chapter]/v[NNN]/
Phase 3B: writer-agent (prose-generation)
    ↓     Output: .book-generation/drafts/[chapter]/v[NNN]/
Phase 4:  book-editor (content-editing)
          Output: .book-generation/reviews/[chapter]/v[NNN]/
```

**No agent may be skipped.** Each agent's output is REQUIRED input for the next agent.

### Skill Assignments

- `research-agent` → `research-methodology` (MANDATORY)
- `writer-agent` → `prose-generation` (MANDATORY)
- `book-editor` → `content-editing` (MANDATORY)
- Structural agents → No skills (specialized design)

### Versioning Structure

All agent outputs follow this pattern:

```
[output-type]/[slug]/v[NNN]/
├── [file].md          # Main output
├── version.json       # Version metadata
└── _current.json      # Pointer to latest version
```

### Quality Gates

- **Research → Outlining**: 10+ Tier 1/2 sources, 60%+ Tier 1, versioned
- **Outlining → Writing**: Complete structure, all sections have purpose, versioned
- **Writing → Editing**: Complete draft, voice consistent, citations integrated, versioned
- **Editing → Publication**: All 5 passes complete, metrics meet targets, no blockers, versioned

## Integration

These workflows integrate with:

- **CURSOR.md**: Main Cursor Agent configuration (references these workflows)
- **Constitution**: `.specify/memory/constitution.md` (all agents must comply)
- **Specify Kit Plus**: Commands like `/sp.specify`, `/sp.plan`, `/sp.tasks`
- **Agent Definitions**: `.cursor/agents/` (agent-specific instructions)
- **Skill Definitions**: `.cursor/skills/` (skill procedures)

## Version History

- **1.0.0** (2025-11-26): Initial workflow definitions with all 6 agents and 3 skills mapped

---

**For detailed information, see the individual workflow files.**

