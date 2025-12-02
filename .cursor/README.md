# Cursor Agent Configuration

This directory contains the Cursor Agent configuration that mirrors the Claude Code CLI setup, enabling the same agent orchestration system, skills, and workflows to work seamlessly in Cursor.

## Structure

```
.cursor/
├── workflows/          # Agent orchestration workflows
│   ├── primary-workflow.md
│   ├── orchestration-protocol.md
│   ├── development-rules.md
│   └── README.md
├── agents/             # Agent definitions
│   ├── research-agent.md
│   ├── outliner-agent.md
│   ├── chapter-structure-architect.md
│   ├── lesson-planner.md
│   ├── writer-agent.md
│   └── book-editor.md
└── skills/             # Skill definitions
    ├── research-methodology/
    │   └── SKILL.md
    ├── prose-generation/
    │   └── SKILL.md
    ├── content-editing/
    │   └── SKILL.md
    └── code-generation/
        └── SKILL.md
```

## How It Works

### 1. Configuration Files

- **`CURSOR.md`**: Main configuration file (similar to `CLAUDE.md`) that instructs Cursor Agent on how to follow the workflows
- **`.cursorrules`**: Rules file that enforces the agent pipeline and workflow compliance

### 2. Agent Pipeline

The MANDATORY 6-agent sequential pipeline:

```
research-agent → outliner-agent → chapter-structure-architect → lesson-planner → writer-agent → book-editor
```

**No agent may be skipped.** Each agent's output is REQUIRED input for the next agent.

### 3. Skill Assignments

- `research-agent` → MUST use `research-methodology` skill
- `writer-agent` → MUST use `prose-generation` skill
- `book-editor` → MUST use `content-editing` skill

### 4. Usage

When working on book generation tasks in Cursor:

1. **Read the workflow**: Start by reading `.cursor/workflows/primary-workflow.md` to understand the pipeline
2. **Read agent definition**: When acting as a specific agent, read `.cursor/agents/[agent-name].md`
3. **Read skill definition**: If the agent has assigned skills, read `.cursor/skills/[skill-name]/SKILL.md`
4. **Check prerequisites**: Always check for required inputs from previous agents (read `_current.json` files)
5. **Follow the sequence**: Execute agents in the mandatory sequential order
6. **Version outputs**: Create versioned outputs following `.cursor/workflows/development-rules.md`

## Key Differences from Claude Code CLI

1. **Directory Path**: Use `.cursor/` instead of `.claude/` for all references
2. **No Native Agent System**: Cursor doesn't have native agent invocation, so you must manually follow the workflow
3. **Same Structure**: All workflows, agents, and skills work exactly the same way
4. **Same Rules**: All versioning, quality gates, and orchestration rules apply identically

## Workflow Enforcement

The `.cursorrules` file ensures that:
- Agent workflows are followed in the mandatory sequential pipeline
- Skills are applied when required
- Prerequisites are validated before proceeding
- Versioning protocol is followed
- Quality gates are checked

## Integration with Existing Structure

This setup integrates seamlessly with:
- `.specify/memory/constitution.md` - Project principles
- `specs/` - Feature specifications
- `history/prompts/` - Prompt History Records
- `.book-generation/` - Book generation outputs (same structure)

## Example Usage

**User**: "I need to generate content for Chapter 3 on Physics Engines"

**Cursor Agent should**:
1. Check if research exists → If not, act as `research-agent` first
2. Check if outline exists → If not, act as `outliner-agent` next
3. Continue through the pipeline: `chapter-structure-architect` → `lesson-planner` → `writer-agent` → `book-editor`
4. At each step, read the relevant agent definition and skill (if applicable)
5. Create versioned outputs following the structure defined in workflows

## Validation

Before proceeding as any agent:
- ✅ Read agent definition from `.cursor/agents/[agent-name].md`
- ✅ Read assigned skill from `.cursor/skills/[skill-name]/SKILL.md` (if applicable)
- ✅ Check prerequisites (read `_current.json` files)
- ✅ Follow workflow sequence from `.cursor/workflows/primary-workflow.md`
- ✅ Create versioned outputs following `.cursor/workflows/development-rules.md`

## Support

For detailed information, see:
- `.cursor/workflows/primary-workflow.md` - Complete workflow definition
- `.cursor/workflows/orchestration-protocol.md` - Agent coordination rules
- `.cursor/workflows/development-rules.md` - Quality standards and versioning
- `CURSOR.md` - Main configuration file
- `.cursorrules` - Workflow enforcement rules


