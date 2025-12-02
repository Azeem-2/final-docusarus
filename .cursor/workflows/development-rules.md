# Development Rules

**Version**: 1.0.0  
**Last Updated**: 2025-11-26  
**Purpose**: Define code quality standards, agent orchestration rules, and development practices

## Code Quality Standards

### Constitution Compliance

All development must adhere to `.specify/memory/constitution.md`:

- **Core Principles**: Every feature must align with project principles
- **Quality Gates**: All quality gates must pass before proceeding
- **Testing Requirements**: Test-first development mandatory where applicable
- **Documentation**: All changes must be documented appropriately

### Agent Output Standards

#### Versioning Requirements

**ALL agent outputs MUST be versioned**:

```text
[output-type]/[slug]/v[NNN]/
├── [file].md          # Main output file
├── version.json       # Version metadata
└── _current.json      # Pointer to latest version
```

**Version Numbering**:
- Format: `v001`, `v002`, `v003` (3-digit padded, always increment)
- Increment on: Rewrites, major revisions, feedback incorporation
- Never overwrite: Always create new version directory

#### File Naming Conventions

- **Research**: `.book-generation/research/[topic-slug]/v[NNN]/research.md`
- **Outlines**: `.book-generation/outlines/[book-slug]/v[NNN]/outline.md`
- **Drafts**: `.book-generation/drafts/[chapter-slug]/v[NNN]/draft.md`
- **Reviews**: `.book-generation/reviews/[chapter-slug]/v[NNN]/review.md`

**Slug Rules**:
- Lowercase
- Hyphens for word separation
- No special characters
- Descriptive but concise

#### Metadata Requirements

Every `version.json` MUST include:

```json
{
  "version": "v001",
  "agent": "[agent-name]",
  "created": "ISO-8601 timestamp",
  "[entity]": "[slug]",
  "inputs": { /* referenced inputs */ },
  "outputs": { /* output file info */ },
  "metrics": { /* quality metrics */ },
  "comparison": { /* vs previous version */ },
  "quality_score": { /* calculated scores */ }
}
```

---

## Agent Orchestration Rules

### Rule 1: MANDATORY Sequential Pipeline

**ALL 6 agents MUST be invoked in this exact sequence - NO EXCEPTIONS:**

```
research-agent → outliner-agent → chapter-structure-architect → lesson-planner → writer-agent → book-editor
```

| Phase | Agent | Output Path |
|-------|-------|-------------|
| 1 | `research-agent` | `.book-generation/research/[topic]/v[NNN]/` |
| 2A | `outliner-agent` | `.book-generation/outlines/[book]/v[NNN]/` |
| 2B | `chapter-structure-architect` | `.book-generation/structures/[chapter]/v[NNN]/` |
| 3A | `lesson-planner` | `.book-generation/lessons/[chapter]/v[NNN]/` |
| 3B | `writer-agent` | `.book-generation/drafts/[chapter]/v[NNN]/` |
| 4 | `book-editor` | `.book-generation/reviews/[chapter]/v[NNN]/` |

**No agent may be skipped.** Each agent's output is REQUIRED input for the next agent.

**Exception**: Research can be invoked AGAIN at any time if gaps are identified during later phases.

### Rule 2: Prerequisite Validation

Before proceeding, each agent MUST:

1. **Check for prerequisites**: Verify required inputs exist
2. **Validate completeness**: Ensure inputs are complete (no placeholders)
3. **Read latest version**: Use `_current.json` to find latest version
4. **Report missing inputs**: If prerequisites missing, report clearly and suggest previous agent

**DO NOT**:
- Proceed with incomplete inputs
- Generate placeholder content
- Auto-invoke previous agents (user must approve)

### Rule 3: Skill Application

Agents with assigned skills MUST:

1. **Reference skill explicitly**: "MUST reference `.cursor/skills/[skill-name]/SKILL.md`"
2. **Follow procedures exactly**: Do not modify or skip skill steps
3. **Report application**: Document skill usage in `version.json`
4. **Never duplicate**: Do not copy skill content into agent files

**Skill Assignments**:
- `research-agent` → `research-methodology` (MANDATORY)
- `writer-agent` → `prose-generation` (MANDATORY)
- `book-editor` → `content-editing` (MANDATORY)
- Structural agents → No skills (specialized design)

### Rule 4: Proactive Suggestions

After completing work, agents MUST:

1. **Signal completion**: Clear message indicating phase complete
2. **Suggest next agent**: Recommend which agent to invoke next
3. **Provide context**: Include paths to outputs and any relevant notes
4. **Flag issues**: Report any gaps or concerns discovered

**Format**:
```markdown
✅ [Phase] Complete
- Output: [description]
- Path: `[path-to-output]`
- Next: Invoke `[next-agent]` to [next-action]
```

---

## Pre-Commit Procedures

### Before Committing Agent Outputs

1. **Version Check**: Ensure output is properly versioned
2. **Metadata Check**: Verify `version.json` is complete
3. **Current Pointer**: Ensure `_current.json` points to latest version
4. **Skill Application**: Verify skill was applied correctly (if applicable)
5. **Quality Metrics**: Check that quality targets are met

### PHR Creation

**MANDATORY**: After every user interaction that results in work:

1. **Determine Stage**: `constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general`
2. **Generate Title**: 3-7 words, create slug
3. **Route**: 
   - `constitution` → `history/prompts/constitution/`
   - Feature stages → `history/prompts/<feature-name>/`
   - `general` → `history/prompts/general/`
4. **Create PHR**: Use `.specify/scripts/bash/create-phr.sh` or agent-native tools
5. **Fill Placeholders**: Complete all YAML and body fields
6. **Validate**: No unresolved placeholders, correct path, coherent metadata

**Skip PHR**: Only for `/sp.phr` command itself

### ADR Suggestions

**When to Suggest ADR**:

After design/architecture work, test for significance:

- **Impact**: Long-term consequences? (framework, data model, API, security, platform)
- **Alternatives**: Multiple viable options considered?
- **Scope**: Cross-cutting and influences system design?

**If ALL true**, suggest:
```markdown
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`
```

**DO NOT**:
- Auto-create ADRs (wait for user consent)
- Suggest for trivial decisions
- Group unrelated decisions

---

## Quality Gates

### Research Quality Gate

Before proceeding to outlining:

- [ ] Minimum 10 Tier 1/2 sources per major section
- [ ] 60%+ sources are Tier 1 (highly authoritative)
- [ ] 0 sources from Tier 3 (NO Wikipedia, user-editable platforms)
- [ ] All sources authenticated and verified
- [ ] Research completed within 3-4 hour target
- [ ] Research versioned and saved

### Outline Quality Gate

Before proceeding to writing:

- [ ] Complete book/chapter structure defined
- [ ] All sections have purpose statements
- [ ] Key points listed with research references
- [ ] Prerequisites clearly defined
- [ ] Estimated lengths provided
- [ ] Outline versioned and saved

### Draft Quality Gate

Before proceeding to editing:

- [ ] Complete draft (no placeholders, no "[content here]")
- [ ] Voice consistent throughout
- [ ] Citations integrated (not just appended)
- [ ] Transitions present between sections
- [ ] Chapter opening uses appropriate hook
- [ ] Chapter closing bridges to next chapter
- [ ] Draft versioned and saved

### Editorial Quality Gate

Before marking as publication-ready:

- [ ] All five editorial passes complete
- [ ] Quality metrics meet targets:
  - Flesch Reading Ease > 60
  - Passive Voice < 20%
  - Citation Coverage 100%
- [ ] No blocking issues remain
- [ ] Cross-chapter consistency verified (if applicable)
- [ ] Review versioned and saved

---

## Error Handling

### Missing Prerequisites

**Response**:
1. Identify missing inputs clearly
2. Suggest which previous agent needs to run
3. Provide example: "Run `research-agent` first to generate research notes"
4. Do NOT proceed with incomplete inputs

### Skill Application Failures

**Response**:
1. Re-read skill file
2. Verify procedures against output
3. Re-apply skill correctly
4. Document issue in `version.json`

### Version Conflicts

**Response**:
1. Read `_current.json` (always use pointed version)
2. If `_current.json` missing, use highest version number
3. If conflict persists, report to user (do not guess)

### Quality Gate Failures

**Response**:
1. Report specific failures clearly
2. Provide actionable fixes
3. Do NOT proceed until gates pass (unless user explicitly overrides)
4. Document override in `version.json` if user approves

---

## Documentation Standards

### Agent Documentation

Each agent file (`.cursor/agents/[agent-name].md`) MUST include:

1. **Frontmatter**:
   - `name`: Agent identifier
   - `description`: When to use this agent (with examples)
   - `model`: Model preference (if specified)
   - `color`: UI color (if specified)
   - `skills`: List of assigned skills

2. **Core Mission**: What the agent does
3. **Input Requirements**: What inputs are needed
4. **Output Format**: What outputs are produced
5. **Process**: Step-by-step procedure
6. **Completion Criteria**: Checklist for completion
7. **Error Handling**: How to handle common errors

### Skill Documentation

Each skill file (`.cursor/skills/[skill-name]/SKILL.md`) MUST include:

1. **Frontmatter**:
   - `name`: Skill identifier
   - `description`: What the skill does and when to use it
   - `version`: Version number
   - `tags`: Relevant tags
   - `changelog`: Version history

2. **When to Use**: Clear triggers for skill application
3. **Procedures**: Step-by-step instructions
4. **Standards**: Quality standards and metrics
5. **Examples**: Concrete usage examples
6. **Quality Assurance**: Checklists and validation

---

## Version History

- **1.0.0** (2025-11-26): Initial development rules with agent orchestration and quality gates

---

**Related Documents**:
- `.cursor/workflows/primary-workflow.md` - Main workflow
- `.cursor/workflows/orchestration-protocol.md` - Agent coordination
- `.specify/memory/constitution.md` - Project principles
- `CURSOR.md` - Main Cursor Agent configuration

