# Agent Orchestration Protocol

**Version**: 1.0.0  
**Last Updated**: 2025-11-26  
**Purpose**: Define how agents coordinate, communicate, and hand off work to each other

## Core Principles

### 1. Agent Autonomy with Coordination

Each agent operates autonomously within its domain but must:
- Check for prerequisite outputs from previous agents
- Validate input completeness before proceeding
- Signal completion and suggest next agent in sequence
- Never duplicate work already done by another agent

### 2. Skill Application Mandate

Agents with assigned skills MUST:
- Reference skill file explicitly (e.g., "MUST reference `.cursor/skills/prose-generation/SKILL.md`")
- Follow skill procedures exactly without modification
- NOT duplicate skill content in agent files
- Report skill application in version metadata

### 3. Versioned Output Chain

All agents produce versioned outputs that form a traceable chain. **ALL 6 agents are MANDATORY in sequence**:

```
research-agent → .book-generation/research/[topic]/v[NNN]/
    ↓
outliner-agent → .book-generation/outlines/[book]/v[NNN]/
    ↓
chapter-structure-architect → .book-generation/structures/[chapter]/v[NNN]/
    ↓
lesson-planner → .book-generation/lessons/[chapter]/v[NNN]/
    ↓
writer-agent → .book-generation/drafts/[chapter]/v[NNN]/
    ↓
book-editor → .book-generation/reviews/[chapter]/v[NNN]/
```

Each agent must:
- Read previous agent's `_current.json` to find latest version
- Reference previous versions in its own `version.json`
- Never overwrite previous versions (always increment)

---

## Agent Handoff Protocol

### Handoff: research-agent → outliner-agent

**Trigger**: Research complete, user requests structure

**research-agent Responsibilities**:
1. Complete research within 3-4 hour target
2. Save to `.book-generation/research/[topic]/v[NNN]/research.md`
3. Create `version.json` with source counts and quality metrics
4. Update `_current.json` pointer
5. **Signal**: "Research complete. Ready for outlining. Suggest invoking outliner-agent."

**outliner-agent Responsibilities**:
1. Check for research in `.book-generation/research/[topic]/_current.json`
2. Read latest research version
3. Verify research completeness (10+ sources, Tier 1/2 only)
4. If research incomplete, request additional research before proceeding
5. Use research to inform outline structure

**Handoff Message Format**:
```markdown
✅ Research Phase Complete
- Topic: [topic-slug]
- Sources: [X] Tier 1, [Y] Tier 2
- Research Path: `.book-generation/research/[topic]/v[NNN]/`
- Next: Invoke `outliner-agent` to create book structure
```

---

### Handoff: outliner-agent → chapter-structure-architect

**Trigger**: Outline complete (MANDATORY next step)

**outliner-agent Responsibilities**:
1. Create complete book/chapter outline
2. Save to `.book-generation/outlines/[book]/v[NNN]/outline.md`
3. Create `version.json` with chapter/section counts
4. Update `_current.json` pointer
5. **Signal**: "Outline complete. Ready for chapter structure design. Suggest invoking chapter-structure-architect."

**chapter-structure-architect Responsibilities**:
1. Check for outline in `.book-generation/outlines/[book]/_current.json`
2. Read latest outline version
3. For EACH chapter in outline:
   - Analyze concept density
   - Apply 8-9 lesson formula for standard chapters
   - Map 4-layer pedagogical progression
   - Define 5 AI integration touchpoints per lesson
4. Save structural blueprint to `.book-generation/structures/[chapter]/v[NNN]/structure.md`
5. Create `version.json` with lesson counts and layer mappings
6. Update `_current.json` pointer

**Handoff Message Format**:
```markdown
✅ Outline Phase Complete
- Book: [book-slug]
- Chapters: [N]
- Outline Path: `.book-generation/outlines/[book]/v[NNN]/`
- Research Path: `.book-generation/research/[topic]/v[NNN]/` (if applicable)
- Next: Invoke `chapter-structure-architect` to design lesson frameworks for each chapter
```

---

### Handoff: chapter-structure-architect → lesson-planner

**Trigger**: Chapter structure blueprint complete (MANDATORY next step)

**chapter-structure-architect Responsibilities**:
1. Create structural blueprint (NOT content)
2. Define lesson framework with AI touchpoints
3. Map 4-layer pedagogical progression
4. Save to `.book-generation/structures/[chapter]/v[NNN]/structure.md`
5. Create `version.json` with lesson counts and layer mappings
6. Update `_current.json` pointer
7. **Signal**: "Chapter structure complete. Ready for lesson content. Suggest invoking lesson-planner."

**lesson-planner Responsibilities**:
1. Check for structure in `.book-generation/structures/[chapter]/_current.json`
2. Read latest structure version
3. Verify blueprint completeness (all lessons defined, AI touchpoints mapped)
4. If blueprint incomplete, request completion before proceeding
5. For EACH lesson in structure:
   - Implement 5-part template (Diagnostic Hook → Concept Theory → Walkthrough → Iterative Challenge → Spaced-Repetition)
   - Apply master methodologies (Visual Intuition First, Stepwise Scaffolding, Pattern-Based Chunking)
   - Integrate all 5 AI touchpoints at defined positions
6. Save to `.book-generation/lessons/[chapter]/v[NNN]/lessons.md`
7. Create `version.json` with lesson details
8. Update `_current.json` pointer

**Handoff Message Format**:
```markdown
✅ Chapter Structure Complete
- Chapter: [chapter-title]
- Lessons: [N] (justified by concept density)
- Pedagogical Layers: [Layer mapping]
- Structure Path: `.book-generation/structures/[chapter]/v[NNN]/`
- Next: Invoke `lesson-planner` to create complete lesson content
```

---

### Handoff: lesson-planner → writer-agent

**Trigger**: Lesson content complete (MANDATORY next step)

**lesson-planner Responsibilities**:
1. Complete all lesson content for the chapter
2. Save to `.book-generation/lessons/[chapter]/v[NNN]/lessons.md`
3. Create `version.json` with lesson details and AI touchpoint mappings
4. Update `_current.json` pointer
5. **Signal**: "Lesson content complete. Ready for prose generation. Suggest invoking writer-agent."

**writer-agent Responsibilities**:
1. Check for lessons in `.book-generation/lessons/[chapter]/_current.json`
2. Read latest lesson version
3. Check for structure in `.book-generation/structures/[chapter]/_current.json`
4. Check for outline in `.book-generation/outlines/[book]/_current.json`
5. Check for research in `.book-generation/research/[topic]/_current.json`
6. Verify lesson content completeness (all 5 parts present, AI touchpoints integrated)
7. If lesson content incomplete, request completion before proceeding
8. Apply `prose-generation` skill to transform lesson content into publication-ready prose
9. Maintain voice consistency with previous chapters

**Handoff Message Format**:
```markdown
✅ Lesson Content Complete
- Chapter: [chapter-title]
- Lessons: [N]
- Lessons Path: `.book-generation/lessons/[chapter]/v[NNN]/`
- Structure Path: `.book-generation/structures/[chapter]/v[NNN]/`
- Next: Invoke `writer-agent` with `prose-generation` skill to create chapter draft
```

---

### Handoff: writer-agent → book-editor

**Trigger**: Draft complete (MANDATORY next step)

**writer-agent Responsibilities**:
1. Complete chapter draft (no placeholders)
2. Apply `prose-generation` skill throughout
3. Save to `.book-generation/drafts/[chapter]/v[NNN]/draft.md`
4. Create `version.json` with word count, Flesch score, citation count
5. Update `_current.json` pointer
6. **Signal**: "Draft complete. Ready for editorial review. Suggest invoking book-editor."

**book-editor Responsibilities**:
1. Check for draft in `.book-generation/drafts/[chapter]/_current.json`
2. Read latest draft version
3. Check for research in `.book-generation/research/[topic]/_current.json` (for citation verification)
4. Verify draft completeness (all sections written, no placeholders)
5. If draft incomplete, request completion before editing
6. Apply `content-editing` skill's five-pass system

**Handoff Message Format**:
```markdown
✅ Writing Phase Complete
- Chapter: [chapter-slug]
- Word Count: [N]
- Voice: [perspective] / [tone]
- Draft Path: `.book-generation/drafts/[chapter]/v[NNN]/`
- Research Path: `.book-generation/research/[topic]/v[NNN]/` (for citation verification)
- Next: Invoke `book-editor` with `content-editing` skill for editorial review
```

---

### Handoff: book-editor → writer-agent (Revision Loop)

**Trigger**: Editorial review identifies issues requiring revision

**book-editor Responsibilities**:
1. Complete five-pass editorial review
2. Save to `.book-generation/reviews/[chapter]/v[NNN]/review.md`
3. Create `version.json` with change counts and quality metrics
4. Update `_current.json` pointer
5. **Signal**: "Review complete. [X] blocking issues found. Suggest invoking writer-agent for revisions."

**writer-agent Responsibilities**:
1. Read editorial review from `.book-generation/reviews/[chapter]/_current.json`
2. Read original draft from `.book-generation/drafts/[chapter]/_current.json`
3. Address all blocking issues
4. Create new draft version (increment version number)
5. **Signal**: "Revision complete. Suggest invoking book-editor for re-review."

**Revision Loop Rules**:
- Maximum 3 revision cycles per chapter
- If issues persist after 3 cycles, escalate to user
- Track revision history in `version.json.comparison`

---

## Cross-Agent Validation

**MANDATORY PIPELINE**: All 6 agents must run in sequence. Each agent validates outputs from ALL previous agents.

### research-agent Validates: Nothing (first in chain)

### outliner-agent Validates:
- [ ] Research exists at `.book-generation/research/[topic]/_current.json`
- [ ] Research has minimum 10 Tier 1/2 sources per major section
- [ ] Research versioned and accessible

### chapter-structure-architect Validates:
- [ ] Outline exists at `.book-generation/outlines/[book]/_current.json`
- [ ] Outline has all chapters defined with purpose statements
- [ ] Research exists at `.book-generation/research/[topic]/_current.json`

### lesson-planner Validates:
- [ ] Structure exists at `.book-generation/structures/[chapter]/_current.json`
- [ ] All lessons defined with AI touchpoints
- [ ] 4-layer pedagogical progression mapped
- [ ] Outline and research accessible

### writer-agent Validates:
- [ ] Lessons exist at `.book-generation/lessons/[chapter]/_current.json`
- [ ] All 5 lesson parts present (Diagnostic Hook, Concept Theory, Walkthrough, Iterative Challenge, Spaced-Repetition)
- [ ] All AI touchpoints integrated
- [ ] Structure, outline, and research accessible
- [ ] Previous chapter drafts exist (for voice consistency)

### book-editor Validates:
- [ ] Draft exists at `.book-generation/drafts/[chapter]/_current.json`
- [ ] Draft is complete (no placeholders)
- [ ] Lessons, structure, outline, and research accessible
- [ ] Previous chapter reviews exist (for cross-chapter consistency)

---

## Skill Application Verification

Each agent must verify skill application:

### research-agent + research-methodology
- [ ] Skill file referenced explicitly
- [ ] Tier 1/2 source standards followed
- [ ] 3-4 hour time target met
- [ ] NO Wikipedia or user-editable platforms used
- [ ] Citation format matches skill requirements

### writer-agent + prose-generation
- [ ] Skill file referenced explicitly
- [ ] Voice configuration applied (perspective, tone, technical level)
- [ ] Paragraph structure follows 4-part pattern
- [ ] Transition techniques applied
- [ ] Chapter flow patterns followed
- [ ] Readability targets met (Flesch score)

### book-editor + content-editing
- [ ] Skill file referenced explicitly
- [ ] All five passes completed
- [ ] Quality metrics calculated (Flesch, passive voice %, citation coverage)
- [ ] Consistency checks performed (Grep for terminology)
- [ ] Edit summary report generated

---

## Error Recovery

### Missing Prerequisites

**If agent invoked without required inputs**:

1. **Identify missing inputs**: List specific files/versions needed
2. **Suggest previous agent**: "Run [previous-agent] first to generate [required-output]"
3. **Do NOT proceed**: Never generate placeholder or incomplete outputs
4. **Wait for user**: Do not auto-invoke previous agent (user must approve)

### Version Conflicts

**If multiple versions exist**:

1. **Read `_current.json`**: Always use the version pointed to by `_current.json`
2. **If `_current.json` missing**: Use highest version number
3. **If version number conflict**: Report to user, do not guess

### Skill Application Failures

**If agent fails to apply skill correctly**:

1. **Re-read skill file**: Agent must reference skill file again
2. **Verify procedures**: Check each step in skill against agent output
3. **Re-apply skill**: Generate corrected output with proper skill application
4. **Document in version.json**: Note skill application issues in metadata

---

## Proactive Agent Suggestions

Agents should proactively suggest next steps:

### research-agent Suggestions
- "Research complete. Suggest invoking `outliner-agent` to structure this research into a book outline."
- "Research gaps identified: [list]. Suggest additional research before outlining."

### outliner-agent Suggestions
- "Outline complete. Suggest invoking `writer-agent` to begin chapter drafts."
- "Outline reveals research gaps: [list]. Suggest invoking `research-agent` for additional sources."

### chapter-structure-architect Suggestions
- "Chapter structure complete. Suggest invoking `lesson-planner` to create lesson content."
- "Structure reveals specification gaps: [list]. Suggest updating chapter spec before content creation."

### lesson-planner Suggestions
- "Lesson content complete. Suggest invoking `book-editor` for quality review."
- "Content reveals structural issues: [list]. Suggest reviewing chapter structure blueprint."

### writer-agent Suggestions
- "Draft complete. Suggest invoking `book-editor` for editorial review."
- "Draft reveals research gaps: [list]. Suggest invoking `research-agent` for additional sources."

### book-editor Suggestions
- "Review complete. [X] blocking issues found. Suggest invoking `writer-agent` for revisions."
- "Review complete. No blocking issues. Content ready for publication."

---

## Version History

- **1.0.0** (2025-11-26): Initial orchestration protocol with handoff rules and validation procedures

---

**Related Documents**:
- `.cursor/workflows/primary-workflow.md` - Main workflow definition
- `.cursor/workflows/development-rules.md` - Development standards
- `.specify/memory/constitution.md` - Project principles

