# Agent Prompt & Workflow Improvements After Pilot (P1-C1 … P7-C1)

**Feature**: 1-robotics-book-spec  
**Date**: 2025-12-01  
**Scope**: Refinements to agent prompts and usage patterns based on the 7 pilot chapters (P1-C1 … P7-C1). No breaking changes; all improvements are backward‑compatible.

---

## research-agent

**Observations from Pilot**
- Met or exceeded the “≥10 Tier 1 sources, 0 Wikipedia” requirement for all pilot chapters.
- Produced good synthesis sections, but in‑chapter drafts sometimes lacked explicit in‑text attributions to specific Tier 1 sources.

**Improvements**
- **Prompt emphasis**:  
  - Add guidance that, for each major claim, at least one Tier 1 source should be explicitly named (e.g., “According to IFR’s World Robotics report…”).  
  - Encourage short “citation snippets” that the writer‑agent can copy directly into prose.
- **Output format tweak**:  
  - Under each key finding, include a **“Suggested in‑text citation”** line, e.g.:  
    - `Suggested in‑text: "IFR (World Robotics, 2024) reports that …"`  

---

## outliner-agent

**Observations from Pilot**
- Correctly produced 14‑section outlines plus any chapter‑specific extras.
- For synthesis chapters (P7‑C1), some application sections and mini‑cases were only loosely connected to metrics and validation gates.

**Improvements**
- **Prompt emphasis**:  
  - For each section that introduces real‑world applications or case studies, require at least one **placeholder for quantitative metrics** (e.g., throughput, scrap reduction, safety incidents) and a note about which validator(s) apply (dual‑domain, citation, safety, etc.).
- **Structure hints**:  
  - When chapter type is “Synthesis/Survey,” explicitly reserve a section for **“Patterns & Metrics”** so writer‑agent is nudged toward more data‑backed narrative.

---

## chapter-structure-architect

**Observations from Pilot**
- 4‑layer progression (Foundation, Application, Analysis, Synthesis) worked well across P1–P7 pilots.
- AI touchpoint mapping (6 touchpoints) is strongest in more technical/project chapters; synthesis chapters use them more lightly.

**Improvements**
- **Prompt emphasis**:  
  - For non‑technical/synthesis chapters, explicitly allow **“lightweight” SDD‑RI challenges** (e.g., spec for policy, diagrams, or analysis) so the framework is maintained without forcing code.  
  - Ask the agent to identify **which RI components** (prompt patterns/skills) are most reusable across professional/career chapters.

---

## lesson-planner

**Observations from Pilot**
- Generated detailed 6‑part lesson blueprints with AI touchpoints.
- Sometimes left citation strategy implicit, relying on writer‑agent to recall research file.

**Improvements**
- **Prompt emphasis**:  
  - Add a mini‑section in each lesson plan’s Theory or Walkthrough part: **“Citations to surface”**, listing 2–3 Tier 1 sources by name and how they should appear in text.  
  - In SDD‑RI challenge descriptions, remind that **specs should reference at least one Tier 1 source** when appropriate (e.g., standard, report, textbook).

---

## writer-agent

**Observations from Pilot**
- Produces clear, engaging prose at the right technical level.
- Occasionally under‑uses the available Tier 1 citations in the research output (claims are correct but unattributed).

**Improvements**
- **Prompt emphasis**:  
  - Require that each major section (Introduction, Core Concepts, Applications, etc.) includes at least **one explicit Tier 1 attribution** drawn from research output.  
  - Remind the agent to check the **“Citations to surface”** hints in lesson plans and convert them into inline references (not formal bibliography).
- **Workflow hint**:  
  - When generating drafts, have the writer‑agent create a short **“Citations Checklist”** at the bottom of the draft that lists which Tier 1 sources were used in the prose; this aids the book-editor pass.

---

## book-editor

**Observations from Pilot**
- Multi‑pass review caught structural and citation issues effectively.
- Citation pass detected missing explicit references but had to rely on manual checking.

**Improvements**
- **Prompt emphasis**:  
  - In Pass 3 (Citations), explicitly ask: *“For each major claim, is at least one Tier 1 source named in text? If not, suggest a concrete phrasing.”*  
  - Encourage marking suggested citation insertions as bullet points in the Issues section for easier iteration by writer‑agent.

---

## diagram-generator (informal)

**Observations from Pilot**
- Diagrams were consistent where created; a few chapters (P6‑C1, P7‑C1) benefited greatly from high‑level architecture and stakeholder maps.

**Improvements**
- **Prompt emphasis**:  
  - For each chapter, ask lesson‑planner to propose **at least one diagram that is clearly tied to an assessment activity** (e.g., concept map, architecture that students annotate), not just for illustration.


