# Pilot Retrospective: P1-C1 … P7-C1

**Feature**: 1-robotics-book-spec  
**Date**: 2025-12-01  
**Scope**: Retrospective on Phase 2 Pilot Chapters (P1-C1 to P7-C1)

---

## 1. Pilot Summary

**Pilot Goal**: Validate the 6‑agent pipeline, versioned outputs, validators, and diagram workflow across 7 representative chapters (one from each part).

**Chapters**:
- P1-C1: What is Physical AI – PUBLISHED (v002, 97/100)
- P2-C1: Mechanical Structures – PUBLISHED (95/100)
- P3-C1: Physics Engines – PUBLISHED (92/100)
- P4-C1: Vision Models – PUBLISHED (92/100)
- P5-C1: Humanoid Kinematics & Dynamics – REVIEW COMPLETE (92/100, ApprovedWithMinorRevisions)
- P6-C1: Build a Mobile Robot – REVIEW COMPLETE (91/100, ApprovedWithMinorRevisions)
- P7-C1: Industry Applications – REVIEW COMPLETE (90/100, ApprovedWithMinorRevisions)

All chapters successfully passed constitutional, dual‑domain, and safety validation gates as designed.

---

## 2. Quality Metrics (T131)

### 2.1 Readability and Structure

- All pilot chapters fall within or near the **target Flesch–Kincaid Grade 12–14** range for technical content.  
- Structural compliance with the **14 mandatory sections** is high:
  - P1–P4: Fully compliant (all 14 sections present).  
  - P5–P7: Compliant with minor adaptations (e.g., synthesis chapters use lighter lab sections but maintain dual‑domain explanation and applications).
- Lesson‑planner outputs consistently generate the **6‑part lesson template** (Hook, Theory, Walkthrough, Challenge, Takeaways, Learn with AI).

### 2.2 Dual-Domain Balance

- All pilot chapters achieve **dual‑domain coverage ≥ 0.7** (physical + simulation) per validators:
  - P2/P5 emphasize physical content but include simulation perspectives.  
  - P3/P4 emphasize simulation and AI but tie back to physical robots and real‑world constraints.  
  - P6 balances mobile robot simulation and hardware implementation.  
  - P7 links real‑world deployments back to both physical and simulated/digital‑twin concepts.

### 2.3 Citations and Academic Integrity

- Each pilot chapter includes **≥ 10 Tier 1 sources**, with no Wikipedia or user‑editable sources.
- Citation validator passes for:
  - URL accessibility (HTTP 200),  
  - IEEE‑style bibliography (where present),  
  - Tier 1/Tier 2 ratios.
- Book‑editor reviews revealed that **explicit in‑text attributions** (e.g., naming IFR or a specific survey) are occasionally underused, which has been addressed in `agent-improvements.md`.

---

## 3. Pipeline and Agent Behavior (T132)

### 3.1 6-Agent Pipeline Robustness

The mandatory pipeline:

```text
research-agent → outliner-agent → chapter-structure-architect → lesson-planner → writer-agent → book-editor
```

worked as intended for all seven pilot chapters:

- Versioned directories (`v001`, `v002`, …) were correctly created for research, outlines, structures, lessons, drafts, and reviews.
- `_current.json` pointers were maintained consistently.
- Book-editor passes (structural, content, citation, consistency, factual) surfaced actionable issues without blocking progress.

### 3.2 Prompt & Workflow Refinements

Key refinements captured in `.book-generation/agent-improvements.md`:

- **research-agent**: Emphasize suggested in‑text citations for major claims (not just bibliography entries).  
- **outliner-agent**: For application/synthesis chapters, require metric placeholders and clearer linkage to validators.  
- **chapter-structure-architect**: Clarify how to use the 4‑layer progression and 6 AI touchpoints for non‑code chapters.  
- **lesson-planner**: Add “Citations to surface” hints in each lesson’s theory/walkthrough.  
- **writer-agent**: Require explicit Tier 1 attributions per major section; add a simple citations checklist.  
- **book-editor**: In the citation pass, explicitly check for missing named sources and propose phrasing.

---

## 4. Validation Thresholds (T133)

### 4.1 Existing Thresholds

- **Readability**: Target FK Grade 12–14 (warning only if outside range; not blocking).  
- **Dual‑domain balance**: Required ratio ≥ 0.7 for each chapter.  
- **Citations**: ≥ 10 Tier 1 sources, 0 excluded sources (Wikipedia, user‑editable, anonymous).  
- **Safety**: Physical labs must include explicit hazard warnings and emergency stop descriptions.

### 4.2 Pilot Findings

- Readability and dual‑domain thresholds worked well; chapters that deviated slightly were still acceptable after editor feedback.  
- Citation and safety validators correctly flagged issues when present; no need to relax constraints.

### 4.3 Decision

- **No numeric threshold changes are required** at this time.  
- Instead, we tightened **agent prompts and editorial checks** (see Section 3 and `agent-improvements.md`) to improve:
  - Explicit in‑text citation usage.  
  - Integration between lesson plans and validator expectations.

This satisfies T133 by confirming thresholds and documenting that they remain appropriate given pilot outcomes.

---

## 5. Lessons Learned (T134)

### 5.1 What Worked Well

- The 6‑agent pipeline is **stable, repeatable, and scalable** across very different chapter types (conceptual, technical, project‑based, synthesis).  
- Versioned outputs and `_current.json` made it easy to iterate without losing history.  
- Validators (constitutional, dual‑domain, citation, safety, readability, diagram style) caught real issues early.  
- The lesson‑planner’s 6‑part template and AI touchpoints created a consistent pedagogical structure that the writer‑agent could follow.

### 5.2 Pain Points and Mitigations

- **Citation explicitness**:  
  - Pain: Drafts sometimes relied on generic phrases (“studies show”) instead of naming Tier 1 sources.  
  - Mitigation: Updated research/lesson/writer/editor prompts to surface and enforce explicit attributions.

- **Synthesis chapters and labs**:  
  - Pain: Some synthesis/professional chapters (e.g., P7‑C1) don’t naturally include formal labs.  
  - Mitigation: Allow **lightweight AI‑driven activities and RI components** in place of full labs while preserving dual‑domain and assessment structure.

- **Diagram planning**:  
  - Pain: Diagrams were sometimes specified late in the pipeline.  
  - Mitigation: Encourage the lesson‑planner and outliner to propose diagrams tied to concrete learning activities early.

### 5.3 Recommendations for Parts 1–7 Production

- Continue using the same **validation thresholds**, but:
  - Be stricter in Go/No‑Go checks for missing explicit Tier 1 attributions in content‑heavy chapters.  
  - Make sure each chapter defines at least one **RI component** (prompt/skill) that students can reuse.
- For each new chapter:
  - Confirm up front which **diagram types** are required and assign them in lesson planning.  
  - Treat the pilot outputs (especially P1‑C1 and P6‑C1) as **reference patterns** for structure, tone, and dual‑domain integration.

---

## 6. Next Steps

- Use these retrospective findings to guide:
  - **Phase 3–9** chapter generation (Parts 1–7 production).  
  - Future `/sp.implement` runs, starting with T135–T138 (Part 1 production) when ready.  
- Periodically re‑run a **mini‑retrospective** after finishing each major part (e.g., after Parts 1, 2–3, 4–5, 6–7) to keep prompts and validators aligned with observed behavior.


