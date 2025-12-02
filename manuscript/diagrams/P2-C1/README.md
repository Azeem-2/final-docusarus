# Chapter P2-C1 Diagrams: Mechanical Structures

**Chapter**: P2-C1 - Mechanical Structures
**Created**: 2025-11-30
**Total Diagrams**: 5
**Format**: Mermaid.js (.mmd)

---

## Diagram Inventory

| Figure # | Filename | Type | Purpose |
|----------|----------|------|---------|
| Figure 1 | `figure-1-joint-types.mmd` | Comparison | Show revolute, prismatic, spherical joints with motion characteristics |
| Figure 2 | `figure-2-serial-parallel.mmd` | Architecture | Compare serial vs parallel mechanisms with trade-offs |
| Figure 3 | `figure-3-urdf-tree.mmd` | Tree Structure | Demonstrate URDF hierarchical link-joint relationships |
| Figure 4 | `figure-4-physical-sim-mapping.mmd` | Flowchart | Complete CAD → Simulation → Build workflow |
| Figure 5 | `figure-5-materials.mmd` | Decision Matrix | Material properties and selection criteria |

---

## Rendering Instructions

### For Markdown Previews
All `.mmd` files contain complete Mermaid code blocks and can be rendered directly in Markdown-compatible viewers (GitHub, VS Code with Mermaid extension, Obsidian).

### For Book Publication
1. Use Mermaid CLI to convert to SVG:
   ```bash
   mmdc -i figure-1-joint-types.mmd -o figure-1-joint-types.svg
   ```

2. Or use Mermaid Live Editor: https://mermaid.live/

3. Verify compliance with style guide:
   - Colors: Blue (#0066CC), Green (#00CC66), Orange (#FF9900), Gray (#666666)
   - Font size: ≥12pt
   - Grayscale readability: Test by converting to grayscale

---

## Style Compliance

✅ **Color Palette**: All diagrams use standard colors from `.book-generation/style-guide/diagrams.md`
✅ **Semantic Colors**:
  - Blue (#0066CC): Physical robotics components
  - Green (#00CC66): Simulation environments
  - Orange (#FF9900): AI/ML components
  - Gray (#666666): General framework elements

✅ **Notation Standards**:
  - Solid arrows (`-->`) for data flow
  - Dashed arrows (`-.->`) for control flow/relationships
  - Labeled connections where appropriate

✅ **Labeling**:
  - All components have clear labels
  - Figure captions follow format: "**Figure N: Title**"
  - Multi-line labels for detailed information

✅ **Readability**:
  - Complexity <20 nodes per diagram
  - Clear hierarchy (top-down or left-right flow)
  - No overlapping text or unlabeled connections

---

## Integration with Chapter

Each diagram corresponds to content in `draft.md`:

- **Figure 1**: Section 5.2 (Joint Types and Their Characteristics)
- **Figure 2**: Section 5.1 (Understanding Robot Morphologies - Serial vs Parallel)
- **Figure 3**: Section 6.2 (URDF - Unified Robot Description Format)
- **Figure 4**: Section 7.1 (The Physical-to-Simulation Pipeline)
- **Figure 5**: Section 5.3 (Materials and Manufacturing: Strength-to-Weight Trade-offs)

---

## Accessibility Notes

All diagrams are designed for:
- **Grayscale compatibility**: Distinguishable without color (patterns, labels, line styles)
- **Screen reader support**: Descriptive captions and labels
- **Print-friendly**: High contrast, readable at 100% zoom

---

## Maintenance

To update diagrams:
1. Edit `.mmd` source files directly
2. Test rendering in Mermaid Live Editor
3. Verify style guide compliance (color palette, font size, complexity)
4. Update this README if new diagrams are added

---

## Contact

For diagram-related questions or updates, reference:
- Style guide: `.book-generation/style-guide/diagrams.md`
- Chapter content: `.book-generation/drafts/P2-C1/v002/draft.md`
- Constitution: `.specify/memory/constitution.md` (Article 10: Visualization Requirements)
