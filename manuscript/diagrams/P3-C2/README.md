# P3-C2 – Environment Modeling – Diagram Pack

This folder contains diagrams for **Part 3, Chapter 2: Environment Modeling**.  
All diagrams follow the global style guide in `.book-generation/style-guide/diagrams.md` and the Mermaid theme in `.book-generation/style-guide/mermaid-theme.json`.

## Diagram List

1. `figure-1-basic-scene.mmd`  
   - **Concept**: Simple top-down environment for a mobile robot (floor, walls, obstacles, goal).  
   - **Usage**: Early in the chapter when introducing basic scene composition and friction choices.

2. `figure-2-collision-vs-visual.mmd`  
   - **Concept**: Distinction between visual meshes and simplified collision geometry.  
   - **Usage**: In the geometry/materials section to motivate separate representations for rendering vs physics.

3. `figure-3-perception-views.mmd`  
   - **Concept**: How a single environment appears from a top-down view vs a camera image, including background clutter.  
   - **Usage**: In the perception-focused section to illustrate why textures, lighting, and layout matter.

4. `figure-4-domain-randomization.mmd`  
   - **Concept**: A base scene spawning multiple randomized variants (textures, lighting, object positions).  
   - **Usage**: In the domain randomization section to visualize environment-level variability across episodes.

## Notes

- These diagrams are **conceptual** and are meant to support intuition about environment composition, perception, and randomization.  
- When exporting for print, convert Mermaid outputs to SVG and verify that labels remain legible in grayscale.


