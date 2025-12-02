# P2-C7 – Control Systems – Diagram Pack

This folder contains the primary diagrams for **Part 2, Chapter 7: Control Systems**.  
All diagrams follow the global style guide in `.book-generation/style-guide/diagrams.md` and the Mermaid theme in `.book-generation/style-guide/mermaid-theme.json`.

## Diagram List

1. `figure-1-feedback-loop.mmd`  
   - **Concept**: Basic feedback control loop for a robot joint (reference, error, controller, plant, measurement).  
   - **Usage**: Early in the chapter when introducing feedback and block diagrams.

2. `figure-2-pid-effects.mmd`  
   - **Concept**: Qualitative comparison of P, PI, and PID responses to a step input (intuition only).  
   - **Usage**: In the PID control section to motivate the roles of P, I, and D.

3. `figure-3-saturation-and-limits.mmd`  
   - **Concept**: Actuator saturation, clipped commands, and conceptual impact on behavior (e.g., integrator windup).  
   - **Usage**: In the tuning/limits section to reinforce real-world constraints.

4. `figure-4-robust-vs-fragile-control.mmd`  
   - **Concept**: Contrast between a high-gain, fragile controller and a moderate-gain, robust controller under parameter changes.  
   - **Usage**: In the robustness/safety section to visualize stable vs unstable behavior under uncertainty.

## Notes

- These diagrams are **conceptual**, not precise plots; they are meant to build intuition about feedback, PID behavior, saturation, and robustness.  
- When exporting for print, convert these Mermaid sources to SVG and ensure labels remain legible in grayscale.  


