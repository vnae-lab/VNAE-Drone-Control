# VNAE-Drone-Control
Practical example of the Victoria-Nash Asymmetric Equilibrium (VNAE) applied to multi-agent drone control. Paper: "Riemmanian Manifolds of Asymmetric Equilibria: The Victoria-Nash Geometry" by DH Pereira (2026).

This implementation in R focuses on the **software-level control logic**
and **geometric stability structure** of VNAE-based multi-agent systems.

It is intended as a **reference and starting point** for control,
automation, and robotics engineers, who may adapt the dynamics,
constraints, sensing, and actuation layers to their specific
hardware platforms and real-world conditions.

# Features
- Quadratic/Canonical VNAE formulation
- Multi-agent (10 drones) control
- Curvature-based stability diagnostics
- Parameter sensitivity analysis
- 2D and 3D visualizations

# Requirements
- R >= 4.0
- Packages:
  - ggplot2
  - plotly
  - rgl
  - gridExtra

# Reference

Pereira, D. H. (2025). Riemannian Manifolds of Asymmetric Equilibria: The Victoria-Nash Geometry.

