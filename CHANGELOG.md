# Changelog

All notable changes to the SIckKinematiX project will be documented in this file.

## [1.0.0] - 2026-06-10
### Added
- Core kinematics computation engine implemented in pure C.
- Robust Forward Kinematics chain calculation via Modified Denavit-Hartenberg (MDH) parameters.
- Position-only Inverse Kinematics solver using the Cyclic Coordinate Descent (CCD) algorithm.
- Complete 6-DoF Pose Inverse Kinematics solver using the Damped Least Squares (DLS) method with singularity damping.
- Random Monte-Carlo workspace analysis evaluating the Yoshikawa Manipulability Index.
- High-performance multi-threading optimization via OpenMP (supported on Linux environments).
- Interactive 3D visualization control panel built with Plotly Dash.
- Automated pytest suite covering data boundaries, edge cases, and argument validation.
- Multi-OS build testing suite utilizing GitHub Actions CI/CD workflows.