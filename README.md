# SIckKinematiX

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![Core Engine](https://img.shields.io/badge/Core_Engine-Pure_C-red.svg)]()
[![CI/CD Pipeline](https://github.com/tomekstolarczyk/SIckKinematiX/actions/workflows/ci.yml/badge.svg)](https://github.com/tomekstolarczyk/SIckKinematiX/actions/workflows/ci.yml)

**A high-performance, universal 6-DoF Manipulator Kinematics Solver built with pure C and wrapped in a Python API.**

---

> 🛑 **STOP! DON'T READ RAW CODE!**
>
> All the essential project details, including key features, complete setup steps, and the full Python API Reference, are hosted on our webiste.
>
> 🌐 **[VISIT THE OFFICIAL HOMEPAGE & DOCUMENTATION](https://tomekstolarczyk.github.io/SIckKinematiX/)**

---

## Technical Features Overview
* **Custom Computational Core:** A proprietary matrix math library (4x4 and 6x6 operations, Gauss-Jordan inversion, and 3D vector math) written entirely from scratch in pure C for absolute efficiency.
* **Universal MDH Engine:** Dynamically parses Modified Denavit-Hartenberg parameters for any serial 6-DoF arm.
* **Forward Kinematics (FK):** Fast transform matrix chain calculation delivering live joint positions and rotation axes.
* **Inverse Kinematics (IK) & Hybrid Solvers:** High-speed positional convergence via **CCD** and precise 6D pose tracking (Position + Orientation) via **Damped Least Squares (DLS)**.
* **Agility Mapping:** Parallelized Monte-Carlo workspace simulation using **OpenMP** to compute the Yoshikawa Manipulability Index.
* **Interactive Dashboard:** Real-time 3D manipulator control panel environment built using **Plotly Dash**.

## Development & Changelog
For a comprehensive breakdown of release history, implementation steps, and engineering updates, please consult the official developer logs:
📄 **[Read the CHANGELOG.md](CHANGELOG.md)**

## License
This project is licensed under the terms of the MIT license. See the [LICENSE](LICENSE) file for details.