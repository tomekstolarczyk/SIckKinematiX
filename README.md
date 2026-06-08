# 🤖 SICK KINEMATIX

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![Core](https://img.shields.io/badge/Core_Engine-Pure_C-red.svg)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**A high-performance, universal 6-DoF Manipulator Kinematics Solver built with pure C and wrapped in a Python API.**

This project features custom-built solvers for Forward and Inverse Kinematics, alongside a fully interactive 3D visualization dashboard.

---

## Official Documentation & Guide

> **STOP! Don't read raw code.**
> 
> We have prepared a beautiful, comprehensive documentation website featuring API references, installation guides, and interactive dashboard demos. 
> 
> **[Click here to visit the website](https://[TWÓJ_LOGIN].github.io/SIckKinematiX/)** 

---

## Core Features

* **Forward Kinematics (FK):** Fast pose calculation using Modified Denavit-Hartenberg (MDH) parameters.
* **Inverse Kinematics (IK):** 
    * **CCD** (Cyclic Coordinate Descent) for fast positional targeting.
  * **DLS** (Damped Least Squares) for precise position and orientation targeting.
* **Workspace Analysis:** Monte-Carlo based point cloud generation evaluating the Yoshikawa Manipulability Index.
* **Dashboard:** Interactive `Plotly Dash` web application for real-time 3D visualization.

## Quick Install

```bash
# Clone the repository
git clone [https://github.com/](https://github.com/)[TWÓJ_LOGIN]/SIckKinematiX.git
cd SIckKinematiX

# Install the C-extension package
pip install .

# Install requirements
pip install -r requirements.txt
```