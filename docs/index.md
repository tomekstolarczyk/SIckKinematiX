# SIck KinematiX 

**High-Performance 6-DoF Manipulator Kinematics Engine**

SIck KinematiX is a professional-grade, universal solver for 6-Degree-of-Freedom serial manipulators. By offloading complex matrix operations to a custom-built **Pure C Engine**, it achieves near-native performance while providing an intuitive **Pythonic API**.

## Core Capabilities
* **Universal Modeling:** Supports any 6-DoF robot via Modified Denavit-Hartenberg (MDH) parameters.
* **Hybrid Solvers:** High-speed positional convergence (CCD) and high-precision pose tracking (DLS).
* **Agility Mapping:** Real-time workspace analysis evaluating the **Yoshikawa Manipulability Index**.
* **Parallel Scaling:** Multi-threaded computation optimized with **OpenMP**.

## Requirements

* **Operating System:** Linux (fully optimized with OpenMP multi-threading), Windows, or macOS.
* **Language Runtimes:** Python >= 3.10 and a C99-compliant compiler (`gcc`, `clang`, or `msvc`).
* **Core Build Dependencies:** 
    * `setuptools` (for compiling the C-Extension module)
    * `numpy >= 2.0.0` (provides C header structures for array handling via NumPy C-API)
* **Test Stack:** `pytest` (for automated unit testing and validation).
* **Visualization & UI Stack:** 
    * `dash` (Plotly Dash framework for the UI server)
    * `plotly` (for interactive 3D WebGL rendering of the manipulator)
    
## Installation

```bash
# Clone the repository
git clone [https://github.com/tomekstolarczyk/SIckKinematiX](https://github.com/tomekstolarczyk/SIckKinematiX)
cd SIckKinematiX

# Install the C-extension package locally
pip install -e .

# Install dashboard requirements
pip install -r requirements.txt
```

## Quick Start 

Build a UR5 robot model and compute Forward Kinematics in 3 lines:

``` Python
import sickkinematix.kinematix as sk
import math

# Define robot's MDH Parameters
# UR5 as an example:
a = [0.0, 0.0, -0.425, -0.39225, 0.0, 0.0]
alpha = [0.0, math.pi/2.0, 0.0, 0.0, math.pi/2.0, -math.pi/2.0]
d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Initialize C-Engine Robot Object
robot = sk.robot(a, alpha, d, offsets)

# Compute Pose
pose = sk.forward_kinematics(robot, 0, 0, 0, 0, 0, 0)
print(f"End-effector Position: {pose[-1][:3]}")
```

## Interactive Dashboard

We also provide a fully interactive dashboard built with Plotly Dash to visualize the robot's kinematics and workspace. Run it locally:
Bash

```bash
python dashboard/app.py
```