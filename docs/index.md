# SIck KinematiX 

**High-Performance 6-DoF Manipulator Kinematics Engine**

SIck KinematiX is a professional-grade, universal solver for 6-Degree-of-Freedom serial manipulators. By offloading complex matrix operations to a custom-built **Pure C Engine**, it achieves near-native performance while providing an intuitive **Pythonic API**.

## Technical Features Overview
* **Custom Computational Core:** A proprietary matrix math library (4x4 and 6x6 operations, Gauss-Jordan inversion, and 3D vector math) written entirely from scratch in pure C for absolute efficiency.
* **Universal MDH Engine:** Dynamically parses Modified Denavit-Hartenberg parameters for any serial 6-DoF arm.
* **Forward Kinematics (FK):** Fast transform matrix chain calculation delivering live joint positions and rotation axes.
* **Inverse Kinematics (IK) & Hybrid Solvers:** High-speed positional convergence via **CCD** and precise 6D pose tracking (Position + Orientation) via **Damped Least Squares (DLS)**.
* **Agility Mapping:** Parallelized Monte-Carlo workspace simulation using **OpenMP** to compute the Yoshikawa Manipulability Index.
* **Interactive Dashboard:** Real-time 3D manipulator control panel environment built using **Plotly Dash**.

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
git clone https://github.com/tomekstolarczyk/SIckKinematiX
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

## Interactive 3D Dashboard

SIckKinematiX includes a fully interactive, real-time control and diagnostic dashboard built with Plotly Dash. It allows for immediate verification of numerical kinematics stability and cloud density metrics.

<p align="center">
  <video src="media/demo_fk.mp4" width="45%" autoplay loop muted playsinline style="margin-right: 4%; border-radius: 6px;"></video>
  <video src="media/demo_ik.mp4" width="45%" autoplay loop muted playsinline style="border-radius: 6px;"></video>
</p>

**Running the Dashboard Locally:**

```bash
python dashboard/app.py
```
