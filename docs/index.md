# SIck KinematiX 

**6-DoF Manipulator Kinematics Solver built with pure C and Python API**

This project provides a robust, fast, and universal solver for 6-Degree-of-Freedom robot arms. It features Forward Kinematics, Inverse Kinematics (using CCD and DLS algorithms), and Workspace Analysis (evaluating the Yoshikawa Manipulability Index). All mathematical operations are implemented in pure C for maximum performance, while keeping a user-friendly Python interface.

## Installation

To install the package locally, clone the repository and run the following commands:

```bash
# Install the C-extension package
pip install -e .
# Install required dependencies for visualization
pip install -r requirements.txt
```

## Quick Start & Examples

Here is a quick example of how to build a UR5 robot and calculate Forward Kinematics for a zero-position:

``` Python
import sickkinematix.c_kinematix as sk
import math

# Define robot parameters (uR5 here as an example)
a = [0.0, 0.0, -0.425, -0.39225, 0.0, 0.0]
alpha = [0.0, math.pi/2.0, 0.0, 0.0, math.pi/2.0, -math.pi/2.0]
d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Build the robot 
robot = sk.build_robot(a, alpha, d, offsets)

# Calculate Forward Kinematics
angles = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
results = sk.fk(robot, *angles)

print("End-effector pose (X, Y, Z, Zx, Zy, Zz):")
print(results[-1])
```

Expected Output:
```Plaintext
End-effector pose (X, Y, Z, Zx, Zy, Zz):
(0.81725, 0.0, -0.005491, 0.0, 1.0, 0.0)
```

## Interactive Dashboard

We also provide a fully interactive dashboard built with Plotly Dash to visualize the robot's kinematics and workspace. Run it locally:
Bash

```bash
python dashboard/app.py
```