# API Reference

---

## `build_robot(a, alpha, d, offsets)`
Builds a 6-DoF robot arm capsule using Modified Denavit-Hartenberg (MDH) parameters.

* **Parameters:**
    * `a` *(list of 6 floats)*: Link lengths.
    * `alpha` *(list of 6 floats)*: Link twists.
    * `d` *(list of 6 floats)*: Link offsets.
    * `offsets` *(list of 6 floats)*: Joint angle offsets.
* **Returns:** A `PyCapsule` object containing the C-struct of the robot.
* **Raises:** `ValueError` if lists are not exactly 6 elements long. `TypeError` if elements are not numbers.

---

## `fk(robot, t1, t2, t3, t4, t5, t6)`
Calculates the Forward Kinematics for a given set of joint angles.

* **Parameters:**
    * `robot` *(PyCapsule)*: The robot object returned by `build_robot`.
    * `t1`...`t6` *(float)*: Joint angles in radians.
* **Returns:** A list of 7 tuples. Each tuple contains 6 floats `(X, Y, Z, Zx, Zy, Zz)` representing the Cartesian position and the Z-axis rotation vector for the base and each of the 6 joints.

---

## `ik_ccd(robot, tx, ty, tz, [t1, t2, t3, t4, t5, t6], [max_iters], [tolerance])`
Solves Inverse Kinematics for a target XYZ position using the Cyclic Coordinate Descent (CCD) algorithm.

* **Parameters:**
    * `robot` *(PyCapsule)*: The robot object.
    * `tx`, `ty`, `tz` *(float)*: Target X, Y, Z coordinates in meters.
    * `t1`...`t6` *(float, optional)*: Initial joint angles guess. Defaults to `0.0`.
    * `max_iters` *(int, optional)*: Maximum iterations. Defaults to `5000`.
    * `tolerance` *(float, optional)*: Acceptable error margin. Defaults to `0.001`.
* **Returns:** A tuple of 6 floats representing the calculated joint angles in radians.

---

## `ik_dls(robot, target_pose_list, [t1...t6], [max_iters], [tolerance], [step_size], [lambda])`
Solves Inverse Kinematics for a target 4x4 pose (Position + Rotation) using Damped Least Squares (DLS).

* **Parameters:**
    * `robot` *(PyCapsule)*: The robot object.
    * `target_pose_list` *(list of 16 floats)*: A flat 4x4 transformation matrix representing the target pose.
    * `step_size` *(float, optional)*: Integration step size. Defaults to `0.1`.
    * `lambda` *(float, optional)*: Damping factor for singularities. Defaults to `0.1`.
* **Returns:** A tuple of 6 floats representing the joint angles.

---

## `workspace(robot, pointsNumber)`
Generates a random point cloud representing the robot's reachable workspace and evaluates agility.

* **Parameters:**
    * `robot` *(PyCapsule)*: The robot object.
    * `pointsNumber` *(int)*: Number of Monte-Carlo samples to generate.
* **Returns:** A tuple of 4 NumPy arrays `(X, Y, Z, Yoshikawa_Index)`.

---

## `yoshikawa(robot, t1, t2, t3, t4, t5, t6)`
Calculates the Yoshikawa Manipulability Index for a specific pose, indicating how far the robot is from a singularity.

* **Parameters:**
    * `robot` *(PyCapsule)*: The robot object.
    * `t1`...`t6` *(float)*: Joint angles in radians.
* **Returns:** A `float` representing the manipulability index $w = \sqrt{\det(J J^T)}$.