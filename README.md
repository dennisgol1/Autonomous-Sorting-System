# Autonomous Sorting System: Project Architecture

## 1. System Overview
The system utilizes a modular "Perceive-Reason-Act" pipeline to bridge high-level perception with low-level hardware execution.

## 2. Component Breakdown
* **Perception (Open-Source VLM):** Processes the RGB-D feed to identify object classes and estimate 3D coordinates.
* **Reasoning Layer (Open-Source Reasoning Model):** Interprets scene metadata to determine sorting logic (e.g., "Class C -> Box 3") and manages the state machine for error recovery.
* **Execution (Robotics):** Handles motion planning and grasping using the Franka Panda arm and its Inverse Kinematics (IK) solver.

## 3. API Definition & Error Handling
| Method | Success Code | Error Codes |
| :--- | :--- | :--- |
| `goto_pose(x, y, z)` | 200: AT_TARGET | 401: IK_UNREACHABLE, 402: COLLISION |
| `execute_grasp()` | 201: GRASP_SUCCESS | 403: OBJECT_SLIPPED, 404: MISSING_OBJ |
| `place_in_box(id)` | 202: PLACE_SUCCESS | 405: DOES_NOT_FIT, 406: OBJECT_FELL |
