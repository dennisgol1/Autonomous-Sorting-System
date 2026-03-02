# Autonomous Sorting System: Project Architecture

## 1. System Overview
The system utilizes a modular "Perceive-Reason-Act" pipeline to bridge high-level perception with low-level hardware execution.

## 2. Component Breakdown
* **Perception Layer (Qwen 3 VL):** Processes the RGB-D feed from the simulation. We selected Qwen 3 VL for its state-of-the-art visual grounding, allowing for highly accurate extraction of object classes and estimation of 3D coordinates.
* **Reasoning Layer (DeepSeek-R1 Distilled):** Interprets scene metadata to determine sorting logic (e.g., "Class C -> Box 3"). DeepSeek-R1 was chosen for its native Chain-of-Thought reasoning and self-reflection, which ensures robust management of the state machine during mid-task error recovery.
* **Execution Layer (Franka Panda via Isaac Sim):** Handles motion planning and grasping using the Franka Panda arm and its Inverse Kinematics (IK) solver.

## 3. API Definition & Error Handling
We enforce a standardized JSON API between the Reasoning Layer and Execution Layer.

| Method | Success Code | Error Codes |
| :--- | :--- | :--- |
| `goto_pose(x, y, z)` | 200: AT_TARGET | 401: IK_UNREACHABLE, 402: COLLISION |
| `execute_grasp()` | 201: GRASP_SUCCESS | 403: OBJECT_SLIPPED, 404: MISSING_OBJ |
| `place_in_box(id)` | 202: PLACE_SUCCESS | 405: DOES_NOT_FIT, 406: OBJECT_FELL |
