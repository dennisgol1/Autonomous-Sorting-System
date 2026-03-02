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

## 4. Model Selection Analysis
*Disclaimer: The following market analysis reflects the state of open-source models as of early 2026, when this architecture was designed.*

### 4.1. Perception Layer (Open-Source VLM)
* **Qwen 3 VL (Selected):**
    * **Pros:** Dominates visual grounding natively. Excels at pinpointing exact pixel coordinates, making the translation to 3D coordinates highly reliable.
    * **Cons:** Slightly heavier VRAM requirement compared to ultra-light models.
* **GLM-4.1V-Thinking:**
    * **Pros:** Incredible at complex visual reasoning via Chain-of-Thought.
    * **Cons:** Inference time is too slow for real-time robotic pipelines.
* **Llama 3.2 Vision:**
    * **Pros:** Highly reliable general object classification and great ecosystem support.
    * **Cons:** Lacks the razor-sharp spatial grounding needed for precise coordinate estimation.

### 4.2. Reasoning Layer (Open-Source Reasoning Model)
* **DeepSeek-R1 Distilled (Selected):**
    * **Pros:** The gold standard for reasoning and self-reflection. Its native ability to "think" prevents the state machine from getting stuck in loops when handling mid-task failures like `OBJECT_FELL`.
    * **Cons:** Output can be verbose, requiring strict system prompts to enforce the JSON API.
* **GLM-5 Reasoning:**
    * **Pros:** Phenomenally obedient at following API schemas and outputting pure JSON data.
    * **Cons:** Self-correction in unpredictable edge cases isn't as robust as reinforcement-learning-heavy models.
* **Llama 4 Scout / Maverick:**
    * **Pros:** Fast, efficient, and handles long context windows flawlessly.
    * **Cons:** A generalist model; strict logical state-machine routing is handled slightly better by purpose-built reasoning models.
