# Autonomous Sorting System: Project Architecture

## 1. System Overview
[cite_start]The system utilizes a modular "Perceive-Reason-Act" pipeline to bridge high-level perception with low-level hardware execution[cite: 6].

## 2. Component Breakdown
* [cite_start]**Perception Layer (Qwen 3 VL):** Processes the RGB-D feed from the simulation[cite: 7]. [cite_start]We selected Qwen 3 VL for its state-of-the-art visual grounding, allowing for highly accurate extraction of object classes and estimation of 3D coordinates[cite: 7].
* [cite_start]**Reasoning Layer (DeepSeek-R1 Distilled):** Interprets scene metadata to determine sorting logic (e.g., "Class C -> Box 3")[cite: 8]. [cite_start]DeepSeek-R1 was chosen for its native Chain-of-Thought reasoning and self-reflection, which ensures robust management of the state machine during mid-task error recovery[cite: 8, 19].
* [cite_start]**Execution Layer (Franka Panda via Isaac Sim):** Handles motion planning and grasping using the Franka Panda arm and its Inverse Kinematics (IK) solver[cite: 9, 14].

## 3. API Definition & Error Handling
[cite_start]We enforce a standardized JSON API between the Reasoning Layer and Execution Layer[cite: 11]. 

| Method | Success Code | Error Codes |
| :--- | :--- | :--- |
| `goto_pose(x, y, z)` | 200: AT_TARGET | 401: IK_UNREACHABLE, 402: COLLISION |
| `execute_grasp()` | 201: GRASP_SUCCESS | 403: OBJECT_SLIPPED, 404: MISSING_OBJ |
| `place_in_box(id)` | 202: PLACE_SUCCESS | 405: DOES_NOT_FIT, 406: OBJECT_FELL |

## 4. Model Selection Analysis
*Disclaimer: The following market analysis reflects the state of open-source models as of early 2026, when this architecture was designed.*

### 4.1. [cite_start]Perception Layer (Open-Source VLM) [cite: 7]
* **Qwen 3 VL (Selected):** * **Pros:** Dominates visual grounding natively. [cite_start]Excels at pinpointing exact pixel coordinates, making the translation to 3D coordinates highly reliable[cite: 7].
    * **Cons:** Slightly heavier VRAM requirement compared to ultra-light models.
* **GLM-4.1V-Thinking:**
    * **Pros:** Incredible at complex visual reasoning via Chain-of-Thought.
    * **Cons:** Inference time is too slow for real-time robotic pipelines.
* **Llama 3.2 Vision:**
    * **Pros:** Highly reliable general object classification and great ecosystem support.
    * **Cons:** Lacks the razor-sharp spatial grounding needed for precise coordinate estimation.

### 4.2. [cite_start]Reasoning Layer (Open-Source Reasoning Model) [cite: 8]
* **DeepSeek-R1 Distilled (Selected):**
    * **Pros:** The gold standard for reasoning and self-reflection. [cite_start]Its native ability to "think" prevents the state machine from getting stuck in loops when handling mid-task failures like `OBJECT_FELL`[cite: 19].
    * [cite_start]**Cons:** Output can be verbose, requiring strict system prompts to enforce the JSON API[cite: 11].
* **GLM-5 Reasoning:**
    * **Pros:** Phenomenally obedient at following API schemas and outputting pure JSON data.
    * **Cons:** Self-correction in unpredictable edge cases isn't as robust as reinforcement-learning-heavy models.
* **Llama 4 Scout / Maverick:**
    * **Pros:** Fast, efficient, and handles long context windows flawlessly.
    * **Cons:** A generalist model; strict logical state-machine routing is handled slightly better by purpose-built reasoning models.
