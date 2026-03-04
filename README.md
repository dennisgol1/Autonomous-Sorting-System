# Autonomous Sorting System: Project Architecture

## 1. System Overview
The system utilizes a modular "Perceive-Reason-Act" pipeline to bridge high-level perception with low-level hardware execution.

## 2. Component Breakdown
* **Perception Layer (Qwen2.5-VL 3B):** Processes the RGB-D feed from the simulation. We selected Qwen2.5-VL for its strong visual grounding and instruction-following capability, allowing for accurate extraction of object classes and estimation of 3D coordinates. Model size was tuned to 3B to fit within the 12 GB VRAM budget alongside Isaac Sim (see Section 5 for sizing rationale).
* **Reasoning Layer (DeepSeek-R1 Distilled 1.5B):** Interprets scene metadata to determine sorting logic (e.g., "Class C -> Box 3"). DeepSeek-R1 was chosen for its native Chain-of-Thought reasoning and self-reflection, which ensures robust management of the state machine during mid-task error recovery. The 1.5B distill provides sufficient reasoning quality for the deterministic 3-class mapping task at minimal VRAM cost.
* **Execution Layer (Franka Panda via Isaac Sim):** Handles motion planning and grasping using the Franka Panda arm and its Inverse Kinematics (IK) solver.

## 3. API Definition & Error Handling
We enforce a standardized JSON API between the Reasoning Layer and Execution Layer. 

| Method | Success Code | Error Codes |
| :--- | :--- | :--- |
| `goto_pose(x, y, z)` | 200: AT_TARGET | 401: IK_UNREACHABLE, 402: COLLISION |
| `execute_grasp()` | 201: GRASP_SUCCESS | 403: OBJECT_SLIPPED, 404: MISSING_OBJ |
| `place_in_box(id)` | 202: PLACE_SUCCESS | 405: DOES_NOT_FIT, 406: OBJECT_FELL |

## 4. Module Reference

### Perception — `src/perception/vlm_client.py`
Wraps a Vision Language Model (Qwen2.5-VL 3B via Ollama) with a webcam capture loop and debug output.

**Key class:** `PerceptionClient(model, camera_index)`

| Method | What it does |
| :--- | :--- |
| `perceive(show_preview, save_debug)` | Full entry point: captures frame → runs VLM → returns scene dict |
| `capture_frame(show_preview)` | Opens webcam. If `show_preview=True`, shows live window — press **SPACE** to capture, **Q** to cancel |
| `analyze_frame(frame_bytes, raw_frame, save_debug)` | Sends JPEG to VLM, strips markdown fences, parses and validates JSON response |

**Output schema:**
```json
{
  "objects": [
    {"id": "obj_1", "class_label": "ClassA", "coords": [-0.2, 0.1, 0.3]},
    {"id": "obj_2", "class_label": "ClassB", "coords": [ 0.0, 0.0, 0.3]},
    {"id": "obj_3", "class_label": "ClassC", "coords": [ 0.2,-0.1, 0.3]}
  ]
}
```

**Color-to-class mapping (laptop POC):**

| Object color | Class label | Target box |
| :--- | :--- | :--- |
| Red | ClassA | Box 1 |
| Blue | ClassB | Box 2 |
| Green | ClassC | Box 3 |

**Isaac Sim swap:** replace `capture_frame()` with Isaac Sim RGB-D sensor output. Everything else is unchanged.

---

### Reasoning — `src/reasoning/llm_client.py`
Wraps DeepSeek-R1 1.5B (via Ollama) to map scene metadata to a sort plan. Strips `<think>` blocks and enforces strict JSON output.

**Key class:** `ReasoningClient(model)`

| Method | What it does |
| :--- | :--- |
| `generate_sort_plan(scene_metadata)` | Sends scene dict to LLM, parses response, returns sort plan list |
| `_clean_response(text)` | Strips `<think>...</think>` blocks and markdown fences, extracts JSON array |
| `_validate_plan(plan)` | Checks each item has `name`, `class_label`, `coords`, `target_box` |

**Output schema:**
```json
[
  {"name": "obj_1", "class_label": "ClassA", "coords": [-0.2, 0.1, 0.3], "target_box": 1},
  {"name": "obj_2", "class_label": "ClassB", "coords": [ 0.0, 0.0, 0.3], "target_box": 2},
  {"name": "obj_3", "class_label": "ClassC", "coords": [ 0.2,-0.1, 0.3], "target_box": 3}
]
```

**Note:** `deepseek-r1:1.5b` on Ollama resolves to the Qwen3-based R1-0528 distill. Downsized from 8B to fit within the VRAM budget alongside Isaac Sim and the VLM.

---

### Execution — `src/execution/hardware_api.py`
Defines the robot hardware interface and status codes. Currently a `MockFrankaRobot` that simulates realistic random failures (~10% per call). Swap for a real `FrankaRobot` class at Isaac Sim integration time.

**Key classes:**

`StatusCode` (enum) — full API contract:

| Code | Meaning | State machine response |
| :--- | :--- | :--- |
| 200 AT_TARGET | Navigation succeeded | Proceed to grasp |
| 201 GRASP_SUCCESS | Gripper secured object | Proceed to place |
| 202 PLACE_SUCCESS | Object placed in box | Done — next object |
| 401 IK_UNREACHABLE | No valid joint solution | **Hard halt** |
| 402 COLLISION | Path obstructed | **Hard halt** |
| 403 OBJECT_SLIPPED | Gripper lost contact | Retry from `goto_pose` |
| 404 MISSING_OBJ | Object not at expected coords | **Hard halt** |
| 405 DOES_NOT_FIT | Wrong box for this object | **Hard halt** |
| 406 OBJECT_FELL | Dropped during placement | Retry from `goto_pose` |

`MockFrankaRobot` — three methods that mirror the real Franka API:
- `goto_pose(x, y, z)` → AT_TARGET or IK_UNREACHABLE
- `execute_grasp()` → GRASP_SUCCESS or OBJECT_SLIPPED
- `place_in_box(box_id)` → PLACE_SUCCESS or OBJECT_FELL

**Isaac Sim swap:** `FrankaRobot` class is implemented in this file and active when `USE_ISAAC_SIM=True` in `main.py`.

#### Motion Controller: PickPlaceController + RMPflow Obstacle Registration

The system uses `PickPlaceController` with **explicit obstacle registration** to enable collision-aware arm motion.

**What PickPlaceController actually is:**
Inspection of the Isaac Sim 5.1.0 source (`pick_place_controller.py`) reveals that `PickPlaceController` already uses `RMPFlowController` as its internal c-space controller:
```python
manipulators_controllers.PickPlaceController.__init__(
    self,
    cspace_controller=RMPFlowController(name=..., robot_articulation=robot_articulation),
    ...
)
```
RMPflow is the reactive motion planner that converts end-effector targets to joint commands on every physics step. `PickPlaceController` wraps it with a phase sequencer (approach → grasp → lift → transit → place → release).

**Why the arm was colliding with the table:**
RMPflow's collision avoidance only acts on obstacles **explicitly registered** with it via `add_obstacle()`. Without registration, RMPflow has no knowledge of scene geometry and plans through it. The table top and bin walls were added to the USD stage with physics collision (so dynamic cubes rest on them) but were never registered with RMPflow — so the arm moved through them freely.

**The fix — obstacle registration:**
After the world is reset and the controller is created, the table top and all bin walls are registered as static obstacles:
```python
self.controller._cspace_controller.add_obstacle(self._table_top, static=True)
for wall_prim in self._bin_wall_prims:
    self.controller._cspace_controller.add_obstacle(wall_prim, static=True)
```
RMPflow then generates collision-free paths that route the arm around the table surface and bin walls during every pick and place motion. This matches real-world robot behaviour, where the motion planner always treats physical surfaces as obstacles.

---

### Debug — `debug/`
Auto-created on first run when `USE_VLM=True` and `save_debug=True`. Each run saves three timestamped files:

| File | Contents |
| :--- | :--- |
| `YYYYMMDD_HHMMSS_frame.jpg` | The captured webcam frame sent to the VLM |
| `YYYYMMDD_HHMMSS_vlm_raw.txt` | Exact text the VLM returned (pre-parse) |
| `YYYYMMDD_HHMMSS_scene.json` | Validated scene dict passed to the reasoning layer |

The folder is tracked in git (via `debug/.gitkeep`) but its contents are ignored (see `.gitignore`).

---

## 5. Model Selection Analysis
*Disclaimer: The following market analysis reflects the state of open-source models as of early 2026, when this architecture was designed.*

### 5.1. Perception Layer (Open-Source VLM)
* **Qwen2.5-VL 3B (Selected):**
    * **Pros:** Strong visual grounding and instruction-following capability. Accurately extracts object classes and spatial coordinates from structured prompts. The 3B size fits within the 12 GB VRAM budget alongside Isaac Sim (~5 GB) with headroom to spare.
    * **Cons:** Slightly lower visual accuracy than the 7B variant; acceptable for clearly distinct colored objects.
* **Qwen2.5-VL 7B (Original design, replaced):**
    * **Pros:** Higher accuracy and more reliable JSON schema adherence.
    * **Cons:** ~5.5 GB VRAM. Combined with Isaac Sim's ~5 GB, this exceeds 12 GB on the RTX 4070. Extensive troubleshooting (CPU mode, Windows VA fragmentation, PC restart) could not stabilize it. See HANDOFF.md Section 15.
* **moondream (~1.7 GB, tested and rejected):**
    * **Pros:** Very small, loads easily alongside Isaac Sim.
    * **Cons:** Designed for image captioning, not structured JSON output. Produced hallucinated repetition loops (38k chars, ~200 copies of the same entry), ignored the `{"objects":[...]}` schema, and output 4-element coords. Rejected as unsuitable for structured perception.
* **GLM-4.1V-Thinking:**
    * **Pros:** Incredible at complex visual reasoning via Chain-of-Thought.
    * **Cons:** Inference time is too slow for real-time robotic pipelines.
* **Llama 3.2 Vision:**
    * **Pros:** Highly reliable general object classification and great ecosystem support.
    * **Cons:** Lacks the razor-sharp spatial grounding needed for precise coordinate estimation.

### 5.2. Reasoning Layer (Open-Source Reasoning Model)
* **DeepSeek-R1 Distilled 1.5B (Selected):**
    * **Pros:** Retains Chain-of-Thought reasoning capability at minimal cost (~1.1 GB VRAM). Native `<think>` blocks are stripped automatically. Sufficient for the deterministic 3-class → 3-box mapping task.
    * **Cons:** Less robust than larger variants for complex edge cases; acceptable given the fixed sorting rules.
* **DeepSeek-R1 8B (Original design, replaced):**
    * **Pros:** Stronger reasoning and self-reflection.
    * **Cons:** ~5.2 GB VRAM. Combined with Isaac Sim + VLM, this exceeds 12 GB. Downsized to 1.5B for VRAM fit.
* **GLM-5 Reasoning:**
    * **Pros:** Phenomenally obedient at following API schemas and outputting pure JSON data.
    * **Cons:** Self-correction in unpredictable edge cases isn't as robust as reinforcement-learning-heavy models.
* **Llama 4 Scout / Maverick:**
    * **Pros:** Fast, efficient, and handles long context windows flawlessly.
    * **Cons:** A generalist model; strict logical state-machine routing is handled slightly better by purpose-built reasoning models.
