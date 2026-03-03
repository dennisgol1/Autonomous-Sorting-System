# AI Context & Handoff Document
## Autonomous Sorting System — For Continuing Development on a New Machine

---

## 1. Project Summary

A modular robotic sorting system built on a **Perceive → Reason → Act** pipeline.
The robot identifies three objects from three different classes, decides which box each
belongs in, and executes the pick-and-place sequence — recovering gracefully from hardware
failures. Built as a home assignment for a Robotics Engineer position.

**Deliverables required by the assignment:**
1. Architecture document — **DONE** (`README.md`)
2. Python implementation (Isaac Sim + AI controller) — **DONE** (all 4 modules written)
3. Video: successful 3-object sort — **PENDING** (needs Isaac Sim)
4. Video: failure recovery — **PENDING** (needs Isaac Sim)

---

## 2. Architecture

```
[Camera / Isaac Sim RGB-D sensor]
         |
         v
[Perception Layer]  src/perception/vlm_client.py
  Model: Qwen2.5-VL 7B via Ollama
  Role: Identifies objects (class + 3D coords) from a single frame.
  Output: {"objects": [{"id", "class_label", "coords": [x,y,z]}, ...]}
         |
         v
[Reasoning Layer]   src/reasoning/llm_client.py
  Model: DeepSeek-R1 8B (Qwen3-distilled) via Ollama
  Role: Maps object classes to target boxes. Strips <think> blocks.
  Output: [{"name", "class_label", "coords", "target_box"}, ...]
         |
         v
[Execution Layer]   src/execution/hardware_api.py
  NOW:   MockFrankaRobot (simulates random hardware failures)
  LATER: Real Franka Panda via Isaac Sim Lula IK solver
         |
         v
[State Machine]     src/main.py  (SortingStateMachine)
  Orchestrates the pick-and-place loop with full error recovery.
```

### Class-to-Box Mapping
| Class   | Color (laptop POC) | Target Box |
|---------|--------------------|------------|
| ClassA  | Red object         | Box 1      |
| ClassB  | Blue object        | Box 2      |
| ClassC  | Green object       | Box 3      |

### API Error Contract
| Method            | Success           | Recoverable (retry)                     | Fatal (halt)                        |
|-------------------|-------------------|-----------------------------------------|-------------------------------------|
| `goto_pose(x,y,z)`| 200: AT_TARGET    | —                                       | 401: IK_UNREACHABLE, 402: COLLISION |
| `execute_grasp()` | 201: GRASP_SUCCESS| 403: OBJECT_SLIPPED (retry from nav)    | 404: MISSING_OBJ                    |
| `place_in_box(id)`| 202: PLACE_SUCCESS| 406: OBJECT_FELL (retry full cycle)     | 405: DOES_NOT_FIT                   |

Max retries: **3** (configured via `MAX_RETRIES` in `main.py`)

---

## 3. Repository Structure

```
AUTONOMOUS-SORTING-SYSTEM/
├── README.md                     # Architecture document (assignment deliverable 1)
├── requirements.txt              # ollama, opencv-python
├── .gitignore
├── docs/
│   └── HANDOFF.md                # This file — AI context for new machine/session
├── debug/                        # Auto-created. Populated when USE_VLM=True.
│   ├── README.md                     # Explains debug file format
│   ├── YYYYMMDD_HHMMSS_frame.jpg     # Captured webcam/sim frame
│   ├── YYYYMMDD_HHMMSS_vlm_raw.txt   # Exact VLM response (pre-parse)
│   └── YYYYMMDD_HHMMSS_scene.json    # Validated scene dict
└── src/
    ├── README.md                 # Pipeline diagram + mode flags reference
    ├── main.py                   # Orchestrator — 3 pipeline modes via flags
    ├── execution/
    │   ├── README.md             # StatusCode table, mock vs real robot
    │   └── hardware_api.py       # MockFrankaRobot + StatusCode enum
    ├── perception/
    │   ├── README.md             # Webcam flow, output schema, Isaac Sim swap
    │   └── vlm_client.py         # PerceptionClient — webcam + Qwen VL
    └── reasoning/
        ├── README.md             # LLM flow, think-tag stripping, output schema
        └── llm_client.py         # ReasoningClient — DeepSeek-R1 + JSON parser
```

---

## 4. Pipeline Modes (main.py flags)

Edit the top of `src/main.py` to switch modes:

| `USE_VLM` | `USE_LLM` | What runs                                      | Requires        |
|-----------|-----------|------------------------------------------------|-----------------|
| `False`   | `False`   | Hardcoded objects → mock robot                 | Nothing         |
| `False`   | `True`    | Mock scene → DeepSeek-R1 → mock robot          | Ollama running  |
| `True`    | `True`    | Webcam → Qwen VL → DeepSeek-R1 → mock robot   | Ollama + camera |

`SHOW_PREVIEW = True` — shows a live webcam window when `USE_VLM=True`.
Press **SPACE** to capture, **Q** to cancel.

---

## 5. Setup on the Isaac Sim Windows Machine

### 5.1 System dependencies
- **Python 3.10+** — download from python.org if not installed
- **Git** — download from git-scm.com if not installed
- **Ollama** — download the Windows installer from [ollama.com](https://ollama.com/download/windows)

After installing Ollama, it runs as a background service automatically on Windows.

### 5.2 Clone the repository and install Python deps
```powershell
git clone <repo-url>
cd Autonomous-Sorting-System
pip install -r requirements.txt
```

### 5.3 Pull AI models (one-time, ~11GB total)
```powershell
ollama pull qwen2.5vl:7b       # ~5.5GB — perception layer
ollama pull deepseek-r1:8b     # ~5.2GB — reasoning layer
```

### 5.4 Verify Ollama is running
```powershell
ollama list     # Should show both models
# Ollama runs as a Windows service — if not running, launch the Ollama app from Start menu
```

### 5.5 Run
```powershell
cd src
python main.py
```

---

## 6. Target Hardware (Isaac Sim machine)

- **OS:** Windows (not Linux)
- **GPU:** NVIDIA RTX 4070 (12GB VRAM) — both models fit comfortably in VRAM
- **CPU:** Intel Core i9-13700KF (13th gen, 16 cores)
- **RAM:** 32GB

Both models will run significantly faster on this machine than on the laptop:
- Qwen2.5-VL 7B: ~1-2s per inference on RTX 4070
- DeepSeek-R1 8B: ~2-4s per inference on RTX 4070

---

## 7. Isaac Sim Integration — What Needs to Change

The codebase was designed to be hardware-agnostic. Only **two methods** need replacing:

### 7.1 Swap the camera capture
In `src/perception/vlm_client.py`, replace `capture_frame()`:
```python
# CURRENT (laptop webcam):
def capture_frame(self, show_preview=True) -> tuple:
    cap = cv2.VideoCapture(self.camera_index)
    ...

# REPLACE WITH (Isaac Sim RGB-D sensor):
def capture_frame(self, show_preview=False) -> tuple:
    # Pull RGB frame from Isaac Sim sensor
    rgb_frame = isaac_sim_camera.get_rgb()
    depth_frame = isaac_sim_camera.get_depth()
    # Use real depth to fill Z in coords (replace the z=0.3 hardcode in the prompt)
    _, jpeg = cv2.imencode(".jpg", rgb_frame)
    return jpeg.tobytes(), rgb_frame
```

### 7.2 Swap the mock robot
In `src/execution/hardware_api.py`, add a real `FrankaRobot` class alongside `MockFrankaRobot`:
```python
class FrankaRobot:
    def goto_pose(self, x, y, z) -> StatusCode:
        # Call Isaac Sim Lula IK solver
        result = lula_kinematics.compute_ik(x, y, z)
        ...
```

In `src/main.py`, swap the import:
```python
# from execution.hardware_api import MockFrankaRobot as Robot
from execution.hardware_api import FrankaRobot as Robot
```

**Everything else — the state machine, LLM client, VLM client, error handling — stays identical.**

---

## 8. Known Limitations & Notes

- **Z coordinate on laptop:** Always 0.3 (hardcoded in VLM prompt). Will be real depth data in Isaac Sim.
- **Coordinate space:** VLM outputs normalized image coords [-0.5, 0.5]. Isaac Sim uses meters in robot frame. A coordinate transform will be needed at the Isaac Sim integration stage.
- **Model name note:** `deepseek-r1:8b` now resolves to the Qwen3-based R1-0528 distill. If exact reproducibility matters, pin to `deepseek-r1:8b-0528-qwen3-q4_K_M`.
- **VLM model name:** The project targets Qwen 3 VL (architecture doc). The current closest available on Ollama is `qwen2.5vl:7b`. Update the model name in `vlm_client.py` when Qwen 3 VL becomes available.
- **Laptop RAM:** `qwen2.5vl:7b` requires ~5.8 GB free RAM. The dev laptop cannot run it. Full pipeline (Mode 3) must be tested on the Isaac Sim machine. Mode 2 (LLM-only) works on the laptop.

---

## 9. Development History

| Stage | File | Status | Description |
|-------|------|--------|-------------|
| 2B | `main.py` | Done | State machine with full error recovery, hardcoded data |
| 2A | `reasoning/llm_client.py` | Done | DeepSeek-R1 wrapper, JSON parsing, schema validation |
| 1  | `perception/vlm_client.py` | Done | Qwen VL wrapper, webcam capture, debug output |
| —  | `execution/hardware_api.py` | Done | MockFrankaRobot with realistic failure rates |
| —  | Integration | Done | All layers wired in main.py with 3-mode flags |
| —  | Documentation | Done | README.md + per-folder READMEs + HANDOFF.md in docs/ |
| —  | Live test (laptop) | Blocked | qwen2.5vl:7b OOMs on laptop — test on Isaac Sim machine |
| —  | Isaac Sim | Pending | Swap 2 methods (see Section 7) |
| —  | Videos | Pending | Record after Isaac Sim integration |

---

## 10. What to Do First on the Isaac Sim Windows Machine

1. Follow Section 5 (setup + model pull)
2. Run Mode 2 first to verify DeepSeek-R1 reasoning works: set `USE_VLM=False, USE_LLM=True` in `src/main.py`
3. Run Mode 3 (full pipeline) with 3 colored objects in front of camera: set `USE_VLM=True, USE_LLM=True`
4. Check `debug/` folder — verify `_vlm_raw.txt` and `_scene.json` look correct
5. Proceed to Isaac Sim integration (Section 7)

---

## 11. Remaining Development Stages (discussed 2026-03-03)

### Stage 3 — Live pipeline test on Isaac Sim Windows machine
**Critical. Cannot skip.**
- Pull both models, run Mode 2 then Mode 3 (see Section 10)
- Validates full stack before touching Isaac Sim

### Stage 4A — Isaac Sim sensor swap (perception)
Replace `capture_frame()` in `src/perception/vlm_client.py` with an Isaac Sim RGB-D feed.

**Chosen approach: Direct Isaac Sim Python API** (no ROS2 required, least friction)
**ROS2 bridge kept as optional adapter** (see Section 12 for architecture)

### Stage 4B — Isaac Sim robot swap (execution)
Replace `MockFrankaRobot` with a real Franka Panda using Isaac Sim's Lula IK solver.
- Can defer until 4A is working
- Only two methods need implementing: `goto_pose`, `execute_grasp`, `place_in_box`
- See Section 7.2 for the swap pattern

### Stage 5 — Coordinate transform
**Can skip for initial demo** by reading object positions directly from Isaac Sim world state instead of projecting VLM image coordinates.

If implementing properly:
- VLM outputs normalized image coords `[-0.5, 0.5]` + real Z from depth sensor
- Need camera intrinsics + camera-to-world transform (both available from Isaac Sim)
- Add `project_to_world(x_norm, y_norm, z_depth)` to `vlm_client.py`

### Stage 6 — Prompt & parser hardening
**Optional for demo, skip if time-constrained.**
- Stress-test VLM with partial occlusion, two same-color objects
- Verify `_clean_response()` handles all VLM output edge cases

### Stage 7 — Documentation, screenshots, video
**Required for assignment deliverables.**
- Terminal output of successful 3-object cycle
- Video: success run + failure recovery sequence
- Add screenshots to README.md

### Recommended fastest path to working demo
1. Linux live test (Stage 3)
2. Isaac Sim: build scene (see Section 13), swap sensor (4A), skip coord transform
3. Swap mock robot with Lula IK (4B)
4. Record demo video (Stage 7)

---

## 12. ROS2 Bridge — Optional Adapter Architecture

Isaac Sim has a built-in ROS2 bridge extension (`omni.isaac.ros2_bridge`) that publishes
camera topics automatically — no external ROS2 node setup needed. We keep ROS2 as an
optional adapter alongside the direct API path.

**Windows note:** ROS2 on Windows is officially supported (ROS2 Humble/Iron have Windows builds)
but is significantly more painful to set up than on Linux. If Isaac Sim integration is working
well without ROS2, skip it. Only pursue this path if you specifically need the ROS2 ecosystem
(e.g., connecting to a physical robot, using Nav2, or reusing existing ROS2 nodes).

**Implementation plan (do this during Stage 4A):**

Add `USE_ROS2` flag to `src/main.py` (alongside `USE_VLM` / `USE_LLM`).

In `src/perception/vlm_client.py`, swap `capture_frame()` based on the flag:

```python
# Direct Isaac Sim API (USE_ROS2=False):
def capture_frame(self, show_preview=False):
    rgb = isaac_sim_camera.get_rgb()
    depth = isaac_sim_camera.get_depth()
    _, jpeg = cv2.imencode(".jpg", rgb)
    return jpeg.tobytes(), rgb

# ROS2 bridge (USE_ROS2=True):
# Create a thin Ros2SensorClient that subscribes to:
#   /camera/rgb   (sensor_msgs/Image)
#   /camera/depth (sensor_msgs/Image)
# and exposes the same (jpeg_bytes, raw_frame) return signature
```

Everything downstream (VLM analysis, state machine, robot) stays identical either way.

**To enable the ROS2 bridge in Isaac Sim:**
- Extensions → search `ros2_bridge` → enable
- Isaac Sim will start publishing camera topics on launch
- On Windows, also requires a sourced ROS2 environment in the same terminal before launching Isaac Sim

---

## 13. Isaac Sim Scene Construction

**Before writing the scene script, check your Isaac Sim version:**

Check the NVIDIA Omniverse Launcher → Isaac Sim → version number shown in the library tab.
Alternatively, inside Isaac Sim: Help → About.

The API differs significantly:
- **Isaac Sim 2023.x** — `from omni.isaac.core import World`, `from omni.isaac.franka import Franka`
- **Isaac Sim 4.x (2024+)** — `isaacsim` package, different import paths

**Ask the next Claude Code session to write the scene setup script** once the version is known.

The scene needs:
1. World + ground plane + table surface
2. Franka Panda robot at a fixed base position
3. 3 colored objects on the table: red cube (ClassA), blue cube (ClassB), green cube (ClassC)
4. RGB-D camera mounted above the scene pointing down at the workspace
5. 3 sorting bins positioned within robot reach (Box 1 / Box 2 / Box 3)
6. (Optional) ROS2 bridge extension enabled for camera topic publishing

Object placement constraints:
- All 3 objects must be within Franka reach envelope (~0.85m radius from base)
- Camera should cover all objects in a single frame
- Bins should be outside the object pickup area but still reachable
