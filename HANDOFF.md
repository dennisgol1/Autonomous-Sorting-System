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
├── HANDOFF.md                    # This file
├── requirements.txt              # ollama, opencv-python
├── .gitignore
├── debug/                        # Auto-created. Populated when USE_VLM=True.
│   ├── YYYYMMDD_HHMMSS_frame.jpg     # Captured webcam/sim frame
│   ├── YYYYMMDD_HHMMSS_vlm_raw.txt   # Exact VLM response (pre-parse)
│   └── YYYYMMDD_HHMMSS_scene.json    # Validated scene dict
└── src/
    ├── main.py                   # Orchestrator — 3 pipeline modes via flags
    ├── execution/
    │   └── hardware_api.py       # MockFrankaRobot + StatusCode enum
    ├── perception/
    │   └── vlm_client.py         # PerceptionClient — webcam + Qwen VL
    └── reasoning/
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

## 5. Setup on a New Linux Machine

### 5.1 System dependencies
```bash
sudo apt update && sudo apt install -y python3 python3-pip git
# Install Ollama
curl -fsSL https://ollama.com/install.sh | sh
```

### 5.2 Clone the repository and install Python deps
```bash
git clone <repo-url>
cd Autonomous-Sorting-System
pip install -r requirements.txt
```

### 5.3 Pull AI models (one-time, ~11GB total)
```bash
ollama pull qwen2.5vl:7b       # ~5.5GB — perception layer
ollama pull deepseek-r1:8b     # ~5.2GB — reasoning layer
```

### 5.4 Verify Ollama is running
```bash
ollama list     # Should show both models
ollama serve    # Start the server if not already running
```

### 5.5 Run
```bash
cd src
python main.py
```

---

## 6. Target Hardware (Isaac Sim machine)

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

---

## 9. Development History

| Stage | File | Status | Description |
|-------|------|--------|-------------|
| 2B | `main.py` | Done | State machine with full error recovery, hardcoded data |
| 2A | `reasoning/llm_client.py` | Done | DeepSeek-R1 wrapper, JSON parsing, schema validation |
| 1  | `perception/vlm_client.py` | Done | Qwen VL wrapper, webcam capture, debug output |
| —  | `execution/hardware_api.py` | Done | MockFrankaRobot with realistic failure rates |
| —  | Integration | Done | All layers wired in main.py with 3-mode flags |
| —  | Isaac Sim | Pending | Swap 2 methods (see Section 7) |
| —  | Videos | Pending | Record after Isaac Sim integration |
