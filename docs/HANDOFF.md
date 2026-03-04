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
  Model: Qwen2.5-VL 3B via Ollama  (downsized from 7B — see Section 15)
  Role: Identifies objects (class + 3D coords) from a single frame.
  Output: {"objects": [{"id", "class_label", "coords": [x,y,z]}, ...]}
         |
         v
[Reasoning Layer]   src/reasoning/llm_client.py
  Model: DeepSeek-R1 1.5B via Ollama  (downsized from 8B — see Section 15)
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

### 5.3 Pull AI models (one-time, ~4GB total)
```powershell
ollama pull qwen2.5vl:3b       # ~2.5GB — perception layer
ollama pull deepseek-r1:1.5b   # ~1.1GB — reasoning layer
```
Note: models were originally 7B/8B but downsized to 3B/1.5B due to VRAM constraints.
See Section 15 for full explanation.

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
- **GPU:** NVIDIA RTX 4070 (12GB VRAM)
- **CPU:** Intel Core i9-13700KF (13th gen, 16 cores)
- **RAM:** 32GB

VRAM budget with current models (Isaac Sim headless + both models on GPU):
- Isaac Sim headless: ~4.5–5 GB
- qwen2.5vl:3b: ~2.5 GB
- deepseek-r1:1.5b: ~1.1 GB
- Windows overhead: ~1 GB
- **Total: ~9 GB → fits within 12 GB**

Expected GPU inference times:
- qwen2.5vl:3b: ~10–20s per frame on RTX 4070
- deepseek-r1:1.5b: ~2–5s per query on RTX 4070

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
- **Model note:** `deepseek-r1:1.5b` on Ollama resolves to the Qwen3-based R1-0528 distill. This is intentional — smaller and faster than the original 8B.
- **VLM model:** Using `qwen2.5vl:3b` (downsized from 7B for VRAM fit). See Section 15 for the full downsizing story.
- **headless=True:** `src/main.py` runs Isaac Sim in headless mode to free ~2GB VRAM for the VLM. Switch to `headless=False` for demo video recording (but disable VLM first, or expect VRAM pressure).
- **VLM fallback:** If VLM parsing fails, the pipeline falls back to known Isaac Sim world positions and continues the sort cycle. The fallback is logged: `[Main] VLM perception failed: ... Falling back to known Isaac Sim positions.`

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
| 3  | Isaac Sim scene | Done | IsaacScene: table, 3 cubes, 3 bins, overhead RGB-D camera |
| 4  | Isaac Sim execution | Done | FrankaRobot + PickPlaceController: 3/3 sort success |
| 5a | VLM model sizing | Done | Downsized to 3b/1.5b; moondream tested as fallback (see Sec 15) |
| 5b | VLM integration | In Progress | qwen2.5vl:3b wired; parser hardened; fallback to known positions |
| 6  | Prompt/parser hardening | Pending | Stress-test VLM with partial occlusion, verify output stability |
| 7  | Demo videos | Pending | headless=False, USE_VLM=False; record 3-object sort + failure recovery |

---

## 10. Current State (2026-03-04) — Picking Up from Here

Isaac Sim integration is complete (Stages 3–4). VLM integration is actively being tested (Stage 5).

**Current pipeline flags (`src/main.py`):**
```python
USE_ISAAC_SIM = True
USE_VLM       = True
USE_LLM       = False   # Not yet tested with Isaac Sim; use mock sort plan
headless      = True    # Frees VRAM for VLM; switch to False for demo videos
```

**To run:**
```powershell
# Terminal 1 — start Ollama (GPU mode, no env vars needed)
ollama serve

# Terminal 2 — run pipeline
D:\isaac-sim-standalone-5.1.0-windows-x86_64\python.bat src/main.py
```

**Known Ollama requirement:** `ollama` must be installed in Isaac Sim's bundled Python (one-time):
```powershell
D:\isaac-sim-standalone-5.1.0-windows-x86_64\python.bat -m pip install ollama
```

**Next immediate step:** Confirm `qwen2.5vl:3b` parses correctly (see Section 15). If it works, `debug/*_scene.json` should show 3 objects with correct class labels. If it falls back again, check `debug/*_vlm_raw.txt` and adjust the parser.

---

## 11. Remaining Development Stages

### Stage 5b — Confirm VLM output (IN PROGRESS)
- `qwen2.5vl:3b` is wired and parser is hardened (bare-array + 4-coord fallbacks added)
- Next run: confirm `debug/*_scene.json` has 3 objects with correct ClassA/B/C labels
- If still failing: check `debug/*_vlm_raw.txt` and adjust parser or VISION_PROMPT
- Fallback always runs the sort cycle using known Isaac Sim positions

### Stage 5c — Enable LLM reasoning (optional)
- Set `USE_LLM=True` in `src/main.py` to test `deepseek-r1:1.5b` sort planning
- VRAM adds ~1.1 GB on top of Isaac Sim + VLM — should still fit in 12 GB

### Stage 6 — Prompt/parser hardening (optional)
- Stress-test VLM with partial occlusion, two same-color objects
- Verify `_clean_response()` handles all VLM output edge cases
- Skip if time-constrained — fallback mechanism already handles parse failures

### Stage 7 — Demo videos (REQUIRED for assignment)
- Switch to `headless=False`, `USE_VLM=False` (no Ollama, no VRAM pressure)
- Record: successful 3-object sort
- Record: failure recovery (OBJECT_SLIPPED or OBJECT_FELL)
- Add screenshots/terminal output to README.md

### Fastest path to completing the assignment
1. Confirm VLM works with qwen2.5vl:3b (Stage 5b) — or accept fallback for demo
2. Switch to headless=False + USE_VLM=False
3. Record demo videos (Stage 7)

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

---

## 14. Camera System — Known Issues & Next Steps

**Current state (Stage 4 commit baseline):**
- Camera prim: `/World/CameraStandPole/Camera`
- Position: `[0.38, 0.0, 1.35]`
- Resolution: `640×480`
- Orientation: `euler_angles_to_quats([0, 90, 0], degrees=True)` ← **BUG**

### Bug: Camera looks sideways, not at workspace

`Ry(90°)` rotates the camera's default look direction (-Z) to **-X** (horizontal),
not downward at the table. Captured frames in `debug/` show only the floor grid,
not the scene objects.

**Fix needed** in `src/isaac_scene.py` `_add_camera_stand()`:
```python
# WRONG — looks along -X (horizontal):
orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True)

# CORRECT — tilt ~35° from vertical toward +Y (workspace side):
# Rx(-35°) rotates the -Z look direction toward +Y (tilts camera to look down and forward)
# Tune the angle to match the viewport view (workspace visible, ~30-45° from vertical)
orientation=rot_utils.euler_angles_to_quats(np.array([-35, 0, 0]), degrees=True)
```

**Isaac Sim camera orientation convention:**
- Identity quaternion `[1,0,0,0]` = camera looks straight down (-Z in world Z-up frame)
- `Rx(angle)` tilts the look direction toward the +Y or -Y axis
- `Ry(angle)` tilts the look direction toward the +X or -X axis
- The camera stand pole is at `(0.35, -0.90)` — workspace center is at `(0.38, 0.0)`,
  so tilting toward +Y (Rx negative angle) aims at the workspace
- Start with `Rx(-35°)` and tune in 5° increments

### DLSS Sub-Resolution Issue (non-headless mode)

- 640×480 camera → DLSS renders internally at 320×240 (50% scale)
- 240 < DLSS minimum 300px → `ERROR_OUT_OF_DEVICE_MEMORY`
- **With ONE camera + viewport**: warning appears but MAY NOT crash (Stage 4 never called `get_rgba()` so DLSS render product was never activated — no OOM)
- **With TWO cameras + viewport**: definitely OOMs
- `headless=True` disables DLSS entirely (safe workaround)
- **Resolution options**: 640×480 (current, may warn), 800×600 (safe, ~25% more VRAM), 1280×720 (large)
- Keep 640×480 first; if OOM when calling `get_rgba()`, bump to 800×600

### capture_frame.py — Deleted by Revert

`src/capture_frame.py` was a standalone script for testing camera without the arm controller.
It was deleted when reverting to Stage 4 commit. If needed:
- Create `src/capture_frame.py` that runs `headless=False`, `with_controller=False`
- Skip `PickPlaceController` init to avoid Warp/LLVM CUDA JIT OOM
- Save captured frames to `debug/` with timestamp

### Stage 5 Camera Work Plan

1. Fix orientation bug (change `Ry(90°)` → `Rx(-35°)` or similar)
2. Run `main.py` (headless=False, existing pipeline) — verify camera sees scene in viewport
3. Call `IsaacScene.capture_frame()` once before the sort loop — save to `debug/`
4. Check `debug/` image — confirm table + colored cubes + bins are visible
5. If image is black/corrupt: increase resolution to 800×600 or run headless=True
6. Wire `capture_frame()` output to `PerceptionClient.analyze_frame()` (Stage 5)

### Camera Position Geometry

Stand pole: `(x=0.35, y=-0.90)` — behind the table edge (table goes to y=-0.60)
Camera prim: world position `(x=0.38, y=0.0, z=1.35)` — above workspace center
- The camera prim position is independent of the stand pole prim hierarchy (`position=` is world-space)
- Workspace objects: x≈0.35–0.55, y≈-0.15–+0.38, z≈0.425 (table top + cube)
- Camera height 1.35m, workspace at 0.40m → vertical gap ≈ 0.95m
- At Rx(-35°): camera looks down and toward +Y → should cover x=0.1–0.7, y=-0.5–+0.6 approximately

---

## 15. Model Downsizing Decision (2026-03-04)

### Why we switched from 7B/8B to 3B/1.5B

The original design used `qwen2.5vl:7b` (~5.5 GB) for perception and `deepseek-r1:8b` (~5.2 GB) for reasoning.
When we first attempted to run Isaac Sim + VLM together on the RTX 4070, we hit a wall.

**Problem 1 — GPU VRAM contention:**
Isaac Sim headless consumes ~4.5–5 GB VRAM. `qwen2.5vl:7b` needs ~5.5 GB.
Combined: ~10–11 GB → over budget on a 12 GB card once Windows WDDM overhead is counted.
Ollama returned: `memory layout cannot be allocated (status code: 500)` within 2 seconds.

**Problem 2 — Windows virtual address space fragmentation:**
Even in CPU mode (`CUDA_VISIBLE_DEVICES=-1`), the OS couldn't find a contiguous block to mmap the 7B model
into RAM. This is a Windows-specific issue: even with 20+ GB free RAM, fragmentation prevents large
contiguous allocations. Symptom: same 500 error in 2 seconds even with `total_vram="0 B"` confirmed.

**Fix: PC restart** clears the virtual address space. After restart, CPU mode loaded `qwen2.5vl:7b`
partially (16-second wait) but then crashed during vision encoding (RAM spike for the image encoder).

**Decision:** Switch to `qwen2.5vl:3b` (~2.5 GB) and `deepseek-r1:1.5b` (~1.1 GB).
These were recommended by Gemini as a Consulted AI during a parallel debugging session (see `docs/HANDOFF_GEMINI.md`).

### moondream — tested as a lightweight fallback

Before confirming qwen2.5vl:3b worked, we tested `moondream` (~1.7 GB) as an even smaller fallback.

**What moondream returned** (38k chars, `debug/20260304_210849_vlm_raw.txt`):
- Output structure: bare JSON array `[{...}]` instead of the required `{"objects": [...]}`
- First 3 entries had **correct class labels**: `obj_1=ClassA, obj_2=ClassB, obj_3=ClassC`
- **4-element coords**: `[x, y, z, w]` instead of `[x, y, z]`
- **Hallucination loop**: after the 3 real objects, moondream repeated ~200 copies of near-identical
  entries (`obj_4..obj_19`, all ClassA) filling the entire 38k response

**Conclusion:** moondream is a captioning model, not an instruction-following model. It doesn't respect
complex JSON schemas and hallucinates repetitive output under structured prompting. Not suitable for this task.

**However**, the moondream test produced two useful parser improvements that remain in `vlm_client.py`:
1. `_clean_response` now handles bare arrays (wraps them as `{"objects": [...]}`)
2. `_validate_scene` truncates coords > 3 elements (was `!= 3`, now `< 3` + `[:3]` slice)

### Ollama CPU mode procedure (if GPU fails again)

Kill all Ollama instances including the system tray icon, then:
```powershell
$env:CUDA_VISIBLE_DEVICES = "-1"
$env:OLLAMA_NUM_GPU = "0"
ollama serve
```
Confirm CPU mode: look for `inference compute id=cpu library=cpu` and `total_vram="0 B"` in Ollama logs.
CPU inference with moondream: ~8–9 minutes per frame. Not practical for regular use.

### Current VLM status
- `qwen2.5vl:3b` is the active model (set as default in `vlm_client.py`)
- Confirmed to run on GPU with sufficient VRAM headroom
- Parser hardened to handle both `{"objects":[...]}` and bare array `[...]` responses
- Fallback: if VLM parse fails, pipeline uses exact Isaac Sim world positions and continues sorting
