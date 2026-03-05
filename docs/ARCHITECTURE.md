# Architecture Document
## Autonomous Sorting System

---

## 1. System Overview

A modular robotic sorting system built on a **Perceive → Reason → Act** pipeline.
The robot identifies three objects from distinct classes, decides which bin each belongs in,
and executes the pick-and-place sequence — recovering gracefully from physical failures.

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Autonomous Sorting System                        │
│                                                                     │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐  │
│  │  PERCEIVE    │───▶│    REASON    │───▶│        ACT           │  │
│  │              │    │              │    │                      │  │
│  │ Qwen2.5-VL   │    │ DeepSeek-R1  │    │  FrankaRobot         │  │
│  │ 3B via Ollama│    │ 1.5B via     │    │  (Isaac Sim +        │  │
│  │              │    │ Ollama       │    │   PickPlaceCtrl)     │  │
│  └──────────────┘    └──────────────┘    └──────────────────────┘  │
│         ▲                                         │                │
│         │           ┌──────────────┐              │                │
│         └───────────│  STATE       │◀─────────────┘                │
│                     │  MACHINE     │  StatusCode enum              │
│                     │  (main.py)   │  200/201/202 = success        │
│                     │              │  4xx = error → retry/halt     │
│                     └──────────────┘                               │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Data Flow

```
Isaac Sim RGB-D Camera
        │
        │  (640×480 JPEG + depth array)
        ▼
PerceptionClient.perceive()                     src/perception/vlm_client.py
  ├─ capture_frame()        → JPEG bytes + raw BGR frame
  ├─ analyze_frame()        → VLM call → JSON parse + validate
  └─ Output: scene_metadata dict
        │
        │  {"objects": [{"id","class_label","coords":[x,y,z]}, ...]}
        ▼
IsaacScene.localize_by_color()                  src/isaac_scene.py
  ├─ HSV color segmentation per class label
  ├─ Depth mask: only pixels 1.00–1.95 m from camera (rejects floor)
  ├─ Contour centroid → _pixel_to_world() back-projection
  └─ Overrides VLM coords with precise camera-measured world XY
        │
        │  coords updated to [x, y, _OBJ_Z] in robot frame (metres)
        ▼
ReasoningClient.generate_sort_plan()            src/reasoning/llm_client.py
  ├─ Sends scene_metadata to DeepSeek-R1
  ├─ Strips <think>...</think> blocks
  └─ Output: sort_plan list
        │
        │  [{"name","class_label","coords","target_box"}, ...]
        ▼
SortingStateMachine.run(sort_plan)              src/main.py
  └─ For each object:
       goto_pose(x,y,z) → execute_grasp() → place_in_box(id)
       On OBJECT_FELL: camera re-localize → retry (up to 3×)
        │
        │  method calls + StatusCode returns
        ▼
FrankaRobot                                     src/execution/hardware_api.py
  ├─ goto_pose()      → validates reach, stores _pick_pos
  ├─ execute_grasp()  → no-op (PickPlaceController handles grasp)
  └─ place_in_box()   → run_pick_and_place() + verify_cube_in_bin()
        │
        ▼
IsaacScene.run_pick_and_place()                 src/isaac_scene.py
  ├─ PickPlaceController.forward() loop (up to 2000 steps)
  ├─ RMPflow avoids table top + 12 bin walls (explicitly registered)
  ├─ is_object_near(place_pos) after settle → confirms physical success
  └─ Returns bool → place_in_box decides PLACE_SUCCESS or OBJECT_FELL

IsaacScene.verify_cube_in_bin()
  ├─ Overhead camera frame → localize_by_color()
  ├─ Not visible → inside bin walls → confirmed ✓
  ├─ Within 20 cm of bin → confirmed ✓
  └─ > 20 cm from bin → still on table → OBJECT_FELL + _reloc_pos
```

---

## 3. Class Structure

### `src/main.py`

```
SortingStateMachine
├─ robot: FrankaRobot | MockFrankaRobot
├─ run(sort_plan)
│    └─ _sort_one(obj) → goto_pose → execute_grasp → place_in_box
│         └─ retry loop (MAX_RETRIES = 3)
└─ Pipeline flags (module-level):
     USE_ISAAC_SIM, USE_VLM, USE_LLM, DEMO_MODE, RANDOM_SPAWN
```

### `src/perception/vlm_client.py`

```
PerceptionClient
├─ model: str          (qwen2.5vl:3b)
├─ camera_index: int
├─ perceive()
├─ capture_frame()     → (jpeg_bytes, bgr_frame)
├─ analyze_frame()     → scene_metadata dict
├─ _clean_response()   → strips markdown fences
└─ _validate_scene()   → enforces schema
```

### `src/reasoning/llm_client.py`

```
ReasoningClient
├─ model: str          (deepseek-r1:1.5b)
├─ generate_sort_plan(scene_metadata) → sort_plan list
├─ _clean_response()   → strips <think> blocks + fences
└─ _validate_plan()    → enforces required fields
```

### `src/execution/hardware_api.py`

```
StatusCode (Enum)
├─ 200 AT_TARGET        202 PLACE_SUCCESS
├─ 201 GRASP_SUCCESS
├─ 401 IK_UNREACHABLE   404 MISSING_OBJ     405 DOES_NOT_FIT  [fatal]
├─ 403 OBJECT_SLIPPED   406 OBJECT_FELL                       [retry]
└─ 402 COLLISION                                              [fatal]

MockFrankaRobot           ← used when USE_ISAAC_SIM=False
├─ goto_pose(x,y,z)    → AT_TARGET  | IK_UNREACHABLE  (random, ~10%)
├─ execute_grasp()     → GRASP_SUCCESS | OBJECT_SLIPPED (random, ~10%)
└─ place_in_box(id)    → PLACE_SUCCESS | OBJECT_FELL    (random, ~10%)

FrankaRobot               ← used when USE_ISAAC_SIM=True
├─ _pick_pos: np.ndarray           stored by goto_pose()
├─ _reloc_pos: np.ndarray | None   camera-fresh position for retry
├─ _demo_drop: bool                Video 2 flag (see demo_drop_second)
├─ goto_pose(x,y,z)    → AT_TARGET | IK_UNREACHABLE (reach check)
│    └─ uses _reloc_pos if set (overrides stale VLM coords on retry)
├─ execute_grasp()     → GRASP_SUCCESS (always; PickPlaceCtrl handles grasp)
└─ place_in_box(id)
     ├─ run_pick_and_place() → bool
     │    True  → verify_cube_in_bin() → PLACE_SUCCESS | OBJECT_FELL + _reloc_pos
     │    False → localize_by_color() table scan → OBJECT_FELL + _reloc_pos
     └─ force_drop path: gripper-state detection (finger joint < 0.03 m)
```

### `src/isaac_scene.py`

```
IsaacScene
│
├─ Scene construction
│   ├─ _add_ground_and_table()
│   ├─ _add_bins()              3 bins + RMPflow obstacle registration
│   ├─ _add_franka()            Franka Panda + ParallelGripper
│   ├─ _add_camera_stand()      Overhead RGB-D camera at [0.35, 0.00, 2.20]
│   └─ _add_demo_camera()       /World/DemoCamera — perspective recording view
│
├─ Simulation control
│   ├─ setup()                  build scene + reset world
│   ├─ settle_physics(steps)    step without rendering to settle objects
│   ├─ randomize_object_positions()   random spawn (RANDOM_SPAWN=True)
│   ├─ park_arm_for_capture()   fold arm away before VLM snapshot
│   └─ return_home()            smooth arm return to upright home pose
│
├─ Perception helpers
│   ├─ capture_frame()          → (jpeg, bgr, depth_array)
│   ├─ _pixel_to_world(px,py,d) pinhole back-projection
│   └─ localize_by_color(label, bgr, depth)   HSV seg → world XY
│
├─ Execution helpers
│   ├─ get_bin_position(box_id) → np.ndarray
│   ├─ get_object_world_position(obj_id) → np.ndarray
│   ├─ is_object_near(pos, tol=0.12)     proximity check
│   └─ verify_cube_in_bin(label, bin_pos) → (confirmed, fresh_xyz|None)
│
└─ Pick-and-place
    └─ run_pick_and_place(pick, place, force_drop=False)
         ├─ PickPlaceController.forward() loop
         ├─ force_drop: monitor finger joint, override to open on grasp
         └─ Returns bool (True = object near bin after settle)
```

---

## 4. Error Recovery Flow

```
SortingStateMachine._sort_one(obj)
│
├─ goto_pose(x, y, z)
│     IK_UNREACHABLE / COLLISION → HARD HALT (object skipped, logged)
│
├─ execute_grasp()
│     OBJECT_SLIPPED → retry from goto_pose (up to MAX_RETRIES=3)
│
└─ place_in_box(id)
      PLACE_SUCCESS → done ✓
      │
      OBJECT_FELL   → retry from goto_pose
                        │
                        goto_pose checks _reloc_pos:
                          set   → use camera-fresh XY (cube moved)
                          None  → use original VLM coords (cube at same spot)
```

**OBJECT_FELL sources and their _reloc_pos behaviour:**

| Cause | `run_pick_and_place` returns | `_reloc_pos` set? | Retry coords |
|-------|------------------------------|-------------------|--------------|
| Controller timeout / missed grasp | False | Yes, if cube visible on table | Camera-fresh or original |
| force_drop (gripper opened mid-transit) | False | Yes (cube near pick pos) | Camera-fresh |
| Placed but bounced out of bin | True → verify fails | Yes (fresh_xyz returned) | Camera-fresh |
| Placed but color not visible in bin | True → verify: not visible → ✓ | No (confirmed in bin) | N/A |

---

## 5. Key Constants & Configuration

### Scene geometry (`src/isaac_scene.py`)

| Constant | Value | Meaning |
|----------|-------|---------|
| `TABLE_H` | 0.40 m | Table surface height |
| `_OBJ_Z` | 0.425 m | Cube centre height (pick target Z) |
| `BIN_POSITIONS[1]` | [0.45, 0.38, 0.54] | Box 1 — front-left |
| `BIN_POSITIONS[2]` | [0.73, 0.01, 0.54] | Box 2 — right edge |
| `BIN_POSITIONS[3]` | [0.55, -0.36, 0.54] | Box 3 — front-right |
| `SPAWN_X_RANGE` | (0.35, 0.60) m | Cube spawn X range |
| `SPAWN_Y_RANGE` | (-0.22, 0.25) m | Cube spawn Y range |
| `SPAWN_MIN_SEP` | 0.25 m | Minimum cube-to-cube separation |
| `end_effector_initial_height` | 0.65 m | PickPlaceController hover height |
| Camera position | [0.35, 0.00, 2.20] | Overhead stand camera |

### Color-to-class mapping (HSV, `src/isaac_scene.py`)

| Class | Color | H range | S min | V min |
|-------|-------|---------|-------|-------|
| ClassA | Red | 0–10 + 170–180 | 120 | 70 |
| ClassB | Blue | 100–130 | 120 | 70 |
| ClassC | Green | 40–80 | 70 | 70 |

Depth mask applied before segmentation: `1.00 m < depth < 1.95 m` (rejects floor at 2.20 m).

---

## 6. Isaac Sim Integration Notes

**Python environment:** Isaac Sim 5.1.0 bundled Python 3.11 (`python.bat`). Do not use system Python for Isaac Sim modes.

**Import style (5.x):**
```python
from isaacsim import SimulationApp
from isaacsim.core.api import World
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
```

**RMPflow + collision:** RMPflow only avoids geometry **explicitly registered** via `add_obstacle()`. USD/PhysX collision and RMPflow obstacle avoidance are independent systems. 13 obstacles registered: 1 table top + 4 walls × 3 bins.

**Camera VRAM:** Each `camera.initialize()` allocates an RTX render target. Only one overhead camera active (the stand camera at `/World/Camera`). VLM runs 0/37 GPU layers (full CPU, ~50 s) due to VRAM pressure from Isaac Sim — works correctly due to prompt hardening.

**`release_camera()` is a no-op:** `camera.get_render_product_path()` returns the shared Hydra prim `/Render/OmniverseKit/HydraTextures/Replicator`. Removing it via `stage.RemovePrim()` breaks the render pipeline. Left as documented no-op.

---

## 7. Pipeline Mode Flags

Edit the top of `src/main.py` before running:

| `USE_ISAAC_SIM` | `USE_VLM` | `USE_LLM` | Mode | Python to use |
|-----------------|-----------|-----------|------|---------------|
| False | False | False | Pure mock — hardcoded objects, MockFrankaRobot | System Python |
| False | False | True | Mock scene → DeepSeek-R1 → MockFrankaRobot | System Python |
| True | False | False | Isaac Sim smoke test — hardcoded scene | python.bat |
| True | True | True | **Full pipeline** — camera → VLM → LLM → FrankaRobot | python.bat |

Additional flags:
- `DEMO_MODE = True` — `headless=False` + perspective DemoCamera viewport
- `RANDOM_SPAWN = True` — cubes teleported to random positions before each run
- `demo_drop_second=True` in `FrankaRobot(scene, ...)` — Video 2 demo drop (remove for production)
