# Autonomous Sorting System — Claude Memory

## Project Location
`D:\Autonomous-Sorting-System` (Windows, git repo on main branch)

## What This Project Is
Home assignment for a Robotics Engineer position.
Modular robotic sorting system: **Perceive → Reason → Act** pipeline.
Identifies 3 objects, decides which box each belongs in, executes pick-and-place with error recovery.

## Key Files
| File | Role |
|------|------|
| `src/main.py` | Orchestrator + SortingStateMachine + pipeline mode flags |
| `src/perception/vlm_client.py` | PerceptionClient — webcam + Qwen2.5-VL 7B via Ollama |
| `src/reasoning/llm_client.py` | ReasoningClient — DeepSeek-R1 8B via Ollama |
| `src/execution/hardware_api.py` | MockFrankaRobot + StatusCode enum |
| `docs/HANDOFF.md` | Full architecture, integration plan, remaining stages |
| `README.md` | Assignment deliverable — architecture document |

## Pipeline Mode Flags (top of src/main.py)
| USE_VLM | USE_LLM | Mode |
|---------|---------|------|
| False | False | Hardcoded objects → mock robot (no deps) |
| False | True | Mock scene → DeepSeek-R1 → mock robot |
| True | True | Webcam → Qwen VL → DeepSeek-R1 → mock robot |

Currently set to: `USE_ISAAC_SIM = True, USE_VLM = True, USE_LLM = True` (Mode 6 — full pipeline)
Additional flags: `DEMO_MODE = False` (headless=False + perspective cam), `RANDOM_SPAWN = True` (random cube positions)

## Target Hardware
- Windows machine, NVIDIA RTX 4070 (12GB VRAM), i9-13700KF, 32GB RAM
- Isaac Sim installed

## Remaining Stages (as of 2026-03-05)
- **Stage 4 DONE** — Isaac Sim scene working: 3/3 sort success, clean arm motion ✓
- **Stage 5 DONE** — Full VLM pipeline: qwen2.5vl:3b labels + HSV color seg + depth back-projection → 3/3 sort, ~10mm XY ✓
- **Stage 5b DONE** — VLM robustness: prompt hardening + degenerate-class guard + render fix ✓
  - Phases 1-3 verified: smoke test ✓, full pipeline ✓, random spawn ✓
- **Stage 6** — Prompt/parser hardening (optional, skip if time-constrained)
- **Stage 7 NEXT** — Demo videos
  - Phase 4 PENDING: demo camera (DEMO_MODE=T, USE_VLM=F, USE_LLM=F) — visual check only
  - Video 1: DEMO_MODE=T, RANDOM_SPAWN=T, USE_VLM=T, USE_LLM=T
  - Video 2: USE_ISAAC_SIM=F → MockFrankaRobot failure recovery

## Isaac Sim Installation
- Version: **5.1.0-rc.19** (standalone, no Omniverse Launcher)
- Location: `D:\isaac-sim-standalone-5.1.0-windows-x86_64\`
- Bundled Python: **3.11** — use `python.bat` for Isaac Sim scripts, NOT system Python
- Launch GUI: `isaac-sim.bat`
- Import style (5.x): `from isaacsim import SimulationApp` (NOT `from omni.isaac.kit`)

## Isaac Sim Integration — Only 2 Things Need Changing
1. `vlm_client.py` → replace `capture_frame()` with Isaac Sim RGB-D feed
2. `hardware_api.py` → add `FrankaRobot` class with Lula IK; swap import in `main.py`

## Known Issues / Notes
- `qwen2.5vl:7b` OOMs on the old laptop — must test on the RTX 4070 machine (this one)
- `deepseek-r1:8b` now resolves to Qwen3-based R1-0528 distill (~5.2GB)
- Z coordinate hardcoded to 0.3 on webcam; real depth comes from Isaac Sim now
- VLM coords replaced by HSV color seg + depth back-projection (Stage 5c) — ~10mm XY accuracy
- Blue cube BIN_POSITIONS[2] placement slightly off — minor XY tuning needed (not blocking)
- VLM runs 0/37 GPU layers (full CPU, ~50s) when Isaac Sim is active — VRAM too tight. Works correctly due to prompt fix + degenerate-class guard.
- `camera.get_render_product_path()` returns shared Hydra prim `/Render/OmniverseKit/HydraTextures/Replicator`. DO NOT call `stage.RemovePrim()` on it — breaks render pipeline with error code 6 on every world.step(). `release_camera()` is a no-op.

## Overhead Camera — CONFIRMED WORKING (2026-03-04)
- **Position**: `[0.35, 0.00, 2.20]`, orientation `Ry(90°)` = `euler_angles_to_quats([0, 90, 0])` → straight down
- **FOV**: `focal_length=8.0mm` set via `UsdGeom.Camera(...).GetFocalLengthAttr().Set(8.0)` AFTER Camera() constructor (constructor rejects `focal_length` kwarg)
- **Physical rig**: vertical pole (0.35, -0.90) + horizontal arm → camera above workspace center. Camera at top-level `/World/Camera` (NOT child of pole — avoids scale inheritance)
- **Sight cone**: child of `/World/Camera/SightCone` → moves with camera in viewport
- Resolution 640×480. Full workspace (table + all bins) visible in Camera viewport.

## Camera GPU Memory Constraints (discovered 2026-03-04)
- Each `camera.initialize()` allocates a full RTX render target on first `world.step(render=True)`
- Main viewport + 2 cameras = 3 RTX render targets → OOM on 12 GB RTX 4070
- **DLSS root cause**: 640×480 → DLSS renders at 320×240 (50% scale) → height 240 < DLSS min 300 → OOM. `headless=True` disables DLSS.
- Stage 4 (main.py headless=False) worked because `get_rgba()` was never called → DLSS render product never activated → no OOM.
- With ONE camera non-headless: calling `get_rgba()` MAY warn but not crash. Try 640×480 first; bump to 800×600 if OOM.
- `with_controller=False` skips PickPlaceController/RMPflow → avoids Warp/LLVM CUDA JIT OOM (for camera-only testing).

## Isaac Sim PickPlaceController — Key Learnings
- `end_effector_initial_height` = hover height for Phase 0 (approach) and Phase 4 (lift). **Default 0.3m is below table (0.40m)** — always set this above table surface. We use **0.65m**.
- RMPflow is the internal motion planner inside PickPlaceController (`_cspace_controller`). It only avoids geometry **explicitly registered** via `add_obstacle(obj, static=True)`. USD/PhysX collision and RMPflow are **independent systems**.
- Total cycle needs ~2284 physics steps (`events_dt` sum). `max_steps=2000` works in practice.
- 13 obstacles registered: 1 table top + 4 walls × 3 bins = 13.
- BIN_POSITIONS z set to `TABLE_H + 0.14 = 0.54m` (4cm above bin wall tops at 0.50m) for clean drop-in.

## Assignment Deliverables Status
1. Architecture document (`README.md`) — DONE
2. Python implementation — DONE
3. Video: successful 3-object sort — PENDING
4. Video: failure recovery — PENDING
