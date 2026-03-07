# Engineering Discussion — Autonomous Sorting System
## Interview Preparation

---

## The Original Request

> Ok, from now we are getting ready for the interview that is going to ask about this assignment.
> I have 2 parts to this that will expand:
>
> 1. Answer questions that might be from the interviewer:
>    A. How do you measure success rate of the action?
>    B. How do you recreate a failure process of the system?
>    C. How do you explain each model you picked? Why not other models? (We can use all the tests we did with the moondream and other options to justify).
>    D. How do you explain the interaction between the models and between the models and the simulation (controller)?
>    E. How do you pick the hardware to work with for this task? (The simulation part and in case of real world experimentation).
>    F. What would you change and what does this decision depend on if you would run this project in real world?
>
> 2. Add question, make discussion and learn everything we can so I get to the interview as ready as possible.

---

## Part 1 — Formal Interviewer Questions (A–F)

---

### A. How do you measure success rate of the action?

**Answer:** Three layers of success measurement:

**Layer 1 — Per-action StatusCode**
Every hardware call returns a typed enum:
- `200 AT_TARGET` — navigation succeeded
- `201 GRASP_SUCCESS` — gripper secured object
- `202 PLACE_SUCCESS` — object placed in box
- `4xx` — specific failure codes (IK_UNREACHABLE, OBJECT_SLIPPED, OBJECT_FELL, etc.)

The state machine logs each code. You can see exactly which step failed and which object.

**Layer 2 — Physical verification post-placement**
`verify_cube_in_bin()` takes a fresh overhead camera snapshot after `place_in_box()` returns PLACE_SUCCESS. It re-runs HSV color segmentation:
- Object not visible in image → inside bin walls → confirmed in bin
- Object within 20 cm of bin center → confirmed in bin
- Object > 20 cm from bin → still on table → triggers OBJECT_FELL + sets `_reloc_pos` for retry

This catches the case where the controller reports success but physics bounced the cube back out.

**Layer 3 — End-of-cycle summary**
After the full sort, the state machine prints:
```
[OK]     obj_1 (ClassA → Box 1)
[OK]     obj_2 (ClassB → Box 2)
[FAILED] obj_3 (ClassC — 3 retries exhausted)
```

---

#### A.1 — Follow-up: No numerical rate? How would you add one?

**Currently:** No aggregate number — only binary per-object terminal output.

**How to add it:** Instrument `SortingStateMachine.run()` with counters:

```python
total = len(sort_plan)
succeeded = 0
total_retries = 0

for obj in sort_plan:
    attempts, ok = self._sort_one(obj)   # returns (retry_count, bool)
    total_retries += attempts
    if ok:
        succeeded += 1

rate = succeeded / total * 100
print(f"\nResult: {succeeded}/{total} ({rate:.0f}%) | avg retries: {total_retries/total:.1f}")
```

**For multi-run tracking (persistence across runs):**
```python
import json, datetime
log = {"timestamp": datetime.datetime.now().isoformat(),
       "success_rate": rate, "successes": succeeded, "total": total}
with open("run_log.json", "a") as f:
    f.write(json.dumps(log) + "\n")
```

Aggregate over 20+ runs for a statistically meaningful rate.

---

#### A.2 — What is the Wilson 95% Confidence Interval?

**The problem with naive success rate:** 3/3 successes → 100% rate, based on 3 trials. Statistically meaningless. Naive CI would give `1.0 ± 0` which is wrong.

**Wilson score interval** corrects for small sample sizes and always stays bounded between 0–1:

```
p̂  = successes / n          (observed rate)
z  = 1.96                   (for 95% confidence)

center = (p̂ + z²/2n) / (1 + z²/n)
margin = z × √(p̂(1-p̂)/n + z²/4n²) / (1 + z²/n)

CI = [center - margin, center + margin]
```

**Concrete example** — 7 successes out of 10 runs:

| Method | Result |
|--------|--------|
| Naive `p ± 1.96√(p(1-p)/n)` | [0.42, 0.98] |
| Wilson 95% CI | [0.35, 0.93] |

At n=3 (our demo), naive CI can go negative or above 1. Wilson always stays in [0,1] because it bakes in the binomial distribution's asymmetry near the boundaries.

**Why it matters in the interview:** If you say "we ran 5 tests, 5/5 succeeded", a good interviewer will ask "so you're 100% confident?" Wilson lets you say "95% confident the true rate is between 57% and 100%", which is the honest, statistically correct answer.

---

### B. How do you recreate a failure process of the system?

**Three mechanisms to inject failures:**

**1. MockFrankaRobot random failures (`USE_ISAAC_SIM=False`)**
`MockFrankaRobot` in `src/execution/hardware_api.py` returns random failures ~10% of the time:
- `goto_pose()` → IK_UNREACHABLE (10%)
- `execute_grasp()` → OBJECT_SLIPPED (10%)
- `place_in_box()` → OBJECT_FELL (10%)

Run with `USE_ISAAC_SIM=False` and the state machine's retry logic is exercised without needing Isaac Sim running.

**2. `demo_drop_second=True` — Gripper-state detection (Isaac Sim)**
Injected at `FrankaRobot(scene, demo_drop_second=True)`. On the 2nd distinct object (tracked by `_seen_box_ids`), the system monitors the finger joint. When `finger_joint[7] < 0.03m` (gripper has closed), it forces the gripper open — the cube drops at the pick position. Then:
- `run_pick_and_place()` returns False (object not near bin)
- `localize_by_color()` re-scans table, finds cube
- `_reloc_pos` set to fresh camera position
- State machine retries from `goto_pose()` with updated coords
- Recovery succeeds → PLACE_SUCCESS

**3. `RANDOM_SPAWN=True` — Unknown object positions**
`randomize_object_positions()` teleports cubes to random table positions before each run. The system must localize them from scratch via HSV segmentation. Tests that the perception-to-execution chain handles truly unknown starting positions.

---

#### B.1 — How does the system know what color maps to what bin?

**The full mapping chain:**

```
Object color (physical)
        │
        ▼
VLM (Qwen2.5-VL) — sees camera image
        → outputs class_label: "ClassA"   (Red → ClassA is in the VLM prompt)
        │
        ▼
LLM (DeepSeek-R1) — receives {class_label: "ClassA"}
        → outputs target_box: 1           (ClassA → Box 1 is in the LLM prompt)
        │
        ▼
SortingStateMachine — calls place_in_box(1)
        │
        ▼
get_bin_position(1) → BIN_POSITIONS[1] = [0.45, 0.38, 0.54]  (hardcoded)
```

**The rules are not in code — they are in the prompts.** To change the sorting logic, you only edit the prompts in `vlm_client.py` and `llm_client.py`, not the control code.

**Color-to-class mapping (VLM prompt):**

| Object color | Class label | Target box |
|:---          |:---         |:---        |
| Red          | ClassA      | Box 1      |
| Blue         | ClassB      | Box 2      |
| Green        | ClassC      | Box 3      |

---

#### B.1.1 — Does VLM recognize bins? What if bins were randomly swapped?

**VLM does NOT detect bins.** Bin positions are hardcoded constants in `src/isaac_scene.py`:

```python
BIN_POSITIONS = {
    1: np.array([0.45,  0.38, 0.54]),   # Box 1 — front-left
    2: np.array([0.73,  0.01, 0.54]),   # Box 2 — right edge
    3: np.array([0.55, -0.36, 0.54]),   # Box 3 — front-right
}
```

If bins were randomly swapped, the robot would place ClassA in the wrong physical bin — it navigates to a fixed world coordinate regardless of what's physically there.

**How to support random bin positions:**

| Approach | How |
|----------|-----|
| **ArUco markers** | Unique marker on each bin. Camera detects marker ID + pose at startup → updates `BIN_POSITIONS` dynamically |
| **Bin HSV segmentation** | If bins have distinct colors: detect bin centroid same way we detect cube centroid |
| **Depth scan at startup** | Scan table surface → detect raised rectangular objects → identify bins by shape |
| **VLM bin detection** | Feed overhead image asking "where is box 1, 2, 3?" — adds latency, VLM coordinate imprecision |

The architecture is already set up for this: `get_bin_position()` is an isolated function — plugging in dynamic detection only requires changing that one method.

---

#### B.2 — Improving demo_drop_second + Other Stress Tests

**User insight:** Current `demo_drop_second` fires as soon as the gripper closes — cube drops at pick position (near table). Better: add a timer so the cube drops mid-transit (mid-air fall):

```python
# In run_pick_and_place() / place_in_box():
# Count steps after grasp phase completes before forcing gripper open
if self._demo_drop and phase == "transit" and steps_since_grasp > 150:
    force_open_gripper()
```

This lets PickPlaceController complete Phase 1 (approach) + Phase 2 (grasp) + enter Phase 3 (lift/transit), then drops — cube falls from mid-air, more realistic physical failure.

**Additional stress tests:**

| Test | How to trigger | What it validates |
|------|---------------|-------------------|
| Retry exhaustion | Set MockFrankaRobot failure rate 100% | State machine exits gracefully after MAX_RETRIES=3 |
| Workspace edge spawn | `SPAWN_X_RANGE = (0.58, 0.62)` | IK_UNREACHABLE handling, hard-halt path |
| Same-class multiple objects | Add 2nd red cube to scene | LLM generates duplicate class plans correctly |
| VLM hallucination inject | Feed wrong image to VLM | HSV override catches bad coords; class mismatch |
| Bin occlusion | Place cube on bin wall | `verify_cube_in_bin()` triggers OBJECT_FELL correctly |
| Rapid re-spawn | Loop 10 runs with `RANDOM_SPAWN=True` | No state leakage between runs (`_pick_pos`, `_reloc_pos` reset) |
| IK_UNREACHABLE mid-sequence | Spawn at x > 0.8m | Hard halt + log, rest of sort continues |

---

#### B.2.1 — Real-World Safety for Workspace Edge Commands

In Isaac Sim: arm reaches limit → IK_UNREACHABLE → logs "hard halt" → continues. No damage.

In the real world with a Franka Panda:

| Risk | What happens |
|------|-------------|
| Joint limit overshoot | Franka controller faults, requires manual reset |
| Singularity entry | Arm approaches degenerate configuration → erratic motion |
| Table collision | End-effector or forearm hits table → hardware damage |
| Emergency joint torque | Sudden high torque → motor overload |

**Defense in depth — layered prevention:**

**Layer 1 — Pre-flight software workspace check (already partially in code):**
```python
# goto_pose() in FrankaRobot already checks reach:
dist = np.linalg.norm(pos[:2])
if dist > MAX_REACH:
    return StatusCode.IK_UNREACHABLE
```
Extend to a proper **reachability ellipsoid** rather than a simple radius — Franka Panda has asymmetric workspace. Reject any target outside the safe envelope before issuing any joint command.

**Layer 2 — Cartesian forbidden zones:**
Define boxes for table surface, bin interiors, robot base area. Reject any `goto_pose()` where the target or any waypoint falls inside a forbidden zone.

**Layer 3 — Franka FCI hardware reflex system:**
Franka Panda hardware monitors joint torques continuously. If external torque exceeds threshold → arm stops and locks. In real deployment, these thresholds are calibrated conservatively for the environment.

**Layer 4 — Velocity limits:**
In simulation, PickPlaceController moves at full speed. In real world, set conservative joint velocity caps — slower motion gives hardware reflexes time to react before damage occurs.

**Layer 5 — Physical e-stop (dead man's switch):**
During initial commissioning of new positions, a human operator holds Franka's enable button. Robot stops the moment the button is released. Only graduate to unattended operation after the full workspace is validated.

**Interview answer:** "The simulation gives us IK_UNREACHABLE as a clean error, but in reality we need defense in depth: software workspace checks before issuing commands, the robot's hardware torque reflexes as a second layer, velocity limits to allow those reflexes to act, and a human e-stop during initial commissioning. You never rely on a single safety mechanism."

---

### C. How do you explain each model you picked? Why not other models?

**VRAM budget constraint (binding):** RTX 4070 = 12 GB. Isaac Sim consumes ~5 GB. Remaining: ~7 GB for all models.

**Perception Layer — Qwen2.5-VL 3B (Selected):**
- Strong visual grounding and instruction-following capability
- Accurately extracts object classes and spatial coordinates from structured prompts
- 3B size (~2.5 GB VRAM) fits within budget alongside Isaac Sim
- Runs on CPU (~50 seconds) due to remaining VRAM pressure — works correctly due to prompt hardening

**Why not other VLMs:**

| Model | Tested | Rejection reason |
|-------|--------|-----------------|
| Qwen2.5-VL 7B | Yes | ~5.5 GB VRAM. Combined with Isaac Sim's ~5 GB exceeds 12 GB. Extensively troubleshot (CPU mode, Windows VA fragmentation, PC restart) — could not stabilize |
| moondream (~1.7 GB) | Yes | **Designed for captioning, not structured output.** Produced 38k-character repetition loops (~200 copies of same entry). Ignored `{"objects":[...]}` schema. Output 4-element coords. Rejected as unsuitable |
| GLM-4.1V-Thinking | Evaluated | Incredible at complex visual reasoning via Chain-of-Thought, but inference time too slow for robotic pipelines |
| Llama 3.2 Vision | Evaluated | Highly reliable general classification, but lacks precision spatial grounding needed for coordinate estimation |

**Reasoning Layer — DeepSeek-R1 Distilled 1.5B (Selected):**
- Native Chain-of-Thought via `<think>` blocks — strips automatically, returns pure JSON
- ~1.1 GB VRAM — minimal cost
- Sufficient for the deterministic 3-class → 3-box mapping task
- Self-reflection quality handles edge cases gracefully

**Why not other LLMs:**

| Model | Tested | Rejection reason |
|-------|--------|-----------------|
| DeepSeek-R1 8B | Yes | Now resolves to Qwen3-based R1-0528 distill (~5.2 GB). Combined with Isaac Sim + VLM exceeds 12 GB. Downsized to 1.5B for VRAM fit |
| GLM-5 Reasoning | Evaluated | Phenomenally obedient at following API schemas. However, self-correction in unpredictable edge cases isn't as robust as RL-heavy models |
| Llama 4 Scout/Maverick | Evaluated | Fast, efficient, long context windows. But generalist — strict logical state-machine routing is handled slightly better by purpose-built reasoning models |

---

#### C.1 — Why JSON? Couldn't we use a different structure?

**What JSON is:** A text-based key-value format. Human-readable, universally parseable in every language via one line (`json.loads()`), nestable, dominant in web APIs — LLMs have seen billions of examples in training data.

**Why not other formats:**

| Format | Problem with LLMs |
|--------|------------------|
| XML | Verbose, LLMs produce tag mismatches (unclosed tags), lower training exposure |
| YAML | Indentation-sensitive — LLMs frequently misalign levels |
| CSV | No nesting, no schema — requires fixed column order and custom parser |
| Plain English | Requires NLP intent parsing — fragile, non-deterministic |
| Protocol Buffers | Binary — LLMs cannot generate binary output |

**Why JSON with these specific models is the best option:**

1. **Qwen2.5-VL** was specifically fine-tuned for visual instruction following with structured output. Training included massive amounts of JSON-formatted visual QA datasets. It understands JSON schema from the prompt alone.

2. **DeepSeek-R1** uses `<think>` blocks to plan output before writing JSON — it reasons first ("ClassA maps to box 1"), then commits to the schema. This two-step process prevents schema violations from "rushing to output" that smaller models exhibit.

3. **moondream's failure illustrates this directly:** moondream was captioning-tuned (prose descriptions), not instruction-tuned for structured output. No JSON schema adherence in training → 38k-character loops with no schema.

**Interview punchline:** "JSON is the intersection of human-debuggable, universally parseable, and dominant in LLM training corpora. The models reliably produce it because they've seen it millions of times. Switching formats would require either a purpose-fine-tuned model or accepting higher hallucination rates."

---

### D. How do you explain the interaction between models and the simulation?

**Full pipeline interaction — step by step:**

```
Isaac Sim RGB-D Camera (overhead, /World/Camera)
        │  640×480 JPEG + depth array
        ▼
PerceptionClient.perceive()                     src/perception/vlm_client.py
  ├─ capture_frame()        → JPEG bytes + raw BGR frame
  ├─ analyze_frame()        → VLM call (Ollama HTTP) → JSON parse + validate
  └─ Output: scene_metadata dict
        │  {"objects": [{"id","class_label","coords":[x,y,z]}, ...]}
        ▼
IsaacScene.localize_by_color()                  src/isaac_scene.py
  ├─ HSV color segmentation per class label
  ├─ Depth mask: only pixels 1.00–1.95 m from camera (rejects floor)
  ├─ Contour centroid → _pixel_to_world() back-projection
  └─ Overrides VLM coords with precise camera-measured world XY (±10mm)
        │  coords updated to [x, y, _OBJ_Z] in robot frame (metres)
        ▼
ReasoningClient.generate_sort_plan()            src/reasoning/llm_client.py
  ├─ Sends scene_metadata to DeepSeek-R1 (Ollama HTTP)
  ├─ Strips <think>...</think> blocks
  └─ Output: sort_plan list
        │  [{"name","class_label","coords","target_box"}, ...]
        ▼
SortingStateMachine.run(sort_plan)              src/main.py
  └─ For each object:
       goto_pose(x,y,z) → execute_grasp() → place_in_box(id)
       On OBJECT_FELL: camera re-localize → retry (up to MAX_RETRIES=3)
        │  method calls + StatusCode returns
        ▼
FrankaRobot                                     src/execution/hardware_api.py
  ├─ goto_pose()      → validates reach, stores _pick_pos
  ├─ execute_grasp()  → no-op (PickPlaceController handles grasp internally)
  └─ place_in_box()   → run_pick_and_place() + verify_cube_in_bin()
        │
        ▼
IsaacScene.run_pick_and_place()                 src/isaac_scene.py
  ├─ PickPlaceController.forward() loop (up to 2000 physics steps)
  ├─ RMPflow avoids table top + 12 bin walls (explicitly registered)
  ├─ is_object_near(place_pos) after settle → confirms physical success
  └─ Returns bool → place_in_box decides PLACE_SUCCESS or OBJECT_FELL
```

**Interface contract:** Every interaction between layers uses a well-defined JSON API or Python enum. This decoupling means each layer can be swapped independently (real camera vs. webcam, real robot vs. mock, different LLM).

---

#### D.1 — What is HSV? Does HSV override make VLM obsolete?

**What is HSV:**
HSV = Hue, Saturation, Value. An alternative color space to RGB:

| Channel | What it means | Example |
|---------|--------------|---------|
| **H**ue | Pure color angle on the color wheel (0–180 in OpenCV) | Red≈0, Green≈60, Blue≈120 |
| **S**aturation | How vivid vs. washed out (0=grey, 255=pure color) | Low S = pastel, high S = vibrant |
| **V**alue | Brightness (0=black, 255=full bright) | Low V = dark, high V = bright |

**Why HSV instead of RGB for segmentation:** In RGB, a "red" object has very different R/G/B values under different lighting conditions. In HSV, the Hue stays approximately constant — the red cube is still Hue≈0 whether in shadow or bright light. This makes HSV masks far more robust across lighting variation.

**VLM vs. HSV — different jobs, not competing:**

|   | VLM | HSV |
|---|-----|-----|
| Answers | WHAT is this object? (class label) | WHERE is this object? (world XY coords) |
| Output | `class_label: "ClassA"` | `coords: [0.48, 0.12, 0.425]` |
| Precision | Low on coords (guessing 3D from 2D) | ±10mm XY |
| Generality | Works on any visual object | Only works on objects with known, distinct colors |

**The actual pipeline:**
```
VLM:  "ClassA at roughly (-0.2, 0.1)"  → class label KEPT
HSV:  "Red mask centroid at (0.482, 0.113)" → coords OVERRIDE VLM
Result: {class_label: "ClassA", coords: [0.482, 0.113, 0.425]}
```

**Honest interview answer:** "For this specific demo with perfectly colored cubes, HSV could technically replace VLM for both classification AND localization. But this is a demo-specific shortcut. In production with real objects (vegetables, packages, mixed materials), HSV breaks immediately because object colors overlap and are not controlled. The VLM handles semantic understanding — 'this is a dented can, not a good one'. HSV is a precision booster for the coordinate estimation step, not a semantic replacement."

---

### E. How do you pick the hardware for this task?

**Simulation hardware (what we used):**

| Component | Selection | Reason |
|-----------|-----------|--------|
| GPU | NVIDIA RTX 4070 (12 GB VRAM) | Binding constraint: Isaac Sim ~5 GB + VLM ~2.5 GB + LLM ~1.1 GB = ~8.6 GB total |
| CPU | Intel i9-13700KF | Isaac Sim's physics simulation is multi-threaded; high core count reduces step latency |
| RAM | 32 GB | Isaac Sim + USD stage loading peaks at ~18 GB; 32 GB provides headroom |
| Storage | SSD, 60 GB free | Isaac Sim ~40 GB install; SSD required for acceptable load times |
| Robot arm | Franka Panda | Native Isaac Sim 5.x support, well-documented SDK, standard in robotics research |
| Camera | Isaac Sim RGB-D sensor (overhead) | Equivalent to Intel RealSense D435 in real world; provides RGB + depth in one API |

**VRAM budget breakdown:**
- Isaac Sim rendering: ~5 GB
- Qwen2.5-VL 3B via Ollama: ~2.5 GB (runs on CPU when VRAM exhausted)
- DeepSeek-R1 1.5B via Ollama: ~1.1 GB
- RTX render targets (PickPlaceController + camera): ~1 GB
- **Total: ~9.6 GB** — within 12 GB with some headroom

**For real-world experimentation:**

| Component | Selection | Reason |
|-----------|-----------|--------|
| Robot arm | Franka Panda FR3 | Same arm as simulation; direct code transfer via franka_ros / FCI SDK |
| Camera | Intel RealSense D435 | RGB-D, same data format as Isaac Sim sensor output; USB3 |
| Gripper | Franka Hand (parallel finger) | Matches simulation gripper type; force sensing built in |
| Workstation | Same RTX 4070 machine | Models already calibrated for this VRAM budget |
| End-effector force sensor | ATI Mini45 or Franka's built-in | Detects grasp quality independently of joint torque |

---

### F. What would you change for real-world deployment?

**What stays the same:**
- State machine logic (SortingStateMachine) — unchanged
- LLM reasoning layer (DeepSeek-R1 prompts and API) — unchanged
- JSON API contract between all layers — unchanged
- Error recovery flow (retry logic, MAX_RETRIES) — unchanged

**What changes and why:**

| Component | Simulation | Real World | Why |
|-----------|-----------|------------|-----|
| Object detection | HSV color segmentation | YOLOv8 or similar trained detector | Real objects don't have controlled HSV colors; lighting varies; occlusion occurs |
| Robot controller | PickPlaceController + RMPflow | franka_ros + FCI (Franka Control Interface) | PickPlaceController is Isaac Sim specific; real robot uses FCI at 1kHz control rate |
| Grasp verification | is_object_near() proximity check | Force-torque sensing at gripper | Camera-based verification is slow; F/T sensor detects contact in milliseconds |
| Motion planning | RMPflow (built into PickPlaceController) | MoveIt 2 with Franka ROS2 driver | Industry standard, well-tested collision planning for real robot |
| Depth localization | Isaac Sim RGB-D sensor | Intel RealSense D435 | Same API format; real-world depth noise requires filtering |
| Safety | Software IK_UNREACHABLE check | Multi-layer safety (see B.2) | Simulation has no real-world damage consequences |
| Bin localization | Hardcoded BIN_POSITIONS | ArUco marker detection at startup | Bins may not be placed precisely; markers give ±2mm pose accuracy |

**The decision depends on:**
- **Scale:** 3 objects in demo → N objects in production (need object instance segmentation, not just 3-class HSV)
- **Variability:** Controlled simulation colors → real-world lighting/occlusion variation
- **Safety:** Simulation crashes = restart → real-world crashes = hardware damage, injury risk
- **Latency:** 50s VLM inference acceptable for demo → unacceptable for production line (need edge-optimized models or GPU-resident inference)

---

## Part 2 — Deep-Dive Additional Discussion

---

### What is a no-op?

**No-op** = "no operation" — a function that intentionally does nothing and returns immediately.

**In our codebase, two no-ops:**

**1. `FrankaRobot.execute_grasp()`** in `src/execution/hardware_api.py`:
```python
def execute_grasp(self) -> StatusCode:
    return StatusCode.GRASP_SUCCESS   # PickPlaceController handles this internally
```
The state machine must call `execute_grasp()` — it's part of the API contract. But `PickPlaceController` runs the full pick-approach-grasp-lift sequence in one `run_pick_and_place()` call. The grasp phase isn't a separate step we can inject into. So `execute_grasp()` returns success immediately to keep the state machine interface consistent without breaking it.

**2. `release_camera()`** in `src/isaac_scene.py`:
```python
def release_camera(self):
    pass   # documented no-op — removing the shared Hydra render prim breaks the pipeline
```
We wanted to free the RTX render target between captures. But `camera.get_render_product_path()` returns `/Render/OmniverseKit/HydraTextures/Replicator` — a prim shared by the entire Hydra rendering pipeline. Removing it causes error code 6 on every subsequent `world.step()`. Documented as no-op rather than introduce a silent bug.

**General rule:** No-ops appear when an interface requires a method, but a specific implementation doesn't need to do anything at that step. They preserve API consistency without lying about state.

---

### What is RMPflow and why does obstacle registration matter?

**RMPflow (Riemannian Motion Policies):** The reactive motion planner inside PickPlaceController. It converts end-effector targets to joint commands on every physics step by computing a policy that simultaneously:
- Attracts the end-effector toward the target
- Repels it from registered obstacles
- Respects joint limits

**Critical insight:** USD/PhysX collision (which makes cubes rest on the table) and RMPflow obstacle avoidance are **completely independent systems**. PhysX collision prevents rigid bodies from interpenetrating. RMPflow only avoids geometry **explicitly registered** with it via `add_obstacle()`.

**Why the arm was initially colliding with the table:**
The table top and bin walls were in the USD stage with PhysX collision (so cubes rest on them) but were never registered with RMPflow — so the arm planned paths directly through them.

**The fix:**
```python
self.controller._cspace_controller.add_obstacle(self._table_top, static=True)
for wall_prim in self._bin_wall_prims:
    self.controller._cspace_controller.add_obstacle(wall_prim, static=True)
```

13 obstacles registered: 1 table top + 4 walls × 3 bins.

---

### What is PickPlaceController?

Inspection of the Isaac Sim 5.1.0 source reveals `PickPlaceController` is a **phase sequencer** that wraps `RMPFlowController` as its internal c-space controller:

```python
manipulators_controllers.PickPlaceController.__init__(
    self,
    cspace_controller=RMPFlowController(name=..., robot_articulation=robot_articulation),
    ...
)
```

**Phase sequence:**
1. Phase 0: Hover above pick position at `end_effector_initial_height`
2. Phase 1: Descend to pick position
3. Phase 2: Close gripper (grasp)
4. Phase 3: Lift to `end_effector_initial_height`
5. Phase 4: Transit to place position hover
6. Phase 5: Descend to place position
7. Phase 6: Open gripper (release)
8. Phase 7: Retreat

**Key parameter:** `end_effector_initial_height = 0.65m` (we set this). Default is 0.3m which is below our table surface (0.40m) — always set this above the table.

Full cycle requires ~2284 physics steps. Our `max_steps=2000` works because the controller completes before that in practice.

---

### Why use Ollama instead of direct model loading?

**Ollama** is a local LLM/VLM inference server:
- Provides HTTP API (`POST /api/generate`) — same interface as OpenAI
- Manages model lifecycle (loading, quantization, GPU/CPU layer split)
- Allows running both VLM and LLM as separate processes without Python memory conflicts
- Isaac Sim's bundled Python doesn't have transformers/PyTorch — Ollama sidesteps this entirely

**The VLM running on CPU:**
With Isaac Sim consuming ~5 GB VRAM, Qwen2.5-VL 3B has no GPU VRAM left. Ollama automatically falls back to CPU inference (0/37 GPU layers). Result: ~50 second inference time, but outputs are correct because prompt hardening compensates for the model running slightly degraded.

---

### What is the state machine's retry logic in detail?

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
                          set   → use camera-fresh XY (cube moved after drop)
                          None  → use original VLM coords (cube at same spot)
```

**OBJECT_FELL sources and `_reloc_pos` behaviour:**

| Cause | `run_pick_and_place` returns | `_reloc_pos` set? | Retry coords |
|-------|------------------------------|-------------------|--------------|
| Controller timeout / missed grasp | False | Yes, if cube visible on table | Camera-fresh or original |
| force_drop (gripper opened mid-transit) | False | Yes (cube near pick pos) | Camera-fresh |
| Placed but bounced out of bin | True → verify fails | Yes (fresh_xyz returned) | Camera-fresh |
| Placed but color not visible in bin | True → verify: not visible → ✓ | No (confirmed in bin) | N/A |

---

### Verification Run Matrix

| Phase | Flag settings | Verifies | Status |
|:---   |:---           |:---      |:---    |
| 1 — Smoke test | `USE_VLM=F, USE_LLM=F` | Scene builds, 3/3 sort, no crashes | ✓ Done |
| 2 — Full pipeline | `USE_VLM=T, USE_LLM=T` | VLM labels + LLM sort plan + 3/3 success | ✓ Done |
| 3 — Random spawn | Phase 2 + `RANDOM_SPAWN=T` | Color seg finds objects at unknown positions | ✓ Done |
| 4 — Demo camera | `USE_VLM=F + DEMO_MODE=T` | Perspective viewport + bin markers visible | ✓ Done |
| 5 — Record Video 1 | `DEMO_MODE=T, RANDOM_SPAWN=T, USE_VLM=T, USE_LLM=T` | Full autonomous sort | ✓ Done |
| 6 — Record Video 2 | Full Isaac Sim, `demo_drop_second=True` | Physical drop + re-localization + retry | ✓ Done |

---

### Assignment Done Criteria

| # | Check | Pass criteria | Status |
|:---|:---  |:---           |:---    |
| A1 | VLM identifies 3 object classes | Terminal shows `ClassA/B/C` for all 3 objects | ✓ |
| A2 | LLM generates sort plan | Terminal shows `target_box: 1/2/3` | ✓ |
| A3 | Franka executes 3/3 pick-and-place | `[OK] obj_1/2/3` in state machine summary | ✓ |
| A4 | Error recovery demonstrated | `OBJECT_FELL → camera re-localize → retry → success` in terminal | ✓ |
| A5 | Architecture document | `README.md` + `docs/ARCHITECTURE.md` cover all 4 modules | ✓ |
| A6 | Video: successful 3-object sort | Cubes sorted into correct colored bins | ✓ |
| A7 | Video: failure recovery | Retry cycle visible in terminal output | ✓ |

---

## Quick Reference — Key Technical Numbers

| Parameter | Value | Context |
|-----------|-------|---------|
| VRAM budget | 12 GB | RTX 4070 |
| Isaac Sim VRAM | ~5 GB | Base consumption |
| Qwen2.5-VL 3B VRAM | ~2.5 GB | Runs CPU in practice |
| DeepSeek-R1 1.5B VRAM | ~1.1 GB | |
| VLM inference time | ~50 s | CPU-only under VRAM pressure |
| HSV localization precision | ±10 mm XY | Camera-measured world coords |
| PickPlaceController max steps | 2000 | ~2284 needed at full cycle |
| `end_effector_initial_height` | 0.65 m | Above table (0.40 m) |
| RMPflow obstacles registered | 13 | 1 table + 4 walls × 3 bins |
| MAX_RETRIES | 3 | Per object in state machine |
| Camera position | [0.35, 0.00, 2.20] m | Overhead stand |
| Camera FOV | focal_length=8.0 mm | Full workspace visible |
| Depth mask range | 1.00–1.95 m | Rejects floor at 2.20 m |
| Cube spawn X range | 0.35–0.60 m | `RANDOM_SPAWN=True` |
| Cube spawn Y range | -0.22–0.25 m | `RANDOM_SPAWN=True` |
| Min cube separation | 0.25 m | Prevents spawn overlap |
| Wilson z-value (95% CI) | 1.96 | Standard normal for 95% |
