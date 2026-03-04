# ---------------------------------------------------------------------------
# Pipeline mode flags
#
#  USE_ISAAC_SIM must be set BEFORE USE_VLM / USE_LLM.
#  When True, SimulationApp is initialized first (required by Isaac Sim).
#
#  Webcam modes (USE_ISAAC_SIM = False):
#    Mode 1 — Pure mock:           USE_VLM=False, USE_LLM=False
#    Mode 2 — LLM only:            USE_VLM=False, USE_LLM=True
#    Mode 3 — Full webcam pipeline: USE_VLM=True,  USE_LLM=True
#
#  Isaac Sim modes (USE_ISAAC_SIM = True):
#    Mode 4 — Sim + mock LLM:      USE_VLM=False, USE_LLM=False
#    Mode 5 — Sim + LLM:           USE_VLM=False, USE_LLM=True
#    Mode 6 — Sim + VLM + LLM:     USE_VLM=True,  USE_LLM=True  ← full pipeline
#
#  Run Isaac Sim modes with Isaac Sim's bundled Python:
#    D:\isaac-sim-standalone-5.1.0-windows-x86_64\python.bat src/main.py
#
#  Run webcam modes with system Python (from src/):
#    python main.py
# ---------------------------------------------------------------------------
USE_ISAAC_SIM = True    # Set True for Isaac Sim integration (Stages 4–7)
USE_VLM       = True    # Use Qwen VL for object detection
USE_LLM       = False   # Use DeepSeek-R1 for sort planning

# Camera sanity check — set True to save one RGB frame + colorized depth map
# to debug/ immediately after physics settle, then continue with mock pipeline.
# No Ollama required. Check debug/capture_test_rgb.jpg and capture_test_depth.png.
CAPTURE_TEST  = False

# Show live webcam preview before capturing (webcam modes only).
# Press SPACE to capture, Q to cancel.
SHOW_PREVIEW = True

MAX_RETRIES = 3

# ---------------------------------------------------------------------------
# Isaac Sim must be initialized before any other isaacsim imports.
# ---------------------------------------------------------------------------
if USE_ISAAC_SIM:
    from isaacsim import SimulationApp
    # TODO Stage 7: switch back to headless=False for demo video recording
    simulation_app = SimulationApp({"headless": True})   # headless=True frees GPU VRAM for VLM

from execution.hardware_api import MockFrankaRobot, StatusCode

# ---------------------------------------------------------------------------
# Fallback / mock data (webcam modes)
# ---------------------------------------------------------------------------

HARDCODED_OBJECTS = [
    {"name": "Object_A", "class_label": "ClassA", "coords": (0.4,  0.1,  0.3), "target_box": 1},
    {"name": "Object_B", "class_label": "ClassB", "coords": (0.2, -0.2,  0.3), "target_box": 2},
    {"name": "Object_C", "class_label": "ClassC", "coords": (-0.1, 0.3,  0.3), "target_box": 3},
]

MOCK_SCENE_METADATA = {
    "objects": [
        {"id": "Object_A", "class_label": "ClassA", "coords": [ 0.4,  0.1, 0.3]},
        {"id": "Object_B", "class_label": "ClassB", "coords": [ 0.2, -0.2, 0.3]},
        {"id": "Object_C", "class_label": "ClassC", "coords": [-0.1,  0.3, 0.3]},
    ]
}


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class SortingStateMachine:
    def __init__(self, robot=None):
        self.robot = robot if robot is not None else MockFrankaRobot()

    def _sort_single_object(self, obj: dict) -> bool:
        """
        Executes the full pick-and-place cycle for one object.
        Retries up to MAX_RETRIES times for recoverable failures.
        Returns True on success, False on an unrecoverable halt.
        """
        name = obj["name"]
        x, y, z = obj["coords"]
        box_id = obj["target_box"]

        print(f"\n--- Sorting {name} ({obj['class_label']}) -> Box {box_id} ---")

        for attempt in range(1, MAX_RETRIES + 1):
            if attempt > 1:
                print(f"[State Machine] Retry {attempt}/{MAX_RETRIES} for {name}...")

            # --- Step 1: Navigate to object ---
            status = self.robot.goto_pose(x, y, z)
            if status == StatusCode.AT_TARGET:
                print("[State Machine] At target. Proceeding to grasp.")
            elif status == StatusCode.IK_UNREACHABLE:
                print(f"[State Machine] HALT: IK_UNREACHABLE — pose ({x}, {y}, {z}) has no valid joint solution.")
                return False
            elif status == StatusCode.COLLISION:
                print("[State Machine] HALT: COLLISION detected — path obstructed. Unsafe to continue.")
                return False
            else:
                print(f"[State Machine] HALT: Unexpected response from goto_pose: {status}")
                return False

            # --- Step 2: Grasp the object ---
            status = self.robot.execute_grasp()
            if status == StatusCode.GRASP_SUCCESS:
                print("[State Machine] Grasp successful. Proceeding to place.")
            elif status == StatusCode.OBJECT_SLIPPED:
                print("[State Machine] OBJECT_SLIPPED — gripper lost contact. Retrying from navigation.")
                continue
            elif status == StatusCode.MISSING_OBJ:
                print("[State Machine] HALT: MISSING_OBJ — object not found at expected coordinates.")
                return False
            else:
                print(f"[State Machine] HALT: Unexpected response from execute_grasp: {status}")
                return False

            # --- Step 3: Place in target box ---
            status = self.robot.place_in_box(box_id)
            if status == StatusCode.PLACE_SUCCESS:
                print(f"[State Machine] SUCCESS: {name} placed in Box {box_id}.")
                return True
            elif status == StatusCode.OBJECT_FELL:
                print("[State Machine] OBJECT_FELL — object dropped during placement. Re-picking.")
                continue
            elif status == StatusCode.DOES_NOT_FIT:
                print(f"[State Machine] HALT: DOES_NOT_FIT — {name} cannot enter Box {box_id}. Check class-to-box mapping.")
                return False
            else:
                print(f"[State Machine] HALT: Unexpected response from place_in_box: {status}")
                return False

        print(f"[State Machine] HALT: Max retries ({MAX_RETRIES}) exceeded for {name}.")
        return False

    def run(self, sort_plan: list[dict]):
        print("=" * 55)
        print("[State Machine] Starting Autonomous Sorting Cycle")
        print(f"[State Machine] Objects queued: {len(sort_plan)}")
        print("=" * 55)

        results = []
        for obj in sort_plan:
            success = self._sort_single_object(obj)
            results.append({"object": obj["name"], "success": success})
            if not success:
                print(f"\n[State Machine] CRITICAL FAILURE on {obj['name']}. Aborting cycle.")
                break

        print("\n" + "=" * 55)
        print("[State Machine] Cycle Complete — Summary:")
        for r in results:
            label = "OK    " if r["success"] else "FAILED"
            print(f"  [{label}] {r['object']}")
        print("=" * 55)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":

    # -----------------------------------------------------------------------
    # Isaac Sim path (Modes 4–6)
    # -----------------------------------------------------------------------
    if USE_ISAAC_SIM:
        from isaac_scene import IsaacScene, OBJECT_CLASS_LABELS, OBJECT_POSITIONS
        from execution.hardware_api import FrankaRobot

        # Build scene + warm up physics
        scene = IsaacScene(simulation_app)
        scene.setup()
        scene.settle_physics(steps=60)

        # --- Camera sanity check (CAPTURE_TEST=True) ---
        # Saves RGB + depth to debug/ so you can visually confirm both channels
        # before enabling USE_VLM. Set CAPTURE_TEST=False once satisfied.
        if CAPTURE_TEST:
            import cv2
            import numpy as np
            from pathlib import Path
            _f_bytes, _bgr, _depth = scene.capture_frame()
            _dbg = Path(__file__).parent.parent / "debug"
            _dbg.mkdir(exist_ok=True)
            cv2.imwrite(str(_dbg / "capture_test_rgb.jpg"), _bgr)
            _d8 = cv2.normalize(_depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            cv2.imwrite(str(_dbg / "capture_test_depth.png"),
                        cv2.applyColorMap(_d8, cv2.COLORMAP_PLASMA))
            print(f"[CAPTURE_TEST] RGB  → debug/capture_test_rgb.jpg")
            print(f"[CAPTURE_TEST] Depth→ debug/capture_test_depth.png  "
                  f"range [{_depth.min():.2f}, {_depth.max():.2f}] m")

        # --- Step 1: Perceive ---
        if USE_VLM:
            from perception.vlm_client import PerceptionClient
            scene.park_arm_for_capture()   # fold arm out of camera view before VLM snapshot
            frame_bytes, raw_frame, depth = scene.capture_frame()
            try:
                scene_metadata = PerceptionClient().analyze_frame(
                    frame_bytes, raw_frame=raw_frame, depth=depth, save_debug=True
                )
            except Exception as e:
                print(f"[Main] VLM perception failed: {e}")
                print("[Main] Falling back to known Isaac Sim positions.")
                scene_metadata = {
                    "objects": [
                        {
                            "id": obj_id,
                            "class_label": OBJECT_CLASS_LABELS[obj_id],
                            "coords": scene.get_object_world_position(obj_id).tolist(),
                        }
                        for obj_id in OBJECT_POSITIONS
                    ]
                }
            # Stage 5c: replaced by localize_by_color() below — comment kept for easy revert
            # Override VLM coordinate estimates with exact Isaac Sim world positions.
            # VLM gives us class labels; Isaac Sim gives us precise coords.
            # for obj in scene_metadata["objects"]:
            #     obj_id = obj["id"]
            #     if obj_id in scene.objects:
            #         obj["coords"] = scene.get_object_world_position(obj_id).tolist()

            # Replace VLM coordinate estimates with color-segmented + back-projected
            # world positions. x,y from pixel centroid, z from depth sensor directly.
            for obj in scene_metadata["objects"]:
                world_xyz = scene.localize_by_color(obj["class_label"], raw_frame, depth)
                if world_xyz is not None:
                    print(f"[Main] {obj['id']} ({obj['class_label']}) localized -> {world_xyz}")
                    obj["coords"] = world_xyz.tolist()
                else:
                    # Color not found — fall back to known Isaac Sim position
                    obj_id = obj["id"]
                    if obj_id in scene.objects:
                        obj["coords"] = scene.get_object_world_position(obj_id).tolist()
                        print(f"[Main] {obj['id']}: color seg failed, using Isaac Sim fallback")
        else:
            # Use known class labels + exact world positions from Isaac Sim
            scene_metadata = {
                "objects": [
                    {
                        "id": obj_id,
                        "class_label": OBJECT_CLASS_LABELS[obj_id],
                        "coords": scene.get_object_world_position(obj_id).tolist(),
                    }
                    for obj_id in OBJECT_POSITIONS
                ]
            }

        # --- Step 2: Reason ---
        if USE_LLM:
            from reasoning.llm_client import ReasoningClient
            sort_plan = ReasoningClient().generate_sort_plan(scene_metadata)
        else:
            CLASS_TO_BOX = {"ClassA": 1, "ClassB": 2, "ClassC": 3}
            sort_plan = [
                {
                    "name": obj["id"],
                    "class_label": obj["class_label"],
                    "coords": obj["coords"],    # VLM label + back-projected x,y,z
                    "target_box": CLASS_TO_BOX.get(obj["class_label"], i + 1),
                }
                for i, obj in enumerate(scene_metadata["objects"])
            ]

        # --- Step 3: Act ---
        robot = FrankaRobot(scene)
        SortingStateMachine(robot=robot).run(sort_plan)

        # Keep the scene open so the user can inspect the result.
        print("\n[Main] Sorting complete. Close the Isaac Sim window to exit.")
        while simulation_app.is_running():
            scene.world.step(render=True)

        simulation_app.close()

    # -----------------------------------------------------------------------
    # Webcam / mock path (Modes 1–3)
    # -----------------------------------------------------------------------
    else:
        # --- Step 1: Perceive ---
        if USE_VLM:
            from perception.vlm_client import PerceptionClient
            scene_metadata = PerceptionClient().perceive(
                show_preview=SHOW_PREVIEW,
                save_debug=True,
            )
        else:
            scene_metadata = MOCK_SCENE_METADATA

        # --- Step 2: Reason ---
        if USE_LLM:
            from reasoning.llm_client import ReasoningClient
            sort_plan = ReasoningClient().generate_sort_plan(scene_metadata)
        else:
            sort_plan = HARDCODED_OBJECTS

        # --- Step 3: Act ---
        SortingStateMachine().run(sort_plan)
