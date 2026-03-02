from execution.hardware_api import MockFrankaRobot, StatusCode

# ---------------------------------------------------------------------------
# Pipeline mode flags
#
#  Mode 1 — Pure mock (no AI, no camera):
#      USE_VLM = False, USE_LLM = False
#      Uses HARDCODED_OBJECTS directly. Fastest — no dependencies needed.
#
#  Mode 2 — LLM only (reasoning tested, perception still mocked):
#      USE_VLM = False, USE_LLM = True
#      Feeds MOCK_SCENE_METADATA to DeepSeek-R1 via Ollama.
#
#  Mode 3 — Full pipeline (webcam → VLM → LLM → robot):
#      USE_VLM = True,  USE_LLM = True
#      Live camera frame → Qwen VL → DeepSeek-R1 → State Machine.
# ---------------------------------------------------------------------------
USE_VLM = True
USE_LLM = True

# Show a live webcam preview window before capturing (only when USE_VLM=True).
# Press SPACE to capture, Q to cancel.
SHOW_PREVIEW = True

MAX_RETRIES = 3

# --- Fallback data (Mode 1) ---
HARDCODED_OBJECTS = [
    {"name": "Object_A", "class_label": "ClassA", "coords": (0.4,  0.1,  0.3), "target_box": 1},
    {"name": "Object_B", "class_label": "ClassB", "coords": (0.2, -0.2,  0.3), "target_box": 2},
    {"name": "Object_C", "class_label": "ClassC", "coords": (-0.1, 0.3,  0.3), "target_box": 3},
]

# --- Mock scene metadata (Mode 2) ---
# Simulates what Qwen VL will produce from an RGB-D frame.
# For the laptop POC: place a red, blue, and green object in front of the camera.
MOCK_SCENE_METADATA = {
    "objects": [
        {"id": "Object_A", "class_label": "ClassA", "coords": [ 0.4,  0.1, 0.3]},
        {"id": "Object_B", "class_label": "ClassB", "coords": [ 0.2, -0.2, 0.3]},
        {"id": "Object_C", "class_label": "ClassC", "coords": [-0.1,  0.3, 0.3]},
    ]
}


class SortingStateMachine:
    def __init__(self):
        self.robot = MockFrankaRobot()

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


if __name__ == "__main__":

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
