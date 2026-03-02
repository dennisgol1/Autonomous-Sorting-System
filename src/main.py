from execution.hardware_api import MockFrankaRobot, StatusCode

# --- Configuration ---
# Set to True to call DeepSeek-R1 via Ollama for the sort plan.
# Set to False to use hardcoded data (no Ollama required).
USE_LLM = False

MAX_RETRIES = 3

# Hardcoded fallback data — used when USE_LLM = False (Stage 2B mode).
HARDCODED_OBJECTS = [
    {"name": "Object_A", "class_label": "ClassA", "coords": (0.4, 0.1, 0.3),  "target_box": 1},
    {"name": "Object_B", "class_label": "ClassB", "coords": (0.2, -0.2, 0.3), "target_box": 2},
    {"name": "Object_C", "class_label": "ClassC", "coords": (-0.1, 0.3, 0.3), "target_box": 3},
]

# Mock scene metadata — simulates what Qwen 3 VL will produce in Stage 1.
# Passed to the LLM when USE_LLM = True.
MOCK_SCENE_METADATA = {
    "objects": [
        {"id": "Object_A", "class_label": "ClassA", "coords": [0.4,  0.1,  0.3]},
        {"id": "Object_B", "class_label": "ClassB", "coords": [0.2, -0.2,  0.3]},
        {"id": "Object_C", "class_label": "ClassC", "coords": [-0.1, 0.3,  0.3]},
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
                continue  # retry full cycle from goto_pose
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
                continue  # retry full cycle from goto_pose
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
    if USE_LLM:
        from reasoning.llm_client import ReasoningClient
        client = ReasoningClient()
        sort_plan = client.generate_sort_plan(MOCK_SCENE_METADATA)
    else:
        sort_plan = HARDCODED_OBJECTS

    machine = SortingStateMachine()
    machine.run(sort_plan)
