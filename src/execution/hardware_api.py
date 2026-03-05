from enum import Enum
import random
import time

import numpy as np


class StatusCode(Enum):
    AT_TARGET = 200
    GRASP_SUCCESS = 201
    PLACE_SUCCESS = 202
    IK_UNREACHABLE = 401
    COLLISION = 402
    OBJECT_SLIPPED = 403
    MISSING_OBJ = 404
    DOES_NOT_FIT = 405
    OBJECT_FELL = 406


class MockFrankaRobot:
    def __init__(self):
        print("[System] Initialized Mock Franka Panda Robot")

    def goto_pose(self, x: float, y: float, z: float) -> StatusCode:
        print(f"[Robot] Moving to (x:{x}, y:{y}, z:{z})...")
        time.sleep(1)
        return StatusCode.IK_UNREACHABLE if random.random() < 0.0 else StatusCode.AT_TARGET

    def execute_grasp(self) -> StatusCode:
        print("[Robot] Attempting grasp...")
        time.sleep(1)
        return StatusCode.OBJECT_SLIPPED if random.random() < 0.1 else StatusCode.GRASP_SUCCESS

    def place_in_box(self, box_id: int) -> StatusCode:
        print(f"[Robot] Placing in Box {box_id}...")
        time.sleep(1)
        return StatusCode.OBJECT_FELL if random.random() < 0.0 else StatusCode.PLACE_SUCCESS


_BOX_TO_CLASS = {1: "ClassA", 2: "ClassB", 3: "ClassC"}


class FrankaRobot:
    """
    Real Franka Panda robot interface backed by Isaac Sim 5.1.0.

    Maps the same 3-method API as MockFrankaRobot to Isaac Sim's
    PickPlaceController. The full pick-and-place motion runs inside
    the sim loop during place_in_box().

    API contract vs MockFrankaRobot:
      goto_pose()    — validates reach, stores pick position. No sim steps.
      execute_grasp()— no-op. PickPlaceController handles grasping internally.
      place_in_box() — runs the full pick-and-place sim loop until done or timeout,
                       then verifies placement with a camera snapshot.
    """

    MAX_REACH = 0.85  # Franka Panda reach radius in meters

    def __init__(self, scene, demo_drop_second: bool = False):
        # scene is an IsaacScene instance (from isaac_scene.py)
        self.scene = scene
        self._pick_pos = None
        self._reloc_pos = None  # camera-fresh position from last failed check; used on next retry
        # demo_drop_second: VIDEO 2 DEMO FLAG — set True only for demo recording.
        # Forces the gripper open mid-transit on the 2nd distinct object (identified by
        # box_id) so the cube physically falls and the recovery cycle is visible on camera.
        # Retries of the same object (same box_id) do NOT re-trigger the drop.
        # isaac_scene.py run_pick_and_place(force_drop=True) detects gripper closure
        # by physics state (finger joint < 0.03 m) and overrides to open.
        # TO RESTORE: pass demo_drop_second=False (default) or just FrankaRobot(scene).
        self._demo_drop = demo_drop_second
        self._demo_drop_triggered = False
        self._seen_box_ids: list = []  # ordered list of unique box_ids encountered
        print("[System] Initialized Franka Panda Robot (Isaac Sim)")

    def goto_pose(self, x: float, y: float, z: float) -> StatusCode:
        # If the camera found the cube at a new position after a failed placement,
        # use that instead of the (potentially stale) sort-plan coordinates.
        if self._reloc_pos is not None:
            x, y, z = self._reloc_pos
            self._reloc_pos = None
            print(f"[Robot] Re-localized pick position from camera: ({x:.3f}, {y:.3f}, {z:.3f})")
        print(f"[Robot] Validating pose (x:{x:.3f}, y:{y:.3f}, z:{z:.3f})...")
        dist = (x ** 2 + y ** 2) ** 0.5
        if dist > self.MAX_REACH:
            print(f"[Robot] IK_UNREACHABLE: distance {dist:.3f}m exceeds reach {self.MAX_REACH}m.")
            return StatusCode.IK_UNREACHABLE
        self._pick_pos = np.array([x, y, z])
        print("[Robot] Pose validated. AT_TARGET.")
        return StatusCode.AT_TARGET

    def execute_grasp(self) -> StatusCode:
        # Grasping is handled internally by PickPlaceController during place_in_box.
        print("[Robot] Grasp queued (executed during placement phase).")
        return StatusCode.GRASP_SUCCESS

    def place_in_box(self, box_id: int) -> StatusCode:
        if box_id not in self._seen_box_ids:
            self._seen_box_ids.append(box_id)
        object_num = self._seen_box_ids.index(box_id) + 1  # 1-indexed, stable across retries
        print(f"[Robot] Executing pick-and-place to Box {box_id}...")
        # Trigger drop on the 2nd distinct object only, and only once (not on retries).
        force_drop = False
        if self._demo_drop and object_num == 2 and not self._demo_drop_triggered:
            self._demo_drop_triggered = True
            force_drop = True
        place_pos = self.scene.get_bin_position(box_id)
        class_label = _BOX_TO_CLASS[box_id]
        success = self.scene.run_pick_and_place(self._pick_pos, place_pos, force_drop=force_drop)
        if success:
            # Arm completed the cycle — verify cube is actually in bin via overhead camera.
            # "color not visible" here legitimately means inside bin walls.
            confirmed, fresh_xyz = self.scene.verify_cube_in_bin(class_label, place_pos)
            if confirmed:
                print(f"[Robot] PLACE_SUCCESS: Object placed in Box {box_id}.")
                return StatusCode.PLACE_SUCCESS
            if fresh_xyz is not None:
                self._reloc_pos = fresh_xyz
                print(f"[Robot] Cube re-located at ({fresh_xyz[0]:.3f}, {fresh_xyz[1]:.3f}) — "
                      "will pick from updated position on retry.")
            print(f"[Robot] OBJECT_FELL: Camera check failed — {class_label} not in bin.")
            return StatusCode.OBJECT_FELL
        # Arm timed out or dropped the cube — do a direct table scan (no "not-visible = in bin"
        # assumption) so the retry can pick from the cube's actual current position.
        _, raw_frame, depth = self.scene.capture_frame()
        fresh_xyz = self.scene.localize_by_color(class_label, raw_frame, depth)
        if fresh_xyz is not None:
            fresh_xyz[2] = self._pick_pos[2]  # keep original Z (cube-centre height)
            self._reloc_pos = fresh_xyz
            print(f"[Robot] Cube found on table at ({fresh_xyz[0]:.3f}, {fresh_xyz[1]:.3f}) — "
                  "will pick from updated position on retry.")
        print("[Robot] OBJECT_FELL: Pick-and-place timed out or cube dropped.")
        return StatusCode.OBJECT_FELL
