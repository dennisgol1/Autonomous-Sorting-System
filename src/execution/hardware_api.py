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
        return StatusCode.IK_UNREACHABLE if random.random() < 0.1 else StatusCode.AT_TARGET

    def execute_grasp(self) -> StatusCode:
        print("[Robot] Attempting grasp...")
        time.sleep(1)
        return StatusCode.OBJECT_SLIPPED if random.random() < 0.1 else StatusCode.GRASP_SUCCESS

    def place_in_box(self, box_id: int) -> StatusCode:
        print(f"[Robot] Placing in Box {box_id}...")
        time.sleep(1)
        return StatusCode.OBJECT_FELL if random.random() < 0.1 else StatusCode.PLACE_SUCCESS


class FrankaRobot:
    """
    Real Franka Panda robot interface backed by Isaac Sim 5.1.0.

    Maps the same 3-method API as MockFrankaRobot to Isaac Sim's
    PickPlaceController. The full pick-and-place motion runs inside
    the sim loop during place_in_box().

    API contract vs MockFrankaRobot:
      goto_pose()    — validates reach, stores pick position. No sim steps.
      execute_grasp()— no-op. PickPlaceController handles grasping internally.
      place_in_box() — runs the full pick-and-place sim loop until done or timeout.
    """

    MAX_REACH = 0.85  # Franka Panda reach radius in meters

    def __init__(self, scene):
        # scene is an IsaacScene instance (from isaac_scene.py)
        self.scene = scene
        self._pick_pos = None
        print("[System] Initialized Franka Panda Robot (Isaac Sim)")

    def goto_pose(self, x: float, y: float, z: float) -> StatusCode:
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
        print(f"[Robot] Executing pick-and-place to Box {box_id}...")
        place_pos = self.scene.get_bin_position(box_id)
        success = self.scene.run_pick_and_place(self._pick_pos, place_pos)
        if success:
            print(f"[Robot] PLACE_SUCCESS: Object placed in Box {box_id}.")
            return StatusCode.PLACE_SUCCESS
        print("[Robot] OBJECT_FELL: Pick-and-place timed out.")
        return StatusCode.OBJECT_FELL
