from enum import Enum
import random
import time


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
