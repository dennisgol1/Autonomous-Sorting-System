# execution/

Defines the robot hardware interface and the state machine that drives the pick-and-place cycle.

## File

`hardware_api.py` — `StatusCode` enum + `MockFrankaRobot` class

## StatusCode — full API contract

| Code | Name | Meaning | State machine action |
| :--- | :--- | :--- | :--- |
| 200 | AT_TARGET | Navigation succeeded | Proceed to grasp |
| 201 | GRASP_SUCCESS | Gripper secured object | Proceed to place |
| 202 | PLACE_SUCCESS | Object placed in box | Done — next object |
| 401 | IK_UNREACHABLE | No valid joint solution for target pose | **Hard halt** |
| 402 | COLLISION | Path obstructed | **Hard halt** |
| 403 | OBJECT_SLIPPED | Gripper lost contact mid-grasp | Retry from `goto_pose` |
| 404 | MISSING_OBJ | Object not at expected coordinates | **Hard halt** |
| 405 | DOES_NOT_FIT | Object incompatible with target box | **Hard halt** |
| 406 | OBJECT_FELL | Dropped during placement | Retry from `goto_pose` |

## MockFrankaRobot

Simulates the Franka Panda API with ~10% random failure injection per call:

| Method | Success | Failure |
| :--- | :--- | :--- |
| `goto_pose(x, y, z)` | AT_TARGET | IK_UNREACHABLE |
| `execute_grasp()` | GRASP_SUCCESS | OBJECT_SLIPPED |
| `place_in_box(box_id)` | PLACE_SUCCESS | OBJECT_FELL |

## Isaac Sim swap

Add a `FrankaRobot` class that calls the Lula IK solver for real motion planning, then update the import in `main.py`:

```python
# from execution.hardware_api import MockFrankaRobot as Robot
from execution.hardware_api import FrankaRobot as Robot
```

The state machine in `main.py` requires no changes — it only sees `StatusCode` values.
