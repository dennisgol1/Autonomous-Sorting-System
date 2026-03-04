"""
Isaac Sim 5.1.0 scene setup for the Autonomous Sorting System.

Run with Isaac Sim's bundled Python:
  D:\\isaac-sim-standalone-5.1.0-windows-x86_64\\python.bat src/main.py

NOT with system Python — this file uses isaacsim packages only available
inside Isaac Sim's Python environment.
"""

import sys

import carb
import cv2
import numpy as np

import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from pxr import Gf, UsdGeom, UsdPhysics, Vt
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.sensors.camera import Camera, SingleViewDepthSensorAsset
from isaacsim.storage.native import get_assets_root_path

# ---------------------------------------------------------------------------
# Scene layout constants
# ---------------------------------------------------------------------------

TABLE_H         = 0.40   # Table surface height (m)
TABLE_CX        = 0.35   # Table center X — robot base (x=0) is on the table
TABLE_WIDTH     = 1.20   # Table X dimension  (x: -0.25 → +0.95)
TABLE_DEPTH     = 1.20   # Table Y dimension  (y: -0.60 → +0.60) — fits all bins
TABLE_THICKNESS = 0.03   # Table top slab thickness

CUBE_SIZE = 0.05         # Sorting object edge length (m)

# Object centers sit on top of the table
_OBJ_Z = TABLE_H + CUBE_SIZE / 2

OBJECT_COLORS = {
    "ClassA": np.array([255,   0,   0]),   # Red
    "ClassB": np.array([  0,   0, 255]),   # Blue
    "ClassC": np.array([  0, 255,   0]),   # Green
}

# Fixed world positions for the 3 sorting objects
OBJECT_POSITIONS = {
    "obj_1": np.array([0.40,  0.10, _OBJ_Z]),   # ClassA — Red
    "obj_2": np.array([0.35, -0.15, _OBJ_Z]),   # ClassB — Blue
    "obj_3": np.array([0.50,  0.00, _OBJ_Z]),   # ClassC — Green
}

OBJECT_CLASS_LABELS = {
    "obj_1": "ClassA",
    "obj_2": "ClassB",
    "obj_3": "ClassC",
}

# Bin drop-off positions (inside the open-top bin, slightly above floor of bin).
# All bins sit on the table surface.
BIN_POSITIONS = {
    1: np.array([0.45,  0.38, TABLE_H + 0.14]),   # Box 1 — ClassA (above bin walls for drop-in)
    2: np.array([0.20,  0.38, TABLE_H + 0.14]),   # Box 2 — ClassB (above bin walls for drop-in)
    3: np.array([0.55, -0.36, TABLE_H + 0.14]),   # Box 3 — ClassC (above bin walls for drop-in)
}


# ---------------------------------------------------------------------------
# IsaacScene
# ---------------------------------------------------------------------------

class IsaacScene:
    """
    Builds the sorting scene and exposes helpers used by main.py / FrankaRobot.

    Scene contents:
      - Ground plane
      - Wooden table with 4 legs
      - 3 colored DynamicCuboids (sorting objects) on the table
      - 3 open-top bins (5-sided VisualCuboid boxes) on the table
      - Overhead camera mounted on a stand
      - Franka Panda robot at world origin
    """

    def __init__(self, simulation_app):
        self.simulation_app = simulation_app
        self.world              = None
        self.franka             = None
        self.camera             = None   # overhead stand camera
        self.wrist_camera       = None   # eye-in-hand camera on panda_hand
        self.objects: dict      = {}
        self.controller         = None
        self.articulation_controller = None
        self._table_top         = None
        self._bin_wall_prims: list = []

    # ------------------------------------------------------------------
    # Scene construction
    # ------------------------------------------------------------------

    def setup(self) -> None:
        print("[Isaac Scene] Building scene...")

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("[Isaac Scene] Could not find Isaac Sim assets folder.")
            self.simulation_app.close()
            sys.exit(1)

        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        self._add_table()
        self._add_sorting_objects()
        self._add_bins()
        self._add_camera_stand()
        self._add_franka(assets_root_path)
        self._add_wrist_camera()
        self._init_world()

        print("[Isaac Scene] Scene ready.")

    # ---- Table ----

    def _add_table(self) -> None:
        leg_h = TABLE_H - TABLE_THICKNESS          # Leg height
        leg_r = 0.04                                # Leg cross-section
        leg_cx = TABLE_WIDTH  / 2 - leg_r          # Leg X offset from table center
        leg_cy = TABLE_DEPTH  / 2 - leg_r          # Leg Y offset from table center
        table_color = np.array([160, 120, 70])      # Wood brown

        # Table top slab (collision-enabled so cubes rest on it; stored for RMPflow registration)
        self._table_top = self.world.scene.add(VisualCuboid(
            prim_path="/World/TableTop",
            name="table_top",
            position=np.array([TABLE_CX, 0.0, TABLE_H - TABLE_THICKNESS / 2]),
            scale=np.array([TABLE_WIDTH, TABLE_DEPTH, TABLE_THICKNESS]),
            size=1.0,
            color=table_color,
        ))
        self._apply_static_collision("/World/TableTop")

        # 4 legs
        for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
            self.world.scene.add(VisualCuboid(
                prim_path=f"/World/TableLeg_{i}",
                name=f"table_leg_{i}",
                position=np.array([TABLE_CX + sx * leg_cx, sy * leg_cy, leg_h / 2]),
                scale=np.array([leg_r, leg_r, leg_h]),
                size=1.0,
                color=table_color,
            ))

    # ---- Sorting objects ----

    def _add_sorting_objects(self) -> None:
        for obj_id, pos in OBJECT_POSITIONS.items():
            class_label = OBJECT_CLASS_LABELS[obj_id]
            cube = self.world.scene.add(DynamicCuboid(
                prim_path=f"/World/{obj_id}",
                name=obj_id,
                position=pos,
                scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
                size=1.0,
                color=OBJECT_COLORS[class_label],
            ))
            self.objects[obj_id] = cube

    # ---- Open-top bins ----

    def _add_bin(
        self,
        name: str,
        cx: float,
        cy: float,
        base_z: float,
        color: np.ndarray,
        bin_w: float = 0.14,
        bin_d: float = 0.14,
        bin_h: float = 0.09,
        wall_t: float = 0.008,
    ) -> None:
        """Creates an open-top rectangular bin from 5 VisualCuboids."""
        half_w = bin_w / 2
        half_d = bin_d / 2

        # Bottom (collision-enabled so cubes land inside the bin)
        self.world.scene.add(VisualCuboid(
            prim_path=f"/World/{name}_floor",
            name=f"{name}_floor",
            position=np.array([cx, cy, base_z + wall_t / 2]),
            scale=np.array([bin_w, bin_d, wall_t]),
            size=1.0, color=color,
        ))
        self._apply_static_collision(f"/World/{name}_floor")
        # +X wall
        w = self.world.scene.add(VisualCuboid(
            prim_path=f"/World/{name}_wx_pos",
            name=f"{name}_wx_pos",
            position=np.array([cx + half_w - wall_t / 2, cy, base_z + wall_t + bin_h / 2]),
            scale=np.array([wall_t, bin_d, bin_h]),
            size=1.0, color=color,
        ))
        self._apply_static_collision(f"/World/{name}_wx_pos")
        self._bin_wall_prims.append(w)
        # -X wall
        w = self.world.scene.add(VisualCuboid(
            prim_path=f"/World/{name}_wx_neg",
            name=f"{name}_wx_neg",
            position=np.array([cx - half_w + wall_t / 2, cy, base_z + wall_t + bin_h / 2]),
            scale=np.array([wall_t, bin_d, bin_h]),
            size=1.0, color=color,
        ))
        self._apply_static_collision(f"/World/{name}_wx_neg")
        self._bin_wall_prims.append(w)
        # +Y wall
        w = self.world.scene.add(VisualCuboid(
            prim_path=f"/World/{name}_wy_pos",
            name=f"{name}_wy_pos",
            position=np.array([cx, cy + half_d - wall_t / 2, base_z + wall_t + bin_h / 2]),
            scale=np.array([bin_w, wall_t, bin_h]),
            size=1.0, color=color,
        ))
        self._apply_static_collision(f"/World/{name}_wy_pos")
        self._bin_wall_prims.append(w)
        # -Y wall
        w = self.world.scene.add(VisualCuboid(
            prim_path=f"/World/{name}_wy_neg",
            name=f"{name}_wy_neg",
            position=np.array([cx, cy - half_d + wall_t / 2, base_z + wall_t + bin_h / 2]),
            scale=np.array([bin_w, wall_t, bin_h]),
            size=1.0, color=color,
        ))
        self._apply_static_collision(f"/World/{name}_wy_neg")
        self._bin_wall_prims.append(w)

    def _add_bins(self) -> None:
        bin_colors = {
            1: np.array([200,  60,  60]),   # Box 1 — red bin
            2: np.array([ 60,  60, 200]),   # Box 2 — blue bin
            3: np.array([ 60, 180,  60]),   # Box 3 — green bin
        }
        bin_centers = {
            1: (BIN_POSITIONS[1][0], BIN_POSITIONS[1][1]),
            2: (BIN_POSITIONS[2][0], BIN_POSITIONS[2][1]),
            3: (BIN_POSITIONS[3][0], BIN_POSITIONS[3][1]),
        }
        for box_id, (cx, cy) in bin_centers.items():
            self._add_bin(
                name=f"bin_{box_id}",
                cx=cx,
                cy=cy,
                base_z=TABLE_H,
                color=bin_colors[box_id],
            )

    # ---- Camera stand ----

    def _add_camera_stand(self) -> None:
        """
        Overhead camera rig: vertical pole at y=-0.90 + horizontal arm extending
        over the workspace. Camera sits 1.80 m above the workspace center and looks
        straight down (-Z). No Euler-angle ambiguity — identity orientation is exact.

        At height 1.80 m, 60°×45° FOV covers ~1.6 m × 1.2 m at table level, which
        fully contains all objects (y: -0.15…0.10) and all bins (y: -0.36…+0.38).
        """
        stand_x, stand_y = 0.35, -0.90   # pole base (outside table edge at y=-0.60)
        cam_x,   cam_y   = 0.35,  0.00   # camera position (above workspace centre)
        pole_h = 2.20                     # pole + arm height (m)
        gray      = np.array([80, 80, 80])
        dark_gray = np.array([40, 40, 40])

        # Vertical pole
        self.world.scene.add(VisualCuboid(
            prim_path="/World/CameraStandPole",
            name="camera_stand_pole",
            position=np.array([stand_x, stand_y, pole_h / 2]),
            scale=np.array([0.03, 0.03, pole_h]),
            size=1.0, color=gray,
        ))
        # Horizontal arm: runs in +Y from pole top to above workspace
        arm_y_len = cam_y - stand_y          # 0.90 m
        arm_y_ctr = (stand_y + cam_y) / 2   # −0.45
        self.world.scene.add(VisualCuboid(
            prim_path="/World/CameraArm",
            name="camera_arm",
            position=np.array([stand_x, arm_y_ctr, pole_h]),
            scale=np.array([0.03, arm_y_len, 0.03]),
            size=1.0, color=gray,
        ))
        # RealSense D455 mesh — replaces the plain VisualCuboid camera body.
        # Loads the USD asset (visual mesh + built-in camera prim).
        # The asset's own camera is at /World/CameraBody/RSD455/Camera_Pseudo_Depth
        # and can be selected in the viewport dropdown for comparison.
        rs_usd = get_assets_root_path() + "/Isaac/Sensors/Intel/RealSense/rsd455.usd"
        rs_prim = add_reference_to_stage(usd_path=rs_usd, prim_path="/World/CameraBody")
        rs_xf = UsdGeom.Xformable(rs_prim)
        rs_xf.ClearXformOpOrder()
        rs_xf.AddTranslateOp().Set(Gf.Vec3d(cam_x, cam_y, pole_h - 0.04))
        rs_xf.AddRotateYOp().Set(90.0)
        rs_xf.AddScaleOp().Set(Gf.Vec3f(0.01, 0.01, 0.01))   # asset is in cm → convert to m

        # --- Option B (did not work — OV9782 camera stays black, mesh not visible) ---
        # rs_color_path = "/World/CameraBody/RSD455/Camera_OmniVision_OV9782_Color"
        # self.camera = Camera(prim_path=rs_color_path, frequency=20, resolution=(640, 480))
        # stage = get_current_stage()
        # cone = UsdGeom.Cone.Define(stage, rs_color_path + "/SightCone")
        # cone.GetHeightAttr().Set(0.30); cone.GetRadiusAttr().Set(0.04)
        # cone.GetAxisAttr().Set("Y")
        # cone.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.5, 0.0)]))
        # xf = UsdGeom.Xformable(cone.GetPrim()); xf.ClearXformOpOrder()
        # xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.25)); xf.AddRotateXOp().Set(-90.0)

        # --- Option A: custom Camera at /World/Camera — confirmed working ---
        cam_pos = np.array([cam_x, cam_y, pole_h])
        self.camera = Camera(
            prim_path="/World/Camera",
            position=cam_pos,
            frequency=20,
            resolution=(640, 480),
            orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
        )
        stage = get_current_stage()
        UsdGeom.Camera(stage.GetPrimAtPath("/World/Camera")).GetFocalLengthAttr().Set(8.0)
        cone = UsdGeom.Cone.Define(stage, "/World/Camera/SightCone")
        cone.GetHeightAttr().Set(0.30)
        cone.GetRadiusAttr().Set(0.04)
        cone.GetAxisAttr().Set("Y")
        cone.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(1.0, 0.5, 0.0)]))
        xf = UsdGeom.Xformable(cone.GetPrim())
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.25))
        xf.AddRotateXOp().Set(-90.0)

    # ---- Franka robot ----

    def _add_franka(self, assets_root_path: str) -> None:
        asset_path = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        robot_prim = add_reference_to_stage(usd_path=asset_path, prim_path="/World/Franka")
        robot_prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
        robot_prim.GetVariantSet("Mesh").SetVariantSelection("Quality")

        # Mount the robot on the table surface.
        xf = UsdGeom.Xformable(robot_prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, TABLE_H))

        gripper = ParallelGripper(
            end_effector_prim_path="/World/Franka/panda_rightfinger",
            joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
            joint_opened_positions=np.array([0.05, 0.05]),
            joint_closed_positions=np.array([0.02, 0.02]),
            action_deltas=np.array([0.01, 0.01]),
        )
        self.franka = self.world.scene.add(SingleManipulator(
            prim_path="/World/Franka",
            name="franka",
            end_effector_prim_path="/World/Franka/panda_rightfinger",
            gripper=gripper,
        ))
        self.franka.gripper.set_default_state(self.franka.gripper.joint_opened_positions)

    # ---- Wrist camera ----

    def _add_wrist_camera(self) -> None:
        """
        Eye-in-hand camera mounted on panda_hand, looking along the gripper axis.
        Being a USD child of panda_hand, it follows all arm motion automatically.

        NOTE: wrist_camera.initialize() is intentionally NOT called in _init_world()
        to avoid a second RTX render target (OOM risk on 12 GB RTX 4070).
        """
        self.wrist_camera = Camera(
            prim_path="/World/Franka/panda_hand/WristCamera",
            frequency=20,
            resolution=(320, 240),
        )
        stage = get_current_stage()
        cam_prim = stage.GetPrimAtPath("/World/Franka/panda_hand/WristCamera")
        xf = UsdGeom.Xformable(cam_prim)
        xf.ClearXformOpOrder()
        # Camera native look direction = camera local +X (confirmed empirically).
        # panda_hand +X = world -Z (downward toward table) in Isaac Sim default arm pose.
        # No rotation needed — with ClearXformOpOrder(), camera local +X = panda_hand +X = DOWN.
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        xf.AddRotateYOp().Set(180.0)
        UsdGeom.Camera(cam_prim).GetFocalLengthAttr().Set(10.0)

        # Blue sight cone along camera +X (= look direction = world -Z = DOWN).
        # Cone at (0.10, 0, 0) axis "X" → apex points in camera local +X = world DOWN ✓
        cone = UsdGeom.Cone.Define(stage, "/World/Franka/panda_hand/WristCamera/SightCone")
        cone.GetHeightAttr().Set(0.20)
        cone.GetRadiusAttr().Set(0.025)
        cone.GetAxisAttr().Set("X")   # apex at +X = camera look direction
        cone.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.2, 0.6, 1.0)]))  # blue
        xf_c = UsdGeom.Xformable(cone.GetPrim())
        xf_c.ClearXformOpOrder()
        xf_c.AddTranslateOp().Set(Gf.Vec3d(0.10, 0.0, 0.0))  # 10 cm in camera +X (look dir)

    # ---- Static collision helper ----

    def _apply_static_collision(self, prim_path: str) -> None:
        """Make a visual prim a kinematic rigid body so physics objects rest on it.

        CollisionAPI alone is not enough for PhysX to block dynamic bodies.
        Pairing it with RigidBodyAPI (kinematicEnabled=True) creates a static
        collider that stays in place but blocks DynamicCuboids.
        """
        stage = get_current_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if prim.IsValid():
            UsdPhysics.CollisionAPI.Apply(prim)
            rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)
            rb_api.GetKinematicEnabledAttr().Set(True)

    # ---- World init ----

    def _init_world(self) -> None:
        self.world.reset()
        self.camera.initialize()
        self.camera.add_distance_to_image_plane_to_frame()  # enables depth output
        # end_effector_initial_height (_h1): the hover height used in Phase 0 (approach)
        # and Phase 4 (lift after grasp). Must be above the table surface (0.40m) and
        # the object centers (0.425m). 0.65m puts the EE 22cm above the table — enough
        # clearance for RMPflow to plan a clean downward approach without straining
        # against the table obstacle.
        self.controller = PickPlaceController(
            name="pick_place_controller",
            gripper=self.franka.gripper,
            robot_articulation=self.franka,
            end_effector_initial_height=0.65,
        )
        self.articulation_controller = self.franka.get_articulation_controller()

        # Register table surface and all bin walls as static RMPflow obstacles.
        # RMPflow only avoids geometry that is explicitly registered here —
        # USD physics collision alone is not seen by the motion planner.
        self.controller._cspace_controller.add_obstacle(self._table_top, static=True)
        for wall in self._bin_wall_prims:
            self.controller._cspace_controller.add_obstacle(wall, static=True)
        print(f"[Isaac Scene] Registered {1 + len(self._bin_wall_prims)} RMPflow obstacles.")

    # ------------------------------------------------------------------
    # Physics warm-up
    # ------------------------------------------------------------------

    def settle_physics(self, steps: int = 60) -> None:
        """Step physics without robot commands so objects settle on the table."""
        print(f"[Isaac Scene] Settling physics ({steps} steps)...")
        for _ in range(steps):
            self.world.step(render=True)

    # ------------------------------------------------------------------
    # Camera capture
    # ------------------------------------------------------------------

    def capture_frame(self) -> tuple:
        """
        Capture one RGB+depth frame from the overhead camera.
        Returns (jpeg_bytes, raw_bgr_frame, depth_array).
          - jpeg_bytes   : JPEG-encoded RGB image (bytes)
          - raw_bgr_frame: BGR numpy array (H×W×3, uint8)
          - depth_array  : distance-to-image-plane in meters (H×W, float32);
                           camera is at z=pole_h=2.20m looking straight down,
                           so depth ≈ (pole_h − object_world_z).
        """
        self.world.step(render=True)
        rgba  = self.camera.get_rgba()
        rgb   = rgba[:, :, :3].astype(np.uint8)
        bgr   = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        depth = self.camera.get_depth()   # (H, W) float32, meters
        _, jpeg = cv2.imencode(".jpg", bgr)
        print(f"[Isaac Scene] Frame captured ({jpeg.nbytes / 1024:.1f} KB), "
              f"depth shape: {depth.shape}, range: [{depth.min():.2f}, {depth.max():.2f}] m.")
        return jpeg.tobytes(), bgr, depth

    # ------------------------------------------------------------------
    # World-state helpers
    # ------------------------------------------------------------------

    def get_object_world_position(self, obj_id: str) -> np.ndarray:
        return self.objects[obj_id].get_world_pose()[0]

    def get_bin_position(self, box_id: int) -> np.ndarray:
        return BIN_POSITIONS[box_id]

    # ------------------------------------------------------------------
    # Pick-and-place execution
    # ------------------------------------------------------------------

    def run_pick_and_place(
        self,
        pick_pos: np.ndarray,
        place_pos: np.ndarray,
        max_steps: int = 2000,
    ) -> bool:
        """
        Run one full pick-and-place cycle using PickPlaceController.
        Returns True on success, False on timeout.
        """
        self.controller.reset()
        for _ in range(max_steps):
            actions = self.controller.forward(
                picking_position=pick_pos,
                placing_position=place_pos,
                current_joint_positions=self.franka.get_joint_positions(),
                end_effector_offset=np.array([0, 0.005, 0]),
            )
            self.articulation_controller.apply_action(actions)
            self.world.step(render=True)
            if self.controller.is_done():
                return True
        return False
