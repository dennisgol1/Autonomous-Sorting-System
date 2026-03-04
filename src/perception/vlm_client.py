import json
import re
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import ollama

# Debug output directory — project root/debug/
DEBUG_DIR = Path(__file__).parent.parent.parent / "debug"

# Color-to-class mapping enforced via the system prompt.
# Laptop POC: place physically colored objects in front of the webcam.
#   Red object   -> ClassA -> Box 1
#   Blue object  -> ClassB -> Box 2
#   Green object -> ClassC -> Box 3
CLASS_COLOR_MAP = {
    "red":   "ClassA",
    "blue":  "ClassB",
    "green": "ClassC",
}

VISION_PROMPT = """You are a robotic vision system analyzing a top-down view of a workspace.
Identify all distinct colored objects (cubes) visible on the table surface.

For each object determine:
- A unique id ("obj_1", "obj_2", etc.)
- class_label based on the object's dominant color:
    * Red object   -> "ClassA"
    * Blue object  -> "ClassB"
    * Green object -> "ClassC"
- Normalized image coordinates [x, y, z]:
    * x: -0.5 (left edge) to +0.5 (right edge)
    * y: -0.5 (bottom edge) to +0.5 (top edge)
    * z: 0.0  (placeholder — real depth will be supplied by the depth sensor)

CRITICAL: Respond ONLY with a valid JSON object. No prose, no markdown code fences.

Required format:
{
  "objects": [
    {"id": "obj_1", "class_label": "ClassA", "coords": [-0.2,  0.1, 0.0]},
    {"id": "obj_2", "class_label": "ClassB", "coords": [ 0.0,  0.0, 0.0]},
    {"id": "obj_3", "class_label": "ClassC", "coords": [ 0.2, -0.1, 0.0]}
  ]
}"""


class PerceptionClient:
    def __init__(self, model: str = "moondream", camera_index: int = 0):
        self.model = model
        self.camera_index = camera_index
        DEBUG_DIR.mkdir(exist_ok=True)
        print(f"[Perception] Initialized PerceptionClient (model: {self.model})")
        print(f"[Perception] Debug output directory: {DEBUG_DIR}")

    # ------------------------------------------------------------------
    # Frame capture
    # ------------------------------------------------------------------

    def capture_frame(self, show_preview: bool = True) -> tuple:
        """
        Opens the webcam and returns (jpeg_bytes, raw_bgr_frame).

        If show_preview=True, a live window is shown so you can frame the shot.
        Press SPACE to capture, Q to cancel.

        On Isaac Sim: replace this method with sim RGB sensor output.
        The rest of the pipeline is unchanged.
        """
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            raise RuntimeError(
                f"[Perception] Cannot open camera (index {self.camera_index}). "
                "Check that a webcam is connected."
            )

        raw_frame = None

        if show_preview:
            print("[Perception] Live preview active.")
            print("[Perception] Press SPACE to capture, Q to cancel.")
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                cv2.imshow("Perception Preview  |  SPACE = capture   Q = cancel", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord(" "):
                    raw_frame = frame
                    break
                elif key == ord("q"):
                    cap.release()
                    cv2.destroyAllWindows()
                    raise RuntimeError("[Perception] Capture cancelled by user.")
            cv2.destroyAllWindows()
        else:
            ret, raw_frame = cap.read()

        cap.release()

        if raw_frame is None:
            raise RuntimeError("[Perception] Failed to capture a frame.")

        _, jpeg = cv2.imencode(".jpg", raw_frame)
        print(f"[Perception] Frame captured ({jpeg.nbytes / 1024:.1f} KB).")
        return jpeg.tobytes(), raw_frame

    # ------------------------------------------------------------------
    # Debug output
    # ------------------------------------------------------------------

    def _save_debug(self, raw_frame, raw_response: str, scene: dict, depth=None) -> None:
        """
        Saves debug artefacts to debug/ with a shared timestamp prefix:
          YYYYMMDD_HHMMSS_frame.jpg    — the captured RGB image
          YYYYMMDD_HHMMSS_depth.png    — colorized depth map (if depth provided)
          YYYYMMDD_HHMMSS_vlm_raw.txt  — the exact text the VLM returned
          YYYYMMDD_HHMMSS_scene.json   — the parsed & validated scene dict
        """
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")

        frame_path = DEBUG_DIR / f"{ts}_frame.jpg"
        cv2.imwrite(str(frame_path), raw_frame)

        if depth is not None:
            depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_PLASMA)
            depth_path = DEBUG_DIR / f"{ts}_depth.png"
            cv2.imwrite(str(depth_path), depth_color)
            print(f"  Depth   : {depth_path.name}")

        raw_path = DEBUG_DIR / f"{ts}_vlm_raw.txt"
        raw_path.write_text(raw_response, encoding="utf-8")

        scene_path = DEBUG_DIR / f"{ts}_scene.json"
        scene_path.write_text(json.dumps(scene, indent=2), encoding="utf-8")

        print(f"[Perception] Debug artefacts saved to {DEBUG_DIR}/")
        print(f"  Frame   : {frame_path.name}")
        print(f"  VLM raw : {raw_path.name}")
        print(f"  Scene   : {scene_path.name}")

    # ------------------------------------------------------------------
    # Response parsing & validation
    # ------------------------------------------------------------------

    def _clean_response(self, text: str) -> str:
        """Strip markdown fences and isolate the outermost JSON object."""
        text = re.sub(r"```(?:json)?\s*", "", text)
        text = text.strip()
        # Use raw_decode so we parse exactly the first complete JSON object and
        # ignore any trailing prose that models like moondream append after the JSON.
        idx = text.find("{")
        if idx == -1:
            raise ValueError(f"No JSON object found in VLM response:\n{text[:300]}")
        try:
            obj, _ = json.JSONDecoder().raw_decode(text, idx)
            return json.dumps(obj)
        except json.JSONDecodeError as e:
            raise ValueError(
                f"No valid JSON object found in VLM response ({e}):\n{text[:300]}"
            )

    def _validate_scene(self, scene: dict) -> None:
        """Ensure the parsed scene has the structure ReasoningClient expects."""
        if "objects" not in scene:
            raise ValueError("Scene metadata missing top-level 'objects' key.")
        required_keys = {"id", "class_label", "coords"}
        for i, obj in enumerate(scene["objects"]):
            missing = required_keys - set(obj.keys())
            if missing:
                raise ValueError(f"Object {i} missing keys: {missing}. Got: {obj}")
            if not isinstance(obj["coords"], list) or len(obj["coords"]) != 3:
                raise ValueError(
                    f"Object {i} 'coords' must be [x, y, z]. Got: {obj['coords']}"
                )

    # ------------------------------------------------------------------
    # VLM inference
    # ------------------------------------------------------------------

    def analyze_frame(
        self,
        frame_bytes: bytes,
        raw_frame=None,
        depth=None,
        save_debug: bool = True,
    ) -> dict:
        """
        Sends a JPEG frame to Qwen VL and returns parsed scene metadata.

        Args:
            frame_bytes : JPEG bytes of the RGB frame.
            raw_frame   : BGR numpy array for debug saving (optional).
            depth       : (H×W) float32 depth array in meters from the depth sensor.
                          If provided, real z values are looked up at each object's
                          normalized (x, y) position and stored in coords[2].
            save_debug  : Save debug artefacts to debug/ if True.
        """
        print(f"[Perception] Sending frame to {self.model}...")

        try:
            response = ollama.chat(
                model=self.model,
                messages=[{
                    "role": "user",
                    "content": VISION_PROMPT,
                    "images": [frame_bytes],
                }],
            )
        except Exception as e:
            raise ConnectionError(
                f"[Perception] Failed to reach Ollama. Is it running? Error: {e}"
            )

        raw = response["message"]["content"]
        print(f"[Perception] Response received ({len(raw)} chars). Parsing...")

        # Always save raw response so failures are diagnosable.
        if save_debug and raw_frame is not None:
            ts = __import__("datetime").datetime.now().strftime("%Y%m%d_%H%M%S")
            DEBUG_DIR.mkdir(exist_ok=True)
            (DEBUG_DIR / f"{ts}_vlm_raw.txt").write_text(raw, encoding="utf-8")
            print(f"[Perception] Raw VLM response saved → debug/{ts}_vlm_raw.txt")

        json_str = self._clean_response(raw)
        scene = json.loads(json_str)
        self._validate_scene(scene)

        # Replace placeholder z=0.0 with real depth sampled at each object's pixel.
        # Normalized coords: x ∈ [-0.5, 0.5] left→right, y ∈ [-0.5, 0.5] bottom→top.
        # Camera looks straight down from height ~2.20 m, so depth ≈ (2.20 - world_z).
        if depth is not None and isinstance(depth, np.ndarray):
            h, w = depth.shape[:2]
            for obj in scene["objects"]:
                x_n, y_n, _ = obj["coords"]
                px = int(np.clip((x_n + 0.5) * w, 0, w - 1))
                py = int(np.clip((0.5 - y_n) * h, 0, h - 1))  # y: +0.5=top=row 0
                obj["coords"][2] = round(float(depth[py, px]), 4)
            print(f"[Perception] Depth z values applied to {len(scene['objects'])} object(s).")

        if save_debug and raw_frame is not None:
            self._save_debug(raw_frame, raw, scene, depth=depth)

        print(f"[Perception] Scene validated: {len(scene['objects'])} object(s) detected.")
        return scene

    # ------------------------------------------------------------------
    # Full pipeline entry point
    # ------------------------------------------------------------------

    def perceive(self, show_preview: bool = True, save_debug: bool = True) -> dict:
        """Capture a frame from the webcam and return scene metadata."""
        frame_bytes, raw_frame = self.capture_frame(show_preview=show_preview)
        return self.analyze_frame(frame_bytes, raw_frame=raw_frame, save_debug=save_debug)
