# perception

Captures a frame from the camera and uses a Vision Language Model to identify objects and their 3D positions.

## File

`vlm_client.py` — `PerceptionClient` class

## How it works

1. `capture_frame()` opens the webcam (OpenCV). With `show_preview=True`, a live window lets you frame the shot — press **SPACE** to capture, **Q** to cancel.
2. The JPEG frame is sent to **Qwen 2.5 VL 7B** via Ollama's `chat()` API.
3. The VLM response is stripped of markdown fences and parsed into a validated scene dict.
4. If `save_debug=True`, three timestamped files are written to `debug/`.

## Output

```json
{
  "objects": [
    {"id": "obj_1", "class_label": "ClassA", "coords": [-0.2, 0.1, 0.3]},
    {"id": "obj_2", "class_label": "ClassB", "coords": [ 0.0, 0.0, 0.3]},
    {"id": "obj_3", "class_label": "ClassC", "coords": [ 0.2,-0.1, 0.3]}
  ]
}
```

## Color → class mapping (laptop POC)

Place physically colored objects in front of the webcam:

| Color | Class | Box |
| :--- | :--- | :--- |
| Red | ClassA | 1 |
| Blue | ClassB | 2 |
| Green | ClassC | 3 |

## Notes

- `coords` are normalized image coordinates: x/y in `[-0.5, 0.5]`, z fixed at `0.3` (no depth sensor on laptop).
- On Isaac Sim, z comes from the real depth channel — replace `capture_frame()` with the sim RGB-D sensor. Everything else stays the same.
- Model: `qwen2.5vl:7b` (~5.5 GB). Requires ~5.8 GB free RAM (fits on Isaac Sim machine, may OOM on a laptop).
