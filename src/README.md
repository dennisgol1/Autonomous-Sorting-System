# src

The main source package. Entry point is `main.py`, which wires the three pipeline layers together.

## Pipeline

```
Webcam / Isaac Sim sensor
        |
        v
perception/vlm_client.py   — identifies objects → scene dict
        |
        v
reasoning/llm_client.py    — maps classes to boxes → sort plan
        |
        v
execution/hardware_api.py  — pick-and-place state machine
```

## Running

```bash
cd src
python main.py
```

## Mode flags (top of `main.py`)

| `USE_VLM` | `USE_LLM` | What runs | Requires |
| :--- | :--- | :--- | :--- |
| `False` | `False` | Hardcoded objects → mock robot | Nothing |
| `False` | `True` | Mock scene → DeepSeek-R1 → mock robot | Ollama |
| `True` | `True` | Webcam → Qwen VL → DeepSeek-R1 → mock robot | Ollama + camera |

`SHOW_PREVIEW = True` — opens a live webcam window when `USE_VLM=True`. Press **SPACE** to capture, **Q** to cancel.

`MAX_RETRIES = 3` — how many times the state machine retries a recoverable failure before halting.
