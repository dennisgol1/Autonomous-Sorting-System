# reasoning

Takes the scene dict from the perception layer and uses a reasoning model to decide which box each object belongs in.

## File

`llm_client.py` — `ReasoningClient` class

## How it works

1. `generate_sort_plan()` formats the scene dict into a prompt and calls **DeepSeek-R1 8B** via Ollama.
2. The response is cleaned: `<think>...</think>` blocks are stripped, then markdown fences are removed.
3. The JSON array is parsed and validated — each item must have `name`, `class_label`, `coords`, and `target_box`.

## Output

```json
[
  {"name": "obj_1", "class_label": "ClassA", "coords": [-0.2, 0.1, 0.3], "target_box": 1},
  {"name": "obj_2", "class_label": "ClassB", "coords": [ 0.0, 0.0, 0.3], "target_box": 2},
  {"name": "obj_3", "class_label": "ClassC", "coords": [ 0.2,-0.1, 0.3], "target_box": 3}
]
```

## Notes

- Model: `deepseek-r1:8b` (~5.2 GB). On Ollama this now resolves to the Qwen3-based R1-0528 distill.
- DeepSeek-R1 uses Chain-of-Thought internally (`<think>` blocks) before producing the final answer. These are stripped automatically — only the JSON reaches the state machine.
- The system prompt enforces strict JSON output and the class-to-box mapping rules.
