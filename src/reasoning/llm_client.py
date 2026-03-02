import json
import re

import ollama

# Sorting rules enforced via the system prompt.
# When the VLM is integrated, class labels will come from it dynamically.
SORTING_RULES = {
    "ClassA": 1,
    "ClassB": 2,
    "ClassC": 3,
}

SYSTEM_PROMPT = """You are a robotic sorting controller. You will receive a JSON object \
describing objects detected in a scene. Your task is to assign each object to its correct \
target box using the sorting rules below.

Sorting Rules:
- ClassA -> Box 1
- ClassB -> Box 2
- ClassC -> Box 3

CRITICAL: Your entire response must be ONLY a valid JSON array.
No prose, no markdown code fences, no <think> blocks.

Required output schema:
[
  {
    "name": "<object id>",
    "class_label": "<class label>",
    "coords": [x, y, z],
    "target_box": <integer box id>
  }
]"""


class ReasoningClient:
    def __init__(self, model: str = "deepseek-r1:8b"):
        self.model = model
        print(f"[Reasoning] Initialized ReasoningClient (model: {self.model})")

    def _clean_response(self, text: str) -> str:
        """
        Strips DeepSeek-R1's <think> reasoning blocks and any markdown fences,
        then isolates the JSON array from the response.
        """
        # Remove <think>...</think> blocks (DeepSeek-R1 chain-of-thought output)
        text = re.sub(r"<think>.*?</think>", "", text, flags=re.DOTALL)
        # Remove markdown code fences (```json ... ```)
        text = re.sub(r"```(?:json)?\s*", "", text)
        text = text.strip()
        # Extract the outermost JSON array
        match = re.search(r"\[.*\]", text, re.DOTALL)
        if not match:
            raise ValueError(f"No JSON array found in LLM response:\n{text}")
        return match.group(0)

    def _validate_plan(self, plan: list) -> None:
        """
        Ensures every action dict has the required keys and correct types
        before handing it to the state machine.
        """
        required_keys = {"name", "class_label", "coords", "target_box"}
        for i, item in enumerate(plan):
            missing = required_keys - set(item.keys())
            if missing:
                raise ValueError(f"Item {i} missing keys: {missing}. Got: {item}")
            if not isinstance(item["coords"], list) or len(item["coords"]) != 3:
                raise ValueError(
                    f"Item {i} 'coords' must be a list of 3 floats. Got: {item['coords']}"
                )
            if not isinstance(item["target_box"], int):
                raise ValueError(
                    f"Item {i} 'target_box' must be an integer. Got: {item['target_box']}"
                )

    def generate_sort_plan(self, scene_metadata: dict) -> list[dict]:
        """
        Queries DeepSeek-R1 to produce a validated sort plan from scene metadata.

        Args:
            scene_metadata: {
                "objects": [
                    {"id": str, "class_label": str, "coords": [x, y, z]},
                    ...
                ]
            }

        Returns:
            A validated list of action dicts ready for SortingStateMachine.
        """
        user_message = json.dumps(scene_metadata, indent=2)
        print(
            f"[Reasoning] Sending {len(scene_metadata['objects'])} object(s) "
            f"to {self.model}..."
        )

        try:
            response = ollama.chat(
                model=self.model,
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": user_message},
                ],
            )
        except Exception as e:
            raise ConnectionError(
                f"[Reasoning] Failed to reach Ollama. Is it running? Error: {e}"
            )

        raw = response["message"]["content"]
        print(f"[Reasoning] Response received ({len(raw)} chars). Parsing...")

        json_str = self._clean_response(raw)
        sort_plan = json.loads(json_str)
        self._validate_plan(sort_plan)

        print(f"[Reasoning] Sort plan validated: {len(sort_plan)} object(s) queued.")
        return sort_plan
