from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(slots=True)
class TraceWriter:
    """Persist system prompt and conversation events for a run."""

    trace_dir: Path
    system_prompt_filename: str = "designer_system_prompt.txt"
    conversation_filename: str = "conversation.jsonl"
    system_prompt_path: Path = field(init=False)
    conversation_path: Path = field(init=False)
    _conversation_file: Any = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.trace_dir.mkdir(parents=True, exist_ok=True)
        self.system_prompt_path = self.trace_dir / self.system_prompt_filename
        self.conversation_path = self.trace_dir / self.conversation_filename
        self._conversation_file = self.conversation_path.open("w", encoding="utf-8")

    def write_system_prompt(self, system_prompt: str) -> None:
        self.system_prompt_path.write_text(system_prompt, encoding="utf-8")

    def write_message(self, message: dict[str, Any]) -> None:
        record = {
            "ts": time.time(),
            "type": "message",
            "message": message,
        }
        self._conversation_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._conversation_file.flush()

    def close(self) -> None:
        try:
            self._conversation_file.close()
        except Exception:
            pass
