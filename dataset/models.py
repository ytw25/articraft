from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass(slots=True)
class DatasetRun:
    category: str
    prompt: str
    output_dir: Path
    status: str = "pending"
    tags: list[str] = field(default_factory=list)
