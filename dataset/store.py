from __future__ import annotations

from pathlib import Path


class DatasetStore:
    def __init__(self, root: Path):
        self.root = root
