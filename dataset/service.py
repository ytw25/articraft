from __future__ import annotations

from dataclasses import dataclass

from dataset.store import DatasetStore


@dataclass(slots=True)
class DatasetService:
    store: DatasetStore
