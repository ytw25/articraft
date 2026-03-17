from __future__ import annotations

from dataclasses import dataclass

from workbench.store import WorkbenchStore


@dataclass(slots=True)
class WorkbenchService:
    store: WorkbenchStore
