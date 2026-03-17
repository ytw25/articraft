from __future__ import annotations

import sys
from pathlib import Path


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from dataset.cli import main as dataset_main
from dataset.models import DatasetRun


def main() -> None:
    assert callable(dataset_main)
    run = DatasetRun(category="tools", prompt="pliers", output_dir=Path("outputs/pliers"))
    assert run.status == "pending"


if __name__ == "__main__":
    main()
