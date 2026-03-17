from __future__ import annotations

import sys
from pathlib import Path


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


from workbench.cli import main as workbench_main
from workbench.models import WorkbenchRun


def main() -> None:
    assert callable(workbench_main)
    run = WorkbenchRun(prompt="pliers", output_dir=Path("outputs/pliers"))
    assert run.status == "pending"


if __name__ == "__main__":
    main()
