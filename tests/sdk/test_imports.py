from __future__ import annotations

import sys
from pathlib import Path


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import sdk


def main() -> None:
    assert hasattr(sdk, "ArticulatedObject")
    assert hasattr(sdk, "ValidationError")
    assert hasattr(sdk, "TestContext")


if __name__ == "__main__":
    main()
