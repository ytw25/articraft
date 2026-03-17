from __future__ import annotations

import sys
from pathlib import Path


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


import sdk_hybrid


def main() -> None:
    assert hasattr(sdk_hybrid, "ArticulatedObject")
    assert hasattr(sdk_hybrid, "ValidationError")
    assert hasattr(sdk_hybrid, "TestContext")


if __name__ == "__main__":
    main()
