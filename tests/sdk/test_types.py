from __future__ import annotations

import pytest

from sdk import ArticulatedObject, Box


def test_part_get_visual_returns_named_visual() -> None:
    model = ArticulatedObject(name="visual_lookup")
    part = model.part("body")
    visual = part.visual(Box((0.1, 0.1, 0.1)), name="shell")

    assert part.get_visual("shell") is visual


def test_part_get_visual_rejects_missing_name() -> None:
    model = ArticulatedObject(name="visual_lookup")
    part = model.part("body")
    part.visual(Box((0.1, 0.1, 0.1)), name="shell")

    with pytest.raises(Exception, match="Unknown visual"):
        part.get_visual("missing")
