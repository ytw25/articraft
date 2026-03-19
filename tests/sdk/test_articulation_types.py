from __future__ import annotations

import pytest

from sdk import ArticulatedObject, MotionLimits, ValidationError


def test_unknown_articulation_type() -> None:
    model = ArticulatedObject(name="planar_not_supported")
    base = model.part("base")
    slider = model.part("slider")

    with pytest.raises(ValidationError, match="Unknown articulation type"):
        model.articulation("base_to_slider", "planar", parent=base, child=slider)


def test_continuous_requires_motion_limits() -> None:
    model = ArticulatedObject(name="continuous_requires_limits")
    base = model.part("base")
    mast = model.part("mast")
    model.articulation("mast_slew", "continuous", parent=base, child=mast)

    with pytest.raises(
        ValidationError,
        match="must set motion_limits=MotionLimits\\(effort=..., velocity=...\\)",
    ):
        model.validate(strict=True)


def test_continuous_forbids_lower_upper_limits() -> None:
    model = ArticulatedObject(name="continuous_forbids_bounds")
    base = model.part("base")
    mast = model.part("mast")
    model.articulation(
        "mast_slew",
        "continuous",
        parent=base,
        child=mast,
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.0,
        ),
    )

    with pytest.raises(ValidationError, match="cannot include lower/upper limits"):
        model.validate(strict=True)


def test_continuous_accepts_effort_velocity_only() -> None:
    model = ArticulatedObject(name="continuous_valid")
    base = model.part("base")
    mast = model.part("mast")
    model.articulation(
        "mast_slew",
        "continuous",
        parent=base,
        child=mast,
        motion_limits=MotionLimits(effort=12.0, velocity=1.5),
    )
    model.validate(strict=True)
