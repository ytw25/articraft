from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_LENGTH = 0.320
BASE_DEPTH = 0.168
BASE_HEIGHT = 0.020
MOTOR_CENTER_Z = 0.186
WHEEL_CENTER_X = 0.176
WHEEL_RADIUS = 0.088
WHEEL_WIDTH = 0.026
GUARD_RADIUS = 0.114
GUARD_WIDTH = 0.082
REST_PIVOT_Y = 0.096
REST_PIVOT_Z = 0.068
DEFLECTOR_PIVOT_Y = 0.106
DEFLECTOR_PIVOT_Z = 0.294
SHIELD_PIVOT_Y = 0.116
SHIELD_PIVOT_Z = 0.404


def _make_frame_mesh():
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_DEPTH, BASE_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT * 0.5))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.006)
    )

    pedestal = (
        cq.Workplane("XY")
        .box(0.170, 0.088, 0.074)
        .translate((0.0, 0.0, 0.056))
        .edges("|Z")
        .fillet(0.016)
    )

    rear_web = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-0.022, 0.020),
                (-0.045, 0.020),
                (-0.030, 0.150),
                (0.030, 0.150),
                (0.045, 0.020),
                (0.022, 0.020),
            ]
        )
        .close()
        .extrude(0.084, both=True)
    )

    motor = (
        cq.Workplane("YZ")
        .circle(0.073)
        .extrude(0.118, both=True)
        .translate((0.0, 0.0, MOTOR_CENTER_Z))
    )
    motor = motor.union(
        cq.Workplane("YZ")
        .circle(0.066)
        .extrude(0.152, both=True)
        .translate((0.0, 0.0, MOTOR_CENTER_Z))
    )

    body = base.union(pedestal).union(rear_web).union(motor)

    for sign in (-1.0, 1.0):
        x = sign * WHEEL_CENTER_X
        neck = (
            cq.Workplane("YZ")
            .circle(0.045)
            .extrude(0.040, both=True)
            .translate((sign * 0.108, 0.0, MOTOR_CENTER_Z))
        )
        guard_shell = (
            cq.Workplane("YZ")
            .circle(GUARD_RADIUS)
            .extrude(GUARD_WIDTH * 0.5, both=True)
            .translate((x, 0.0, MOTOR_CENTER_Z))
        )
        guard_shell = guard_shell.cut(
            cq.Workplane("YZ")
            .circle(WHEEL_RADIUS + 0.015)
            .extrude((GUARD_WIDTH - 0.020) * 0.5, both=True)
            .translate((x, 0.0, MOTOR_CENTER_Z))
        )
        guard_shell = guard_shell.cut(
            cq.Workplane("XY")
            .box(0.140, 0.240, 0.118)
            .translate((x, 0.066, MOTOR_CENTER_Z - 0.060))
        )
        guard_shell = guard_shell.cut(
            cq.Workplane("XY")
            .box(0.140, 0.116, 0.110)
            .translate((x, 0.076, MOTOR_CENTER_Z + 0.068))
        )
        guard_shell = guard_shell.cut(
            cq.Workplane("YZ")
            .circle(0.018)
            .extrude(0.024)
            .translate((x + sign * (GUARD_WIDTH * 0.5 - 0.012), 0.0, MOTOR_CENTER_Z))
        )
        outer_cap = (
            cq.Workplane("YZ")
            .circle(GUARD_RADIUS * 0.98)
            .extrude(0.009)
            .translate((x + sign * (GUARD_WIDTH * 0.5 - 0.0045), 0.0, MOTOR_CENTER_Z))
        )
        outer_cap = outer_cap.cut(
            cq.Workplane("YZ")
            .circle(0.034)
            .extrude(0.012)
            .translate((x + sign * (GUARD_WIDTH * 0.5 - 0.0045), 0.0, MOTOR_CENTER_Z))
        )
        spindle = (
            cq.Workplane("YZ")
            .circle(0.010)
            .extrude(0.022)
            .translate((x + sign * 0.029, 0.0, MOTOR_CENTER_Z))
        )
        rest_support = (
            cq.Workplane("XY")
            .box(0.080, 0.024, 0.026)
            .translate((sign * 0.148, 0.090, 0.066))
        )
        deflector_support = (
            cq.Workplane("XY")
            .box(0.080, 0.050, 0.032)
            .translate((sign * 0.148, 0.082, 0.289))
        )
        shield_support = (
            cq.Workplane("XY")
            .box(0.080, 0.056, 0.132)
            .translate((sign * 0.148, 0.088, 0.338))
        )
        body = body.union(neck).union(guard_shell).union(outer_cap).union(spindle).union(rest_support).union(deflector_support).union(shield_support)

    return body


def _add_wheel(part, *, stone: str, steel: str, side_sign: float) -> None:
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=stone,
        name="wheel_stone",
    )
    part.visual(
        Cylinder(radius=0.028, length=WHEEL_WIDTH + 0.010),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="wheel_flange",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(side_sign * 0.029, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="wheel_hub",
    )


def _add_rest(part, *, steel: str) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="rest_pivot",
    )
    part.visual(
        Box((0.018, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.018, 0.004)),
        material=steel,
        name="rest_arm",
    )
    part.visual(
        Box((0.078, 0.026, 0.006)),
        origin=Origin(xyz=(0.0, 0.037, 0.015)),
        material=steel,
        name="rest_table",
    )
    part.visual(
        Box((0.078, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.050, 0.019)),
        material=steel,
        name="rest_lip",
    )


def _add_deflector(part, *, steel: str) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="deflector_pivot",
    )
    part.visual(
        Box((0.016, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.008, 0.010)),
        material=steel,
        name="deflector_arm",
    )
    part.visual(
        Box((0.074, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, 0.015, 0.022), rpy=(-0.55, 0.0, 0.0)),
        material=steel,
        name="deflector_plate",
    )


def _add_shield(part, *, steel: str, clear_tint: str) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="shield_pivot",
    )
    part.visual(
        Box((0.080, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.006, -0.004)),
        material=steel,
        name="shield_bar",
    )
    part.visual(
        Box((0.010, 0.010, 0.042)),
        origin=Origin(xyz=(0.0, 0.015, -0.022)),
        material=steel,
        name="shield_stem",
    )
    part.visual(
        Box((0.088, 0.003, 0.056)),
        origin=Origin(xyz=(0.0, 0.022, -0.036), rpy=(0.30, 0.0, 0.0)),
        material=clear_tint,
        name="shield_panel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_grinder")

    cast_gray = model.material("cast_gray", rgba=(0.48, 0.51, 0.55, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.30, 0.32, 0.35, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    wheel_stone = model.material("wheel_stone", rgba=(0.70, 0.71, 0.66, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))
    clear_tint = model.material("clear_tint", rgba=(0.78, 0.92, 1.00, 0.34))

    frame = model.part("frame")
    frame.visual(mesh_from_cadquery(_make_frame_mesh(), "bench_grinder_frame"), material=cast_gray, name="frame_shell")
    frame.visual(Box((0.120, 0.032, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.004)), material=dark_cast, name="switch_plinth")
    for x, y in ((0.112, 0.050), (0.112, -0.050), (-0.112, 0.050), (-0.112, -0.050)):
        frame.visual(
            Cylinder(radius=0.015, length=0.010),
            origin=Origin(xyz=(x, y, 0.005)),
            material=rubber,
        )
    left_wheel = model.part("left_wheel")
    right_wheel = model.part("right_wheel")
    left_rest = model.part("left_rest")
    right_rest = model.part("right_rest")
    left_deflector = model.part("left_deflector")
    right_deflector = model.part("right_deflector")
    left_shield = model.part("left_shield")
    right_shield = model.part("right_shield")
    _add_wheel(left_wheel, stone="wheel_stone", steel="steel", side_sign=-1.0)
    _add_wheel(right_wheel, stone="wheel_stone", steel="steel", side_sign=1.0)
    _add_rest(left_rest, steel="steel")
    _add_rest(right_rest, steel="steel")
    _add_deflector(left_deflector, steel="steel")
    _add_deflector(right_deflector, steel="steel")
    _add_shield(left_shield, steel="steel", clear_tint="clear_tint")
    _add_shield(right_shield, steel="steel", clear_tint="clear_tint")

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=left_wheel,
        origin=Origin(xyz=(-WHEEL_CENTER_X, 0.0, MOTOR_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=25.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, 0.0, MOTOR_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=25.0),
    )
    model.articulation(
        "left_rest_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_rest,
        origin=Origin(xyz=(-WHEEL_CENTER_X, REST_PIVOT_Y, REST_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=-0.30, upper=0.22),
    )
    model.articulation(
        "right_rest_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_rest,
        origin=Origin(xyz=(WHEEL_CENTER_X, REST_PIVOT_Y, REST_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.0, lower=-0.30, upper=0.22),
    )
    model.articulation(
        "left_deflector_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_deflector,
        origin=Origin(xyz=(-WHEEL_CENTER_X, DEFLECTOR_PIVOT_Y, DEFLECTOR_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=-0.20, upper=0.55),
    )
    model.articulation(
        "right_deflector_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_deflector,
        origin=Origin(xyz=(WHEEL_CENTER_X, DEFLECTOR_PIVOT_Y, DEFLECTOR_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=-0.20, upper=0.55),
    )
    model.articulation(
        "left_shield_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_shield,
        origin=Origin(xyz=(-WHEEL_CENTER_X, SHIELD_PIVOT_Y, SHIELD_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=-0.45, upper=0.65),
    )
    model.articulation(
        "right_shield_tilt",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_shield,
        origin=Origin(xyz=(WHEEL_CENTER_X, SHIELD_PIVOT_Y, SHIELD_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.2, lower=-0.45, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_rest = object_model.get_part("left_rest")
    right_rest = object_model.get_part("right_rest")
    left_deflector = object_model.get_part("left_deflector")
    right_deflector = object_model.get_part("right_deflector")
    left_shield = object_model.get_part("left_shield")
    right_shield = object_model.get_part("right_shield")

    ctx.allow_overlap(
        frame,
        left_wheel,
        reason="The left wheel, arbor, and guard seat are simplified as a nested assembly inside the cast housing shell.",
    )
    ctx.allow_overlap(
        frame,
        right_wheel,
        reason="The right wheel, arbor, and guard seat are simplified as a nested assembly inside the cast housing shell.",
    )
    ctx.allow_overlap(
        frame,
        left_rest,
        reason="The left tool rest bracket is simplified as a compact pivoting arm that slightly nests into the cast support.",
    )
    ctx.allow_overlap(
        frame,
        right_rest,
        reason="The right tool rest bracket is simplified as a compact pivoting arm that slightly nests into the cast support.",
    )
    ctx.allow_overlap(
        frame,
        left_deflector,
        reason="The left spark deflector hinge is simplified as a compact arm nested into the cast support boss.",
    )
    ctx.allow_overlap(
        frame,
        right_deflector,
        reason="The right spark deflector hinge is simplified as a compact arm nested into the cast support boss.",
    )
    ctx.allow_overlap(
        frame,
        left_shield,
        reason="The left eye shield bracket is simplified as a short pivot barrel nested into the support upright.",
    )
    ctx.allow_overlap(
        frame,
        right_shield,
        reason="The right eye shield bracket is simplified as a short pivot barrel nested into the support upright.",
    )
    ctx.expect_origin_gap("right_wheel", "left_wheel", axis="x", min_gap=0.30, name="wheel centers span the grinder width")
    ctx.expect_gap(left_wheel, left_rest, axis="z", min_gap=0.004, max_gap=0.016, name="left rest sits just below the wheel")
    ctx.expect_gap(right_wheel, right_rest, axis="z", min_gap=0.004, max_gap=0.016, name="right rest sits just below the wheel")
    ctx.expect_gap(left_deflector, left_wheel, axis="z", min_gap=0.012, max_gap=0.090, name="left deflector sits above the wheel")
    ctx.expect_gap(right_deflector, right_wheel, axis="z", min_gap=0.012, max_gap=0.090, name="right deflector sits above the wheel")
    ctx.expect_gap(left_shield, left_deflector, axis="z", min_gap=0.012, max_gap=0.090, name="left eye shield sits above the spark deflector")
    ctx.expect_gap(right_shield, right_deflector, axis="z", min_gap=0.012, max_gap=0.090, name="right eye shield sits above the spark deflector")

    return ctx.report()


object_model = build_object_model()
