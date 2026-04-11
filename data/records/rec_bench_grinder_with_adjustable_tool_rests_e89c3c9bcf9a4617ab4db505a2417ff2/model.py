from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

WHEEL_RADIUS = 0.062
WHEEL_THICKNESS = 0.020
WHEEL_CENTER_X = 0.128
WHEEL_BORE_RADIUS = 0.011
def _wheel_ring_mesh(name: str) -> object:
    ring = cq.Workplane("YZ").circle(WHEEL_RADIUS).extrude(WHEEL_THICKNESS / 2.0, both=True)
    ring = ring.cut(cq.Workplane("YZ").circle(WHEEL_BORE_RADIUS).extrude(WHEEL_THICKNESS * 0.55, both=True))
    return mesh_from_cadquery(ring, name)


def _tool_rest_mesh(name: str) -> object:
    platform = cq.Workplane("XY").box(0.032, 0.036, 0.004).translate((0.0, 0.028, 0.011))
    neck = cq.Workplane("XY").box(0.010, 0.018, 0.024).translate((0.0, 0.012, 0.004))
    ear = cq.Workplane("XY").box(0.008, 0.010, 0.016).translate((0.0, 0.004, 0.0))
    return mesh_from_cadquery(platform.union(neck).union(ear), name)


def _switch_mesh() -> object:
    lever = cq.Workplane("XY").box(0.012, 0.016, 0.026).translate((0.0, 0.008, 0.013))
    return mesh_from_cadquery(lever, "power_switch")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_grinder")

    housing_paint = model.material("housing_paint", rgba=(0.36, 0.39, 0.43, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    coarse_stone = model.material("coarse_stone", rgba=(0.47, 0.43, 0.36, 1.0))
    fine_stone = model.material("fine_stone", rgba=(0.72, 0.74, 0.75, 1.0))
    rest_steel = model.material("rest_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    flange_steel = model.material("flange_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    switch_red = model.material("switch_red", rgba=(0.72, 0.10, 0.09, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.190, 0.118, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.093)),
        material=housing_paint,
        name="base",
    )
    housing.visual(
        Box((0.108, 0.084, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=housing_paint,
        name="pedestal",
    )
    housing.visual(
        Cylinder(radius=0.041, length=0.150),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_paint,
        name="motor_body",
    )
    housing.visual(
        Cylinder(radius=0.046, length=0.014),
        origin=Origin(xyz=(-0.071, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_paint,
        name="left_endbell",
    )
    housing.visual(
        Cylinder(radius=0.046, length=0.014),
        origin=Origin(xyz=(0.071, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_paint,
        name="right_endbell",
    )
    housing.visual(
        Box((0.030, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, 0.045, -0.020)),
        material=housing_paint,
        name="switch_bezel",
    )
    for x_pos in (-0.056, 0.056):
        for y_pos in (-0.032, 0.032):
            housing.visual(
                Box((0.030, 0.018, 0.007)),
                origin=Origin(xyz=(x_pos, y_pos, -0.1055)),
                material=foot_rubber,
                name=f"foot_{int(x_pos > 0)}_{int(y_pos > 0)}",
            )
    housing.inertial = Inertial.from_geometry(
        Box((0.320, 0.140, 0.210)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
    )

    left_guard = model.part("left_guard")
    left_guard.visual(
        Box((0.050, 0.046, 0.056)),
        origin=Origin(xyz=(-0.098, 0.0, 0.0)),
        material=housing_paint,
        name="neck",
    )
    left_guard.visual(
        Cylinder(radius=0.074, length=0.005),
        origin=Origin(xyz=(-0.114, -0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_paint,
        name="side_plate",
    )
    left_guard.visual(
        Box((0.018, 0.066, 0.010)),
        origin=Origin(xyz=(-0.120, -0.005, 0.069)),
        material=housing_paint,
        name="top_band",
    )
    left_guard.visual(
        Box((0.018, 0.010, 0.096)),
        origin=Origin(xyz=(-0.120, -0.069, 0.0)),
        material=housing_paint,
        name="rear_band",
    )
    left_guard.visual(
        Box((0.018, 0.026, 0.010)),
        origin=Origin(xyz=(-0.120, -0.040, -0.064)),
        material=housing_paint,
        name="lower_band",
    )
    left_guard.visual(
        Box((0.034, 0.026, 0.030)),
        origin=Origin(xyz=(-WHEEL_CENTER_X, 0.022, -0.072)),
        material=housing_paint,
        name="bracket_web",
    )
    left_guard.visual(
        Box((0.006, 0.008, 0.018)),
        origin=Origin(xyz=(-WHEEL_CENTER_X - 0.009, 0.034, -0.074)),
        material=housing_paint,
        name="plate_0",
    )
    left_guard.visual(
        Box((0.006, 0.008, 0.018)),
        origin=Origin(xyz=(-WHEEL_CENTER_X + 0.009, 0.034, -0.074)),
        material=housing_paint,
        name="plate_1",
    )
    left_guard.inertial = Inertial.from_geometry(
        Box((0.080, 0.140, 0.160)),
        mass=0.9,
        origin=Origin(xyz=(-0.118, -0.008, 0.0)),
    )

    right_guard = model.part("right_guard")
    right_guard.visual(
        Box((0.050, 0.046, 0.056)),
        origin=Origin(xyz=(0.098, 0.0, 0.0)),
        material=housing_paint,
        name="neck",
    )
    right_guard.visual(
        Cylinder(radius=0.074, length=0.005),
        origin=Origin(xyz=(0.114, -0.005, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_paint,
        name="side_plate",
    )
    right_guard.visual(
        Box((0.018, 0.066, 0.010)),
        origin=Origin(xyz=(0.120, -0.005, 0.069)),
        material=housing_paint,
        name="top_band",
    )
    right_guard.visual(
        Box((0.018, 0.010, 0.096)),
        origin=Origin(xyz=(0.120, -0.069, 0.0)),
        material=housing_paint,
        name="rear_band",
    )
    right_guard.visual(
        Box((0.018, 0.026, 0.010)),
        origin=Origin(xyz=(0.120, -0.040, -0.064)),
        material=housing_paint,
        name="lower_band",
    )
    right_guard.visual(
        Box((0.034, 0.026, 0.030)),
        origin=Origin(xyz=(WHEEL_CENTER_X, 0.022, -0.072)),
        material=housing_paint,
        name="bracket_web",
    )
    right_guard.visual(
        Box((0.006, 0.008, 0.018)),
        origin=Origin(xyz=(WHEEL_CENTER_X - 0.009, 0.034, -0.074)),
        material=housing_paint,
        name="plate_0",
    )
    right_guard.visual(
        Box((0.006, 0.008, 0.018)),
        origin=Origin(xyz=(WHEEL_CENTER_X + 0.009, 0.034, -0.074)),
        material=housing_paint,
        name="plate_1",
    )
    right_guard.inertial = Inertial.from_geometry(
        Box((0.080, 0.140, 0.160)),
        mass=0.9,
        origin=Origin(xyz=(0.118, -0.008, 0.0)),
    )

    ring_mesh = _wheel_ring_mesh("grinder_wheel_ring")

    left_wheel = model.part("left_wheel")
    left_wheel.visual(ring_mesh, material=coarse_stone, name="abrasive")
    left_wheel.visual(
        Cylinder(radius=0.018, length=0.003),
        origin=Origin(xyz=(-0.0085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_steel,
        name="inner_flange",
    )
    left_wheel.visual(
        Cylinder(radius=0.018, length=0.003),
        origin=Origin(xyz=(0.0085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_steel,
        name="outer_flange",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_THICKNESS),
        mass=0.72,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(ring_mesh, material=fine_stone, name="abrasive")
    right_wheel.visual(
        Cylinder(radius=0.018, length=0.003),
        origin=Origin(xyz=(-0.0085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_steel,
        name="inner_flange",
    )
    right_wheel.visual(
        Cylinder(radius=0.018, length=0.003),
        origin=Origin(xyz=(0.0085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=flange_steel,
        name="outer_flange",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_THICKNESS),
        mass=0.70,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rest_mesh = _tool_rest_mesh("tool_rest")

    left_rest = model.part("left_rest")
    left_rest.visual(rest_mesh, material=rest_steel, name="tray")
    left_rest.inertial = Inertial.from_geometry(
        Box((0.032, 0.046, 0.034)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.024, 0.007)),
    )

    right_rest = model.part("right_rest")
    right_rest.visual(rest_mesh, material=rest_steel, name="tray")
    right_rest.inertial = Inertial.from_geometry(
        Box((0.032, 0.046, 0.034)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.024, 0.007)),
    )

    power_switch = model.part("power_switch")
    power_switch.visual(_switch_mesh(), material=switch_red, name="lever")
    power_switch.inertial = Inertial.from_geometry(
        Box((0.012, 0.016, 0.026)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.008, 0.013)),
    )

    model.articulation(
        "housing_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_wheel,
        origin=Origin(xyz=(-WHEEL_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=30.0),
    )
    model.articulation(
        "housing_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_wheel,
        origin=Origin(xyz=(WHEEL_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=30.0),
    )
    model.articulation(
        "housing_to_left_guard",
        ArticulationType.FIXED,
        parent=housing,
        child=left_guard,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_right_guard",
        ArticulationType.FIXED,
        parent=housing,
        child=right_guard,
        origin=Origin(),
    )
    model.articulation(
        "housing_to_left_rest",
        ArticulationType.REVOLUTE,
        parent=left_guard,
        child=left_rest,
        origin=Origin(xyz=(-WHEEL_CENTER_X, 0.036, -0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=1.5, lower=-0.20, upper=0.20),
    )
    model.articulation(
        "housing_to_right_rest",
        ArticulationType.REVOLUTE,
        parent=right_guard,
        child=right_rest,
        origin=Origin(xyz=(WHEEL_CENTER_X, 0.036, -0.078)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=1.5, lower=-0.20, upper=0.20),
    )
    model.articulation(
        "housing_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=power_switch,
        origin=Origin(xyz=(0.0, 0.052, -0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=2.0, lower=-0.35, upper=0.35),
    )

    return model


def _aabb_max_z(aabb) -> float | None:
    if aabb is None:
        return None
    return float(aabb[1][2])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_rest = object_model.get_part("left_rest")
    right_rest = object_model.get_part("right_rest")
    power_switch = object_model.get_part("power_switch")

    left_spin = object_model.get_articulation("housing_to_left_wheel")
    right_spin = object_model.get_articulation("housing_to_right_wheel")
    left_rest_joint = object_model.get_articulation("housing_to_left_rest")
    right_rest_joint = object_model.get_articulation("housing_to_right_rest")
    switch_joint = object_model.get_articulation("housing_to_power_switch")

    ctx.check(
        "wheel joints are continuous on a common shaft axis",
        left_spin.articulation_type == ArticulationType.CONTINUOUS
        and right_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_spin.motion_limits is not None
        and right_spin.motion_limits is not None
        and left_spin.motion_limits.lower is None
        and left_spin.motion_limits.upper is None
        and right_spin.motion_limits.lower is None
        and right_spin.motion_limits.upper is None
        and abs(left_spin.origin.xyz[1] - right_spin.origin.xyz[1]) < 1e-9
        and abs(left_spin.origin.xyz[2] - right_spin.origin.xyz[2]) < 1e-9,
        details=f"left_origin={left_spin.origin.xyz}, right_origin={right_spin.origin.xyz}",
    )

    ctx.check(
        "tool rests and power switch use bounded revolute joints",
        left_rest_joint.articulation_type == ArticulationType.REVOLUTE
        and right_rest_joint.articulation_type == ArticulationType.REVOLUTE
        and switch_joint.articulation_type == ArticulationType.REVOLUTE
        and left_rest_joint.motion_limits is not None
        and right_rest_joint.motion_limits is not None
        and switch_joint.motion_limits is not None,
        details="Expected the adjustable rests and front switch to be bounded revolute articulations.",
    )

    ctx.expect_gap(
        left_wheel,
        left_rest,
        axis="z",
        positive_elem="abrasive",
        negative_elem="tray",
        min_gap=0.0,
        max_gap=0.012,
        name="left rest sits just below the left wheel",
    )
    ctx.expect_gap(
        right_wheel,
        right_rest,
        axis="z",
        positive_elem="abrasive",
        negative_elem="tray",
        min_gap=0.0,
        max_gap=0.012,
        name="right rest sits just below the right wheel",
    )
    ctx.expect_overlap(
        left_wheel,
        left_rest,
        axes="x",
        elem_a="abrasive",
        elem_b="tray",
        min_overlap=0.018,
        name="left rest spans the wheel width",
    )
    ctx.expect_overlap(
        right_wheel,
        right_rest,
        axes="x",
        elem_a="abrasive",
        elem_b="tray",
        min_overlap=0.018,
        name="right rest spans the wheel width",
    )

    left_rest_home = _aabb_max_z(ctx.part_element_world_aabb(left_rest, elem="tray"))
    with ctx.pose({left_rest_joint: left_rest_joint.motion_limits.upper}):
        left_rest_up = _aabb_max_z(ctx.part_element_world_aabb(left_rest, elem="tray"))
    ctx.check(
        "left rest pivots upward",
        left_rest_home is not None and left_rest_up is not None and left_rest_up > left_rest_home + 0.005,
        details=f"home={left_rest_home}, up={left_rest_up}",
    )

    right_rest_home = _aabb_max_z(ctx.part_element_world_aabb(right_rest, elem="tray"))
    with ctx.pose({right_rest_joint: right_rest_joint.motion_limits.upper}):
        right_rest_up = _aabb_max_z(ctx.part_element_world_aabb(right_rest, elem="tray"))
    ctx.check(
        "right rest pivots upward",
        right_rest_home is not None and right_rest_up is not None and right_rest_up > right_rest_home + 0.005,
        details=f"home={right_rest_home}, up={right_rest_up}",
    )

    switch_home = _aabb_max_z(ctx.part_element_world_aabb(power_switch, elem="lever"))
    with ctx.pose({switch_joint: switch_joint.motion_limits.upper}):
        switch_up = _aabb_max_z(ctx.part_element_world_aabb(power_switch, elem="lever"))
    ctx.check(
        "power switch flips upward",
        switch_home is not None and switch_up is not None and switch_up > switch_home + 0.0035,
        details=f"home={switch_home}, up={switch_up}",
    )

    ctx.expect_origin_gap(
        power_switch,
        housing,
        axis="y",
        min_gap=0.035,
        name="power switch is mounted on the front of the housing",
    )

    return ctx.report()


object_model = build_object_model()
