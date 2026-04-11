from __future__ import annotations

import cadquery as cq
from math import pi

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


BASE_LENGTH = 0.34
BASE_DEPTH = 0.18
BASE_THICKNESS = 0.012
FOOT_LENGTH = 0.048
FOOT_DEPTH = 0.038
FOOT_HEIGHT = 0.008
HOUSING_RADIUS = 0.08
HOUSING_LENGTH = 0.18
HOUSING_CENTER_Z = 0.185
WHEEL_RADIUS = 0.075
WHEEL_THICKNESS = 0.025
WHEEL_CENTER_X = 0.194
WHEEL_JOINT_X = 0.1655
TOOL_REST_PIVOT_Y = 0.053
TOOL_REST_PIVOT_Z = 0.086
SHIELD_HINGE_Y = 0.117
SHIELD_HINGE_Z = 0.289
SWITCH_PIVOT_Y = 0.103
SWITCH_PIVOT_Z = 0.145


def x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def x_ring(
    inner_radius: float,
    outer_radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate(center)
    )


def box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def merge_shapes(*shapes: cq.Workplane) -> cq.Workplane:
    merged = shapes[0]
    for shape in shapes[1:]:
        merged = merged.union(shape)
    return merged


def make_guard_shape(center_x: float) -> cq.Workplane:
    roof = box_shape((0.058, 0.108, 0.014), (center_x, -0.024, 0.286))
    rear = box_shape((0.058, 0.018, 0.17), (center_x, -0.104, HOUSING_CENTER_Z))
    return merge_shapes(roof, rear)


def make_body_shape() -> cq.Workplane:
    base_plate = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_DEPTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    ).translate((0.0, 0.0, FOOT_HEIGHT))

    pedestal = box_shape((0.11, 0.082, 0.102), (0.0, 0.0, 0.071))
    saddle = box_shape((0.15, 0.092, 0.028), (0.0, 0.0, 0.136))
    housing = x_cylinder(HOUSING_RADIUS, HOUSING_LENGTH, (0.0, 0.0, HOUSING_CENTER_Z))
    left_end_bell = x_cylinder(0.086, 0.03, (-0.105, 0.0, HOUSING_CENTER_Z))
    right_end_bell = x_cylinder(0.086, 0.03, (0.105, 0.0, HOUSING_CENTER_Z))

    left_guard = make_guard_shape(-WHEEL_CENTER_X)
    right_guard = make_guard_shape(WHEEL_CENTER_X)
    left_guard_arm = box_shape((0.05, 0.022, 0.08), (-0.142, -0.100, 0.185))
    right_guard_arm = box_shape((0.05, 0.022, 0.08), (0.142, -0.100, 0.185))
    left_spindle_boss = x_cylinder(0.018, 0.022, (-0.1545, 0.0, HOUSING_CENTER_Z))
    right_spindle_boss = x_cylinder(0.018, 0.022, (0.1545, 0.0, HOUSING_CENTER_Z))

    left_tool_rest_stem = box_shape((0.018, 0.018, 0.094), (-WHEEL_CENTER_X, 0.027, 0.063))
    right_tool_rest_stem = box_shape((0.018, 0.018, 0.094), (WHEEL_CENTER_X, 0.027, 0.063))
    left_tool_rest_bracket = box_shape((0.026, 0.018, 0.03), (-WHEEL_CENTER_X, 0.035, 0.094))
    right_tool_rest_bracket = box_shape((0.026, 0.018, 0.03), (WHEEL_CENTER_X, 0.035, 0.094))

    left_shield_post = box_shape((0.012, 0.02, 0.046), (-WHEEL_CENTER_X, 0.03, 0.261))
    right_shield_post = box_shape((0.012, 0.02, 0.046), (WHEEL_CENTER_X, 0.03, 0.261))
    left_shield_bar = box_shape((0.07, 0.012, 0.012), (-WHEEL_CENTER_X, 0.049, SHIELD_HINGE_Z - 0.002))
    right_shield_bar = box_shape((0.07, 0.012, 0.012), (WHEEL_CENTER_X, 0.049, SHIELD_HINGE_Z - 0.002))

    control_pod = box_shape((0.056, 0.038, 0.044), (0.0, 0.078, SWITCH_PIVOT_Z))
    switch_recess = box_shape((0.034, 0.026, 0.024), (0.0, 0.084, SWITCH_PIVOT_Z))

    body = merge_shapes(
        base_plate,
        pedestal,
        saddle,
        housing,
        left_end_bell,
        right_end_bell,
        left_guard,
        right_guard,
        left_guard_arm,
        right_guard_arm,
        left_spindle_boss,
        right_spindle_boss,
        left_tool_rest_stem,
        right_tool_rest_stem,
        left_tool_rest_bracket,
        right_tool_rest_bracket,
        left_shield_post,
        right_shield_post,
        left_shield_bar,
        right_shield_bar,
        control_pod,
    )

    return body.cut(switch_recess)


def make_wheel_shape(direction: float) -> cq.Workplane:
    hub = x_cylinder(0.01, 0.016, (direction * 0.008, 0.0, 0.0))
    inner_flange = x_cylinder(0.022, 0.004, (direction * 0.014, 0.0, 0.0))
    stone = x_cylinder(WHEEL_RADIUS, WHEEL_THICKNESS, (direction * 0.0285, 0.0, 0.0))
    outer_flange = x_cylinder(0.018, 0.006, (direction * 0.044, 0.0, 0.0))
    return merge_shapes(hub, inner_flange, stone, outer_flange)


def make_tool_rest_shape() -> cq.Workplane:
    pivot = x_cylinder(0.007, 0.026, (0.0, 0.0, 0.0))
    mount = box_shape((0.026, 0.01, 0.018), (0.0, -0.004, 0.0))
    arm = box_shape((0.026, 0.012, 0.016), (0.0, 0.009, 0.005))
    web = box_shape((0.022, 0.01, 0.014), (0.0, 0.019, 0.009))
    table = box_shape((0.05, 0.028, 0.004), (0.0, 0.031, 0.011))
    return merge_shapes(pivot, mount, arm, web, table)


def make_eye_shield_bracket_shape() -> cq.Workplane:
    hinge = x_cylinder(0.004, 0.068, (0.0, 0.0, 0.0))
    clamp = box_shape((0.068, 0.01, 0.012), (0.0, 0.006, -0.006))
    tab_0 = box_shape((0.012, 0.01, 0.02), (-0.025, 0.006, -0.016))
    tab_1 = box_shape((0.012, 0.01, 0.02), (0.025, 0.006, -0.016))
    return merge_shapes(hinge, clamp, tab_0, tab_1)


def make_switch_shape() -> cq.Workplane:
    pivot = x_cylinder(0.0035, 0.026, (0.0, 0.0, 0.0))
    mount = box_shape((0.026, 0.008, 0.012), (0.0, -0.002, 0.0))
    cap = box_shape((0.028, 0.01, 0.018), (0.0, 0.008, 0.003))
    top = box_shape((0.028, 0.006, 0.01), (0.0, 0.013, 0.007))
    return merge_shapes(pivot, mount, cap, top)


def make_side_frame(direction: float) -> cq.Workplane:
    guard_arm = box_shape((0.07, 0.018, 0.08), (direction * 0.128, -0.086, 0.185))
    guard_rear = box_shape((0.058, 0.018, 0.17), (direction * WHEEL_CENTER_X, -0.104, HOUSING_CENTER_Z))
    guard_roof = box_shape((0.058, 0.10, 0.014), (direction * WHEEL_CENTER_X, -0.049, 0.286))
    spindle_boss = x_cylinder(0.018, 0.022, (direction * 0.1545, 0.0, HOUSING_CENTER_Z))
    boss_web = box_shape((0.02, 0.086, 0.028), (direction * 0.1545, -0.043, HOUSING_CENTER_Z))
    rest_stem = box_shape((0.018, 0.018, 0.094), (direction * WHEEL_CENTER_X, 0.027, 0.063))
    rest_bracket = box_shape((0.026, 0.018, 0.03), (direction * WHEEL_CENTER_X, 0.035, 0.094))
    shield_post = box_shape((0.014, 0.012, 0.27), (direction * WHEEL_CENTER_X, 0.105, 0.149))
    shield_bar = box_shape((0.07, 0.012, 0.012), (direction * WHEEL_CENTER_X, 0.107, SHIELD_HINGE_Z - 0.002))
    lower_runner = box_shape((0.018, 0.08, 0.022), (direction * WHEEL_CENTER_X, 0.068, 0.064))
    top_bridge = box_shape((0.018, 0.21, 0.02), (direction * WHEEL_CENTER_X, 0.011, 0.281))
    return merge_shapes(
        guard_arm,
        guard_rear,
        guard_roof,
        spindle_boss,
        boss_web,
        rest_stem,
        rest_bracket,
        shield_post,
        shield_bar,
        lower_runner,
        top_bridge,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_grinder")

    model.material("body_paint", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("wheel_stone", rgba=(0.60, 0.63, 0.66, 1.0))
    model.material("steel", rgba=(0.72, 0.73, 0.75, 1.0))
    model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("shield_clear", rgba=(0.78, 0.92, 0.98, 0.35))
    model.material("switch_red", rgba=(0.78, 0.10, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        Box((BASE_LENGTH, BASE_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT + BASE_THICKNESS / 2.0)),
        material="body_paint",
        name="base",
    )
    body.visual(
        Box((0.11, 0.082, 0.114)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material="body_paint",
        name="pedestal",
    )
    body.visual(
        Box((0.15, 0.092, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.134)),
        material="body_paint",
        name="saddle",
    )
    body.visual(
        Cylinder(radius=HOUSING_RADIUS, length=HOUSING_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, HOUSING_CENTER_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material="body_paint",
        name="housing",
    )
    for idx, direction in enumerate((-1.0, 1.0)):
        body.visual(
            Cylinder(radius=0.086, length=0.03),
            origin=Origin(
                xyz=(direction * 0.105, 0.0, HOUSING_CENTER_Z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="body_paint",
            name=f"bell_{idx}",
        )
    body.visual(
        Box((0.056, 0.038, 0.044)),
        origin=Origin(xyz=(0.0, 0.078, SWITCH_PIVOT_Z)),
        material="body_paint",
        name="switch_pod",
    )

    for idx, direction in enumerate((-1.0, 1.0)):
        side = model.part(f"side_{idx}")
        side.visual(
            mesh_from_cadquery(make_side_frame(direction), f"side_{idx}"),
            material="body_paint",
            name="frame",
        )
        model.articulation(
            f"body_to_side_{idx}",
            ArticulationType.FIXED,
            parent=body,
            child=side,
        )

    foot_positions = (
        (-0.11, -0.055, FOOT_HEIGHT / 2.0),
        (-0.11, 0.055, FOOT_HEIGHT / 2.0),
        (0.11, -0.055, FOOT_HEIGHT / 2.0),
        (0.11, 0.055, FOOT_HEIGHT / 2.0),
    )
    for idx, xyz in enumerate(foot_positions):
        body.visual(
            Box((FOOT_LENGTH, FOOT_DEPTH, FOOT_HEIGHT)),
            origin=Origin(xyz=xyz),
            material="rubber",
            name=f"foot_{idx}",
        )

    for idx, direction in enumerate((-1.0, 1.0)):
        wheel = model.part(f"wheel_{idx}")
        wheel.visual(
            mesh_from_cadquery(make_wheel_shape(direction), f"wheel_{idx}"),
            material="wheel_stone",
            name="wheel",
        )
        model.articulation(
            f"body_to_wheel_{idx}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(direction * WHEEL_JOINT_X, 0.0, HOUSING_CENTER_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=40.0),
        )

    for idx, direction in enumerate((-1.0, 1.0)):
        tool_rest = model.part(f"tool_rest_{idx}")
        tool_rest.visual(
            mesh_from_cadquery(make_tool_rest_shape(), f"tool_rest_{idx}"),
            material="steel",
            name="rest",
        )
        model.articulation(
            f"body_to_tool_rest_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=tool_rest,
            origin=Origin(xyz=(direction * WHEEL_CENTER_X, TOOL_REST_PIVOT_Y, TOOL_REST_PIVOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=1.5,
                lower=-0.35,
                upper=0.35,
            ),
        )

    for idx, direction in enumerate((-1.0, 1.0)):
        eye_shield = model.part(f"eye_shield_{idx}")
        eye_shield.visual(
            mesh_from_cadquery(make_eye_shield_bracket_shape(), f"eye_shield_{idx}_bracket"),
            material="steel",
            name="bracket",
        )
        eye_shield.visual(
            Box((0.082, 0.0025, 0.068)),
            origin=Origin(xyz=(0.0, 0.011, -0.04)),
            material="shield_clear",
            name="shield",
        )
        model.articulation(
            f"body_to_eye_shield_{idx}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=eye_shield,
            origin=Origin(xyz=(direction * WHEEL_CENTER_X, SHIELD_HINGE_Y, SHIELD_HINGE_Z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.5,
                lower=0.0,
                upper=1.1,
            ),
        )

    switch = model.part("switch")
    switch.visual(
        mesh_from_cadquery(make_switch_shape(), "switch"),
        material="switch_red",
        name="cap",
    )
    model.articulation(
        "body_to_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=switch,
        origin=Origin(xyz=(0.0, SWITCH_PIVOT_Y, SWITCH_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-0.32,
            upper=0.32,
        ),
    )

    return model


def _aabb_max_z(aabb):
    return None if aabb is None else aabb[1][2]


def _aabb_min_z(aabb):
    return None if aabb is None else aabb[0][2]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    side_0 = object_model.get_part("side_0")
    side_1 = object_model.get_part("side_1")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")
    tool_rest_0 = object_model.get_part("tool_rest_0")
    tool_rest_1 = object_model.get_part("tool_rest_1")
    eye_shield_0 = object_model.get_part("eye_shield_0")
    eye_shield_1 = object_model.get_part("eye_shield_1")
    switch = object_model.get_part("switch")

    ctx.allow_overlap(
        body,
        side_0,
        reason="The left guard casting is simplified as a fixed side frame with a small embedded mount into the motor end bell.",
    )
    ctx.allow_overlap(
        body,
        side_1,
        reason="The right guard casting is simplified as a fixed side frame with a small embedded mount into the motor end bell.",
    )
    ctx.allow_overlap(
        side_0,
        tool_rest_0,
        reason="The left tool rest pivot block is intentionally represented as seated within the fixed bracket proxy.",
    )
    ctx.allow_overlap(
        side_1,
        tool_rest_1,
        reason="The right tool rest pivot block is intentionally represented as seated within the fixed bracket proxy.",
    )

    ctx.expect_origin_gap(
        wheel_1,
        wheel_0,
        axis="x",
        min_gap=0.32,
        max_gap=0.34,
        name="wheels span both sides of the housing",
    )
    ctx.expect_origin_distance(
        wheel_0,
        wheel_1,
        axes="yz",
        max_dist=0.001,
        name="wheels share one horizontal shaft line",
    )

    for idx, wheel, rest in (
        (0, wheel_0, tool_rest_0),
        (1, wheel_1, tool_rest_1),
    ):
        ctx.expect_gap(
            wheel,
            rest,
            axis="z",
            min_gap=0.008,
            max_gap=0.018,
            elem_a="wheel",
            elem_b="rest",
            name=f"tool rest {idx} sits just below its wheel",
        )
        ctx.expect_overlap(
            wheel,
            rest,
            axes="x",
            min_overlap=0.03,
            elem_a="wheel",
            elem_b="rest",
            name=f"tool rest {idx} stays under wheel width",
        )

    for idx, wheel, shield in (
        (0, wheel_0, eye_shield_0),
        (1, wheel_1, eye_shield_1),
    ):
        ctx.expect_origin_gap(
            shield,
            wheel,
            axis="z",
            min_gap=0.08,
            max_gap=0.12,
            name=f"eye shield {idx} hangs above its wheel center",
        )
        ctx.expect_origin_gap(
            shield,
            body,
            axis="y",
            min_gap=0.11,
            max_gap=0.12,
            name=f"eye shield {idx} sits forward of the guard",
        )

    ctx.expect_origin_distance(
        switch,
        body,
        axes="x",
        max_dist=0.001,
        name="power switch stays centered on the grinder",
    )
    ctx.expect_origin_gap(
        switch,
        body,
        axis="y",
        min_gap=0.095,
        max_gap=0.105,
        name="power switch is mounted on the front face",
    )

    for joint_name in ("body_to_wheel_0", "body_to_wheel_1"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )

    for idx in (0, 1):
        rest_joint = object_model.get_articulation(f"body_to_tool_rest_{idx}")
        rest_limits = rest_joint.motion_limits
        if rest_limits is not None and rest_limits.lower is not None and rest_limits.upper is not None:
            with ctx.pose({rest_joint: rest_limits.lower}):
                low_aabb = ctx.part_element_world_aabb(f"tool_rest_{idx}", elem="rest")
            with ctx.pose({rest_joint: rest_limits.upper}):
                high_aabb = ctx.part_element_world_aabb(f"tool_rest_{idx}", elem="rest")
            ctx.check(
                f"tool rest {idx} tilts upward through its adjustment arc",
                low_aabb is not None
                and high_aabb is not None
                and _aabb_max_z(high_aabb) is not None
                and _aabb_max_z(low_aabb) is not None
                and _aabb_max_z(high_aabb) > _aabb_max_z(low_aabb) + 0.01,
                details=f"low={low_aabb}, high={high_aabb}",
            )

    for idx in (0, 1):
        shield_joint = object_model.get_articulation(f"body_to_eye_shield_{idx}")
        shield_limits = shield_joint.motion_limits
        if shield_limits is not None and shield_limits.lower is not None and shield_limits.upper is not None:
            with ctx.pose({shield_joint: shield_limits.lower}):
                low_aabb = ctx.part_element_world_aabb(f"eye_shield_{idx}", elem="shield")
            with ctx.pose({shield_joint: shield_limits.upper}):
                high_aabb = ctx.part_element_world_aabb(f"eye_shield_{idx}", elem="shield")
            ctx.check(
                f"eye shield {idx} lifts above the wheel when opened",
                low_aabb is not None
                and high_aabb is not None
                and _aabb_min_z(low_aabb) is not None
                and _aabb_min_z(high_aabb) is not None
                and _aabb_min_z(high_aabb) > _aabb_min_z(low_aabb) + 0.02,
                details=f"low={low_aabb}, high={high_aabb}",
            )

    switch_joint = object_model.get_articulation("body_to_switch")
    switch_limits = switch_joint.motion_limits
    if switch_limits is not None and switch_limits.lower is not None and switch_limits.upper is not None:
        with ctx.pose({switch_joint: switch_limits.lower}):
            low_aabb = ctx.part_element_world_aabb("switch", elem="cap")
        with ctx.pose({switch_joint: switch_limits.upper}):
            high_aabb = ctx.part_element_world_aabb("switch", elem="cap")
        ctx.check(
            "rocker switch changes attitude across its pivot",
            low_aabb is not None
            and high_aabb is not None
            and _aabb_max_z(high_aabb) is not None
            and _aabb_max_z(low_aabb) is not None
            and _aabb_max_z(high_aabb) > _aabb_max_z(low_aabb) + 0.003,
            details=f"low={low_aabb}, high={high_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
