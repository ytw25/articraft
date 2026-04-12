from __future__ import annotations

from math import atan2, sqrt

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


METAL_T = 0.0018
METAL_Z_UPPER = 0.00135
METAL_Z_LOWER = -0.00135
GRIP_T = 0.0048
GRIP_LEN = 0.036
GRIP_W = 0.010
TAIL_PIVOT_XY = (-0.060, 0.0058)
CATCH_PIN_XY = (-0.057, -0.0048)


def _mirror_y(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return [(x, -y) for x, y in points]


def _extruded_profile(
    points: list[tuple[float, float]],
    thickness: float,
    z_center: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .polyline(points)
        .close()
        .extrude(thickness)
        .translate((0.0, 0.0, z_center - thickness / 2.0))
    )


def _capsule_xy(length: float, width: float, thickness: float) -> cq.Workplane:
    straight = max(length - width, 0.0)
    half_span = straight / 2.0
    body = cq.Workplane("XY").rect(straight, width).extrude(thickness)
    ends = (
        cq.Workplane("XY")
        .pushPoints([(-half_span, 0.0), (half_span, 0.0)])
        .circle(width / 2.0)
        .extrude(thickness)
    )
    return body.union(ends)


def _jaw_shape(points: list[tuple[float, float]], z_center: float) -> cq.Workplane:
    return _extruded_profile(points, METAL_T, z_center)


def _frame_shape(
    sign: float,
    z_center: float,
    *,
    has_pivot_hole: bool,
    add_lower_details: bool = False,
) -> cq.Workplane:
    pivot_cheek = (
        cq.Workplane("XY")
        .circle(0.0063 if not has_pivot_hole else 0.0060)
        .extrude(METAL_T)
        .translate((0.0, 0.0, z_center - METAL_T / 2.0))
    )
    handle = (
        _capsule_xy(0.066, 0.0088, METAL_T)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 6.0 * sign)
        .translate((-0.036, -0.0046 * sign, z_center - METAL_T / 2.0))
    )
    shoulder = (
        _capsule_xy(0.011, 0.0060, METAL_T)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 88.0)
        .translate((-0.0026, 0.0035 * sign, z_center - METAL_T / 2.0))
    )
    throat = (
        _capsule_xy(0.018, 0.0054, METAL_T)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 16.0 * sign)
        .translate((-0.0042, 0.0004 * sign, z_center - METAL_T / 2.0))
    )

    shape = pivot_cheek.union(handle).union(shoulder).union(throat)
    if has_pivot_hole:
        hole = (
            cq.Workplane("XY")
            .circle(0.0019)
            .extrude(0.008)
            .translate((0.0, 0.0, -0.004))
        )
        shape = shape.cut(hole)
    if add_lower_details:
        main_pin = (
            cq.Workplane("XY")
            .circle(0.00125)
            .extrude(0.0054)
            .translate((0.0, 0.0, -0.0031))
        )
        rivet_head = (
            cq.Workplane("XY")
            .circle(0.0038)
            .extrude(0.0012)
            .translate((0.0, 0.0, -0.0036))
        )
        tail_post = (
            cq.Workplane("XY")
            .circle(0.0007)
            .extrude(0.0044)
            .translate((TAIL_PIVOT_XY[0], TAIL_PIVOT_XY[1], -0.0047))
        )
        shape = shape.union(main_pin).union(rivet_head).union(tail_post)
    return shape


def _grip_shape(center_xy: tuple[float, float], angle_deg: float, z_center: float) -> cq.Workplane:
    return (
        _capsule_xy(GRIP_LEN, GRIP_W, GRIP_T)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        .translate((center_xy[0], center_xy[1], z_center - GRIP_T / 2.0))
    )


def _latch_shape() -> cq.Workplane:
    thickness = 0.0015
    z_center = -0.0038
    hook_center = (CATCH_PIN_XY[0] - TAIL_PIVOT_XY[0], CATCH_PIN_XY[1] - TAIL_PIVOT_XY[1])

    pivot_eye = (
        cq.Workplane("XY")
        .circle(0.0024)
        .extrude(thickness)
        .translate((0.0, 0.0, z_center - thickness / 2.0))
    )
    pivot_hole = (
        cq.Workplane("XY")
        .circle(0.0010)
        .extrude(0.006)
        .translate((0.0, 0.0, z_center - 0.003))
    )

    arm_len = sqrt(hook_center[0] * hook_center[0] + hook_center[1] * hook_center[1]) + 0.0045
    arm_angle = 57.29577951308232 * atan2(hook_center[1], hook_center[0])
    arm_center = (hook_center[0] * 0.5, hook_center[1] * 0.5)
    arm = (
        _capsule_xy(arm_len, 0.0022, thickness)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), arm_angle)
        .translate((arm_center[0], arm_center[1], z_center - thickness / 2.0))
    )

    hook_outer = (
        cq.Workplane("XY")
        .circle(0.0025)
        .extrude(thickness)
        .translate((hook_center[0], hook_center[1], z_center - thickness / 2.0))
    )
    hook_inner = (
        cq.Workplane("XY")
        .circle(0.00135)
        .extrude(0.006)
        .translate((hook_center[0], hook_center[1], z_center - 0.003))
    )
    hook_slot = (
        cq.Workplane("XY")
        .box(0.0032, 0.0017, 0.006)
        .translate((hook_center[0] - 0.0011, hook_center[1] + 0.0015, z_center))
    )

    return pivot_eye.cut(pivot_hole).union(arm).union(hook_outer.cut(hook_inner).cut(hook_slot))


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def _distance(a, b):
    if a is None or b is None:
        return None
    return sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jewelry_needle_nose_pliers")

    steel = model.material("steel", rgba=(0.63, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.40, 0.42, 0.45, 1.0))
    grip_red = model.material("grip_red", rgba=(0.60, 0.17, 0.18, 1.0))

    upper_jaw_pts = [
        (-0.0035, 0.0038),
        (0.0030, 0.0047),
        (0.0160, 0.0032),
        (0.0320, 0.0018),
        (0.0440, 0.00090),
        (0.0510, 0.00048),
        (0.0510, 0.00012),
        (0.0410, 0.00023),
        (0.0260, 0.00058),
        (0.0090, 0.00115),
        (-0.0005, 0.00190),
        (-0.0035, 0.00280),
    ]

    lower_half = model.part("lower_half")
    lower_half.visual(
        mesh_from_cadquery(_jaw_shape(_mirror_y(upper_jaw_pts), METAL_Z_LOWER), "lower_jaw"),
        material=steel,
        name="jaw",
    )
    lower_half.visual(
        mesh_from_cadquery(_frame_shape(-1.0, METAL_Z_LOWER, has_pivot_hole=False, add_lower_details=True), "lower_frame"),
        material=dark_steel,
        name="frame",
    )
    lower_half.visual(
        mesh_from_cadquery(_grip_shape((-0.046, 0.0046), -6.0, METAL_Z_LOWER), "lower_grip"),
        material=grip_red,
        name="grip",
    )
    upper_half = model.part("upper_half")
    upper_half.visual(
        mesh_from_cadquery(_jaw_shape(upper_jaw_pts, METAL_Z_UPPER), "upper_jaw"),
        material=steel,
        name="jaw",
    )
    upper_half.visual(
        mesh_from_cadquery(_frame_shape(1.0, METAL_Z_UPPER, has_pivot_hole=True), "upper_frame"),
        material=dark_steel,
        name="frame",
    )
    upper_half.visual(
        mesh_from_cadquery(_grip_shape((-0.046, -0.0046), 6.0, METAL_Z_UPPER), "upper_grip"),
        material=grip_red,
        name="grip",
    )
    upper_half.visual(
        Cylinder(radius=0.0008, length=0.0070),
        origin=Origin(xyz=(CATCH_PIN_XY[0], CATCH_PIN_XY[1], -0.0011)),
        material=dark_steel,
        name="catch_pin",
    )

    tail_latch = model.part("tail_latch")
    tail_latch.visual(
        mesh_from_cadquery(_latch_shape(), "tail_latch"),
        material=dark_steel,
        name="hook",
    )

    model.articulation(
        "main_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_half,
        child=upper_half,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.72, effort=12.0, velocity=3.0),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_half,
        child=tail_latch,
        origin=Origin(xyz=(TAIL_PIVOT_XY[0], TAIL_PIVOT_XY[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.0, effort=2.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_half = object_model.get_part("lower_half")
    upper_half = object_model.get_part("upper_half")
    tail_latch = object_model.get_part("tail_latch")
    main_pivot = object_model.get_articulation("main_pivot")
    latch_pivot = object_model.get_articulation("latch_pivot")

    with ctx.pose({main_pivot: 0.0, latch_pivot: 0.0}):
        ctx.expect_gap(
            upper_half,
            lower_half,
            axis="y",
            positive_elem="jaw",
            negative_elem="jaw",
            max_gap=0.0014,
            max_penetration=0.0,
            name="slim jaws close with a fine gap",
        )
        ctx.expect_overlap(
            upper_half,
            lower_half,
            axes="x",
            elem_a="jaw",
            elem_b="jaw",
            min_overlap=0.030,
            name="jaw tips stay aligned along their length",
        )
        ctx.expect_overlap(
            tail_latch,
            upper_half,
            axes="xy",
            elem_a="hook",
            elem_b="catch_pin",
            min_overlap=0.0010,
            name="tail latch sits over the storage catch when closed",
        )

    main_limits = main_pivot.motion_limits
    if main_limits is not None and main_limits.upper is not None:
        with ctx.pose({main_pivot: 0.0}):
            jaw_closed = _center_from_aabb(ctx.part_element_world_aabb(upper_half, elem="jaw"))
        with ctx.pose({main_pivot: main_limits.upper}):
            jaw_open = _center_from_aabb(ctx.part_element_world_aabb(upper_half, elem="jaw"))
        ctx.check(
            "upper half rotates upward about the central pivot",
            jaw_closed is not None and jaw_open is not None and jaw_open[1] > jaw_closed[1] + 0.008,
            details=f"closed={jaw_closed}, open={jaw_open}",
        )

    latch_limits = latch_pivot.motion_limits
    if latch_limits is not None and latch_limits.upper is not None:
        with ctx.pose({main_pivot: 0.0, latch_pivot: 0.0}):
            hook_closed = _center_from_aabb(ctx.part_element_world_aabb(tail_latch, elem="hook"))
            catch_closed = _center_from_aabb(ctx.part_element_world_aabb(upper_half, elem="catch_pin"))
        with ctx.pose({main_pivot: 0.0, latch_pivot: latch_limits.upper}):
            hook_open = _center_from_aabb(ctx.part_element_world_aabb(tail_latch, elem="hook"))
            catch_open = _center_from_aabb(ctx.part_element_world_aabb(upper_half, elem="catch_pin"))
        closed_dist = _distance(hook_closed, catch_closed)
        open_dist = _distance(hook_open, catch_open)
        ctx.check(
            "tail latch swings away from the catch when released",
            closed_dist is not None and open_dist is not None and open_dist > closed_dist + 0.006,
            details=f"closed_dist={closed_dist}, open_dist={open_dist}",
        )

    return ctx.report()


object_model = build_object_model()
