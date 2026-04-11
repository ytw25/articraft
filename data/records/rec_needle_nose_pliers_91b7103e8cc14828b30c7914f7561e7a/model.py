from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOOL_LENGTH = 0.200
JAW_LENGTH = 0.082
HANDLE_LENGTH = TOOL_LENGTH - JAW_LENGTH
HALF_THICKNESS = 0.0055
HALF_Z_OFFSET = 0.00345
PIVOT_CAP_THICKNESS = 0.0022
PIVOT_CAP_RADIUS = 0.0105
PIVOT_POST_RADIUS = 0.0048
CLOSED_HALF_ANGLE = math.radians(1.8)
OPEN_TRAVEL = math.radians(38.0)


def _extruded_profile(
    points: list[tuple[float, float]],
    *,
    thickness: float,
    angle_rad: float = 0.0,
    z_offset: float = 0.0,
) -> cq.Workplane:
    shape = cq.Workplane("XY").polyline(points).close().extrude(thickness / 2.0, both=True)
    if angle_rad:
        shape = shape.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(angle_rad))
    if z_offset:
        shape = shape.translate((0.0, 0.0, z_offset))
    return shape


def _pivot_cap(*, z_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(PIVOT_CAP_RADIUS)
        .extrude(PIVOT_CAP_THICKNESS / 2.0, both=True)
        .translate((0.0, 0.0, z_center))
    )


def _pivot_post(*, length: float, z_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(PIVOT_POST_RADIUS)
        .extrude(length / 2.0, both=True)
        .translate((0.0, 0.0, z_center))
    )


def _jaw_shape(*, angle_rad: float, z_offset: float) -> cq.Workplane:
    jaw_points = [
        (-0.015, -0.0042),
        (-0.004, -0.0066),
        (0.014, -0.0084),
        (0.040, -0.0063),
        (0.064, -0.0038),
        (JAW_LENGTH, -0.0016),
        (JAW_LENGTH, 0.0016),
        (0.064, 0.0038),
        (0.040, 0.0063),
        (0.014, 0.0084),
        (-0.004, 0.0066),
        (-0.015, 0.0042),
    ]
    return _extruded_profile(jaw_points, thickness=HALF_THICKNESS, angle_rad=angle_rad, z_offset=z_offset)


def _handle_tang_shape(*, angle_rad: float, z_offset: float) -> cq.Workplane:
    tang_points = [
        (-HANDLE_LENGTH, -0.0038),
        (-0.114, -0.0046),
        (-0.060, -0.0048),
        (-0.020, -0.0045),
        (0.010, -0.0058),
        (0.018, -0.0052),
        (0.018, 0.0052),
        (0.010, 0.0058),
        (-0.020, 0.0045),
        (-0.060, 0.0048),
        (-0.114, 0.0046),
        (-HANDLE_LENGTH, 0.0038),
    ]
    return _extruded_profile(tang_points, thickness=HALF_THICKNESS, angle_rad=angle_rad, z_offset=z_offset)


def _grip_shape(*, angle_rad: float, z_offset: float) -> cq.Workplane:
    grip_points = [
        (-0.122, -0.0068),
        (-0.117, -0.0075),
        (-0.060, -0.0078),
        (-0.030, -0.0073),
        (-0.024, -0.0062),
        (-0.024, 0.0062),
        (-0.030, 0.0073),
        (-0.060, 0.0078),
        (-0.117, 0.0075),
        (-0.122, 0.0068),
    ]
    return _extruded_profile(grip_points, thickness=0.0068, angle_rad=angle_rad, z_offset=z_offset)


def _add_half(
    model: ArticulatedObject,
    *,
    name: str,
    angle_rad: float,
    z_offset: float,
    cap_sign: float,
    steel_material: str,
    grip_material: str,
    with_pivot_post: bool = False,
) -> None:
    half = model.part(name)
    half.visual(
        mesh_from_cadquery(_jaw_shape(angle_rad=angle_rad, z_offset=z_offset), f"{name}_jaw"),
        material=steel_material,
        name="jaw",
    )
    half.visual(
        mesh_from_cadquery(
            _handle_tang_shape(angle_rad=angle_rad, z_offset=z_offset),
            f"{name}_handle_tang",
        ),
        material=steel_material,
        name="handle_tang",
    )
    half.visual(
        mesh_from_cadquery(_grip_shape(angle_rad=angle_rad, z_offset=z_offset), f"{name}_grip"),
        material=grip_material,
        name="grip",
    )
    half.visual(
        mesh_from_cadquery(
            _pivot_cap(
                z_center=z_offset + cap_sign * (HALF_THICKNESS / 2.0 + PIVOT_CAP_THICKNESS / 2.0)
            ),
            f"{name}_pivot_cap",
        ),
        material=steel_material,
        name="pivot_cap",
    )
    if with_pivot_post:
        half.visual(
            mesh_from_cadquery(
                _pivot_post(
                    length=2.0 * HALF_Z_OFFSET - HALF_THICKNESS,
                    z_center=0.0,
                ),
                f"{name}_pivot_post",
            ),
            material=steel_material,
            name="pivot_post",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="needle_nose_pliers")

    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    grip = model.material("grip", rgba=(0.12, 0.16, 0.32, 1.0))

    _add_half(
        model,
        name="half_0",
        angle_rad=CLOSED_HALF_ANGLE,
        z_offset=-HALF_Z_OFFSET,
        cap_sign=-1.0,
        steel_material=steel,
        grip_material=grip,
        with_pivot_post=True,
    )
    _add_half(
        model,
        name="half_1",
        angle_rad=-CLOSED_HALF_ANGLE,
        z_offset=HALF_Z_OFFSET,
        cap_sign=1.0,
        steel_material=steel,
        grip_material=grip,
    )

    model.articulation(
        "half_0_to_half_1",
        ArticulationType.REVOLUTE,
        parent="half_0",
        child="half_1",
        origin=Origin(),
        # The closed child half is authored at a slight negative plan-view angle.
        # Using -Z makes positive q open the jaws wider instead of closing them.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OPEN_TRAVEL,
            effort=20.0,
            velocity=3.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    half_0 = object_model.get_part("half_0")
    half_1 = object_model.get_part("half_1")
    pivot = object_model.get_articulation("half_0_to_half_1")
    limits = pivot.motion_limits
    
    def elem_center_y(part_obj, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return (lower[1] + upper[1]) / 2.0

    ctx.expect_overlap(
        half_0,
        half_1,
        axes="x",
        elem_a="jaw",
        elem_b="jaw",
        min_overlap=0.070,
        name="closed jaws align along their tapered length",
    )
    ctx.expect_gap(
        half_1,
        half_0,
        axis="z",
        positive_elem="jaw",
        negative_elem="jaw",
        min_gap=0.001,
        max_gap=0.002,
        name="jaw halves stay stacked without touching",
    )
    ctx.expect_gap(
        half_1,
        half_0,
        axis="z",
        positive_elem="grip",
        negative_elem="grip",
        min_gap=0.0,
        max_gap=0.0002,
        name="grips sit in adjacent layers around the pivot plane",
    )

    closed_jaw_y_delta = None
    closed_grip_y_delta = None
    jaw0_y = elem_center_y(half_0, "jaw")
    jaw1_y = elem_center_y(half_1, "jaw")
    grip0_y = elem_center_y(half_0, "grip")
    grip1_y = elem_center_y(half_1, "grip")
    if jaw0_y is not None and jaw1_y is not None:
        closed_jaw_y_delta = abs(jaw0_y - jaw1_y)
    if grip0_y is not None and grip1_y is not None:
        closed_grip_y_delta = abs(grip0_y - grip1_y)

    ctx.check(
        "closed jaws remain nearly collinear",
        closed_jaw_y_delta is not None and closed_jaw_y_delta <= 0.003,
        details=f"closed_jaw_center_delta_y={closed_jaw_y_delta}",
    )
    ctx.check(
        "closed handles show a visible hand gap",
        closed_grip_y_delta is not None and 0.003 <= closed_grip_y_delta <= 0.010,
        details=f"closed_grip_center_delta_y={closed_grip_y_delta}",
    )

    if limits is not None and limits.upper is not None:
        with ctx.pose({pivot: limits.upper}):
            open_jaw_y_delta = None
            open_grip_y_delta = None
            jaw0_y = elem_center_y(half_0, "jaw")
            jaw1_y = elem_center_y(half_1, "jaw")
            grip0_y = elem_center_y(half_0, "grip")
            grip1_y = elem_center_y(half_1, "grip")
            if jaw0_y is not None and jaw1_y is not None:
                open_jaw_y_delta = abs(jaw0_y - jaw1_y)
            if grip0_y is not None and grip1_y is not None:
                open_grip_y_delta = abs(grip0_y - grip1_y)

            ctx.check(
                "open jaws spread for gripping",
                open_jaw_y_delta is not None and open_jaw_y_delta >= 0.018,
                details=f"open_jaw_center_delta_y={open_jaw_y_delta}",
            )
            ctx.check(
                "open handles swing apart",
                open_grip_y_delta is not None and open_grip_y_delta >= 0.045,
                details=f"open_grip_center_delta_y={open_grip_y_delta}",
            )

    return ctx.report()


object_model = build_object_model()
