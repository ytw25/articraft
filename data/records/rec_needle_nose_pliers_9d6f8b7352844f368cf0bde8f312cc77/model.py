from __future__ import annotations

from math import cos, hypot, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)

HALF_REST_ANGLE = 0.08
METAL_THICKNESS = 0.0022
METAL_CENTER_OFFSET = 0.0016
GRIP_THICKNESS = 0.0038
GRIP_CENTER_OFFSET = 0.0033
PIVOT_RADIUS = 0.0058
PIVOT_BOSS_THICKNESS = 0.0030
LATCH_THICKNESS = 0.0012
LATCH_SWING = 2.35


def _mirror_outline(upper_edge: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return upper_edge + [(x, -z) for x, z in reversed(upper_edge[:-1])]


def _extrude_xz_profile(
    upper_edge: list[tuple[float, float]],
    *,
    thickness: float,
    y_center: float,
) -> cq.Workplane:
    outline = _mirror_outline(upper_edge)
    y_start = y_center - thickness * 0.5
    return cq.Workplane("XZ").polyline(outline).close().extrude(thickness).translate((0.0, y_start, 0.0))


def _rotate_y(shape: cq.Workplane, angle: float) -> cq.Workplane:
    return shape.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle * 180.0 / pi)


def _rotate_point_y(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    return (x * cos(angle) + z * sin(angle), y, -x * sin(angle) + z * cos(angle))


def _half_upper_edge() -> list[tuple[float, float]]:
    return [
        (0.050, 0.00055),
        (0.044, 0.00085),
        (0.034, 0.00160),
        (0.024, 0.00255),
        (0.014, 0.00395),
        (0.005, 0.00545),
        (0.000, 0.00600),
        (-0.012, 0.00455),
        (-0.029, 0.00395),
        (-0.047, 0.00335),
        (-0.063, 0.00285),
        (-0.073, 0.00180),
        (-0.076, 0.00000),
    ]


def _grip_upper_edge() -> list[tuple[float, float]]:
    return [
        (-0.018, 0.00475),
        (-0.034, 0.00450),
        (-0.051, 0.00410),
        (-0.066, 0.00360),
        (-0.073, 0.00270),
        (-0.076, 0.00000),
    ]


def _latch_shape() -> cq.Workplane:
    plate_outline = [
        (0.0000, 0.0021),
        (0.0048, 0.0020),
        (0.0102, 0.0016),
        (0.0140, 0.0010),
        (0.0152, 0.0002),
        (0.0144, -0.0016),
        (0.0112, -0.0023),
        (0.0094, -0.0010),
        (0.0036, -0.0008),
        (0.0000, -0.0014),
    ]
    plate = (
        cq.Workplane("XZ")
        .polyline(plate_outline)
        .close()
        .extrude(LATCH_THICKNESS)
        .translate((0.0, -LATCH_THICKNESS * 0.5, 0.0))
    )
    pivot_eye = (
        cq.Workplane("XZ")
        .circle(0.00225)
        .extrude(LATCH_THICKNESS)
        .translate((0.0, -LATCH_THICKNESS * 0.5, 0.0))
    )
    hook_tooth = cq.Workplane("XY").box(0.0026, LATCH_THICKNESS, 0.0011).translate((0.0136, 0.0, -0.0017))
    return plate.union(pivot_eye).union(hook_tooth)


def _add_half_visuals(
    part,
    *,
    prefix: str,
    angle: float,
    sign: float,
    steel,
    boss_steel,
    grip_material,
    add_spring: bool = False,
) -> None:
    metal_shape = _rotate_y(
        _extrude_xz_profile(
            _half_upper_edge(),
            thickness=METAL_THICKNESS,
            y_center=sign * METAL_CENTER_OFFSET,
        ),
        angle,
    )
    part.visual(mesh_from_cadquery(metal_shape, f"{prefix}_metal"), material=steel, name="metal")

    metal_y_min = sign * METAL_CENTER_OFFSET - METAL_THICKNESS * 0.5
    metal_y_max = sign * METAL_CENTER_OFFSET + METAL_THICKNESS * 0.5
    boss_y_start = metal_y_min - (PIVOT_BOSS_THICKNESS - 0.0012) if sign < 0.0 else metal_y_max - 0.0012
    pivot_boss = cq.Workplane("XZ").circle(PIVOT_RADIUS).extrude(PIVOT_BOSS_THICKNESS).translate((0.0, boss_y_start, 0.0))
    part.visual(mesh_from_cadquery(pivot_boss, f"{prefix}_pivot_boss"), material=boss_steel, name="pivot_boss")

    grip_shape = _rotate_y(
        _extrude_xz_profile(
            _grip_upper_edge(),
            thickness=GRIP_THICKNESS,
            y_center=sign * GRIP_CENTER_OFFSET,
        ),
        angle,
    )
    part.visual(mesh_from_cadquery(grip_shape, f"{prefix}_grip"), material=grip_material, name="grip")

    tip_pad = _rotate_y(
        cq.Workplane("XZ")
        .rect(0.0042, 0.0010)
        .extrude(METAL_THICKNESS + 0.0002)
        .translate((0.0477, sign * METAL_CENTER_OFFSET - (METAL_THICKNESS + 0.0002) * 0.5, 0.0)),
        angle,
    )
    part.visual(mesh_from_cadquery(tip_pad, f"{prefix}_tip_pad"), material=boss_steel, name="tip_pad")

    tail_cap = _rotate_y(
        cq.Workplane("XZ")
        .rect(0.0032, 0.0025)
        .extrude(METAL_THICKNESS + 0.0004)
        .translate((-0.0720, sign * METAL_CENTER_OFFSET - (METAL_THICKNESS + 0.0004) * 0.5, 0.0)),
        angle,
    )
    part.visual(mesh_from_cadquery(tail_cap, f"{prefix}_tail_cap"), material=boss_steel, name="tail_cap")

    if add_spring:
        spring_points = [
            _rotate_point_y(point, angle)
            for point in [
                (-0.020, -0.0024, 0.0005),
                (-0.023, -0.0016, 0.0018),
                (-0.031, -0.0004, 0.0054),
                (-0.043, -0.0002, 0.0066),
                (-0.053, -0.0002, 0.0041),
            ]
        ]
        spring = tube_from_spline_points(
            spring_points,
            radius=0.00055,
            samples_per_segment=14,
            radial_segments=14,
            cap_ends=True,
        )
        part.visual(mesh_from_geometry(spring, f"{prefix}_return_spring"), material=boss_steel, name="return_spring")


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
    return ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5)


def _xz_distance(point_a: tuple[float, float, float] | None, point_b: tuple[float, float, float] | None) -> float | None:
    if point_a is None or point_b is None:
        return None
    return hypot(point_a[0] - point_b[0], point_a[2] - point_b[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="spring_return_needle_nose_pliers")

    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    boss_steel = model.material("boss_steel", rgba=(0.31, 0.33, 0.36, 1.0))
    grip_material = model.material("grip_material", rgba=(0.14, 0.43, 0.60, 1.0))

    lower_half = model.part("lower_half")
    upper_half = model.part("upper_half")
    latch = model.part("latch")

    _add_half_visuals(
        lower_half,
        prefix="lower_half",
        angle=-HALF_REST_ANGLE,
        sign=-1.0,
        steel=steel,
        boss_steel=boss_steel,
        grip_material=grip_material,
        add_spring=True,
    )
    _add_half_visuals(
        upper_half,
        prefix="upper_half",
        angle=HALF_REST_ANGLE,
        sign=1.0,
        steel=steel,
        boss_steel=boss_steel,
        grip_material=grip_material,
    )

    latch.visual(
        mesh_from_cadquery(_latch_shape(), "storage_latch"),
        material=boss_steel,
        origin=Origin(rpy=(0.0, pi, 0.0)),
        name="hook",
    )

    latch_rivet_center = _rotate_point_y((-0.0605, 0.0, 0.0), -HALF_REST_ANGLE)
    latch_rivet = (
        cq.Workplane("XZ")
        .circle(0.0017)
        .extrude(0.0026)
        .translate((latch_rivet_center[0], -0.0018, latch_rivet_center[2]))
    )
    lower_half.visual(mesh_from_cadquery(latch_rivet, "lower_half_latch_rivet"), material=boss_steel, name="latch_rivet")

    model.articulation(
        "jaw_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_half,
        child=upper_half,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=-0.30, upper=0.15),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=lower_half,
        child=latch,
        origin=Origin(xyz=latch_rivet_center),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0, lower=0.0, upper=LATCH_SWING),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_half = object_model.get_part("lower_half")
    upper_half = object_model.get_part("upper_half")
    latch = object_model.get_part("latch")
    jaw_pivot = object_model.get_articulation("jaw_pivot")
    latch_pivot = object_model.get_articulation("latch_pivot")

    rest_upper_tip = _aabb_center(ctx.part_element_world_aabb(upper_half, elem="tip_pad"))
    rest_lower_tip = _aabb_center(ctx.part_element_world_aabb(lower_half, elem="tip_pad"))
    rest_tip_gap = None if rest_upper_tip is None or rest_lower_tip is None else abs(rest_upper_tip[2] - rest_lower_tip[2])
    ctx.check(
        "rest pose keeps the needle tips spring-open",
        rest_tip_gap is not None and rest_tip_gap > 0.006,
        details=f"tip_gap_z={rest_tip_gap}",
    )

    jaw_limits = jaw_pivot.motion_limits
    if jaw_limits is not None and jaw_limits.upper is not None and jaw_limits.lower is not None:
        with ctx.pose({jaw_pivot: jaw_limits.upper}):
            closed_upper_tip = _aabb_center(ctx.part_element_world_aabb(upper_half, elem="tip_pad"))
            closed_lower_tip = _aabb_center(ctx.part_element_world_aabb(lower_half, elem="tip_pad"))
            closed_tip_gap = (
                None
                if closed_upper_tip is None or closed_lower_tip is None
                else abs(closed_upper_tip[2] - closed_lower_tip[2])
            )
            ctx.check(
                "upper jaw limit nearly closes the fine tips",
                closed_tip_gap is not None and closed_tip_gap < 0.0015,
                details=f"tip_gap_z={closed_tip_gap}",
            )

        with ctx.pose({jaw_pivot: jaw_limits.lower}):
            wide_upper_tip = _aabb_center(ctx.part_element_world_aabb(upper_half, elem="tip_pad"))
            wide_lower_tip = _aabb_center(ctx.part_element_world_aabb(lower_half, elem="tip_pad"))
            wide_tip_gap = (
                None
                if wide_upper_tip is None or wide_lower_tip is None
                else abs(wide_upper_tip[2] - wide_lower_tip[2])
            )
            ctx.check(
                "lower jaw limit opens the tips wider than the spring rest pose",
                rest_tip_gap is not None and wide_tip_gap is not None and wide_tip_gap > rest_tip_gap + 0.008,
                details=f"rest_tip_gap_z={rest_tip_gap}, wide_tip_gap_z={wide_tip_gap}",
            )

        with ctx.pose({jaw_pivot: jaw_limits.upper, latch_pivot: 0.0}):
            stored_hook = _aabb_center(ctx.part_element_world_aabb(latch, elem="hook"))
            lower_grip = _aabb_center(ctx.part_element_world_aabb(lower_half, elem="grip"))
            upper_grip = _aabb_center(ctx.part_element_world_aabb(upper_half, elem="grip"))

        latch_limits = latch_pivot.motion_limits
        engaged_latch_q = 0.0 if latch_limits is None or latch_limits.upper is None else latch_limits.upper
        with ctx.pose({jaw_pivot: jaw_limits.upper, latch_pivot: engaged_latch_q}):
            engaged_hook = _aabb_center(ctx.part_element_world_aabb(latch, elem="hook"))

        ctx.check(
            "latch swings forward into its locking position",
            stored_hook is not None
            and engaged_hook is not None
            and engaged_hook[0] > stored_hook[0] + 0.005
            and engaged_hook[2] < stored_hook[2] - 0.003,
            details=f"stored_hook={stored_hook}, engaged_hook={engaged_hook}",
        )
        ctx.check(
            "engaged latch sits between the two handle tails",
            lower_grip is not None
            and upper_grip is not None
            and engaged_hook is not None
            and lower_grip[1] < engaged_hook[1] < upper_grip[1],
            details=f"lower_grip={lower_grip}, engaged_hook={engaged_hook}, upper_grip={upper_grip}",
        )

    return ctx.report()


object_model = build_object_model()
