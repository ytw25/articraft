from __future__ import annotations

from math import radians

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


STEEL_THICKNESS = 0.0038
GRIP_THICKNESS = 0.0105
PIVOT_CAP_THICKNESS = 0.0040
REST_HALF_ANGLE = radians(3.6)
BEND_ANGLE_DEG = 42.0


def _symmetric_profile(upper_edge: list[tuple[float, float]]) -> list[tuple[float, float]]:
    return upper_edge + [(x, -y) for x, y in reversed(upper_edge)]


def _extruded_profile(profile: list[tuple[float, float]], thickness: float) -> cq.Workplane:
    return cq.Workplane("XY").polyline(profile).close().extrude(thickness, both=True)


def _steel_body_shape() -> cq.Workplane:
    body_profile = _symmetric_profile(
        [
            (-0.115, 0.0046),
            (-0.096, 0.0050),
            (-0.072, 0.0055),
            (-0.043, 0.0063),
            (-0.021, 0.0079),
            (-0.006, 0.0105),
            (0.006, 0.0124),
            (0.017, 0.0107),
            (0.033, 0.0074),
            (0.048, 0.0050),
            (0.056, 0.0042),
        ]
    )
    body = _extruded_profile(body_profile, STEEL_THICKNESS)
    pivot_boss = cq.Workplane("XY").circle(0.0132).extrude(STEEL_THICKNESS, both=True)
    return body.union(pivot_boss)


def _jaw_tip_shape() -> cq.Workplane:
    tip_profile = _symmetric_profile(
        [
            (0.055, 0.0028),
            (0.066, 0.0021),
            (0.077, 0.0016),
            (0.086, 0.0012),
            (0.089, 0.00095),
        ]
    )
    tip = _extruded_profile(tip_profile, STEEL_THICKNESS)
    return tip.rotate((0.055, 0.0, 0.0), (0.055, 1.0, 0.0), -BEND_ANGLE_DEG)


def _handle_grip_shape() -> cq.Workplane:
    main = (
        cq.Workplane("XY")
        .box(0.082, 0.0112, GRIP_THICKNESS)
        .edges("|X")
        .fillet(0.0022)
        .translate((-0.067, 0.0, 0.0))
    )
    butt = (
        cq.Workplane("XY")
        .box(0.020, 0.0128, 0.0116)
        .edges("|X")
        .fillet(0.0024)
        .translate((-0.111, 0.0, 0.0))
    )
    ferrule = (
        cq.Workplane("XY")
        .box(0.020, 0.0092, 0.0080)
        .edges("|X")
        .fillet(0.0016)
        .translate((-0.019, 0.0, 0.0))
    )
    return main.union(butt).union(ferrule)


def _handle_end_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.018, 0.0126, 0.0114)
        .edges("|X")
        .fillet(0.0024)
        .translate((-0.112, 0.0, 0.0))
    )


def _pivot_cap_shape() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.0066).extrude(PIVOT_CAP_THICKNESS, both=True)


def _add_half_visuals(
    part,
    *,
    mesh_prefix: str,
    yaw: float,
    z_sign: float,
    steel_material,
    grip_material,
    bolt_material,
) -> None:
    steel_origin = Origin(xyz=(0.0, 0.0, z_sign * STEEL_THICKNESS * 0.5), rpy=(0.0, 0.0, yaw))
    grip_origin = Origin(xyz=(0.0, 0.0, z_sign * GRIP_THICKNESS * 0.5), rpy=(0.0, 0.0, yaw))
    pivot_origin = Origin(
        xyz=(0.0, 0.0, z_sign * 0.0022),
        rpy=(0.0, 0.0, yaw),
    )

    part.visual(
        mesh_from_cadquery(_steel_body_shape(), f"{mesh_prefix}_steel_body"),
        origin=steel_origin,
        material=steel_material,
        name="steel_body",
    )
    part.visual(
        mesh_from_cadquery(_jaw_tip_shape(), f"{mesh_prefix}_jaw_tip"),
        origin=steel_origin,
        material=steel_material,
        name="jaw_tip",
    )
    part.visual(
        mesh_from_cadquery(_handle_grip_shape(), f"{mesh_prefix}_handle_grip"),
        origin=grip_origin,
        material=grip_material,
        name="handle_grip",
    )
    part.visual(
        mesh_from_cadquery(_handle_end_shape(), f"{mesh_prefix}_handle_end"),
        origin=grip_origin,
        material=grip_material,
        name="handle_end",
    )
    part.visual(
        mesh_from_cadquery(_pivot_cap_shape(), f"{mesh_prefix}_pivot_cap"),
        origin=pivot_origin,
        material=bolt_material,
        name="pivot_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bent_nose_needle_nose_pliers")

    black_oxide = model.material("black_oxide", rgba=(0.17, 0.18, 0.19, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.63, 0.65, 0.68, 1.0))
    insulated_red = model.material("insulated_red", rgba=(0.80, 0.15, 0.11, 1.0))

    upper_half = model.part("upper_half")
    lower_half = model.part("lower_half")

    _add_half_visuals(
        upper_half,
        mesh_prefix="upper_half",
        yaw=-REST_HALF_ANGLE,
        z_sign=1.0,
        steel_material=black_oxide,
        grip_material=insulated_red,
        bolt_material=bolt_steel,
    )
    _add_half_visuals(
        lower_half,
        mesh_prefix="lower_half",
        yaw=REST_HALF_ANGLE,
        z_sign=-1.0,
        steel_material=black_oxide,
        grip_material=insulated_red,
        bolt_material=bolt_steel,
    )

    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=upper_half,
        child=lower_half,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=5.0, lower=0.0, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    upper_half = object_model.get_part("upper_half")
    lower_half = object_model.get_part("lower_half")
    pivot = object_model.get_articulation("pivot")

    ctx.expect_gap(
        lower_half,
        upper_half,
        axis="y",
        positive_elem="jaw_tip",
        negative_elem="jaw_tip",
        min_gap=0.0008,
        max_gap=0.008,
        name="jaw tips rest narrowly apart",
    )
    ctx.expect_overlap(
        upper_half,
        lower_half,
        axes="x",
        elem_a="jaw_tip",
        elem_b="jaw_tip",
        min_overlap=0.028,
        name="bent tips remain aligned along the nose",
    )
    ctx.expect_overlap(
        upper_half,
        lower_half,
        axes="xy",
        elem_a="pivot_cap",
        elem_b="pivot_cap",
        min_overlap=0.010,
        name="cross-bolt stays concentric at the pivot",
    )

    limits = pivot.motion_limits
    if limits is not None and limits.upper is not None:
        with ctx.pose({pivot: limits.upper}):
            ctx.expect_gap(
                upper_half,
                lower_half,
                axis="y",
                positive_elem="handle_end",
                negative_elem="handle_end",
                min_gap=0.060,
                name="insulated handles open wide",
            )
            ctx.expect_gap(
                lower_half,
                upper_half,
                axis="y",
                positive_elem="jaw_tip",
                negative_elem="jaw_tip",
                min_gap=0.020,
                name="jaws spread as the handles open",
            )

    return ctx.report()


object_model = build_object_model()
