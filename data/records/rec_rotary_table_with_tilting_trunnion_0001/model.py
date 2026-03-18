from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

PIVOT_Z = 0.305
SUPPORT_CENTER_X = 0.165
SUPPORT_WIDTH = 0.07
SUPPORT_INNER_X = SUPPORT_CENTER_X - SUPPORT_WIDTH / 2.0

TRUNNION_SPAN = 2.0 * SUPPORT_INNER_X
TRUNNION_CENTER_X = TRUNNION_SPAN / 2.0

TABLE_RADIUS = 0.100
TABLE_THICKNESS = 0.028


def _build_turntable_shape() -> cq.Workplane:
    ring_depth = 0.004
    slot_depth = 0.0045
    bolt_pocket_depth = 0.003

    body = (
        cq.Workplane("XY")
        .circle(TABLE_RADIUS * 0.94)
        .workplane(offset=0.002)
        .circle(TABLE_RADIUS)
        .workplane(offset=TABLE_THICKNESS - 0.004)
        .circle(TABLE_RADIUS)
        .workplane(offset=0.002)
        .circle(TABLE_RADIUS * 0.94)
        .loft(combine=True)
    )

    recess = (
        cq.Workplane("XY", origin=(0.0, 0.0, TABLE_THICKNESS - ring_depth))
        .circle(TABLE_RADIUS * 0.80)
        .circle(0.026)
        .extrude(ring_depth)
    )
    body = body.cut(recess)

    center_bore = cq.Workplane("XY").circle(0.016).extrude(TABLE_THICKNESS)
    body = body.cut(center_bore)

    for angle_deg in range(0, 360, 60):
        slot = (
            cq.Workplane("XY", origin=(0.0, 0.0, TABLE_THICKNESS - slot_depth))
            .transformed(rotate=(0.0, 0.0, angle_deg))
            .moveTo(0.054, 0.0)
            .slot2D(0.050, 0.012)
            .extrude(slot_depth)
        )
        body = body.cut(slot)

    for angle_deg in range(0, 360, 30):
        pocket = (
            cq.Workplane("XY", origin=(0.0, 0.0, TABLE_THICKNESS - bolt_pocket_depth))
            .transformed(rotate=(0.0, 0.0, angle_deg))
            .moveTo(0.080, 0.0)
            .circle(0.0042)
            .extrude(bolt_pocket_depth)
        )
        body = body.cut(pocket)

    return body


def _x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_trunnion_table", assets=ASSETS)

    model.material("cast_iron", rgba=(0.31, 0.33, 0.35, 1.0))
    model.material("machine_gray", rgba=(0.55, 0.58, 0.61, 1.0))
    model.material("machined_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("dark_oxide", rgba=(0.18, 0.19, 0.21, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.44, 0.30, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material="cast_iron",
    )
    base.visual(
        Box((0.24, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material="cast_iron",
    )
    base.visual(
        Box((0.30, 0.16, 0.05)),
        origin=Origin(xyz=(0.0, -0.03, 0.165)),
        material="machine_gray",
    )
    for x_sign in (-1.0, 1.0):
        x_pos = x_sign * SUPPORT_CENTER_X
        base.visual(
            Box((SUPPORT_WIDTH, 0.14, 0.19)),
            origin=Origin(xyz=(x_pos, 0.0, 0.215)),
            material="cast_iron",
        )
        base.visual(
            Cylinder(radius=0.035, length=0.020),
            origin=_x_axis_origin((x_sign * 0.210, 0.0, PIVOT_Z)),
            material="machined_steel",
        )
    base.visual(
        Box((0.34, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.075, 0.18)),
        material="machine_gray",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.44, 0.30, 0.30)),
        mass=38.0,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )

    trunnion = model.part("trunnion")
    trunnion.visual(
        Cylinder(radius=0.078, length=0.070),
        origin=Origin(xyz=(TRUNNION_CENTER_X, 0.0, -0.038)),
        material="machine_gray",
    )
    trunnion.visual(
        Cylinder(radius=0.055, length=0.006),
        origin=Origin(xyz=(TRUNNION_CENTER_X, 0.0, -0.003)),
        material="machined_steel",
    )
    trunnion.visual(
        Box((0.070, 0.080, 0.090)),
        origin=Origin(xyz=(0.050, -0.040, -0.055)),
        material="machine_gray",
    )
    trunnion.visual(
        Box((0.070, 0.080, 0.090)),
        origin=Origin(xyz=(0.210, -0.040, -0.055)),
        material="machine_gray",
    )
    trunnion.visual(
        Box((0.180, 0.035, 0.100)),
        origin=Origin(xyz=(TRUNNION_CENTER_X, -0.075, -0.050)),
        material="machine_gray",
    )
    trunnion.visual(
        Box((0.110, 0.090, 0.040)),
        origin=Origin(xyz=(TRUNNION_CENTER_X, -0.035, -0.100)),
        material="dark_oxide",
    )
    trunnion.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=_x_axis_origin((0.015, 0.0, 0.0)),
        material="machined_steel",
    )
    trunnion.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=_x_axis_origin((TRUNNION_SPAN - 0.015, 0.0, 0.0)),
        material="machined_steel",
    )
    trunnion.inertial = Inertial.from_geometry(
        Box((0.26, 0.12, 0.14)),
        mass=14.0,
        origin=Origin(xyz=(TRUNNION_CENTER_X, -0.035, -0.050)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        mesh_from_cadquery(_build_turntable_shape(), "turntable.obj", assets=ASSETS),
        material="machined_steel",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=TABLE_RADIUS, length=TABLE_THICKNESS),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, TABLE_THICKNESS / 2.0)),
    )

    model.articulation(
        "trunnion_tilt",
        ArticulationType.REVOLUTE,
        parent="base",
        child="trunnion",
        origin=Origin(xyz=(-SUPPORT_INNER_X, 0.0, PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-0.35,
            upper=1.35,
            effort=120.0,
            velocity=0.9,
        ),
    )
    model.articulation(
        "table_spin",
        ArticulationType.CONTINUOUS,
        parent="trunnion",
        child="turntable",
        origin=Origin(xyz=(TRUNNION_CENTER_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.allow_overlap(
        "trunnion",
        "base",
        reason="pivot hubs sit tightly inside the side supports and generated collision is conservative",
    )
    ctx.allow_overlap(
        "turntable",
        "trunnion",
        reason="the rotary faceplate is intentionally seated on the trunnion nose with negligible clearance",
    )
    ctx.check_no_overlaps(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("trunnion", "base")
    ctx.expect_aabb_contact("turntable", "trunnion")
    ctx.expect_aabb_overlap("turntable", "base", axes="xy", min_overlap=0.18)
    ctx.expect_origin_distance("turntable", "base", axes="xy", max_dist=0.005)
    ctx.expect_joint_motion_axis(
        "trunnion_tilt",
        "trunnion",
        world_axis="y",
        direction="positive",
        min_delta=0.03,
    )

    with ctx.pose(table_spin=math.pi / 2.0):
        ctx.expect_aabb_contact("turntable", "trunnion")
        ctx.expect_origin_distance("turntable", "base", axes="xy", max_dist=0.005)
        ctx.expect_aabb_overlap("turntable", "base", axes="xy", min_overlap=0.18)

    with ctx.pose(trunnion_tilt=-0.30):
        ctx.expect_aabb_contact("trunnion", "base")
        ctx.expect_aabb_contact("turntable", "trunnion")
        ctx.expect_origin_distance("turntable", "base", axes="xy", max_dist=0.005)

    with ctx.pose(trunnion_tilt=1.20):
        ctx.expect_aabb_contact("trunnion", "base")
        ctx.expect_aabb_contact("turntable", "trunnion")
        ctx.expect_origin_distance("turntable", "base", axes="xy", max_dist=0.005)

    with ctx.pose({"trunnion_tilt": 1.20, "table_spin": math.pi}):
        ctx.expect_aabb_contact("trunnion", "base")
        ctx.expect_aabb_contact("turntable", "trunnion")
        ctx.expect_origin_distance("turntable", "base", axes="xy", max_dist=0.005)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
