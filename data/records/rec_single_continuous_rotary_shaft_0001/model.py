from __future__ import annotations

from pathlib import Path

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    cadquery_available,
    mesh_from_cadquery,
)

HERE = Path(__file__).resolve().parent
MESH_DIR = HERE / "meshes"
MESH_DIR.mkdir(exist_ok=True)


# >>> USER_CODE_START
BASE_WIDTH = 0.46
BASE_DEPTH = 0.34
LOWER_BODY_HEIGHT = 0.18
UPPER_BODY_HEIGHT = 0.10
UPPER_BODY_WIDTH = 0.36
UPPER_BODY_DEPTH = 0.26
PAN_OUTER_RADIUS = 0.16
PAN_INNER_RADIUS = 0.045
PAN_HEIGHT = 0.022
COLLAR_RADIUS = 0.018
COLLAR_HEIGHT = 0.028

SHAFT_RADIUS = 0.014
SHAFT_HEIGHT = 0.022
COUPLING_LUG_SIZE = (0.012, 0.008, 0.010)
COUPLING_LUG_ORIGIN = (0.020, 0.0, 0.007)

WHEELHEAD_RADIUS = 0.14
WHEELHEAD_THICKNESS = 0.020
WHEELHEAD_HUB_RADIUS = 0.030
WHEELHEAD_HUB_HEIGHT = 0.018
GROOVE_DEPTH = 0.0015
GROOVE_RADII = (
    (0.115, 0.111),
    (0.090, 0.086),
    (0.065, 0.061),
)

BASE_TOP_Z = LOWER_BODY_HEIGHT + UPPER_BODY_HEIGHT
COLLAR_TOP_Z = BASE_TOP_Z + COLLAR_HEIGHT


def _make_base_visual_mesh():
    if not cadquery_available():
        return None

    import cadquery as cq

    lower_body = (
        cq.Workplane("XY")
        .box(BASE_WIDTH, BASE_DEPTH, LOWER_BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.03)
    )
    upper_body = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_BODY_HEIGHT)
        .box(
            UPPER_BODY_WIDTH,
            UPPER_BODY_DEPTH,
            UPPER_BODY_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.02)
    )
    splash_pan = (
        cq.Workplane("XY")
        .workplane(offset=BASE_TOP_Z)
        .circle(PAN_OUTER_RADIUS)
        .circle(PAN_INNER_RADIUS)
        .extrude(PAN_HEIGHT)
    )
    spindle_collar = (
        cq.Workplane("XY").workplane(offset=BASE_TOP_Z).circle(COLLAR_RADIUS).extrude(COLLAR_HEIGHT)
    )
    base_shape = lower_body.union(upper_body).union(splash_pan).union(spindle_collar)
    return mesh_from_cadquery(base_shape, MESH_DIR / "pottery_wheel_base.obj")


def _make_wheelhead_visual_mesh():
    if not cadquery_available():
        return None

    import cadquery as cq

    wheelhead = cq.Workplane("XY").circle(WHEELHEAD_HUB_RADIUS).extrude(WHEELHEAD_HUB_HEIGHT)
    wheelhead = wheelhead.union(
        cq.Workplane("XY")
        .workplane(offset=WHEELHEAD_HUB_HEIGHT)
        .circle(WHEELHEAD_RADIUS)
        .extrude(WHEELHEAD_THICKNESS)
    )
    for outer_radius, inner_radius in GROOVE_RADII:
        wheelhead = (
            wheelhead.faces(">Z")
            .workplane()
            .circle(outer_radius)
            .circle(inner_radius)
            .cutBlind(-GROOVE_DEPTH)
        )
    return mesh_from_cadquery(wheelhead, MESH_DIR / "pottery_wheel_head.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pottery_wheel")

    model.material("body_gray", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("wheelhead", rgba=(0.80, 0.82, 0.84, 1.0))

    base_visual_mesh = _make_base_visual_mesh()
    wheelhead_visual_mesh = _make_wheelhead_visual_mesh()

    base = model.part("base")
    if base_visual_mesh is not None:
        base.visual(base_visual_mesh, material="body_gray")
    else:
        base.visual(
            Box((BASE_WIDTH, BASE_DEPTH, LOWER_BODY_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, LOWER_BODY_HEIGHT / 2.0)),
            material="body_gray",
        )
        base.visual(
            Box((UPPER_BODY_WIDTH, UPPER_BODY_DEPTH, UPPER_BODY_HEIGHT)),
            origin=Origin(xyz=(0.0, 0.0, LOWER_BODY_HEIGHT + (UPPER_BODY_HEIGHT / 2.0))),
            material="body_gray",
        )
        base.visual(
            Cylinder(radius=PAN_OUTER_RADIUS, length=PAN_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + (PAN_HEIGHT / 2.0))),
            material="trim_black",
        )
        base.visual(
            Cylinder(radius=COLLAR_RADIUS, length=COLLAR_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + (COLLAR_HEIGHT / 2.0))),
            material="steel",
        )

    base.collision(
        Box((0.44, 0.32, LOWER_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_BODY_HEIGHT / 2.0)),
    )
    base.collision(
        Box((0.36, 0.26, UPPER_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_BODY_HEIGHT + (UPPER_BODY_HEIGHT / 2.0))),
    )
    base.collision(
        Cylinder(radius=PAN_OUTER_RADIUS, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + 0.009)),
    )
    base.collision(
        Cylinder(radius=COLLAR_RADIUS, length=COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z + (COLLAR_HEIGHT / 2.0))),
    )
    base.inertial = Inertial.from_geometry(
        Box((0.44, 0.32, 0.31)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT / 2.0)),
        material="steel",
    )
    shaft.visual(
        Box(COUPLING_LUG_SIZE),
        origin=Origin(xyz=COUPLING_LUG_ORIGIN),
        material="steel",
    )
    shaft.collision(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT / 2.0)),
    )
    shaft.collision(
        Box(COUPLING_LUG_SIZE),
        origin=Origin(xyz=COUPLING_LUG_ORIGIN),
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_HEIGHT),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT / 2.0)),
    )

    wheelhead = model.part("wheelhead")
    if wheelhead_visual_mesh is not None:
        wheelhead.visual(wheelhead_visual_mesh, material="wheelhead")
    else:
        wheelhead.visual(
            Cylinder(radius=WHEELHEAD_HUB_RADIUS, length=WHEELHEAD_HUB_HEIGHT),
            origin=Origin(xyz=(0.0, 0.0, WHEELHEAD_HUB_HEIGHT / 2.0)),
            material="steel",
        )
        wheelhead.visual(
            Cylinder(radius=WHEELHEAD_RADIUS, length=WHEELHEAD_THICKNESS),
            origin=Origin(
                xyz=(
                    0.0,
                    0.0,
                    WHEELHEAD_HUB_HEIGHT + (WHEELHEAD_THICKNESS / 2.0),
                )
            ),
            material="wheelhead",
        )

    wheelhead.collision(
        Cylinder(radius=WHEELHEAD_HUB_RADIUS, length=WHEELHEAD_HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, WHEELHEAD_HUB_HEIGHT / 2.0)),
    )
    wheelhead.collision(
        Cylinder(radius=WHEELHEAD_RADIUS, length=WHEELHEAD_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, WHEELHEAD_HUB_HEIGHT + (WHEELHEAD_THICKNESS / 2.0))),
    )
    wheelhead.inertial = Inertial.from_geometry(
        Cylinder(
            radius=WHEELHEAD_RADIUS,
            length=WHEELHEAD_HUB_HEIGHT + WHEELHEAD_THICKNESS,
        ),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (WHEELHEAD_HUB_HEIGHT + WHEELHEAD_THICKNESS) / 2.0)),
    )

    model.articulation(
        "base_to_shaft",
        ArticulationType.CONTINUOUS,
        parent="base",
        child="shaft",
        origin=Origin(xyz=(0.0, 0.0, COLLAR_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=12.0),
    )
    model.articulation(
        "shaft_to_wheelhead",
        ArticulationType.FIXED,
        parent="shaft",
        child="wheelhead",
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_joint_origin_near_geometry(tol=0.02)
    ctx.check_joint_origin_near_physical_geometry(tol=0.02)
    ctx.check_no_overlaps(
        max_pose_samples=96,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )

    ctx.expect_xy_distance("shaft", "base", max_dist=0.006)
    ctx.expect_aabb_overlap_xy("shaft", "base", min_overlap=0.012)
    ctx.expect_aabb_gap_z("shaft", "base", max_gap=0.002, max_penetration=0.0)

    ctx.expect_xy_distance("wheelhead", "shaft", max_dist=0.006)
    ctx.expect_aabb_overlap_xy("wheelhead", "shaft", min_overlap=0.012)
    ctx.expect_aabb_gap_z("wheelhead", "shaft", max_gap=0.002, max_penetration=0.0)

    ctx.expect_above("wheelhead", "base", min_clearance=0.015)
    ctx.expect_xy_distance("wheelhead", "base", max_dist=0.01)
    ctx.expect_aabb_overlap_xy("wheelhead", "base", min_overlap=0.20)
    ctx.expect_aabb_gap_z("wheelhead", "base", max_gap=0.03, max_penetration=0.0)
    ctx.expect_above("shaft", "base", min_clearance=0.0)
    ctx.expect_above("wheelhead", "shaft", min_clearance=0.0)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
