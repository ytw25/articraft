from __future__ import annotations

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
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
MOTOR_BASE_SIZE = 0.14
MOTOR_BASE_HEIGHT = 0.04
MOTOR_BOLT_SPACING = 0.09
MOTOR_BOLT_RADIUS = 0.0035
MOTOR_BASE_FILLET = 0.006

PEDESTAL_RADIUS = 0.032
PEDESTAL_HEIGHT = 0.055

HOUSING_OUTER_RADIUS = 0.055
HOUSING_INNER_RADIUS = 0.018
HOUSING_HEIGHT = 0.028
HOUSING_BASE_Z = MOTOR_BASE_HEIGHT + PEDESTAL_HEIGHT
YAW_JOINT_Z = HOUSING_BASE_Z + (HOUSING_HEIGHT / 2.0)

ROTOR_SHAFT_RADIUS = 0.015
ROTOR_SHAFT_BOTTOM_Z = -0.014
ROTOR_SHAFT_TOP_Z = 0.018
ROTOR_FLANGE_RADIUS = 0.042
ROTOR_FLANGE_BOTTOM_Z = 0.015
ROTOR_FLANGE_TOP_Z = 0.025

TOP_PLATE_SIZE = 0.18
TOP_PLATE_THICKNESS = 0.012
TOP_PLATE_HOLE_SPACING = 0.12
TOP_PLATE_HOLE_RADIUS = 0.004
TOP_PLATE_EDGE_FILLET = 0.01
TOP_PLATE_TAB_LENGTH = 0.02
TOP_PLATE_TAB_WIDTH = 0.03


def _make_base_shape() -> cq.Workplane:
    motor_base = (
        cq.Workplane("XY")
        .box(MOTOR_BASE_SIZE, MOTOR_BASE_SIZE, MOTOR_BASE_HEIGHT)
        .edges("|Z")
        .fillet(MOTOR_BASE_FILLET)
        .faces(">Z")
        .workplane()
        .rect(MOTOR_BOLT_SPACING, MOTOR_BOLT_SPACING, forConstruction=True)
        .vertices()
        .circle(MOTOR_BOLT_RADIUS)
        .cutBlind(-MOTOR_BASE_HEIGHT)
        .translate((0.0, 0.0, MOTOR_BASE_HEIGHT / 2.0))
    )

    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, MOTOR_BASE_HEIGHT))
    )

    bearing_housing = (
        cq.Workplane("XY")
        .circle(HOUSING_OUTER_RADIUS)
        .extrude(HOUSING_HEIGHT)
        .translate((0.0, 0.0, HOUSING_BASE_Z))
    )
    bearing_bore = (
        cq.Workplane("XY")
        .circle(HOUSING_INNER_RADIUS)
        .extrude(HOUSING_HEIGHT + 0.004)
        .translate((0.0, 0.0, HOUSING_BASE_Z - 0.002))
    )

    return motor_base.union(pedestal).union(bearing_housing).cut(bearing_bore)


def _make_rotor_shape() -> cq.Workplane:
    shaft = (
        cq.Workplane("XY")
        .circle(ROTOR_SHAFT_RADIUS)
        .extrude(ROTOR_SHAFT_TOP_Z - ROTOR_SHAFT_BOTTOM_Z)
        .translate((0.0, 0.0, ROTOR_SHAFT_BOTTOM_Z))
    )
    flange = (
        cq.Workplane("XY")
        .circle(ROTOR_FLANGE_RADIUS)
        .extrude(ROTOR_FLANGE_TOP_Z - ROTOR_FLANGE_BOTTOM_Z)
        .translate((0.0, 0.0, ROTOR_FLANGE_BOTTOM_Z))
    )
    return shaft.union(flange)


def _make_top_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(TOP_PLATE_SIZE, TOP_PLATE_SIZE, TOP_PLATE_THICKNESS)
        .edges("|Z")
        .fillet(TOP_PLATE_EDGE_FILLET)
        .faces(">Z")
        .workplane()
        .rect(TOP_PLATE_HOLE_SPACING, TOP_PLATE_HOLE_SPACING, forConstruction=True)
        .vertices()
        .circle(TOP_PLATE_HOLE_RADIUS)
        .cutThruAll()
    )
    cable_tab = cq.Workplane("XY").box(
        TOP_PLATE_TAB_LENGTH,
        TOP_PLATE_TAB_WIDTH,
        TOP_PLATE_THICKNESS,
    )
    cable_tab = cable_tab.translate(
        (
            (TOP_PLATE_SIZE / 2.0) + (TOP_PLATE_TAB_LENGTH / 2.0),
            0.0,
            0.0,
        )
    )
    return plate.union(cable_tab).translate((0.0, 0.0, TOP_PLATE_THICKNESS / 2.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_yaw_module", assets=ASSETS)

    model.material("motor_black", rgba=(0.17, 0.18, 0.20, 1.0))
    model.material("bearing_steel", rgba=(0.52, 0.54, 0.58, 1.0))
    model.material("plate_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))

    base = model.part("pedestal_base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "pedestal_base.obj", assets=ASSETS),
        material="motor_black",
    )
    base.collision(
        Box((MOTOR_BASE_SIZE, MOTOR_BASE_SIZE, MOTOR_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MOTOR_BASE_HEIGHT / 2.0)),
    )
    base.collision(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, MOTOR_BASE_HEIGHT + (PEDESTAL_HEIGHT / 2.0))),
    )
    housing_wall_thickness = HOUSING_OUTER_RADIUS - HOUSING_INNER_RADIUS
    housing_wall_center = HOUSING_INNER_RADIUS + (housing_wall_thickness / 2.0)
    housing_center_z = HOUSING_BASE_Z + (HOUSING_HEIGHT / 2.0)
    base.collision(
        Box((2.0 * HOUSING_OUTER_RADIUS, housing_wall_thickness, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, housing_wall_center, housing_center_z)),
    )
    base.collision(
        Box((2.0 * HOUSING_OUTER_RADIUS, housing_wall_thickness, HOUSING_HEIGHT)),
        origin=Origin(xyz=(0.0, -housing_wall_center, housing_center_z)),
    )
    base.collision(
        Box((housing_wall_thickness, 2.0 * HOUSING_INNER_RADIUS, HOUSING_HEIGHT)),
        origin=Origin(xyz=(housing_wall_center, 0.0, housing_center_z)),
    )
    base.collision(
        Box((housing_wall_thickness, 2.0 * HOUSING_INNER_RADIUS, HOUSING_HEIGHT)),
        origin=Origin(xyz=(-housing_wall_center, 0.0, housing_center_z)),
    )
    base.inertial = Inertial.from_geometry(
        Box((MOTOR_BASE_SIZE, MOTOR_BASE_SIZE, HOUSING_BASE_Z + HOUSING_HEIGHT)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, (HOUSING_BASE_Z + HOUSING_HEIGHT) / 2.0)),
    )

    rotor = model.part("yaw_rotor")
    rotor.visual(
        mesh_from_cadquery(_make_rotor_shape(), "yaw_rotor.obj", assets=ASSETS),
        material="bearing_steel",
    )
    rotor.collision(
        Cylinder(
            radius=ROTOR_SHAFT_RADIUS,
            length=ROTOR_SHAFT_TOP_Z - ROTOR_SHAFT_BOTTOM_Z,
        ),
        origin=Origin(xyz=(0.0, 0.0, (ROTOR_SHAFT_TOP_Z + ROTOR_SHAFT_BOTTOM_Z) / 2.0)),
    )
    rotor.collision(
        Cylinder(
            radius=ROTOR_FLANGE_RADIUS,
            length=ROTOR_FLANGE_TOP_Z - ROTOR_FLANGE_BOTTOM_Z,
        ),
        origin=Origin(xyz=(0.0, 0.0, (ROTOR_FLANGE_TOP_Z + ROTOR_FLANGE_BOTTOM_Z) / 2.0)),
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=ROTOR_FLANGE_RADIUS, length=ROTOR_FLANGE_TOP_Z + 0.01),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    top_plate = model.part("top_plate")
    top_plate.visual(
        mesh_from_cadquery(_make_top_plate_shape(), "top_plate.obj", assets=ASSETS),
        material="plate_aluminum",
    )
    top_plate.collision(
        Box((TOP_PLATE_SIZE, TOP_PLATE_SIZE, TOP_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_THICKNESS / 2.0)),
    )
    top_plate.collision(
        Box((TOP_PLATE_TAB_LENGTH, TOP_PLATE_TAB_WIDTH, TOP_PLATE_THICKNESS)),
        origin=Origin(
            xyz=(
                (TOP_PLATE_SIZE / 2.0) + (TOP_PLATE_TAB_LENGTH / 2.0),
                0.0,
                TOP_PLATE_THICKNESS / 2.0,
            )
        ),
    )
    top_plate.inertial = Inertial.from_geometry(
        Box(
            (
                TOP_PLATE_SIZE + TOP_PLATE_TAB_LENGTH,
                TOP_PLATE_SIZE,
                TOP_PLATE_THICKNESS,
            )
        ),
        mass=0.55,
        origin=Origin(xyz=(TOP_PLATE_TAB_LENGTH / 2.0, 0.0, TOP_PLATE_THICKNESS / 2.0)),
    )

    model.articulation(
        "pedestal_to_rotor",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, YAW_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-1.5708,
            upper=1.5708,
            effort=12.0,
            velocity=4.0,
        ),
    )
    model.articulation(
        "rotor_to_top_plate",
        ArticulationType.FIXED,
        parent=rotor,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, ROTOR_FLANGE_TOP_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_joint_origin_near_geometry(tol=0.02)
    ctx.check_joint_origin_near_physical_geometry(tol=0.02)
    ctx.check_no_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_xy_distance("yaw_rotor", "pedestal_base", max_dist=0.003)
    ctx.expect_xy_distance("top_plate", "pedestal_base", max_dist=0.015)
    ctx.expect_aabb_overlap_xy("yaw_rotor", "pedestal_base", min_overlap=0.03)
    ctx.expect_aabb_overlap_xy("top_plate", "pedestal_base", min_overlap=0.12)
    ctx.expect_above("top_plate", "pedestal_base", min_clearance=0.0)
    ctx.expect_aabb_gap_z(
        "top_plate",
        "pedestal_base",
        max_gap=0.02,
        max_penetration=0.0,
    )
    ctx.expect_aabb_gap_z(
        "top_plate",
        "yaw_rotor",
        max_gap=0.002,
        max_penetration=0.0,
    )
    ctx.expect_joint_motion_axis(
        "pedestal_to_rotor",
        "top_plate",
        world_axis="y",
        direction="positive",
        min_delta=0.015,
    )

    with ctx.pose(pedestal_to_rotor=1.2):
        ctx.expect_xy_distance("top_plate", "pedestal_base", max_dist=0.015)
        ctx.expect_aabb_overlap_xy("top_plate", "pedestal_base", min_overlap=0.12)
        ctx.expect_aabb_gap_z(
            "top_plate",
            "pedestal_base",
            max_gap=0.02,
            max_penetration=0.0,
        )

    with ctx.pose(pedestal_to_rotor=-1.7):
        ctx.expect_xy_distance("top_plate", "pedestal_base", max_dist=0.015)
        ctx.expect_aabb_gap_z(
            "top_plate",
            "yaw_rotor",
            max_gap=0.002,
            max_penetration=0.0,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
