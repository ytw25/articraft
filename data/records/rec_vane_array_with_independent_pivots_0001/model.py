from __future__ import annotations

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
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
FRAME_OUTER_WIDTH = 0.62
FRAME_OUTER_HEIGHT = 0.48
FRAME_DEPTH = 0.08
SIDE_RAIL_WIDTH = 0.045
TOP_RAIL_HEIGHT = 0.05
OPENING_WIDTH = FRAME_OUTER_WIDTH - 2.0 * SIDE_RAIL_WIDTH
OPENING_HEIGHT = FRAME_OUTER_HEIGHT - 2.0 * TOP_RAIL_HEIGHT

LOUVER_COUNT = 5
LOUVER_PITCH = 0.072
LOUVER_CHORD = 0.064
LOUVER_THICKNESS = 0.010
LOUVER_COLLISION_THICKNESS = 0.008
LOUVER_SHAFT_RADIUS = 0.005
LOUVER_HUB_RADIUS = 0.008
LOUVER_HUB_LENGTH = 0.008
LOUVER_BODY_LENGTH = OPENING_WIDTH - 0.018
LOUVER_BODY_X_OFFSET = (OPENING_WIDTH - LOUVER_BODY_LENGTH) / 2.0
LOUVER_BODY_Y_OFFSET = 0.005
LOUVER_BODY_Z_OFFSET = -0.006

LEFT_AXIS_X = -OPENING_WIDTH / 2.0
LOUVER_NAMES = [f"vane_{index + 1}" for index in range(LOUVER_COUNT)]
LOUVER_JOINT_NAMES = [f"frame_to_vane_{index + 1}" for index in range(LOUVER_COUNT)]
LOUVER_Z_OFFSETS = [
    (LOUVER_COUNT - 1) * LOUVER_PITCH / 2.0 - index * LOUVER_PITCH for index in range(LOUVER_COUNT)
]


def _make_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)
    opening = cq.Workplane("XY").box(OPENING_WIDTH, FRAME_DEPTH + 0.004, OPENING_HEIGHT)
    frame = frame.cut(opening)
    return frame.edges("|Z").fillet(0.006)


def _make_louver_shape() -> cq.Workplane:
    blade_profile = (
        cq.Workplane("YZ")
        .moveTo(-LOUVER_CHORD * 0.50, -0.001)
        .threePointArc(
            (-LOUVER_CHORD * 0.16, LOUVER_THICKNESS * 0.62),
            (LOUVER_CHORD * 0.50, 0.000),
        )
        .threePointArc(
            (-LOUVER_CHORD * 0.06, -LOUVER_THICKNESS * 0.98),
            (-LOUVER_CHORD * 0.50, -0.001),
        )
        .close()
        .extrude(LOUVER_BODY_LENGTH)
        .translate((LOUVER_BODY_X_OFFSET, LOUVER_BODY_Y_OFFSET, LOUVER_BODY_Z_OFFSET))
    )

    shaft = cq.Workplane("YZ").circle(LOUVER_SHAFT_RADIUS).extrude(OPENING_WIDTH)
    left_hub = cq.Workplane("YZ").circle(LOUVER_HUB_RADIUS).extrude(LOUVER_HUB_LENGTH)
    right_hub = (
        cq.Workplane("YZ")
        .circle(LOUVER_HUB_RADIUS)
        .extrude(LOUVER_HUB_LENGTH)
        .translate((OPENING_WIDTH - LOUVER_HUB_LENGTH, 0.0, 0.0))
    )
    return shaft.union(left_hub).union(right_hub).union(blade_profile)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="adjustable_louver_bank", assets=ASSETS)
    model.material("frame_powdercoat", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("blade_aluminum", rgba=(0.73, 0.76, 0.79, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_shape(), "frame.obj", assets=ASSETS),
        material="frame_powdercoat",
    )
    frame.collision(
        Box((SIDE_RAIL_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=(-(OPENING_WIDTH / 2.0 + SIDE_RAIL_WIDTH / 2.0), 0.0, 0.0)),
    )
    frame.collision(
        Box((SIDE_RAIL_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        origin=Origin(xyz=((OPENING_WIDTH / 2.0 + SIDE_RAIL_WIDTH / 2.0), 0.0, 0.0)),
    )
    frame.collision(
        Box((OPENING_WIDTH, FRAME_DEPTH, TOP_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT / 2.0 + TOP_RAIL_HEIGHT / 2.0)),
    )
    frame.collision(
        Box((OPENING_WIDTH, FRAME_DEPTH, TOP_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, -(OPENING_HEIGHT / 2.0 + TOP_RAIL_HEIGHT / 2.0))),
    )
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        mass=3.6,
    )

    for index, (part_name, joint_name, joint_z) in enumerate(
        zip(LOUVER_NAMES, LOUVER_JOINT_NAMES, LOUVER_Z_OFFSETS)
    ):
        vane = model.part(part_name)
        vane.visual(
            mesh_from_cadquery(_make_louver_shape(), f"{part_name}.obj", assets=ASSETS),
            material="blade_aluminum",
        )
        vane.collision(
            Cylinder(radius=LOUVER_SHAFT_RADIUS * 0.95, length=OPENING_WIDTH),
            origin=Origin(
                xyz=(OPENING_WIDTH / 2.0, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
        )
        vane.collision(
            Box((LOUVER_BODY_LENGTH, LOUVER_CHORD * 0.90, LOUVER_COLLISION_THICKNESS)),
            origin=Origin(
                xyz=(
                    LOUVER_BODY_X_OFFSET + LOUVER_BODY_LENGTH / 2.0,
                    LOUVER_BODY_Y_OFFSET,
                    LOUVER_BODY_Z_OFFSET,
                )
            ),
        )
        vane.inertial = Inertial.from_geometry(
            Box((LOUVER_BODY_LENGTH, LOUVER_CHORD * 0.90, LOUVER_COLLISION_THICKNESS)),
            mass=0.22 + 0.01 * index,
            origin=Origin(
                xyz=(
                    LOUVER_BODY_X_OFFSET + LOUVER_BODY_LENGTH / 2.0,
                    LOUVER_BODY_Y_OFFSET,
                    LOUVER_BODY_Z_OFFSET,
                )
            ),
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(LEFT_AXIS_X, 0.0, joint_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-0.95,
                upper=0.95,
                effort=1.5,
                velocity=2.5,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_joint_origin_near_geometry(tol=0.02)
    ctx.check_joint_origin_near_physical_geometry(tol=0.02)
    ctx.check_no_overlaps(
        max_pose_samples=160,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )

    for vane_name in LOUVER_NAMES:
        ctx.expect_aabb_overlap_xy(vane_name, "frame", min_overlap=0.04)

    for upper, lower in zip(LOUVER_NAMES, LOUVER_NAMES[1:]):
        ctx.expect_above(upper, lower, min_clearance=0.03)
        ctx.expect_aabb_gap_z(upper, lower, max_gap=0.08, max_penetration=0.0)

    for joint_name, vane_name in zip(LOUVER_JOINT_NAMES, LOUVER_NAMES):
        ctx.expect_joint_motion_axis(
            joint_name,
            vane_name,
            world_axis="y",
            direction="positive",
            min_delta=0.005,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
