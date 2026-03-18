from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BASE_WIDTH = 0.46
BASE_DEPTH = 0.32
LOWER_DECK_THICKNESS = 0.014
TOP_PLATE_THICKNESS = 0.004
BASE_TOP_Z = LOWER_DECK_THICKNESS + TOP_PLATE_THICKNESS

CUT_START = (-0.165, 0.094)
CUT_END = (0.152, -0.123)
CUT_DX = CUT_END[0] - CUT_START[0]
CUT_DY = CUT_END[1] - CUT_START[1]
CUT_LENGTH = math.hypot(CUT_DX, CUT_DY)
CUT_YAW = math.atan2(CUT_DY, CUT_DX)
CUT_MID = ((CUT_START[0] + CUT_END[0]) / 2.0, (CUT_START[1] + CUT_END[1]) / 2.0)

ARM_REACH = CUT_LENGTH + 0.065
ARM_OPEN_LIMIT = 1.18


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paper_cutter_guillotine", assets=ASSETS)

    body_graphite = model.material("body_graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    work_surface = model.material("work_surface", rgba=(0.82, 0.83, 0.82, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.86, 0.87, 0.89, 1.0))
    arm_finish = model.material("arm_finish", rgba=(0.34, 0.37, 0.40, 1.0))
    print_blue = model.material("print_blue", rgba=(0.42, 0.54, 0.74, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    clear_guard = model.material("clear_guard", rgba=(0.82, 0.90, 0.96, 0.35))
    amber_cursor = model.material("amber_cursor", rgba=(0.90, 0.66, 0.18, 0.65))

    deck_profile = rounded_rect_profile(
        BASE_WIDTH,
        BASE_DEPTH,
        radius=0.020,
        corner_segments=10,
    )
    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(deck_profile, LOWER_DECK_THICKNESS),
        ASSETS.mesh_path("base_deck.obj"),
    )

    base = model.part("base")
    base.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, LOWER_DECK_THICKNESS / 2.0)),
        material=body_graphite,
    )
    base.visual(
        Box((0.438, 0.298, TOP_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_DECK_THICKNESS + TOP_PLATE_THICKNESS / 2.0)),
        material=work_surface,
    )
    base.visual(
        Box((0.332, 0.018, 0.002)),
        origin=Origin(xyz=(-0.005, -0.131, BASE_TOP_Z + 0.001)),
        material=brushed_aluminum,
    )
    base.visual(
        Box((0.018, 0.246, 0.002)),
        origin=Origin(xyz=(0.187, -0.006, BASE_TOP_Z + 0.001)),
        material=brushed_aluminum,
    )
    base.visual(
        Box((CUT_LENGTH * 0.98, 0.012, 0.003)),
        origin=Origin(
            xyz=(CUT_MID[0], CUT_MID[1], BASE_TOP_Z + 0.0015),
            rpy=(0.0, 0.0, CUT_YAW),
        ),
        material=blade_steel,
    )
    base.visual(
        Box((0.078, 0.058, 0.008)),
        origin=Origin(
            xyz=(CUT_START[0] + 0.012, CUT_START[1] - 0.020, BASE_TOP_Z + 0.004),
            rpy=(0.0, 0.0, CUT_YAW),
        ),
        material=body_graphite,
    )
    base.visual(
        Box((CUT_LENGTH * 0.96, 0.0012, 0.0004)),
        origin=Origin(
            xyz=(CUT_MID[0], CUT_MID[1], BASE_TOP_Z + 0.0002),
            rpy=(0.0, 0.0, CUT_YAW),
        ),
        material=amber_cursor,
    )

    for x_line in (-0.105, -0.035, 0.035, 0.105):
        base.visual(
            Box((0.0008, 0.215, 0.0004)),
            origin=Origin(xyz=(x_line, -0.010, BASE_TOP_Z + 0.0002)),
            material=print_blue,
        )

    for y_line in (-0.085, -0.040, 0.005, 0.050):
        base.visual(
            Box((0.250, 0.0008, 0.0004)),
            origin=Origin(xyz=(0.030, y_line, BASE_TOP_Z + 0.0002)),
            material=print_blue,
        )

    for foot_x, foot_y in (
        (-0.170, -0.112),
        (0.170, -0.112),
        (-0.170, 0.112),
        (0.170, 0.112),
    ):
        base.visual(
            Box((0.048, 0.040, 0.003)),
            origin=Origin(xyz=(foot_x, foot_y, -0.0015)),
            material=rubber_black,
        )

    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_TOP_Z + 0.003)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, (BASE_TOP_Z + 0.003) / 2.0)),
    )

    fence = model.part("guide_fence")
    fence.visual(
        Box((0.392, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brushed_aluminum,
    )
    fence.visual(
        Box((0.392, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.011, 0.015)),
        material=brushed_aluminum,
    )
    fence.visual(
        Box((0.034, 0.028, 0.024)),
        origin=Origin(xyz=(-0.138, 0.0, 0.012)),
        material=arm_finish,
    )
    fence.visual(
        Box((0.024, 0.028, 0.008)),
        origin=Origin(xyz=(-0.100, 0.0, 0.008)),
        material=amber_cursor,
    )
    fence.visual(
        Cylinder(radius=0.007, length=0.014),
        origin=Origin(xyz=(-0.138, 0.0, 0.031)),
        material=rubber_black,
    )
    fence.inertial = Inertial.from_geometry(
        Box((0.392, 0.028, 0.026)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    arm = model.part("arm")
    arm.visual(
        Box((0.072, 0.056, 0.026)),
        origin=Origin(xyz=(0.036, -0.028, 0.013)),
        material=arm_finish,
    )
    arm.visual(
        Box((ARM_REACH - 0.072, 0.040, 0.016)),
        origin=Origin(xyz=(0.072 + (ARM_REACH - 0.072) / 2.0, -0.020, 0.008)),
        material=arm_finish,
    )
    arm.visual(
        Box((ARM_REACH - 0.100, 0.022, 0.006)),
        origin=Origin(xyz=(0.094 + (ARM_REACH - 0.100) / 2.0, -0.019, 0.019)),
        material=body_graphite,
    )
    arm.visual(
        Box((CUT_LENGTH * 0.92, 0.004, 0.006)),
        origin=Origin(xyz=(0.030 + (CUT_LENGTH * 0.92) / 2.0, -0.002, 0.003)),
        material=blade_steel,
    )
    arm.visual(
        Box((ARM_REACH - 0.125, 0.003, 0.032)),
        origin=Origin(xyz=(0.115 + (ARM_REACH - 0.125) / 2.0, -0.0415, 0.016)),
        material=clear_guard,
    )
    arm.visual(
        Box((0.055, 0.022, 0.030)),
        origin=Origin(xyz=(ARM_REACH - 0.050, -0.019, 0.015)),
        material=arm_finish,
    )
    arm.visual(
        Box((0.020, 0.036, 0.018)),
        origin=Origin(xyz=(ARM_REACH - 0.010, -0.018, 0.009)),
        material=blade_steel,
    )
    arm.visual(
        Cylinder(radius=0.016, length=0.030),
        origin=Origin(
            xyz=(0.018, -0.010, 0.016),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=blade_steel,
    )
    arm.visual(
        Cylinder(radius=0.012, length=0.090),
        origin=Origin(
            xyz=(ARM_REACH - 0.030, -0.050, 0.028),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber_black,
    )
    arm.inertial = Inertial.from_geometry(
        Box((ARM_REACH, 0.090, 0.040)),
        mass=1.3,
        origin=Origin(xyz=(ARM_REACH / 2.0, -0.028, 0.020)),
    )

    model.articulation(
        "base_to_guide_fence",
        ArticulationType.FIXED,
        parent="base",
        child="guide_fence",
        origin=Origin(xyz=(0.0, 0.138, BASE_TOP_Z)),
    )
    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent="base",
        child="arm",
        origin=Origin(
            xyz=(CUT_START[0], CUT_START[1], BASE_TOP_Z + 0.003),
            rpy=(0.0, 0.0, CUT_YAW),
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=ARM_OPEN_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("guide_fence", "base")
    ctx.expect_aabb_overlap("guide_fence", "base", axes="xy", min_overlap=0.015)

    with ctx.pose(base_to_arm=0.0):
        ctx.expect_aabb_contact("arm", "base")
        ctx.expect_aabb_overlap("arm", "base", axes="xy", min_overlap=0.060)

    ctx.expect_joint_motion_axis(
        "base_to_arm",
        "arm",
        world_axis="z",
        direction="positive",
        min_delta=0.020,
    )

    with ctx.pose(base_to_arm=0.70):
        ctx.expect_aabb_overlap("arm", "base", axes="xy", min_overlap=0.040)
        ctx.expect_aabb_overlap("arm", "guide_fence", axes="x", min_overlap=0.080)

    with ctx.pose(base_to_arm=ARM_OPEN_LIMIT):
        ctx.expect_aabb_overlap("arm", "base", axes="xy", min_overlap=0.025)
        ctx.expect_aabb_overlap("arm", "guide_fence", axes="x", min_overlap=0.060)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
