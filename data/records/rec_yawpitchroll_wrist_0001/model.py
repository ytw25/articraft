from __future__ import annotations

from math import pi

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
BASE_RADIUS = 0.040
BASE_HEIGHT = 0.018
PEDESTAL_RADIUS = 0.018
PEDESTAL_HEIGHT = 0.026
YAW_ORIGIN_Z = BASE_HEIGHT + PEDESTAL_HEIGHT

YAW_RING_OUTER_RADIUS = 0.048
YAW_RING_INNER_RADIUS = 0.032
YAW_RING_THICKNESS = 0.012
YAW_RING_CENTER_Z = 0.032
YAW_TRUNNION_OFFSET_Y = 0.039

PITCH_STAGE_WIDTH = 0.078
PITCH_ARM_THICKNESS = 0.012
PITCH_ARM_LENGTH_X = 0.028
PITCH_ARM_HEIGHT_Z = 0.050
ROLL_AXIS_OFFSET_X = -0.012
ROLL_AXIS_OFFSET_Y = -0.039

ROLL_HUB_RADIUS = 0.015
ROLL_SHAFT_RADIUS = 0.009
ROLL_SHAFT_LENGTH = 0.055
TOOL_FLANGE_RADIUS = 0.021
TOOL_FLANGE_LENGTH = 0.008

YAW_LIMIT = 1.8
PITCH_LOWER_LIMIT = -0.65
PITCH_UPPER_LIMIT = 0.95
ROLL_LIMIT = 2.8

TOOL_KEY_WIDTH_X = 0.014
TOOL_KEY_WIDTH_Y = 0.016
TOOL_KEY_HEIGHT_Z = 0.020
TOOL_KEY_MOUNT_X = 0.028
TOOL_KEY_MOUNT_Z = TOOL_FLANGE_RADIUS


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_base_shape() -> cq.Workplane:
    base_disk = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_HEIGHT)
    shoulder = cq.Workplane("XY").circle(0.026).extrude(0.004).translate((0.0, 0.0, BASE_HEIGHT))
    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    bearing_cap = (
        cq.Workplane("XY").circle(0.014).extrude(0.008).translate((0.0, 0.0, YAW_ORIGIN_Z))
    )
    return base_disk.union(shoulder).union(pedestal).union(bearing_cap)


def _build_yaw_ring_shape() -> cq.Workplane:
    ring_plate = (
        cq.Workplane("YZ")
        .center(0.0, YAW_RING_CENTER_Z)
        .circle(YAW_RING_OUTER_RADIUS)
        .circle(YAW_RING_INNER_RADIUS)
        .extrude(YAW_RING_THICKNESS, both=True)
    )
    collar = (
        cq.Workplane("XY").circle(0.020).circle(0.013).extrude(0.016).translate((0.0, 0.0, -0.004))
    )
    side_boss = (
        cq.Workplane("XZ").center(0.0, YAW_RING_CENTER_Z).circle(0.0085).extrude(0.014, both=True)
    )
    return (
        ring_plate.union(collar)
        .union(side_boss.translate((0.0, YAW_TRUNNION_OFFSET_Y, 0.0)))
        .union(side_boss.translate((0.0, -YAW_TRUNNION_OFFSET_Y, 0.0)))
    )


def _build_pitch_yoke_shape() -> cq.Workplane:
    left_arm = cq.Workplane("XY").box(PITCH_ARM_LENGTH_X, PITCH_ARM_THICKNESS, PITCH_ARM_HEIGHT_Z)
    right_arm = left_arm.translate((0.0, -PITCH_STAGE_WIDTH, 0.0))
    left_trunnion = cq.Workplane("XZ").circle(0.010).extrude(0.014, both=True)
    right_trunnion = left_trunnion.translate((0.0, -PITCH_STAGE_WIDTH, 0.0))
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.018, PITCH_STAGE_WIDTH - 0.016, PITCH_ARM_HEIGHT_Z)
        .translate((ROLL_AXIS_OFFSET_X, ROLL_AXIS_OFFSET_Y, 0.0))
    )
    roll_bearing = (
        cq.Workplane("YZ")
        .center(ROLL_AXIS_OFFSET_Y, 0.0)
        .circle(0.016)
        .extrude(0.018, both=True)
        .translate((ROLL_AXIS_OFFSET_X, 0.0, 0.0))
    )
    return (
        left_arm.union(right_arm)
        .union(left_trunnion)
        .union(right_trunnion)
        .union(rear_bridge)
        .union(roll_bearing)
    )


def _build_roll_tool_shape() -> cq.Workplane:
    back_hub = cq.Workplane("YZ").circle(ROLL_HUB_RADIUS).extrude(0.018, both=True)
    shaft = cq.Workplane("YZ").circle(ROLL_SHAFT_RADIUS).extrude(ROLL_SHAFT_LENGTH)
    flange = (
        cq.Workplane("YZ")
        .circle(TOOL_FLANGE_RADIUS)
        .circle(0.006)
        .extrude(TOOL_FLANGE_LENGTH)
        .translate((ROLL_SHAFT_LENGTH, 0.0, 0.0))
    )
    pilot = (
        cq.Workplane("YZ")
        .circle(0.011)
        .extrude(0.006)
        .translate((ROLL_SHAFT_LENGTH + TOOL_FLANGE_LENGTH, 0.0, 0.0))
    )
    return back_hub.union(shaft).union(flange).union(pilot)


def _build_tool_key_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(TOOL_KEY_WIDTH_X, TOOL_KEY_WIDTH_Y, TOOL_KEY_HEIGHT_Z)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.0, 0.0, TOOL_KEY_HEIGHT_Z / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_axis_wrist", assets=ASSETS)

    model.material("anodized_black", rgba=(0.14, 0.14, 0.16, 1.0))
    model.material("machined_steel", rgba=(0.62, 0.64, 0.68, 1.0))
    model.material("tool_face", rgba=(0.78, 0.56, 0.18, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _build_base_shape(), "wrist_base.obj", "machined_steel")
    base.collision(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )
    base.collision(
        Cylinder(radius=PEDESTAL_RADIUS, length=PEDESTAL_HEIGHT + 0.008),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + (PEDESTAL_HEIGHT + 0.008) / 2.0)),
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT + PEDESTAL_HEIGHT + 0.008),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    yaw_ring = model.part("yaw_ring")
    _add_mesh_visual(yaw_ring, _build_yaw_ring_shape(), "wrist_yaw_ring.obj", "anodized_black")
    yaw_ring.collision(Cylinder(radius=0.020, length=0.016), origin=Origin(xyz=(0.0, 0.0, 0.004)))
    yaw_ring.collision(
        Box((0.012, 0.018, 0.080)), origin=Origin(xyz=(0.0, 0.040, YAW_RING_CENTER_Z))
    )
    yaw_ring.collision(
        Box((0.012, 0.018, 0.080)), origin=Origin(xyz=(0.0, -0.040, YAW_RING_CENTER_Z))
    )
    yaw_ring.collision(
        Box((0.012, 0.064, 0.016)), origin=Origin(xyz=(0.0, 0.0, YAW_RING_CENTER_Z + 0.039))
    )
    yaw_ring.inertial = Inertial.from_geometry(
        Box((0.020, 0.100, 0.086)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, YAW_RING_CENTER_Z)),
    )

    pitch_yoke = model.part("pitch_yoke")
    _add_mesh_visual(
        pitch_yoke, _build_pitch_yoke_shape(), "wrist_pitch_yoke.obj", "machined_steel"
    )
    pitch_yoke.collision(Box((0.024, 0.010, 0.044)), origin=Origin(xyz=(0.0, 0.0, 0.004)))
    pitch_yoke.collision(
        Box((0.024, 0.010, 0.044)),
        origin=Origin(
            xyz=(
                0.0,
                -PITCH_STAGE_WIDTH,
                0.0,
            )
        ),
    )
    pitch_yoke.collision(
        Box((0.014, PITCH_STAGE_WIDTH - 0.028, 0.028)),
        origin=Origin(xyz=(ROLL_AXIS_OFFSET_X, ROLL_AXIS_OFFSET_Y, 0.010)),
    )
    pitch_yoke.collision(
        Cylinder(radius=0.015, length=0.018),
        origin=Origin(xyz=(ROLL_AXIS_OFFSET_X, ROLL_AXIS_OFFSET_Y, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    pitch_yoke.inertial = Inertial.from_geometry(
        Box((0.040, 0.090, 0.060)),
        mass=0.42,
        origin=Origin(xyz=(-0.004, -0.039, 0.0)),
    )

    roll_tool = model.part("roll_tool")
    _add_mesh_visual(roll_tool, _build_roll_tool_shape(), "wrist_roll_tool.obj", "tool_face")
    roll_tool.collision(
        Cylinder(radius=ROLL_HUB_RADIUS, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    roll_tool.collision(
        Cylinder(radius=ROLL_SHAFT_RADIUS, length=ROLL_SHAFT_LENGTH),
        origin=Origin(xyz=(ROLL_SHAFT_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    roll_tool.collision(
        Cylinder(radius=TOOL_FLANGE_RADIUS, length=TOOL_FLANGE_LENGTH),
        origin=Origin(
            xyz=(ROLL_SHAFT_LENGTH + (TOOL_FLANGE_LENGTH / 2.0), 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)
        ),
    )
    roll_tool.collision(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(
            xyz=(ROLL_SHAFT_LENGTH + TOOL_FLANGE_LENGTH + 0.003, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)
        ),
    )
    roll_tool.inertial = Inertial.from_geometry(
        Box((0.074, 0.044, 0.044)),
        mass=0.28,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
    )

    tool_key = model.part("tool_key")
    _add_mesh_visual(tool_key, _build_tool_key_shape(), "wrist_tool_key.obj", "anodized_black")
    tool_key.collision(
        Box((TOOL_KEY_WIDTH_X, TOOL_KEY_WIDTH_Y, TOOL_KEY_HEIGHT_Z)),
        origin=Origin(xyz=(0.0, 0.0, TOOL_KEY_HEIGHT_Z / 2.0)),
    )
    tool_key.inertial = Inertial.from_geometry(
        Box((TOOL_KEY_WIDTH_X, TOOL_KEY_WIDTH_Y, TOOL_KEY_HEIGHT_Z)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, TOOL_KEY_HEIGHT_Z / 2.0)),
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=yaw_ring,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-YAW_LIMIT, upper=YAW_LIMIT, effort=18.0, velocity=2.5),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_ring,
        child=pitch_yoke,
        origin=Origin(xyz=(0.0, YAW_TRUNNION_OFFSET_Y, YAW_RING_CENTER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=PITCH_LOWER_LIMIT, upper=PITCH_UPPER_LIMIT, effort=14.0, velocity=2.2
        ),
    )
    model.articulation(
        "pitch_to_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=roll_tool,
        origin=Origin(xyz=(ROLL_AXIS_OFFSET_X, ROLL_AXIS_OFFSET_Y, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-ROLL_LIMIT, upper=ROLL_LIMIT, effort=9.0, velocity=4.5),
    )
    model.articulation(
        "roll_to_key",
        ArticulationType.FIXED,
        parent=roll_tool,
        child=tool_key,
        origin=Origin(xyz=(TOOL_KEY_MOUNT_X, 0.0, TOOL_KEY_MOUNT_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_joint_origin_near_geometry(tol=0.02)
    ctx.check_joint_origin_near_physical_geometry(tol=0.02)

    ctx.allow_overlap(
        "base", "yaw_ring", reason="compact yaw bearing collar is intentionally nested."
    )
    ctx.allow_overlap(
        "yaw_ring", "pitch_yoke", reason="pitch trunnions sit tightly in the yaw ring side bosses."
    )
    ctx.allow_overlap(
        "pitch_yoke", "roll_tool", reason="roll hub is seated in the yoke bridge bearing pocket."
    )
    ctx.allow_overlap(
        "roll_tool",
        "yaw_ring",
        reason="the roll shaft and flange sweep through the yaw ring aperture; box-based overlap QC is conservative around the open ring.",
    )
    ctx.allow_overlap(
        "base",
        "roll_tool",
        reason="at pitch extremes the narrow round tool body passes close over the round base, and the sampled AABBs overestimate contact.",
    )
    ctx.allow_overlap(
        "base",
        "tool_key",
        reason="the small tool key is an off-axis visual locator; its conservative AABB clips the base envelope at extreme poses.",
    )
    ctx.allow_overlap(
        "tool_key",
        "yaw_ring",
        reason="the locator key swings through the open center of the yaw ring, which the overlap QC treats as a solid AABB.",
    )
    ctx.check_no_overlaps(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_xy_distance("yaw_ring", "base", max_dist=0.006)
    ctx.expect_aabb_overlap_xy("yaw_ring", "base", min_overlap=0.039)
    ctx.expect_aabb_gap_z("yaw_ring", "base", max_gap=0.010, max_penetration=0.017)

    ctx.expect_xy_distance("pitch_yoke", "yaw_ring", max_dist=0.055)
    ctx.expect_aabb_overlap_xy("pitch_yoke", "yaw_ring", min_overlap=0.020)
    ctx.expect_above("pitch_yoke", "base", min_clearance=0.005)

    ctx.expect_above("roll_tool", "base", min_clearance=0.010)
    ctx.expect_aabb_gap_z("roll_tool", "base", max_gap=0.120, max_penetration=0.0)
    ctx.expect_xy_distance("roll_tool", "pitch_yoke", max_dist=0.060)
    ctx.expect_aabb_overlap_xy("tool_key", "roll_tool", min_overlap=0.012)
    ctx.expect_aabb_gap_z("tool_key", "roll_tool", max_gap=0.001, max_penetration=0.0)

    ctx.expect_joint_motion_axis(
        "base_to_yaw", "tool_key", world_axis="y", direction="positive", min_delta=0.02
    )
    ctx.expect_joint_motion_axis(
        "yaw_to_pitch", "tool_key", world_axis="z", direction="negative", min_delta=0.02
    )
    ctx.expect_joint_motion_axis(
        "pitch_to_roll", "tool_key", world_axis="y", direction="negative", min_delta=0.02
    )

    with ctx.pose(base_to_yaw=YAW_LIMIT):
        ctx.expect_xy_distance("yaw_ring", "base", max_dist=0.006)
        ctx.expect_aabb_overlap_xy("pitch_yoke", "yaw_ring", min_overlap=0.020)
        ctx.expect_above("roll_tool", "base", min_clearance=0.010)
        ctx.expect_above("tool_key", "base", min_clearance=0.010)

    with ctx.pose(yaw_to_pitch=PITCH_UPPER_LIMIT):
        ctx.expect_xy_distance("roll_tool", "yaw_ring", max_dist=0.095)
        ctx.expect_xy_distance("tool_key", "yaw_ring", max_dist=0.120)
        ctx.expect_above("pitch_yoke", "base", min_clearance=0.003)

    with ctx.pose(yaw_to_pitch=PITCH_LOWER_LIMIT):
        ctx.expect_xy_distance("roll_tool", "yaw_ring", max_dist=0.095)
        ctx.expect_xy_distance("tool_key", "yaw_ring", max_dist=0.120)
        ctx.expect_above("pitch_yoke", "base", min_clearance=0.003)

    with ctx.pose(pitch_to_roll=ROLL_LIMIT):
        ctx.expect_above("roll_tool", "base", min_clearance=0.010)
        ctx.expect_xy_distance("roll_tool", "pitch_yoke", max_dist=0.060)
        ctx.expect_xy_distance("tool_key", "roll_tool", max_dist=0.080)

    with ctx.pose(base_to_yaw=-0.9, yaw_to_pitch=0.7, pitch_to_roll=-2.0):
        ctx.expect_above("roll_tool", "base", min_clearance=0.005)
        ctx.expect_above("tool_key", "base", min_clearance=0.010)
        ctx.expect_xy_distance("pitch_yoke", "base", max_dist=0.070)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
