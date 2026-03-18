from __future__ import annotations

from pathlib import Path

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
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
try:
    import cadquery as cq
except Exception:  # pragma: no cover - cadquery is optional at import time
    cq = None


PI = 3.141592653589793

BASE_PLATE_SIZE = (0.18, 0.16, 0.024)
BASE_COLUMN_SIZE = (0.078, 0.082, 0.10)
SHOULDER_Z = BASE_PLATE_SIZE[2] + BASE_COLUMN_SIZE[2]
SHOULDER_BLOCK_SIZE = (0.09, 0.05, 0.014)

UPPER_ARM_LENGTH = 0.26
FOREARM_LENGTH = 0.22
WRIST_BODY_LENGTH = 0.09
WRIST_MOUNT_THICKNESS = 0.008

JOINT_BOX_SIZE = (0.026, 0.04, 0.026)
LINK_HUB_COLLISION_SIZE = (0.03, 0.042, 0.03)
WRIST_HUB_COLLISION_SIZE = (0.028, 0.038, 0.028)
UPPER_ARM_BEAM_SIZE = (UPPER_ARM_LENGTH - 0.088, 0.032, 0.028)
FOREARM_BEAM_SIZE = (FOREARM_LENGTH - 0.082, 0.03, 0.026)
WRIST_BEAM_SIZE = (0.052, 0.024, 0.022)
PAD_SIZE = (0.012, 0.09, 0.06)
PAD_CENTER_X = WRIST_BODY_LENGTH + WRIST_MOUNT_THICKNESS + 0.001 + (PAD_SIZE[0] / 2.0)


def _has_cadquery() -> bool:
    return cq is not None and cadquery_available()


def _mesh_or_fallback(shape, filename: str, fallback):
    if _has_cadquery():
        return mesh_from_cadquery(shape, MESH_DIR / filename)
    return fallback


def _make_base_shape():
    plate = cq.Workplane("XY").box(*BASE_PLATE_SIZE).translate((0.0, 0.0, BASE_PLATE_SIZE[2] / 2.0))
    column = (
        cq.Workplane("XY")
        .box(*BASE_COLUMN_SIZE)
        .translate((0.0, 0.0, BASE_PLATE_SIZE[2] + (BASE_COLUMN_SIZE[2] / 2.0)))
    )
    shoulder_block = (
        cq.Workplane("XY")
        .box(*SHOULDER_BLOCK_SIZE)
        .translate((0.0, 0.0, SHOULDER_Z - (SHOULDER_BLOCK_SIZE[2] / 2.0)))
    )
    rear_rib = (
        cq.Workplane("XY").box(0.03, 0.05, 0.06).translate((-0.02, 0.0, BASE_PLATE_SIZE[2] + 0.045))
    )
    return plate.union(column).union(shoulder_block).union(rear_rib)


def _make_arm_link_shape(
    length: float, beam_width: float, beam_height: float, hub_radius: float, hub_width: float
):
    proximal_hub = (
        cq.Workplane("XZ")
        .circle(hub_radius)
        .extrude(hub_width)
        .translate((0.0, -hub_width / 2.0, 0.0))
    )
    beam = (
        cq.Workplane("XY").box(length, beam_width, beam_height).translate((length / 2.0, 0.0, 0.0))
    )
    distal_hub = (
        cq.Workplane("XZ")
        .circle(hub_radius * 0.94)
        .extrude(hub_width * 0.92)
        .translate((length, -(hub_width * 0.92) / 2.0, 0.0))
    )
    lower_stiffener = (
        cq.Workplane("XY")
        .box(length * 0.72, beam_width * 0.6, beam_height * 0.52)
        .translate((length * 0.55, 0.0, -(beam_height * 0.28)))
    )
    return proximal_hub.union(beam).union(distal_hub).union(lower_stiffener)


def _make_wrist_body_shape():
    body = _make_arm_link_shape(
        WRIST_BODY_LENGTH,
        beam_width=0.024,
        beam_height=0.022,
        hub_radius=0.024,
        hub_width=0.042,
    )
    mount = (
        cq.Workplane("XY")
        .box(WRIST_MOUNT_THICKNESS, 0.046, 0.034)
        .translate((WRIST_BODY_LENGTH + (WRIST_MOUNT_THICKNESS / 2.0), 0.0, 0.0))
    )
    return body.union(mount)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="task_arm")

    model.material("base_gray", rgba=(0.23, 0.24, 0.27, 1.0))
    model.material("arm_silver", rgba=(0.77, 0.79, 0.83, 1.0))
    model.material("pad_black", rgba=(0.11, 0.12, 0.13, 1.0))

    base = model.part("base")
    if _has_cadquery():
        base.visual(
            _mesh_or_fallback(_make_base_shape(), "base.obj", Box((0.18, 0.16, SHOULDER_Z))),
            material="base_gray",
        )
    else:
        base.visual(
            Box((0.18, 0.16, SHOULDER_Z)),
            origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z / 2.0)),
            material="base_gray",
        )
    base.collision(Box(BASE_PLATE_SIZE), origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_SIZE[2] / 2.0)))
    base.collision(
        Box(BASE_COLUMN_SIZE),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_SIZE[2] + (BASE_COLUMN_SIZE[2] / 2.0))),
    )
    base.qc_collision(Box(JOINT_BOX_SIZE), origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z - 0.004)))
    base.inertial = Inertial.from_geometry(
        Box((0.18, 0.16, SHOULDER_Z)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z / 2.0)),
    )

    upper_arm = model.part("upper_arm")
    if _has_cadquery():
        upper_arm.visual(
            _mesh_or_fallback(
                _make_arm_link_shape(UPPER_ARM_LENGTH, 0.038, 0.03, 0.028, 0.048),
                "upper_arm.obj",
                Box((UPPER_ARM_LENGTH + 0.056, 0.048, 0.056)),
            ),
            material="arm_silver",
        )
    else:
        upper_arm.visual(
            Box((UPPER_ARM_LENGTH + 0.056, 0.048, 0.056)),
            origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
            material="arm_silver",
        )
    upper_arm.collision(Box(LINK_HUB_COLLISION_SIZE), origin=Origin(xyz=(0.0, 0.0, 0.0)))
    upper_arm.collision(
        Box(UPPER_ARM_BEAM_SIZE), origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0))
    )
    upper_arm.collision(
        Box(LINK_HUB_COLLISION_SIZE), origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0))
    )
    upper_arm.qc_collision(Box(JOINT_BOX_SIZE), origin=Origin(xyz=(0.0, 0.0, 0.0)))
    upper_arm.qc_collision(Box(JOINT_BOX_SIZE), origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)))
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.045, 0.04)),
        mass=1.2,
        origin=Origin(xyz=(UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    if _has_cadquery():
        forearm.visual(
            _mesh_or_fallback(
                _make_arm_link_shape(FOREARM_LENGTH, 0.034, 0.028, 0.026, 0.044),
                "forearm.obj",
                Box((FOREARM_LENGTH + 0.052, 0.044, 0.052)),
            ),
            material="arm_silver",
        )
    else:
        forearm.visual(
            Box((FOREARM_LENGTH + 0.052, 0.044, 0.052)),
            origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.0)),
            material="arm_silver",
        )
    forearm.collision(Box(LINK_HUB_COLLISION_SIZE), origin=Origin(xyz=(0.0, 0.0, 0.0)))
    forearm.collision(Box(FOREARM_BEAM_SIZE), origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.0)))
    forearm.collision(Box(LINK_HUB_COLLISION_SIZE), origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)))
    forearm.qc_collision(Box(JOINT_BOX_SIZE), origin=Origin(xyz=(0.0, 0.0, 0.0)))
    forearm.qc_collision(Box(JOINT_BOX_SIZE), origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)))
    forearm.inertial = Inertial.from_geometry(
        Box((FOREARM_LENGTH, 0.04, 0.036)),
        mass=0.95,
        origin=Origin(xyz=(FOREARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    wrist_pad = model.part("wrist_pad")
    if _has_cadquery():
        wrist_pad.visual(
            _mesh_or_fallback(
                _make_wrist_body_shape(),
                "wrist_body.obj",
                Box((WRIST_BODY_LENGTH + WRIST_MOUNT_THICKNESS + 0.048, 0.046, 0.048)),
            ),
            material="arm_silver",
        )
    else:
        wrist_pad.visual(
            Box((WRIST_BODY_LENGTH + WRIST_MOUNT_THICKNESS + 0.048, 0.046, 0.048)),
            origin=Origin(xyz=((WRIST_BODY_LENGTH + WRIST_MOUNT_THICKNESS) / 2.0, 0.0, 0.0)),
            material="arm_silver",
        )
    wrist_pad.visual(
        Box(PAD_SIZE), origin=Origin(xyz=(PAD_CENTER_X, 0.0, 0.0)), material="pad_black"
    )
    wrist_pad.collision(Box(WRIST_HUB_COLLISION_SIZE), origin=Origin(xyz=(0.0, 0.0, 0.0)))
    wrist_pad.collision(
        Box(WRIST_BEAM_SIZE), origin=Origin(xyz=(WRIST_BODY_LENGTH / 2.0, 0.0, 0.0))
    )
    wrist_pad.collision(Box(PAD_SIZE), origin=Origin(xyz=(PAD_CENTER_X, 0.0, 0.0)))
    wrist_pad.qc_collision(Box(JOINT_BOX_SIZE), origin=Origin(xyz=(0.0, 0.0, 0.0)))
    wrist_pad.inertial = Inertial.from_geometry(
        Box((PAD_CENTER_X + (PAD_SIZE[0] / 2.0), 0.06, 0.05)),
        mass=0.45,
        origin=Origin(xyz=((PAD_CENTER_X + (PAD_SIZE[0] / 2.0)) / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent="base",
        child="upper_arm",
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=1.35, effort=45.0, velocity=1.6),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent="upper_arm",
        child="forearm",
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.95, effort=30.0, velocity=1.8),
    )
    model.articulation(
        "wrist",
        ArticulationType.REVOLUTE,
        parent="forearm",
        child="wrist_pad",
        origin=Origin(xyz=(FOREARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.8, upper=0.95, effort=12.0, velocity=2.2),
    )

    return model


def _pose(ctx: TestContext, **joint_values):
    try:
        return ctx.pose(**joint_values)
    except TypeError:
        return ctx.pose(joint_values)


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_joint_origin_near_geometry(tol=0.02)
    ctx.check_joint_origin_near_physical_geometry(tol=0.02)
    ctx.check_no_overlaps(max_pose_samples=192, overlap_tol=0.0025, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap_xy("base", "upper_arm", min_overlap=0.03)
    ctx.expect_xy_distance("base", "upper_arm", max_dist=0.16)
    ctx.expect_xy_distance("upper_arm", "forearm", max_dist=0.28)
    ctx.expect_xy_distance("forearm", "wrist_pad", max_dist=0.24)

    ctx.expect_joint_motion_axis(
        "shoulder", "upper_arm", world_axis="z", direction="positive", min_delta=0.01
    )
    ctx.expect_joint_motion_axis(
        "elbow", "forearm", world_axis="z", direction="positive", min_delta=0.01
    )
    ctx.expect_joint_motion_axis(
        "wrist", "wrist_pad", world_axis="z", direction="positive", min_delta=0.01
    )

    with _pose(ctx, shoulder=0.55, elbow=0.95, wrist=-0.2):
        ctx.expect_above("forearm", "base", min_clearance=0.02)
        ctx.expect_xy_distance("wrist_pad", "base", max_dist=0.48)

    with _pose(ctx, shoulder=0.82, elbow=1.28, wrist=0.32):
        ctx.expect_above("forearm", "base", min_clearance=0.02)
        ctx.expect_above("wrist_pad", "base", min_clearance=0.05)
        ctx.expect_xy_distance("wrist_pad", "base", max_dist=0.35)
        ctx.expect_aabb_gap_z("wrist_pad", "base", max_gap=0.5, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
