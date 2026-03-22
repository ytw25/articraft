from __future__ import annotations

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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

ARM_WIDTH = 0.020
ARM_HEIGHT = 0.012
HUB_RADIUS = 0.008
HUB_LENGTH = 0.018
BRACKET_DEPTH = 0.020
BRACKET_THICKNESS = 0.005
BRACKET_HEIGHT = 0.042
BRACKET_Y = 0.0125
COLLAR_Y = 0.0165

LOWER_ARM_LENGTH = 0.235
UPPER_ARM_LENGTH = 0.210


def _origin(
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    return Origin(xyz=xyz, rpy=rpy)


def _add_y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin(xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_x_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin(xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_hinge_bracket(
    part,
    *,
    joint_xyz: tuple[float, float, float],
    metal,
    plastic,
    plate_height: float = BRACKET_HEIGHT,
) -> None:
    x, y, z = joint_xyz
    bracket_x = x - (BRACKET_DEPTH / 2.0 + HUB_RADIUS)
    for sign in (-1.0, 1.0):
        side_y = y + sign * BRACKET_Y
        part.visual(
            Box((BRACKET_DEPTH, BRACKET_THICKNESS, plate_height)),
            origin=_origin((bracket_x, side_y, z)),
            material=metal,
        )
        _add_y_cylinder(
            part,
            radius=0.009,
            length=0.003,
            xyz=(bracket_x, y + sign * COLLAR_Y, z),
            material=plastic,
        )


def _add_pivot_hub(part, *, metal) -> None:
    _add_y_cylinder(
        part,
        radius=HUB_RADIUS,
        length=HUB_LENGTH,
        xyz=(0.0, 0.0, 0.0),
        material=metal,
    )


def _build_arm_link(part, *, length: float, metal, pivot_metal, plastic) -> None:
    beam_start = 0.010
    beam_end = length - 0.010
    beam_length = beam_end - beam_start
    beam_center = (beam_start + beam_end) / 2.0

    part.visual(
        Box((beam_length, ARM_WIDTH, ARM_HEIGHT)),
        origin=_origin((beam_center, 0.0, 0.0)),
        material=metal,
        name="beam",
    )
    part.visual(
        Box((0.038, 0.024, 0.018)),
        origin=_origin((0.019, 0.0, 0.0)),
        material=metal,
        name="root_block",
    )
    part.visual(
        Box((beam_length * 0.48, 0.012, 0.004)),
        origin=_origin((beam_center + 0.010, 0.0, 0.008)),
        material=pivot_metal,
        name="stiffener",
    )
    _add_pivot_hub(part, metal=pivot_metal)
    _add_hinge_bracket(part, joint_xyz=(length, 0.0, 0.0), metal=metal, plastic=plastic)


def _expect_hinge_mount(ctx: TestContext, child: str, parent: str) -> None:
    ctx.expect_aabb_contact(child, parent)
    ctx.expect_aabb_overlap(child, parent, axes="yz", min_overlap=0.006)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_mic_boom", assets=ASSETS)

    matte_metal = model.material("matte_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    pivot_metal = model.material("pivot_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.48, 0.50, 0.53, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        Box((0.060, 0.050, 0.018)),
        origin=_origin((0.010, 0.0, 0.112)),
        material=matte_metal,
        name="top_jaw",
    )
    clamp_base.visual(
        Box((0.024, 0.050, 0.120)),
        origin=_origin((-0.022, 0.0, 0.060)),
        material=matte_metal,
        name="rear_spine",
    )
    clamp_base.visual(
        Box((0.050, 0.050, 0.014)),
        origin=_origin((0.000, 0.0, 0.018)),
        material=matte_metal,
        name="lower_jaw",
    )
    _add_y_cylinder(
        clamp_base,
        radius=0.0035,
        length=0.068,
        xyz=(0.006, 0.0, 0.018),
        material=black_plastic,
        name="clamp_handle",
    )
    clamp_base.visual(
        Cylinder(radius=0.011, length=0.048),
        origin=_origin((0.006, 0.0, 0.045)),
        material=pivot_metal,
        name="screw_housing",
    )
    clamp_base.visual(
        Cylinder(radius=0.0065, length=0.080),
        origin=_origin((0.006, 0.0, 0.064)),
        material=satin_steel,
        name="screw_stem",
    )
    clamp_base.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=_origin((0.006, 0.0, 0.100)),
        material=rubber,
        name="desk_pad",
    )
    clamp_base.visual(
        Box((0.028, 0.040, 0.066)),
        origin=_origin((0.006, 0.0, 0.154)),
        material=matte_metal,
        name="mount_block",
    )
    _add_hinge_bracket(
        clamp_base,
        joint_xyz=(0.038, 0.0, 0.160),
        metal=matte_metal,
        plastic=black_plastic,
        plate_height=0.046,
    )
    clamp_base.inertial = Inertial.from_geometry(
        Box((0.080, 0.060, 0.190)),
        mass=3.2,
        origin=_origin((0.000, 0.0, 0.095)),
    )

    lower_arm = model.part("lower_arm")
    _build_arm_link(
        lower_arm,
        length=LOWER_ARM_LENGTH,
        metal=matte_metal,
        pivot_metal=pivot_metal,
        plastic=black_plastic,
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_ARM_LENGTH, 0.030, 0.050)),
        mass=0.70,
        origin=_origin((LOWER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    _build_arm_link(
        upper_arm,
        length=UPPER_ARM_LENGTH,
        metal=matte_metal,
        pivot_metal=pivot_metal,
        plastic=black_plastic,
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((UPPER_ARM_LENGTH, 0.028, 0.048)),
        mass=0.55,
        origin=_origin((UPPER_ARM_LENGTH / 2.0, 0.0, 0.0)),
    )

    mic_head = model.part("mic_head")
    _add_pivot_hub(mic_head, metal=pivot_metal)
    mic_head.visual(
        Box((0.034, 0.024, 0.018)),
        origin=_origin((0.017, 0.0, -0.003)),
        material=matte_metal,
        name="wrist_block",
    )
    _add_x_cylinder(
        mic_head,
        radius=0.015,
        length=0.012,
        xyz=(0.036, 0.0, -0.004),
        material=black_plastic,
        name="adjustment_collar",
    )
    _add_x_cylinder(
        mic_head,
        radius=0.020,
        length=0.074,
        xyz=(0.081, 0.0, -0.006),
        material=matte_metal,
        name="microphone_body",
    )
    _add_x_cylinder(
        mic_head,
        radius=0.019,
        length=0.022,
        xyz=(0.129, 0.0, -0.006),
        material=satin_steel,
        name="grille",
    )
    _add_x_cylinder(
        mic_head,
        radius=0.012,
        length=0.010,
        xyz=(0.046, 0.0, -0.004),
        material=pivot_metal,
        name="neck_sleeve",
    )
    mic_head.inertial = Inertial.from_geometry(
        Box((0.145, 0.050, 0.050)),
        mass=0.42,
        origin=_origin((0.073, 0.0, -0.004)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent="clamp_base",
        child="lower_arm",
        origin=_origin((0.038, 0.0, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.4,
            lower=-0.85,
            upper=1.10,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent="lower_arm",
        child="upper_arm",
        origin=_origin((LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.8,
            lower=-0.10,
            upper=1.65,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent="upper_arm",
        child="mic_head",
        origin=_origin((UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-1.05,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected(use="visual")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    _expect_hinge_mount(ctx, "lower_arm", "clamp_base")
    _expect_hinge_mount(ctx, "upper_arm", "lower_arm")
    _expect_hinge_mount(ctx, "mic_head", "upper_arm")
    ctx.expect_aabb_overlap("lower_arm", "clamp_base", axes="z", min_overlap=0.020)
    ctx.expect_aabb_overlap("upper_arm", "lower_arm", axes="yz", min_overlap=0.010)
    ctx.expect_aabb_overlap("mic_head", "upper_arm", axes="yz", min_overlap=0.010)

    ctx.expect_joint_motion_axis(
        "shoulder_pitch",
        "lower_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.050,
    )
    ctx.expect_joint_motion_axis(
        "elbow_pitch",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.040,
    )
    ctx.expect_joint_motion_axis(
        "wrist_pitch",
        "mic_head",
        world_axis="z",
        direction="positive",
        min_delta=0.015,
    )

    with ctx.pose(shoulder_pitch=0.95):
        _expect_hinge_mount(ctx, "lower_arm", "clamp_base")
        ctx.expect_aabb_overlap("lower_arm", "clamp_base", axes="z", min_overlap=0.015)

    with ctx.pose(elbow_pitch=1.35):
        _expect_hinge_mount(ctx, "upper_arm", "lower_arm")
        ctx.expect_aabb_overlap("upper_arm", "lower_arm", axes="yz", min_overlap=0.008)

    with ctx.pose(wrist_pitch=0.80):
        _expect_hinge_mount(ctx, "mic_head", "upper_arm")
        ctx.expect_aabb_overlap("mic_head", "upper_arm", axes="yz", min_overlap=0.008)

    with ctx.pose(shoulder_pitch=0.65, elbow_pitch=1.20, wrist_pitch=-0.55):
        _expect_hinge_mount(ctx, "lower_arm", "clamp_base")
        _expect_hinge_mount(ctx, "upper_arm", "lower_arm")
        _expect_hinge_mount(ctx, "mic_head", "upper_arm")
        ctx.expect_aabb_overlap("upper_arm", "lower_arm", axes="y", min_overlap=0.012)
        ctx.expect_aabb_overlap("mic_head", "upper_arm", axes="yz", min_overlap=0.006)

    return ctx.report()

# >>> USER_CODE_END

object_model = build_object_model()
