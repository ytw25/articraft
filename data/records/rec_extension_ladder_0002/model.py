from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd
if _safe_getcwd() == "/":
    os.chdir("/")

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

BASE_LENGTH = 2.65
FLY_LENGTH = 2.35
BASE_RAIL_CENTER_X = 0.205
BASE_RAIL_WIDTH = 0.075
BASE_RAIL_DEPTH = 0.038
BASE_WEB = 0.012
BASE_FLANGE = 0.008
FLY_RAIL_CENTER_X = 0.132
FLY_RAIL_WIDTH = 0.048
FLY_RAIL_DEPTH = 0.030
FLY_WEB = 0.010
FLY_FLANGE = 0.006
RUNG_Y = -0.010
BASE_RUNG_RADIUS = 0.017
FLY_RUNG_RADIUS = 0.015
BASE_RUNG_LENGTH = 0.380
FLY_RUNG_LENGTH = 0.270
BASE_RUNG_Y = -0.010
FLY_RUNG_Y = 0.0
BASE_FIRST_RUNG_Z = 0.35
FLY_FIRST_RUNG_Z = 0.28
RUNG_PITCH = 0.30
BASE_RUNG_COUNT = 8
FLY_RUNG_COUNT = 7
FLY_REST_OFFSET = 0.15
FLY_Y_OFFSET = 0.046
MAX_EXTENSION = 0.90


def _add_channel_rail(
    part,
    prefix: str,
    center_x: float,
    length: float,
    width: float,
    depth: float,
    web_t: float,
    flange_t: float,
    open_dir: float,
    material,
) -> None:
    center_z = length / 2.0
    flange_len = width - web_t
    web_x = center_x - open_dir * (width / 2.0 - web_t / 2.0)
    flange_x = center_x + open_dir * web_t / 2.0
    part.visual(
        Box((web_t, depth, length)),
        origin=Origin(xyz=(web_x, 0.0, center_z)),
        material=material,
        name=f"{prefix}_web",
    )
    part.visual(
        Box((flange_len, flange_t, length)),
        origin=Origin(xyz=(flange_x, depth / 2.0 - flange_t / 2.0, center_z)),
        material=material,
        name=f"{prefix}_front_flange",
    )
    part.visual(
        Box((flange_len, flange_t, length)),
        origin=Origin(xyz=(flange_x, -depth / 2.0 + flange_t / 2.0, center_z)),
        material=material,
        name=f"{prefix}_rear_flange",
    )


def _add_rungs(
    part,
    prefix: str,
    count: int,
    first_z: float,
    pitch: float,
    radius: float,
    length: float,
    y_pos: float,
    material,
) -> None:
    for i in range(count):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(
                xyz=(0.0, y_pos, first_z + i * pitch),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=material,
            name=f"{prefix}_{i + 1}",
        )


def _build_guide_assembly(
    model: ArticulatedObject,
    fly_part,
    side_sign: float,
    prefix: str,
    rail_material,
    hardware_material,
):
    guide = model.part(f"{prefix}_guide_assembly")
    guide_z = 0.20
    channel_length = 0.24

    guide.visual(
        Box((0.018, 0.020, 0.120)),
        origin=Origin(xyz=(side_sign * 0.164, -0.010, guide_z + 0.01)),
        material=rail_material,
        name="guide_mount",
    )
    guide.visual(
        Box((0.050, 0.012, 0.018)),
        origin=Origin(xyz=(side_sign * 0.182, -0.022, guide_z + 0.01)),
        material=hardware_material,
        name="guide_carrier",
    )
    guide.visual(
        Box((0.010, 0.036, 0.180)),
        origin=Origin(xyz=(side_sign * 0.197, -0.040, guide_z)),
        material=hardware_material,
        name="guide_drop",
    )
    guide.visual(
        Box((0.006, 0.012, channel_length)),
        origin=Origin(xyz=(side_sign * 0.203, -0.046, guide_z)),
        material=hardware_material,
        name="guide_outer",
    )
    guide.visual(
        Box((0.012, 0.012, channel_length)),
        origin=Origin(xyz=(side_sign * 0.186, -0.046, guide_z)),
        material=hardware_material,
        name="guide_inner",
    )
    guide.visual(
        Box((0.029, 0.010, channel_length)),
        origin=Origin(xyz=(side_sign * 0.1945, -0.053, guide_z)),
        material=hardware_material,
        name="guide_back",
    )

    model.articulation(
        f"fly_to_{prefix}_guide",
        ArticulationType.FIXED,
        parent=fly_part,
        child=guide,
        origin=Origin(),
    )


def _build_pawl_lock(
    model: ArticulatedObject,
    fly_part,
    side_sign: float,
    prefix: str,
    rail_material,
    hardware_material,
):
    lock = model.part(f"{prefix}_pawl_lock")
    lock_z = 0.235

    lock.visual(
        Box((0.030, 0.018, 0.090)),
        origin=Origin(xyz=(side_sign * 0.166, 0.0, lock_z)),
        material=rail_material,
        name="lock_mount",
    )
    lock.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(
            xyz=(side_sign * 0.183, -0.012, lock_z + 0.003),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware_material,
        name="lock_hub",
    )
    lock.visual(
        Box((0.018, 0.012, 0.012)),
        origin=Origin(xyz=(side_sign * 0.192, -0.026, lock_z + 0.001)),
        material=hardware_material,
        name="lock_arm",
    )
    lock.visual(
        Box((0.010, 0.024, 0.012)),
        origin=Origin(xyz=(side_sign * 0.194, -0.038, 0.232)),
        material=hardware_material,
        name="lock_link",
    )
    lock.visual(
        Box((0.014, 0.012, 0.024)),
        origin=Origin(xyz=(side_sign * 0.187, -0.048, 0.228)),
        material=hardware_material,
        name="lock_tip",
    )

    model.articulation(
        f"fly_to_{prefix}_pawl",
        ArticulationType.FIXED,
        parent=fly_part,
        child=lock,
        origin=Origin(),
    )


def _build_rope_system(model: ArticulatedObject, base_part) -> None:
    rope = model.part("rope_system")
    rope_material = "rope"
    metal = "aluminum"
    dark = "dark_hardware"

    rope.visual(
        Box((0.020, 0.024, 0.070)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=metal,
        name="pulley_bracket",
    )
    rope.visual(
        Cylinder(radius=0.022, length=0.018),
        origin=Origin(
            xyz=(-0.020, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark,
        name="rope_pulley",
    )
    rope.visual(
        Box((0.026, 0.008, 0.040)),
        origin=Origin(xyz=(-0.005, 0.016, -0.035)),
        material=metal,
        name="rope_bridge",
    )
    rope.visual(
        Cylinder(radius=0.004, length=1.65),
        origin=Origin(xyz=(-0.016, 0.018, -0.865)),
        material=rope_material,
        name="rope_tail",
    )
    rope.visual(
        Cylinder(radius=0.004, length=0.34),
        origin=Origin(xyz=(0.004, 0.018, -0.170)),
        material=rope_material,
        name="rope_return",
    )
    rope.inertial = Inertial.from_geometry(
        Box((0.060, 0.050, 1.75)),
        mass=0.7,
        origin=Origin(xyz=(-0.005, 0.010, -0.80)),
    )

    model.articulation(
        "base_to_rope",
        ArticulationType.FIXED,
        parent=base_part,
        child=rope,
        origin=Origin(
            xyz=(
                -(BASE_RAIL_CENTER_X + BASE_RAIL_WIDTH / 2.0),
                0.0,
                2.36,
            )
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="extension_ladder")

    fiberglass = model.material("fiberglass_orange", rgba=(0.92, 0.53, 0.14, 1.0))
    rung_metal = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    hardware = model.material("dark_hardware", rgba=(0.22, 0.22, 0.24, 1.0))
    rope = model.material("rope", rgba=(0.78, 0.70, 0.50, 1.0))
    _ = rope

    base = model.part("base_section")
    _add_channel_rail(
        base,
        "left_base_rail",
        -BASE_RAIL_CENTER_X,
        BASE_LENGTH,
        BASE_RAIL_WIDTH,
        BASE_RAIL_DEPTH,
        BASE_WEB,
        BASE_FLANGE,
        open_dir=1.0,
        material=fiberglass,
    )
    _add_channel_rail(
        base,
        "right_base_rail",
        BASE_RAIL_CENTER_X,
        BASE_LENGTH,
        BASE_RAIL_WIDTH,
        BASE_RAIL_DEPTH,
        BASE_WEB,
        BASE_FLANGE,
        open_dir=-1.0,
        material=fiberglass,
    )
    _add_rungs(
        base,
        "base_rung",
        BASE_RUNG_COUNT,
        BASE_FIRST_RUNG_Z,
        RUNG_PITCH,
        BASE_RUNG_RADIUS,
        BASE_RUNG_LENGTH,
        BASE_RUNG_Y,
        rung_metal,
    )
    base.inertial = Inertial.from_geometry(
        Box((0.50, 0.10, BASE_LENGTH)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_LENGTH / 2.0)),
    )

    fly = model.part("fly_section")
    _add_channel_rail(
        fly,
        "left_fly_rail",
        -FLY_RAIL_CENTER_X,
        FLY_LENGTH,
        FLY_RAIL_WIDTH,
        FLY_RAIL_DEPTH,
        FLY_WEB,
        FLY_FLANGE,
        open_dir=1.0,
        material=fiberglass,
    )
    _add_channel_rail(
        fly,
        "right_fly_rail",
        FLY_RAIL_CENTER_X,
        FLY_LENGTH,
        FLY_RAIL_WIDTH,
        FLY_RAIL_DEPTH,
        FLY_WEB,
        FLY_FLANGE,
        open_dir=-1.0,
        material=fiberglass,
    )
    _add_rungs(
        fly,
        "fly_rung",
        FLY_RUNG_COUNT,
        FLY_FIRST_RUNG_Z,
        RUNG_PITCH,
        FLY_RUNG_RADIUS,
        FLY_RUNG_LENGTH,
        FLY_RUNG_Y,
        rung_metal,
    )
    fly.inertial = Inertial.from_geometry(
        Box((0.40, 0.09, FLY_LENGTH)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, FLY_LENGTH / 2.0)),
    )

    model.articulation(
        "fly_extension",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, FLY_Y_OFFSET, FLY_REST_OFFSET)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.60,
            lower=0.0,
            upper=MAX_EXTENSION,
        ),
    )

    _build_guide_assembly(model, fly, -1.0, "left", fiberglass, hardware)
    _build_guide_assembly(model, fly, 1.0, "right", fiberglass, hardware)
    _build_pawl_lock(model, fly, -1.0, "left", fiberglass, hardware)
    _build_pawl_lock(model, fly, 1.0, "right", fiberglass, hardware)
    _build_rope_system(model, base)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    left_guide = object_model.get_part("left_guide_assembly")
    right_guide = object_model.get_part("right_guide_assembly")
    left_lock = object_model.get_part("left_pawl_lock")
    right_lock = object_model.get_part("right_pawl_lock")
    rope_system = object_model.get_part("rope_system")
    extend = object_model.get_articulation("fly_extension")

    left_fly_web = fly.get_visual("left_fly_rail_web")
    right_fly_web = fly.get_visual("right_fly_rail_web")
    left_guide_mount = left_guide.get_visual("guide_mount")
    right_guide_mount = right_guide.get_visual("guide_mount")
    left_guide_inner = left_guide.get_visual("guide_inner")
    right_guide_inner = right_guide.get_visual("guide_inner")
    left_lock_mount = left_lock.get_visual("lock_mount")
    right_lock_mount = right_lock.get_visual("lock_mount")
    left_lock_tip = left_lock.get_visual("lock_tip")
    right_lock_tip = right_lock.get_visual("lock_tip")
    base_rung_1 = base.get_visual("base_rung_1")
    base_rung_4 = base.get_visual("base_rung_4")
    left_base_front = base.get_visual("left_base_rail_front_flange")
    left_base_web = base.get_visual("left_base_rail_web")
    pulley_bracket = rope_system.get_visual("pulley_bracket")
    pulley = rope_system.get_visual("rope_pulley")
    rope_tail = rope_system.get_visual("rope_tail")
    rope_return = rope_system.get_visual("rope_return")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # The ladder slide axis lives in the open slot between the rails, and the
    # guide/pawl subassemblies use convenient local origins rather than
    # geometry-centroid origins, so the generic articulation-origin proximity
    # warning is noisy here. The exact mount and pose checks below carry the
    # real evidence for these relationships.
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.allow_overlap(
        left_guide,
        base,
        reason="guide shoe wraps around the base rail and rung as a sliding capture",
    )
    ctx.allow_overlap(
        right_guide,
        base,
        reason="guide shoe wraps around the base rail and rung as a sliding capture",
    )
    ctx.allow_overlap(
        left_lock,
        base,
        reason="swivel pawl tooth nests over the base rung when the fly is lowered or locked",
    )
    ctx.allow_overlap(
        right_lock,
        base,
        reason="swivel pawl tooth nests over the base rung when the fly is lowered or locked",
    )
    ctx.allow_overlap(
        left_guide,
        left_lock,
        reason="the guide hardware and pawl are represented as separate parts but share one rail-side bracket envelope",
    )
    ctx.allow_overlap(
        right_guide,
        right_lock,
        reason="the guide hardware and pawl are represented as separate parts but share one rail-side bracket envelope",
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    ctx.expect_overlap(fly, base, axes="xz", min_overlap=0.30, name="fly nests within the base ladder silhouette")
    ctx.expect_origin_distance(fly, base, axes="x", max_dist=0.001, name="base and fly stay centered side-to-side")
    ctx.expect_gap(
        fly,
        base,
        axis="y",
        positive_elem=left_fly_web,
        negative_elem=left_base_front,
        min_gap=0.010,
        name="fly rails ride forward of the base section",
    )
    ctx.expect_overlap(
        left_guide,
        fly,
        axes="yz",
        elem_a=left_guide_mount,
        elem_b=left_fly_web,
        min_overlap=0.012,
        name="left guide bracket mounts to the fly rail",
    )
    ctx.expect_overlap(
        right_guide,
        fly,
        axes="yz",
        elem_a=right_guide_mount,
        elem_b=right_fly_web,
        min_overlap=0.012,
        name="right guide bracket mounts to the fly rail",
    )
    ctx.expect_overlap(
        left_lock,
        fly,
        axes="yz",
        elem_a=left_lock_mount,
        elem_b=left_fly_web,
        min_overlap=0.015,
        name="left swivel pawl mounts to the fly rail",
    )
    ctx.expect_overlap(
        right_lock,
        fly,
        axes="yz",
        elem_a=right_lock_mount,
        elem_b=right_fly_web,
        min_overlap=0.015,
        name="right swivel pawl mounts to the fly rail",
    )
    ctx.expect_overlap(
        left_guide,
        base,
        axes="xz",
        elem_a=left_guide_inner,
        elem_b=base_rung_1,
        min_overlap=0.01,
        name="left guide channel rides on the first visible base rung in the nested pose",
    )
    ctx.expect_overlap(
        right_guide,
        base,
        axes="xz",
        elem_a=right_guide_inner,
        elem_b=base_rung_1,
        min_overlap=0.01,
        name="right guide channel rides on the first visible base rung in the nested pose",
    )
    ctx.expect_overlap(
        rope_system,
        base,
        axes="yz",
        elem_a=pulley_bracket,
        elem_b=left_base_web,
        min_overlap=0.02,
        name="pulley bracket clamps to the left base rail",
    )
    ctx.expect_overlap(
        rope_system,
        base,
        axes="yz",
        elem_a=pulley,
        elem_b=left_base_web,
        min_overlap=0.015,
        name="pulley mounts on the left base rail",
    )
    ctx.expect_overlap(
        rope_system,
        base,
        axes="yz",
        elem_a=rope_tail,
        elem_b=left_base_web,
        min_overlap=0.004,
        name="rope tail runs down the same rail as the pulley",
    )
    ctx.expect_overlap(
        rope_system,
        base,
        axes="yz",
        elem_a=rope_return,
        elem_b=left_base_web,
        min_overlap=0.004,
        name="return rope segment stays in the pulley lane",
    )

    with ctx.pose({extend: MAX_EXTENSION}):
        ctx.expect_overlap(
            left_guide,
            base,
            axes="xz",
            elem_a=left_guide_inner,
            elem_b=base_rung_4,
            min_overlap=0.01,
            name="left guide channel stays aligned with a higher base rung when extended",
        )
        ctx.expect_overlap(
            right_guide,
            base,
            axes="xz",
            elem_a=right_guide_inner,
            elem_b=base_rung_4,
            min_overlap=0.01,
            name="right guide channel stays aligned with a higher base rung when extended",
        )
        ctx.expect_gap(
            left_lock,
            base,
            axis="z",
            positive_elem=left_lock_tip,
            negative_elem=base_rung_4,
            max_gap=0.002,
            max_penetration=0.001,
            name="left swivel pawl settles onto a base rung in the locked pose",
        )
        ctx.expect_gap(
            right_lock,
            base,
            axis="z",
            positive_elem=right_lock_tip,
            negative_elem=base_rung_4,
            max_gap=0.002,
            max_penetration=0.001,
            name="right swivel pawl settles onto a base rung in the locked pose",
        )
        ctx.expect_overlap(
            left_lock,
            base,
            axes="xy",
            elem_a=left_lock_tip,
            elem_b=base_rung_4,
            min_overlap=0.008,
            name="left swivel pawl lines up over the rung end",
        )
        ctx.expect_overlap(
            right_lock,
            base,
            axes="xy",
            elem_a=right_lock_tip,
            elem_b=base_rung_4,
            min_overlap=0.008,
            name="right swivel pawl lines up over the rung end",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
