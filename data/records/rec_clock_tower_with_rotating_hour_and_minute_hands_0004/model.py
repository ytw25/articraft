from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return "/"


os.getcwd = _safe_getcwd
os.chdir(_safe_getcwd())
if not os.path.isabs(__file__):
    __file__ = os.path.join(_safe_getcwd(), __file__)

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

TOWER_WIDTH = 0.48
TOWER_DEPTH = 0.30
TOWER_HEIGHT = 1.58
FRONT_FACADE_DEPTH = 0.08
FRONT_Y = TOWER_DEPTH * 0.5
FACADE_CENTER_Y = FRONT_Y - FRONT_FACADE_DEPTH * 0.5

CLOCK_CENTER_Z = 1.17
CLOCK_REVEAL_RADIUS = 0.152
CLOCK_INNER_RADIUS = 0.124
CLOCK_FACE_RADIUS = 0.116
CLOCK_REVEAL_DEPTH = 0.018
CLOCK_SEAT_DEPTH = 0.030
CLOCK_FACE_THICKNESS = 0.012

CURTAIN_CENTER_Z = 0.60
CURTAIN_WIDTH = 0.304
CURTAIN_HEIGHT = 0.392
CURTAIN_RECESS_Y = FRONT_Y - 0.026


def _facade_band_center(z_min: float, z_max: float) -> tuple[float, float]:
    return ((z_min + z_max) * 0.5, z_max - z_min)


def _add_segmented_annulus(
    part,
    *,
    outer_radius: float,
    inner_radius: float,
    center_z: float,
    y_center: float,
    depth: float,
    material,
    prefix: str,
    segments: int = 16,
) -> None:
    mid_radius = 0.5 * (outer_radius + inner_radius)
    radial_thickness = outer_radius - inner_radius
    tangent_length = 2.0 * mid_radius * math.sin(math.pi / segments) * 1.02
    for index in range(segments):
        angle = math.tau * index / segments
        part.visual(
            Box((tangent_length, depth, radial_thickness)),
            origin=Origin(
                xyz=(
                    mid_radius * math.cos(angle),
                    y_center,
                    center_z + mid_radius * math.sin(angle),
                ),
                rpy=(0.0, math.pi * 0.5 - angle, 0.0),
            ),
            material=material,
            name=f"{prefix}_{index}",
        )


def _add_rect_frame(
    part,
    *,
    width: float,
    height: float,
    depth: float,
    frame_thickness: float,
    y_center: float,
    center_z: float,
    material,
) -> None:
    side_height = height
    top_bottom_width = width - 2.0 * frame_thickness
    half_width = width * 0.5 - frame_thickness * 0.5
    half_height = height * 0.5 - frame_thickness * 0.5
    part.visual(
        Box((frame_thickness, depth, side_height)),
        origin=Origin(xyz=(-half_width, y_center, center_z)),
        material=material,
        name="frame_left",
    )
    part.visual(
        Box((frame_thickness, depth, side_height)),
        origin=Origin(xyz=(half_width, y_center, center_z)),
        material=material,
        name="frame_right",
    )
    part.visual(
        Box((top_bottom_width, depth, frame_thickness)),
        origin=Origin(xyz=(0.0, y_center, center_z + half_height)),
        material=material,
        name="frame_top",
    )
    part.visual(
        Box((top_bottom_width, depth, frame_thickness)),
        origin=Origin(xyz=(0.0, y_center, center_z - half_height)),
        material=material,
        name="frame_bottom",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modernist_clock_tower")

    concrete = model.material("concrete", rgba=(0.66, 0.66, 0.64, 1.0))
    off_white = model.material("off_white", rgba=(0.93, 0.93, 0.90, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    anodized = model.material("anodized", rgba=(0.24, 0.26, 0.28, 1.0))
    glass = model.material("glass", rgba=(0.64, 0.77, 0.86, 0.42))

    tower = model.part("tower")
    tower.visual(
        Box((TOWER_WIDTH, TOWER_DEPTH - FRONT_FACADE_DEPTH, TOWER_HEIGHT)),
        origin=Origin(xyz=(0.0, -FRONT_FACADE_DEPTH * 0.5, TOWER_HEIGHT * 0.5)),
        material=concrete,
        name="rear_mass",
    )
    column_width = 0.5 * (TOWER_WIDTH - 2.0 * CLOCK_REVEAL_RADIUS)
    column_x = 0.5 * TOWER_WIDTH - 0.5 * column_width
    tower.visual(
        Box((column_width, FRONT_FACADE_DEPTH, TOWER_HEIGHT)),
        origin=Origin(xyz=(-column_x, FACADE_CENTER_Y, TOWER_HEIGHT * 0.5)),
        material=concrete,
        name="left_column",
    )
    tower.visual(
        Box((column_width, FRONT_FACADE_DEPTH, TOWER_HEIGHT)),
        origin=Origin(xyz=(column_x, FACADE_CENTER_Y, TOWER_HEIGHT * 0.5)),
        material=concrete,
        name="right_column",
    )
    top_center_z, top_height = _facade_band_center(
        CLOCK_CENTER_Z + CLOCK_REVEAL_RADIUS,
        TOWER_HEIGHT,
    )
    mid_center_z, mid_height = _facade_band_center(
        CURTAIN_CENTER_Z + CURTAIN_HEIGHT * 0.5,
        CLOCK_CENTER_Z - CLOCK_REVEAL_RADIUS,
    )
    bottom_center_z, bottom_height = _facade_band_center(
        0.0,
        CURTAIN_CENTER_Z - CURTAIN_HEIGHT * 0.5,
    )
    tower.visual(
        Box((2.0 * CLOCK_REVEAL_RADIUS, FRONT_FACADE_DEPTH, top_height)),
        origin=Origin(xyz=(0.0, FACADE_CENTER_Y, top_center_z)),
        material=concrete,
        name="top_band",
    )
    tower.visual(
        Box((2.0 * CLOCK_REVEAL_RADIUS, FRONT_FACADE_DEPTH, mid_height)),
        origin=Origin(xyz=(0.0, FACADE_CENTER_Y, mid_center_z)),
        material=concrete,
        name="mid_band",
    )
    tower.visual(
        Box((2.0 * CLOCK_REVEAL_RADIUS, FRONT_FACADE_DEPTH, bottom_height)),
        origin=Origin(xyz=(0.0, FACADE_CENTER_Y, bottom_center_z)),
        material=concrete,
        name="bottom_band",
    )
    tower.inertial = Inertial.from_geometry(
        Box((TOWER_WIDTH, TOWER_DEPTH, TOWER_HEIGHT)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, TOWER_HEIGHT * 0.5)),
    )

    clock_reveal = model.part("clock_reveal")
    _add_segmented_annulus(
        clock_reveal,
        outer_radius=CLOCK_REVEAL_RADIUS,
        inner_radius=CLOCK_INNER_RADIUS,
        center_z=0.0,
        y_center=0.0,
        depth=CLOCK_SEAT_DEPTH,
        material=concrete,
        prefix="reveal_outer",
    )
    _add_segmented_annulus(
        clock_reveal,
        outer_radius=CLOCK_INNER_RADIUS,
        inner_radius=CLOCK_FACE_RADIUS + 0.006,
        center_z=0.0,
        y_center=-0.009,
        depth=0.012,
        material=concrete,
        prefix="reveal_inner",
        segments=12,
    )
    clock_reveal.inertial = Inertial.from_geometry(
        Box((0.31, 0.04, 0.31)),
        mass=4.0,
        origin=Origin(),
    )

    curtain_wall = model.part("curtain_wall")
    _add_rect_frame(
        curtain_wall,
        width=CURTAIN_WIDTH,
        height=CURTAIN_HEIGHT,
        depth=0.012,
        frame_thickness=0.038,
        y_center=0.0,
        center_z=0.0,
        material=anodized,
    )
    curtain_wall.visual(
        Box((0.014, 0.012, 0.316)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=anodized,
        name="center_mullion",
    )
    curtain_wall.visual(
        Box((0.228, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=anodized,
        name="transom",
    )
    curtain_wall.visual(
        Box((0.228, 0.004, 0.316)),
        origin=Origin(xyz=(0.0, -0.008, 0.0)),
        material=glass,
        name="glass_panel",
    )
    curtain_wall.inertial = Inertial.from_geometry(
        Box((CURTAIN_WIDTH, 0.02, CURTAIN_HEIGHT)),
        mass=6.0,
        origin=Origin(),
    )

    clock_panel = model.part("clock_panel")
    clock_panel.visual(
        Cylinder(radius=CLOCK_FACE_RADIUS, length=CLOCK_FACE_THICKNESS),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=off_white,
        name="clock_face",
    )
    clock_panel.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="boss_base",
    )
    clock_panel.visual(
        Cylinder(radius=0.009, length=0.0028),
        origin=Origin(xyz=(0.0, 0.0114, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="boss_tip",
    )
    clock_panel.inertial = Inertial.from_geometry(
        Box((0.25, 0.04, 0.25)),
        mass=2.0,
        origin=Origin(),
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Cylinder(radius=0.016, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0108, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="hour_collar",
    )
    hour_hand.visual(
        Box((0.016, 0.0018, 0.072)),
        origin=Origin(xyz=(0.0, 0.0112, 0.020)),
        material=charcoal,
        name="hour_bar",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.03, 0.01, 0.08)),
        mass=0.2,
        origin=Origin(xyz=(0.0, 0.0108, 0.020)),
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Cylinder(radius=0.011, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0131, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="minute_collar",
    )
    minute_hand.visual(
        Box((0.012, 0.0015, 0.102)),
        origin=Origin(xyz=(0.0, 0.0135, 0.029)),
        material=charcoal,
        name="minute_bar",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.03, 0.01, 0.11)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0131, 0.029)),
    )

    model.articulation(
        "tower_to_clock_reveal",
        ArticulationType.FIXED,
        parent=tower,
        child=clock_reveal,
        origin=Origin(
            xyz=(
                0.0,
                FRONT_Y - CLOCK_SEAT_DEPTH * 0.5,
                CLOCK_CENTER_Z,
            )
        ),
    )
    model.articulation(
        "tower_to_curtain_wall",
        ArticulationType.FIXED,
        parent=tower,
        child=curtain_wall,
        origin=Origin(
            xyz=(
                0.0,
                CURTAIN_RECESS_Y,
                CURTAIN_CENTER_Z,
            )
        ),
    )
    model.articulation(
        "clock_reveal_to_clock_panel",
        ArticulationType.FIXED,
        parent=clock_reveal,
        child=clock_panel,
        origin=Origin(
            xyz=(
                0.0,
                -0.021,
                0.0,
            )
        ),
    )
    model.articulation(
        "hour_hand_rotate",
        ArticulationType.CONTINUOUS,
        parent=clock_panel,
        child=hour_hand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=2.0),
    )
    model.articulation(
        "minute_hand_rotate",
        ArticulationType.CONTINUOUS,
        parent=clock_panel,
        child=minute_hand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root="/")
    tower = object_model.get_part("tower")
    clock_reveal = object_model.get_part("clock_reveal")
    clock_panel = object_model.get_part("clock_panel")
    curtain_wall = object_model.get_part("curtain_wall")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_joint = object_model.get_articulation("hour_hand_rotate")
    minute_joint = object_model.get_articulation("minute_hand_rotate")

    clock_face = clock_panel.get_visual("clock_face")
    reveal_outer = clock_reveal.get_visual("reveal_outer_0")
    reveal_outer_left = clock_reveal.get_visual("reveal_outer_8")
    reveal_outer_top = clock_reveal.get_visual("reveal_outer_4")
    reveal_outer_bottom = clock_reveal.get_visual("reveal_outer_12")
    boss_base = clock_panel.get_visual("boss_base")
    boss_tip = clock_panel.get_visual("boss_tip")
    left_column = tower.get_visual("left_column")
    right_column = tower.get_visual("right_column")
    top_band = tower.get_visual("top_band")
    mid_band = tower.get_visual("mid_band")
    bottom_band = tower.get_visual("bottom_band")
    frame_left = curtain_wall.get_visual("frame_left")
    frame_right = curtain_wall.get_visual("frame_right")
    frame_top = curtain_wall.get_visual("frame_top")
    frame_bottom = curtain_wall.get_visual("frame_bottom")
    center_mullion = curtain_wall.get_visual("center_mullion")
    transom = curtain_wall.get_visual("transom")
    glass_panel = curtain_wall.get_visual("glass_panel")
    hour_collar = hour_hand.get_visual("hour_collar")
    hour_bar = hour_hand.get_visual("hour_bar")
    minute_collar = minute_hand.get_visual("minute_collar")
    minute_bar = minute_hand.get_visual("minute_bar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
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
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_contact(clock_reveal, tower, elem_a=reveal_outer)
    ctx.expect_contact(clock_reveal, tower, elem_a=reveal_outer, elem_b=right_column)
    ctx.expect_contact(clock_reveal, tower, elem_a=reveal_outer_left, elem_b=left_column)
    ctx.expect_contact(clock_reveal, tower, elem_a=reveal_outer_top, elem_b=top_band)
    ctx.expect_contact(clock_reveal, tower, elem_a=reveal_outer_bottom, elem_b=mid_band)
    ctx.expect_within(clock_reveal, tower, axes="xz", inner_elem=reveal_outer)
    ctx.expect_origin_distance(clock_reveal, tower, axes="x", max_dist=0.01)
    ctx.expect_origin_distance(clock_panel, clock_reveal, axes="xz", max_dist=0.001)
    ctx.expect_within(clock_panel, clock_reveal, axes="xz", inner_elem=clock_face)
    ctx.expect_gap(
        clock_reveal,
        clock_panel,
        axis="y",
        positive_elem=reveal_outer,
        negative_elem=clock_face,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_origin_distance(clock_panel, tower, axes="x", max_dist=0.01)
    ctx.expect_contact(curtain_wall, tower, elem_a=frame_left)
    ctx.expect_contact(curtain_wall, tower, elem_a=frame_left, elem_b=left_column)
    ctx.expect_contact(curtain_wall, tower, elem_a=frame_right, elem_b=right_column)
    ctx.expect_contact(curtain_wall, tower, elem_a=frame_top, elem_b=mid_band)
    ctx.expect_contact(curtain_wall, tower, elem_a=frame_bottom, elem_b=bottom_band)
    ctx.expect_within(curtain_wall, tower, axes="xz", inner_elem=frame_left)
    ctx.expect_within(curtain_wall, curtain_wall, axes="xz", inner_elem=glass_panel)
    ctx.expect_contact(curtain_wall, curtain_wall, elem_a=glass_panel, elem_b=frame_left)
    ctx.expect_contact(curtain_wall, curtain_wall, elem_a=glass_panel, elem_b=frame_right)
    ctx.expect_contact(curtain_wall, curtain_wall, elem_a=glass_panel, elem_b=center_mullion)
    ctx.expect_contact(curtain_wall, curtain_wall, elem_a=glass_panel, elem_b=transom)
    ctx.expect_gap(clock_reveal, curtain_wall, axis="z", min_gap=0.18, max_gap=0.34)
    ctx.expect_contact(hour_hand, clock_panel, elem_a=hour_collar, elem_b=boss_base)
    ctx.expect_contact(minute_hand, clock_panel, elem_a=minute_collar, elem_b=boss_tip)
    ctx.expect_within(hour_hand, clock_panel, axes="xz", inner_elem=hour_bar, outer_elem=clock_face)
    ctx.expect_within(minute_hand, clock_panel, axes="xz", inner_elem=minute_bar, outer_elem=clock_face)
    with ctx.pose({hour_joint: math.pi / 3.0, minute_joint: -math.pi / 2.0}):
        ctx.expect_within(hour_hand, clock_panel, axes="xz", inner_elem=hour_bar, outer_elem=clock_face)
        ctx.expect_within(minute_hand, clock_panel, axes="xz", inner_elem=minute_bar, outer_elem=clock_face)
        ctx.expect_contact(hour_hand, clock_panel, elem_a=hour_collar, elem_b=boss_base)
        ctx.expect_contact(minute_hand, clock_panel, elem_a=minute_collar, elem_b=boss_tip)
    with ctx.pose({hour_joint: -math.pi / 4.0, minute_joint: math.pi}):
        ctx.expect_within(hour_hand, clock_panel, axes="xz", inner_elem=hour_bar, outer_elem=clock_face)
        ctx.expect_within(minute_hand, clock_panel, axes="xz", inner_elem=minute_bar, outer_elem=clock_face)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
