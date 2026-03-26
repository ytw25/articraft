from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

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
    ValidationError,
)

ASSET_ROOT = "/"
_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir(ASSET_ROOT)
        return ASSET_ROOT


os.getcwd = _safe_getcwd
os.chdir(_safe_getcwd())

BASE_LENGTH = 1.60
BASE_WIDTH = 1.05
BASE_HEIGHT = 0.08
BASE_LINK_Z = 0.12

PLATFORM_LENGTH = 1.50
PLATFORM_WIDTH = 1.00
PLATFORM_THICKNESS = 0.08
PLATFORM_LINK_LOCAL_Z = -0.08

LINK_LENGTH = 1.10
LINK_WIDTH = 0.024
LINK_DEPTH = 0.060
PIN_RADIUS = 0.028
PIN_LENGTH = 0.050
CENTER_PIN_LENGTH = 0.080

PAIR_Y_FRONT = -0.29
PAIR_Y_REAR = 0.29
PAIR_LINK_OFFSET = 0.014

LEFT_FIXED_X = -0.54
CLOSED_SCISSOR_HEIGHT = 0.22
OPEN_SCISSOR_HEIGHT = 0.72

TRACK_LENGTH = 0.40
TRACK_WIDTH = 0.090
TRACK_HEIGHT = 0.036

PLATFORM_CURB_HEIGHT = 0.085
PLATFORM_CURB_THICKNESS = 0.045


def _horizontal_span(height: float) -> float:
    return math.sqrt(max(LINK_LENGTH**2 - height**2, 0.0))


def _vector_angle(height: float) -> float:
    return math.atan2(height, _horizontal_span(height))


CLOSED_LINK_ANGLE = _vector_angle(CLOSED_SCISSOR_HEIGHT)
OPEN_LINK_ANGLE = _vector_angle(OPEN_SCISSOR_HEIGHT)
LINK_OPEN_DELTA = OPEN_LINK_ANGLE - CLOSED_LINK_ANGLE

RIGHT_SLIDE_X_CLOSED = LEFT_FIXED_X + _horizontal_span(CLOSED_SCISSOR_HEIGHT)
RIGHT_SLIDE_X_OPEN = LEFT_FIXED_X + _horizontal_span(OPEN_SCISSOR_HEIGHT)
TRACK_CENTER_X = 0.5 * (RIGHT_SLIDE_X_CLOSED + RIGHT_SLIDE_X_OPEN)

PLATFORM_CENTER_Z_CLOSED = BASE_LINK_Z + CLOSED_SCISSOR_HEIGHT - PLATFORM_LINK_LOCAL_Z
PLATFORM_LIFT_TRAVEL = OPEN_SCISSOR_HEIGHT - CLOSED_SCISSOR_HEIGHT


def _link_pose(phi: float) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    dx = LINK_LENGTH * math.cos(phi)
    dz = LINK_LENGTH * math.sin(phi)
    center = (0.5 * dx, 0.0, 0.5 * dz)
    end = (dx, 0.0, dz)
    rpy = (0.0, -phi, 0.0)
    return center, end, rpy


def _add_link_visuals(part, phi: float, *, slider_name: str, fixed_pin_name: str, center_name: str) -> None:
    center, end, rpy = _link_pose(phi)
    part.visual(
        Box((LINK_LENGTH, LINK_WIDTH, LINK_DEPTH)),
        origin=Origin(xyz=center, rpy=rpy),
        name="main_link",
    )
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name=fixed_pin_name,
    )
    part.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=end, rpy=(math.pi / 2.0, 0.0, 0.0)),
        name=slider_name,
    )
    part.visual(
        Box((0.12, LINK_WIDTH * 1.4, LINK_DEPTH * 1.2)),
        origin=Origin(xyz=center, rpy=rpy),
        name="center_boss",
    )
    part.visual(
        Cylinder(radius=PIN_RADIUS * 0.95, length=CENTER_PIN_LENGTH),
        origin=Origin(xyz=center, rpy=(math.pi / 2.0, 0.0, 0.0)),
        name=center_name,
    )


def _add_track_channel(part, *, name: str, xyz: tuple[float, float, float], material) -> None:
    x, y, z = xyz
    part.visual(
        Box((TRACK_LENGTH, TRACK_WIDTH, TRACK_HEIGHT)),
        origin=Origin(xyz=(x, y, z)),
        material=material,
        name=name,
    )
    cheek_width = 0.016
    cheek_height = TRACK_HEIGHT * 1.8
    cheek_offset = 0.5 * (TRACK_WIDTH - cheek_width)
    cheek_z = z + 0.20 * TRACK_HEIGHT
    for suffix, direction in (("left_cheek", 1.0), ("right_cheek", -1.0)):
        part.visual(
            Box((TRACK_LENGTH, cheek_width, cheek_height)),
            origin=Origin(xyz=(x, y + direction * cheek_offset, cheek_z)),
            material=material,
            name=f"{name}_{suffix}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scissor_lift_freight_elevator")

    dark_steel = model.material("dark_steel", rgba=(0.21, 0.23, 0.25, 1.0))
    platform_steel = model.material("platform_steel", rgba=(0.54, 0.57, 0.60, 1.0))
    hydraulic_gray = model.material("hydraulic_gray", rgba=(0.36, 0.39, 0.42, 1.0))
    pin_black = model.material("pin_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((BASE_LENGTH, 0.09, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.5 * (BASE_WIDTH - 0.09), 0.5 * BASE_HEIGHT)),
        material=dark_steel,
        name="left_rail",
    )
    base.visual(
        Box((BASE_LENGTH, 0.09, BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.5 * (BASE_WIDTH - 0.09), 0.5 * BASE_HEIGHT)),
        material=dark_steel,
        name="right_rail",
    )
    base.visual(
        Box((0.10, BASE_WIDTH - 0.18, BASE_HEIGHT)),
        origin=Origin(xyz=(0.5 * (BASE_LENGTH - 0.10), 0.0, 0.5 * BASE_HEIGHT)),
        material=dark_steel,
        name="front_crossmember",
    )
    base.visual(
        Box((0.10, BASE_WIDTH - 0.18, BASE_HEIGHT)),
        origin=Origin(xyz=(-0.5 * (BASE_LENGTH - 0.10), 0.0, 0.5 * BASE_HEIGHT)),
        material=dark_steel,
        name="rear_crossmember",
    )
    base.visual(
        Box((BASE_LENGTH - 0.08, BASE_WIDTH - 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=platform_steel,
        name="base_plate",
    )
    for track_name, track_y in (("base_track_front", PAIR_Y_FRONT), ("base_track_rear", PAIR_Y_REAR)):
        _add_track_channel(
            base,
            name=track_name,
            xyz=(TRACK_CENTER_X, track_y, BASE_LINK_Z),
            material=dark_steel,
        )
        base.visual(
            Box((TRACK_LENGTH - 0.02, 0.05, 0.106)),
            origin=Origin(xyz=(TRACK_CENTER_X, track_y, 0.053)),
            material=dark_steel,
            name=f"{track_name}_pedestal",
        )
    base.visual(
        Cylinder(radius=0.075, length=0.68),
        origin=Origin(xyz=(-0.02, 0.0, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hydraulic_gray,
        name="hydraulic_housing",
    )
    base.visual(
        Box((0.62, 0.18, 0.05)),
        origin=Origin(xyz=(-0.02, 0.0, 0.145)),
        material=dark_steel,
        name="hydraulic_cradle",
    )
    base.visual(
        Box((0.66, 0.20, 0.12)),
        origin=Origin(xyz=(-0.02, 0.0, 0.07)),
        material=dark_steel,
        name="hydraulic_support_tower",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.20),
        origin=Origin(xyz=(0.25, 0.0, 0.18), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_black,
        name="hydraulic_rod_stub",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)),
        mass=280.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * BASE_HEIGHT)),
    )

    platform = model.part("work_platform")
    platform.visual(
        Box((PLATFORM_LENGTH, PLATFORM_WIDTH, PLATFORM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=platform_steel,
        name="platform_deck",
    )
    for track_name, track_y in (("platform_track_front", PAIR_Y_FRONT), ("platform_track_rear", PAIR_Y_REAR)):
        _add_track_channel(
            platform,
            name=track_name,
            xyz=(TRACK_CENTER_X, track_y, PLATFORM_LINK_LOCAL_Z),
            material=dark_steel,
        )
        platform.visual(
            Box((TRACK_LENGTH - 0.02, 0.05, 0.03)),
            origin=Origin(xyz=(TRACK_CENTER_X, track_y, -0.049)),
            material=dark_steel,
            name=f"{track_name}_hanger",
        )
    for curb_name, curb_y in (("front_curb", 0.5 * (PLATFORM_WIDTH - PLATFORM_CURB_THICKNESS)), ("rear_curb", -0.5 * (PLATFORM_WIDTH - PLATFORM_CURB_THICKNESS))):
        platform.visual(
            Box((PLATFORM_LENGTH, PLATFORM_CURB_THICKNESS, PLATFORM_CURB_HEIGHT)),
            origin=Origin(
                xyz=(0.0, curb_y, 0.5 * PLATFORM_THICKNESS + 0.5 * PLATFORM_CURB_HEIGHT),
            ),
            material=dark_steel,
            name=curb_name,
        )
    for curb_name, curb_x in (("left_stop", -0.5 * (PLATFORM_LENGTH - PLATFORM_CURB_THICKNESS)), ("right_stop", 0.5 * (PLATFORM_LENGTH - PLATFORM_CURB_THICKNESS))):
        platform.visual(
            Box((PLATFORM_CURB_THICKNESS, PLATFORM_WIDTH, PLATFORM_CURB_HEIGHT)),
            origin=Origin(
                xyz=(curb_x, 0.0, 0.5 * PLATFORM_THICKNESS + 0.5 * PLATFORM_CURB_HEIGHT),
            ),
            material=dark_steel,
            name=curb_name,
        )
    platform.inertial = Inertial.from_geometry(
        Box((PLATFORM_LENGTH, PLATFORM_WIDTH, PLATFORM_THICKNESS)),
        mass=190.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "platform_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, PLATFORM_CENTER_Z_CLOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8000.0,
            velocity=0.30,
            lower=0.0,
            upper=PLATFORM_LIFT_TRAVEL,
        ),
    )

    front_rise = model.part("front_rise_link")
    _add_link_visuals(
        front_rise,
        CLOSED_LINK_ANGLE,
        slider_name="top_slider",
        fixed_pin_name="base_pin",
        center_name="center_pin",
    )
    front_rise.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH, LINK_WIDTH, LINK_DEPTH)),
        mass=36.0,
        origin=Origin(xyz=(0.5 * LINK_LENGTH, 0.0, 0.0)),
    )
    model.articulation(
        "front_rise_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=front_rise,
        origin=Origin(xyz=(LEFT_FIXED_X, PAIR_Y_FRONT + PAIR_LINK_OFFSET, BASE_LINK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.90,
            lower=0.0,
            upper=LINK_OPEN_DELTA,
        ),
    )

    front_fall = model.part("front_fall_link")
    _add_link_visuals(
        front_fall,
        -CLOSED_LINK_ANGLE,
        slider_name="bottom_slider",
        fixed_pin_name="top_pin",
        center_name="center_pin",
    )
    front_fall.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH, LINK_WIDTH, LINK_DEPTH)),
        mass=36.0,
        origin=Origin(xyz=(0.5 * LINK_LENGTH, 0.0, 0.0)),
    )
    model.articulation(
        "front_fall_hinge",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=front_fall,
        origin=Origin(xyz=(LEFT_FIXED_X, PAIR_Y_FRONT - PAIR_LINK_OFFSET, PLATFORM_LINK_LOCAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.90,
            lower=0.0,
            upper=LINK_OPEN_DELTA,
        ),
    )

    rear_rise = model.part("rear_rise_link")
    _add_link_visuals(
        rear_rise,
        CLOSED_LINK_ANGLE,
        slider_name="top_slider",
        fixed_pin_name="base_pin",
        center_name="center_pin",
    )
    rear_rise.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH, LINK_WIDTH, LINK_DEPTH)),
        mass=36.0,
        origin=Origin(xyz=(0.5 * LINK_LENGTH, 0.0, 0.0)),
    )
    model.articulation(
        "rear_rise_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rear_rise,
        origin=Origin(xyz=(LEFT_FIXED_X, PAIR_Y_REAR + PAIR_LINK_OFFSET, BASE_LINK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.90,
            lower=0.0,
            upper=LINK_OPEN_DELTA,
        ),
    )

    rear_fall = model.part("rear_fall_link")
    _add_link_visuals(
        rear_fall,
        -CLOSED_LINK_ANGLE,
        slider_name="bottom_slider",
        fixed_pin_name="top_pin",
        center_name="center_pin",
    )
    rear_fall.inertial = Inertial.from_geometry(
        Box((LINK_LENGTH, LINK_WIDTH, LINK_DEPTH)),
        mass=36.0,
        origin=Origin(xyz=(0.5 * LINK_LENGTH, 0.0, 0.0)),
    )
    model.articulation(
        "rear_fall_hinge",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=rear_fall,
        origin=Origin(xyz=(LEFT_FIXED_X, PAIR_Y_REAR - PAIR_LINK_OFFSET, PLATFORM_LINK_LOCAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.90,
            lower=0.0,
            upper=LINK_OPEN_DELTA,
        ),
    )

    for link_part in (front_rise, front_fall, rear_rise, rear_fall):
        for visual_name in ("base_pin", "top_pin", "top_slider", "bottom_slider", "center_pin"):
            try:
                link_part.get_visual(visual_name).material = pin_black
            except ValidationError:
                continue
        link_part.get_visual("main_link").material = dark_steel
        link_part.get_visual("center_boss").material = dark_steel

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSET_ROOT)

    base = object_model.get_part("base_frame")
    platform = object_model.get_part("work_platform")
    front_rise = object_model.get_part("front_rise_link")
    front_fall = object_model.get_part("front_fall_link")
    rear_rise = object_model.get_part("rear_rise_link")
    rear_fall = object_model.get_part("rear_fall_link")

    platform_lift = object_model.get_articulation("platform_lift")
    front_rise_hinge = object_model.get_articulation("front_rise_hinge")
    front_fall_hinge = object_model.get_articulation("front_fall_hinge")
    rear_rise_hinge = object_model.get_articulation("rear_rise_hinge")
    rear_fall_hinge = object_model.get_articulation("rear_fall_hinge")

    platform_deck = platform.get_visual("platform_deck")
    base_plate = base.get_visual("base_plate")
    hydraulic_housing = base.get_visual("hydraulic_housing")
    front_rise_main = front_rise.get_visual("main_link")
    front_fall_main = front_fall.get_visual("main_link")
    rear_rise_main = rear_rise.get_visual("main_link")
    rear_fall_main = rear_fall.get_visual("main_link")
    front_base_track = base.get_visual("base_track_front")
    rear_base_track = base.get_visual("base_track_rear")
    front_platform_track = platform.get_visual("platform_track_front")
    rear_platform_track = platform.get_visual("platform_track_rear")

    front_rise_slider = front_rise.get_visual("top_slider")
    rear_rise_slider = rear_rise.get_visual("top_slider")
    front_fall_slider = front_fall.get_visual("bottom_slider")
    rear_fall_slider = rear_fall.get_visual("bottom_slider")
    front_rise_center = front_rise.get_visual("center_pin")
    rear_rise_center = rear_rise.get_visual("center_pin")
    front_fall_center = front_fall.get_visual("center_pin")
    rear_fall_center = rear_fall.get_visual("center_pin")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(front_rise, front_fall, reason="front scissor pair shares a common center pivot block")
    ctx.allow_overlap(rear_rise, rear_fall, reason="rear scissor pair shares a common center pivot block")
    ctx.allow_overlap(front_rise, platform, reason="front upper slide pin nests inside the platform guide channel")
    ctx.allow_overlap(rear_rise, platform, reason="rear upper slide pin nests inside the platform guide channel")
    ctx.allow_overlap(front_fall, base, reason="front lower slide pin nests inside the base guide channel")
    ctx.allow_overlap(rear_fall, base, reason="rear lower slide pin nests inside the base guide channel")
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(platform, base, axes="xy", min_overlap=0.75, elem_a=platform_deck, elem_b=base_plate)
    ctx.expect_gap(platform, base, axis="z", min_gap=0.33, positive_elem=platform_deck, negative_elem=base_plate)
    ctx.expect_within(base, platform, axes="xy", inner_elem=hydraulic_housing, outer_elem=platform_deck)
    ctx.expect_overlap(base, front_rise, axes="xz", min_overlap=0.12, elem_a=hydraulic_housing, elem_b=front_rise_main)
    ctx.expect_overlap(base, front_fall, axes="xz", min_overlap=0.12, elem_a=hydraulic_housing, elem_b=front_fall_main)
    ctx.expect_overlap(base, rear_rise, axes="xz", min_overlap=0.12, elem_a=hydraulic_housing, elem_b=rear_rise_main)
    ctx.expect_overlap(base, rear_fall, axes="xz", min_overlap=0.12, elem_a=hydraulic_housing, elem_b=rear_fall_main)
    ctx.expect_gap(base, front_rise, axis="y", min_gap=0.14, positive_elem=hydraulic_housing, negative_elem=front_rise_main)
    ctx.expect_gap(base, front_fall, axis="y", min_gap=0.14, positive_elem=hydraulic_housing, negative_elem=front_fall_main)
    ctx.expect_gap(rear_rise, base, axis="y", min_gap=0.14, positive_elem=rear_rise_main, negative_elem=hydraulic_housing)
    ctx.expect_gap(rear_fall, base, axis="y", min_gap=0.14, positive_elem=rear_fall_main, negative_elem=hydraulic_housing)
    ctx.expect_gap(rear_rise, front_rise, axis="y", min_gap=0.52, positive_elem=rear_rise_main, negative_elem=front_rise_main)
    ctx.expect_gap(rear_fall, front_fall, axis="y", min_gap=0.52, positive_elem=rear_fall_main, negative_elem=front_fall_main)

    ctx.expect_within(front_rise, platform, axes="xy", inner_elem=front_rise_slider, outer_elem=front_platform_track)
    ctx.expect_within(rear_rise, platform, axes="xy", inner_elem=rear_rise_slider, outer_elem=rear_platform_track)
    ctx.expect_within(front_fall, base, axes="xy", inner_elem=front_fall_slider, outer_elem=front_base_track)
    ctx.expect_within(rear_fall, base, axes="xy", inner_elem=rear_fall_slider, outer_elem=rear_base_track)
    ctx.expect_overlap(front_rise, platform, axes="xz", min_overlap=0.03, elem_a=front_rise_slider, elem_b=front_platform_track)
    ctx.expect_overlap(rear_rise, platform, axes="xz", min_overlap=0.03, elem_a=rear_rise_slider, elem_b=rear_platform_track)
    ctx.expect_overlap(front_fall, base, axes="xz", min_overlap=0.03, elem_a=front_fall_slider, elem_b=front_base_track)
    ctx.expect_overlap(rear_fall, base, axes="xz", min_overlap=0.03, elem_a=rear_fall_slider, elem_b=rear_base_track)

    ctx.expect_overlap(front_rise, front_fall, axes="xz", min_overlap=0.05, elem_a=front_rise_center, elem_b=front_fall_center)
    ctx.expect_overlap(rear_rise, rear_fall, axes="xz", min_overlap=0.05, elem_a=rear_rise_center, elem_b=rear_fall_center)

    with ctx.pose(
        {
            platform_lift: PLATFORM_LIFT_TRAVEL,
            front_rise_hinge: LINK_OPEN_DELTA,
            front_fall_hinge: LINK_OPEN_DELTA,
            rear_rise_hinge: LINK_OPEN_DELTA,
            rear_fall_hinge: LINK_OPEN_DELTA,
        }
    ):
        ctx.expect_overlap(platform, base, axes="xy", min_overlap=0.75, elem_a=platform_deck, elem_b=base_plate)
        ctx.expect_gap(platform, base, axis="z", min_gap=0.83, positive_elem=platform_deck, negative_elem=base_plate)
        ctx.expect_within(base, platform, axes="xy", inner_elem=hydraulic_housing, outer_elem=platform_deck)
        ctx.expect_overlap(base, front_rise, axes="xz", min_overlap=0.12, elem_a=hydraulic_housing, elem_b=front_rise_main)
        ctx.expect_overlap(base, front_fall, axes="xz", min_overlap=0.12, elem_a=hydraulic_housing, elem_b=front_fall_main)
        ctx.expect_overlap(base, rear_rise, axes="xz", min_overlap=0.12, elem_a=hydraulic_housing, elem_b=rear_rise_main)
        ctx.expect_overlap(base, rear_fall, axes="xz", min_overlap=0.12, elem_a=hydraulic_housing, elem_b=rear_fall_main)
        ctx.expect_gap(base, front_rise, axis="y", min_gap=0.14, positive_elem=hydraulic_housing, negative_elem=front_rise_main)
        ctx.expect_gap(base, front_fall, axis="y", min_gap=0.14, positive_elem=hydraulic_housing, negative_elem=front_fall_main)
        ctx.expect_gap(rear_rise, base, axis="y", min_gap=0.14, positive_elem=rear_rise_main, negative_elem=hydraulic_housing)
        ctx.expect_gap(rear_fall, base, axis="y", min_gap=0.14, positive_elem=rear_fall_main, negative_elem=hydraulic_housing)
        ctx.expect_gap(rear_rise, front_rise, axis="y", min_gap=0.52, positive_elem=rear_rise_main, negative_elem=front_rise_main)
        ctx.expect_gap(rear_fall, front_fall, axis="y", min_gap=0.52, positive_elem=rear_fall_main, negative_elem=front_fall_main)
        ctx.expect_within(front_rise, platform, axes="xy", inner_elem=front_rise_slider, outer_elem=front_platform_track)
        ctx.expect_within(rear_rise, platform, axes="xy", inner_elem=rear_rise_slider, outer_elem=rear_platform_track)
        ctx.expect_within(front_fall, base, axes="xy", inner_elem=front_fall_slider, outer_elem=front_base_track)
        ctx.expect_within(rear_fall, base, axes="xy", inner_elem=rear_fall_slider, outer_elem=rear_base_track)
        ctx.expect_overlap(front_rise, platform, axes="xz", min_overlap=0.03, elem_a=front_rise_slider, elem_b=front_platform_track)
        ctx.expect_overlap(rear_rise, platform, axes="xz", min_overlap=0.03, elem_a=rear_rise_slider, elem_b=rear_platform_track)
        ctx.expect_overlap(front_fall, base, axes="xz", min_overlap=0.03, elem_a=front_fall_slider, elem_b=front_base_track)
        ctx.expect_overlap(rear_fall, base, axes="xz", min_overlap=0.03, elem_a=rear_fall_slider, elem_b=rear_base_track)
        ctx.expect_overlap(front_rise, front_fall, axes="xz", min_overlap=0.05, elem_a=front_rise_center, elem_b=front_fall_center)
        ctx.expect_overlap(rear_rise, rear_fall, axes="xz", min_overlap=0.05, elem_a=rear_rise_center, elem_b=rear_fall_center)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
