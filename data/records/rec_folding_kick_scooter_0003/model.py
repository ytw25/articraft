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
)

def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_wheel_visuals(part, radius: float, width: float, tire_mat, rim_mat, hub_mat) -> None:
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tire_mat,
        name="tire",
    )
    part.visual(
        Cylinder(radius=radius * 0.58, length=width * 0.55),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_mat,
        name="rim",
    )
    part.visual(
        Cylinder(radius=radius * 0.20, length=width * 0.90),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hub_mat,
        name="hub",
    )
    face_offset = width * 0.5
    part.visual(
        Cylinder(radius=radius * 0.42, length=0.002),
        origin=Origin(xyz=(0.0, -face_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_mat,
        name="face_neg",
    )
    part.visual(
        Cylinder(radius=radius * 0.42, length=0.002),
        origin=Origin(xyz=(0.0, face_offset, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim_mat,
        name="face_pos",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wheel_kick_scooter")

    deck_blue = model.material("deck_blue", rgba=(0.18, 0.56, 0.84, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    silver = model.material("silver", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.11, 0.12, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.50, 0.14, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.083)),
        material=deck_blue,
        name="deck_surface",
    )
    deck.visual(
        Box((0.34, 0.09, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=charcoal,
        name="grip_pad",
    )
    deck.visual(
        Box((0.46, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.064, 0.085)),
        material=charcoal,
        name="left_rail",
    )
    deck.visual(
        Box((0.46, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, -0.064, 0.085)),
        material=charcoal,
        name="right_rail",
    )
    deck.visual(
        Box((0.084, 0.058, 0.012)),
        origin=Origin(xyz=(0.215, 0.0, 0.099)),
        material=charcoal,
        name="pivot_plinth",
    )
    deck.visual(
        Box((0.010, 0.026, 0.026)),
        origin=Origin(xyz=(0.192, 0.0, 0.118)),
        material=charcoal,
        name="pivot_rear_cheek",
    )
    deck.visual(
        Box((0.010, 0.026, 0.026)),
        origin=Origin(xyz=(0.238, 0.0, 0.118)),
        material=charcoal,
        name="pivot_front_cheek",
    )
    deck.visual(
        Box((0.075, 0.016, 0.080)),
        origin=Origin(xyz=(-0.2875, 0.032, 0.076)),
        material=charcoal,
        name="rear_left_dropout",
    )
    deck.visual(
        Box((0.075, 0.016, 0.080)),
        origin=Origin(xyz=(-0.2875, -0.032, 0.076)),
        material=charcoal,
        name="rear_right_dropout",
    )
    deck.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(-0.315, 0.022, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_left_stub",
    )
    deck.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(-0.315, -0.022, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_right_stub",
    )
    deck.visual(
        Cylinder(radius=0.0045, length=0.034),
        origin=Origin(xyz=(-0.315, 0.0, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_axle_tube",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.58, 0.16, 0.14)),
        mass=4.8,
        origin=Origin(xyz=(-0.02, 0.0, 0.085)),
    )

    steering = model.part("steering_fork")
    steering.visual(
        Cylinder(radius=0.0105, length=0.036),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="pivot_barrel",
    )
    steering.visual(
        Box((0.012, 0.020, 0.032)),
        origin=Origin(xyz=(0.004, 0.0, 0.016)),
        material=charcoal,
        name="pivot_post",
    )
    steering.visual(
        Box((0.018, 0.028, 0.022)),
        origin=Origin(xyz=(0.012, 0.0, 0.031)),
        material=charcoal,
        name="pivot_yoke",
    )
    steering.visual(
        Box((0.022, 0.094, 0.014)),
        origin=Origin(xyz=(0.050, 0.0, 0.010)),
        material=charcoal,
        name="fork_crown",
    )
    _add_member(
        steering,
        (0.010, 0.0, 0.030),
        (0.040, 0.0, 0.020),
        0.0055,
        silver,
        "crown_link",
    )
    _add_member(
        steering,
        (0.015, 0.0, 0.034),
        (0.098, 0.0, 0.470),
        0.014,
        silver,
        "lower_column",
    )
    steering.visual(
        Cylinder(radius=0.021, length=0.110),
        origin=Origin(xyz=(0.098, 0.0, 0.485)),
        material=charcoal,
        name="clamp_sleeve",
    )
    steering.visual(
        Box((0.018, 0.022, 0.022)),
        origin=Origin(xyz=(0.112, 0.0, 0.485)),
        material=charcoal,
        name="clamp_lug",
    )
    _add_member(
        steering,
        (0.050, 0.032, 0.010),
        (0.112, 0.084, -0.056),
        0.010,
        silver,
        "left_fork_arm",
    )
    _add_member(
        steering,
        (0.050, -0.032, 0.010),
        (0.112, -0.084, -0.056),
        0.010,
        silver,
        "right_fork_arm",
    )
    steering.visual(
        Box((0.032, 0.024, 0.040)),
        origin=Origin(xyz=(0.116, 0.084, -0.056)),
        material=charcoal,
        name="left_hub_block",
    )
    steering.visual(
        Box((0.032, 0.024, 0.040)),
        origin=Origin(xyz=(0.116, -0.084, -0.056)),
        material=charcoal,
        name="right_hub_block",
    )
    steering.visual(
        Cylinder(radius=0.0055, length=0.164),
        origin=Origin(xyz=(0.096, 0.0, -0.032), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="fork_spreader",
    )
    steering.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.124, 0.098, -0.064), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="front_left_stub",
    )
    steering.visual(
        Cylinder(radius=0.008, length=0.014),
        origin=Origin(xyz=(0.124, -0.098, -0.064), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="front_right_stub",
    )
    steering.inertial = Inertial.from_geometry(
        Box((0.28, 0.26, 0.60)),
        mass=1.6,
        origin=Origin(xyz=(0.06, 0.0, 0.16)),
    )

    handlebar = model.part("handlebar_assembly")
    handlebar.visual(
        Cylinder(radius=0.015, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=silver,
        name="stem_insert",
    )
    handlebar.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=charcoal,
        name="height_adjust_collar",
    )
    handlebar.visual(
        Box((0.050, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=charcoal,
        name="bar_clamp",
    )
    handlebar.visual(
        Cylinder(radius=0.013, length=0.320),
        origin=Origin(xyz=(0.0, 0.0, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="handlebar_bar",
    )
    handlebar.visual(
        Cylinder(radius=0.017, length=0.080),
        origin=Origin(xyz=(0.0, 0.185, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.017, length=0.080),
        origin=Origin(xyz=(0.0, -0.185, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )
    handlebar.inertial = Inertial.from_geometry(
        Box((0.10, 0.42, 0.32)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    front_left_wheel = model.part("front_left_wheel")
    _add_wheel_visuals(front_left_wheel, 0.060, 0.032, rubber, silver, charcoal)
    front_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.032),
        mass=0.5,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    front_right_wheel = model.part("front_right_wheel")
    _add_wheel_visuals(front_right_wheel, 0.060, 0.032, rubber, silver, charcoal)
    front_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.032),
        mass=0.5,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    _add_wheel_visuals(rear_wheel, 0.055, 0.034, rubber, silver, charcoal)
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.034),
        mass=0.45,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "steering_tilt",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=steering,
        origin=Origin(xyz=(0.215, 0.0, 0.118)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=-0.30, upper=0.30),
    )
    model.articulation(
        "handlebar_mount",
        ArticulationType.FIXED,
        parent=steering,
        child=handlebar,
        origin=Origin(xyz=(0.098, 0.0, 0.350)),
    )
    model.articulation(
        "front_left_spin",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child=front_left_wheel,
        origin=Origin(xyz=(0.124, 0.118, -0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )
    model.articulation(
        "front_right_spin",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child=front_right_wheel,
        origin=Origin(xyz=(0.124, -0.118, -0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )
    model.articulation(
        "rear_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.315, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    try:
        os.getcwd()
    except FileNotFoundError:
        os.chdir("/")

    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    steering = object_model.get_part("steering_fork")
    handlebar = object_model.get_part("handlebar_assembly")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_wheel = object_model.get_part("rear_wheel")

    steering_tilt = object_model.get_articulation("steering_tilt")

    deck_surface = deck.get_visual("deck_surface")
    pivot_front_cheek = deck.get_visual("pivot_front_cheek")
    pivot_rear_cheek = deck.get_visual("pivot_rear_cheek")
    rear_left_stub = deck.get_visual("rear_left_stub")
    rear_right_stub = deck.get_visual("rear_right_stub")

    pivot_barrel = steering.get_visual("pivot_barrel")
    clamp_sleeve = steering.get_visual("clamp_sleeve")
    front_left_stub = steering.get_visual("front_left_stub")
    front_right_stub = steering.get_visual("front_right_stub")

    stem_insert = handlebar.get_visual("stem_insert")
    handlebar_bar = handlebar.get_visual("handlebar_bar")

    front_left_face_neg = front_left_wheel.get_visual("face_neg")
    front_right_face_pos = front_right_wheel.get_visual("face_pos")
    rear_face_pos = rear_wheel.get_visual("face_pos")
    rear_face_neg = rear_wheel.get_visual("face_neg")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.allow_overlap(
        front_left_wheel,
        steering,
        reason="front left wheel hub sleeves around the left stub axle",
    )
    ctx.allow_overlap(
        front_right_wheel,
        steering,
        reason="front right wheel hub sleeves around the right stub axle",
    )
    ctx.allow_overlap(
        rear_wheel,
        deck,
        reason="rear wheel side faces seat tightly against the fixed rear axle stubs",
    )
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(steering, deck, elem_a=pivot_barrel, elem_b=pivot_front_cheek)
    ctx.expect_contact(steering, deck, elem_a=pivot_barrel, elem_b=pivot_rear_cheek)
    ctx.expect_contact(handlebar, steering, elem_a=stem_insert, elem_b=clamp_sleeve)
    ctx.expect_contact(front_left_wheel, steering, elem_a=front_left_face_neg, elem_b=front_left_stub)
    ctx.expect_contact(front_right_wheel, steering, elem_a=front_right_face_pos, elem_b=front_right_stub)
    ctx.expect_contact(rear_wheel, deck, elem_a=rear_face_pos, elem_b=rear_left_stub)
    ctx.expect_contact(rear_wheel, deck, elem_a=rear_face_neg, elem_b=rear_right_stub)
    ctx.expect_origin_distance(front_left_wheel, front_right_wheel, axes="x", max_dist=0.002)
    ctx.expect_origin_distance(front_left_wheel, front_right_wheel, axes="z", max_dist=0.002)
    ctx.expect_origin_distance(rear_wheel, deck, axes="y", max_dist=0.001)

    ctx.expect_gap(
        handlebar,
        deck,
        axis="z",
        min_gap=0.48,
        positive_elem=handlebar_bar,
        negative_elem=deck_surface,
    )
    ctx.expect_gap(
        front_left_wheel,
        deck,
        axis="x",
        min_gap=0.010,
        negative_elem=deck_surface,
    )
    ctx.expect_gap(
        front_right_wheel,
        deck,
        axis="x",
        min_gap=0.010,
        negative_elem=deck_surface,
    )
    ctx.expect_gap(front_left_wheel, deck, axis="y", min_gap=0.020)
    ctx.expect_gap(deck, front_right_wheel, axis="y", min_gap=0.020)
    ctx.expect_gap(deck, rear_wheel, axis="x", min_gap=0.005, positive_elem=deck_surface)
    ctx.expect_gap(front_left_wheel, front_right_wheel, axis="y", min_gap=0.18)
    ctx.expect_gap(front_left_wheel, rear_wheel, axis="x", min_gap=0.50)
    ctx.expect_gap(front_right_wheel, rear_wheel, axis="x", min_gap=0.50)
    ctx.expect_within(rear_wheel, deck, axes="y")

    with ctx.pose({steering_tilt: 0.35}):
        ctx.expect_contact(steering, deck, elem_a=pivot_barrel, elem_b=pivot_front_cheek)
        ctx.expect_gap(
            handlebar,
            deck,
            axis="z",
            min_gap=0.44,
            positive_elem=handlebar_bar,
            negative_elem=deck_surface,
        )
        ctx.expect_gap(
            front_left_wheel,
            deck,
            axis="x",
            min_gap=0.008,
            negative_elem=deck_surface,
        )
        ctx.expect_gap(
            front_right_wheel,
            deck,
            axis="x",
            min_gap=0.008,
            negative_elem=deck_surface,
        )

    with ctx.pose({steering_tilt: -0.35}):
        ctx.expect_contact(steering, deck, elem_a=pivot_barrel, elem_b=pivot_rear_cheek)
        ctx.expect_gap(
            handlebar,
            deck,
            axis="z",
            min_gap=0.44,
            positive_elem=handlebar_bar,
            negative_elem=deck_surface,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
