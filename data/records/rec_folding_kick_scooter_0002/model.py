from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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

DECK_LENGTH = 0.40
DECK_WIDTH = 0.13
DECK_THICKNESS = 0.024
DECK_CENTER_X = 0.03
DECK_CENTER_Z = 0.084
DECK_TOP_Z = DECK_CENTER_Z + DECK_THICKNESS * 0.5
HINGE_X = -0.18
HINGE_Z = 0.225
SEAT_COLLAR_X = 0.39
SEAT_COLLAR_Z = 0.26
SEAT_POST_RADIUS = 0.0125
SEAT_POST_TRAVEL = 0.12
FRONT_WHEEL_RADIUS = 0.105
REAR_WHEEL_RADIUS = 0.065
WHEEL_WIDTH = 0.024
FRONT_AXLE_X = -0.13
FRONT_AXLE_Z = FRONT_WHEEL_RADIUS - HINGE_Z
REAR_AXLE_X = 0.285
def _segment_length(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _segment_origin(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> Origin:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return Origin(
        xyz=((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5),
        rpy=(0.0, pitch, yaw),
    )


def _add_tube_member(part, name: str, a, b, radius: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=_segment_length(a, b)),
        origin=_segment_origin(a, b),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commuter_scooter")

    frame_gray = model.material("frame_gray", rgba=(0.44, 0.47, 0.50, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.17, 0.18, 0.20, 1.0))
    matte_black = model.material("matte_black", rgba=(0.09, 0.10, 0.11, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    saddle_black = model.material("saddle_black", rgba=(0.13, 0.13, 0.14, 1.0))
    latch_red = model.material("latch_red", rgba=(0.76, 0.13, 0.10, 1.0))

    deck_frame = model.part("deck_frame")
    deck_frame.visual(
        Box((DECK_LENGTH, DECK_WIDTH, DECK_THICKNESS)),
        origin=Origin(xyz=(DECK_CENTER_X, 0.0, DECK_CENTER_Z)),
        material=dark_frame,
        name="deck_shell",
    )
    deck_frame.visual(
        Box((0.29, 0.102, 0.004)),
        origin=Origin(xyz=(-0.055, 0.0, DECK_TOP_Z + 0.002)),
        material=matte_black,
        name="deck_grip",
    )
    deck_frame.visual(
        Box((0.056, 0.082, 0.170)),
        origin=Origin(xyz=(-0.148, 0.0, 0.181)),
        material=frame_gray,
        name="head_block",
    )
    deck_frame.visual(
        Box((0.038, 0.012, 0.044)),
        origin=Origin(xyz=(HINGE_X, 0.019, HINGE_Z)),
        material=frame_gray,
        name="right_hinge_ear",
    )
    deck_frame.visual(
        Box((0.038, 0.012, 0.044)),
        origin=Origin(xyz=(HINGE_X, -0.019, HINGE_Z)),
        material=frame_gray,
        name="left_hinge_ear",
    )
    deck_frame.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(HINGE_X, 0.035, HINGE_Z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=frame_gray,
        name="right_pin_head",
    )
    deck_frame.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(HINGE_X, -0.035, HINGE_Z), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=frame_gray,
        name="left_pin_head",
    )
    deck_frame.visual(
        Box((0.016, 0.044, 0.012)),
        origin=Origin(xyz=(-0.118, 0.0, 0.229)),
        material=latch_red,
        name="latch_block",
    )
    deck_frame.visual(
        Box((0.220, 0.070, 0.050)),
        origin=Origin(xyz=(0.295, 0.0, 0.087)),
        material=frame_gray,
        name="rear_bridge",
    )
    deck_frame.visual(
        Box((0.020, 0.010, 0.062)),
        origin=Origin(xyz=(REAR_AXLE_X, 0.023, 0.095)),
        material=frame_gray,
        name="rear_right_drop",
    )
    deck_frame.visual(
        Box((0.020, 0.010, 0.062)),
        origin=Origin(xyz=(REAR_AXLE_X, -0.023, 0.095)),
        material=frame_gray,
        name="rear_left_drop",
    )
    deck_frame.visual(
        Box((0.040, 0.070, 0.010)),
        origin=Origin(xyz=(SEAT_COLLAR_X, 0.0, DECK_TOP_Z + 0.005)),
        material=frame_gray,
        name="seat_support_pad",
    )
    deck_frame.visual(
        Box((0.026, 0.044, 0.136)),
        origin=Origin(xyz=(SEAT_COLLAR_X, 0.0, 0.180)),
        material=frame_gray,
        name="seat_mast_base",
    )
    _add_tube_member(
        deck_frame,
        "collar_front_brace",
        (0.215, 0.0, 0.108),
        (SEAT_COLLAR_X - 0.018, 0.0, SEAT_COLLAR_Z - 0.050),
        0.008,
        frame_gray,
    )
    deck_frame.inertial = Inertial.from_geometry(
        Box((0.50, 0.16, 0.16)),
        mass=7.8,
        origin=Origin(xyz=(0.03, 0.0, 0.095)),
    )

    seat_collar = model.part("seat_collar")
    seat_collar.visual(
        Box((0.034, 0.060, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, -0.036)),
        material=frame_gray,
        name="collar_mast",
    )
    seat_collar.visual(
        Box((0.034, 0.012, 0.048)),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=frame_gray,
        name="right_cheek",
    )
    seat_collar.visual(
        Box((0.034, 0.012, 0.048)),
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=frame_gray,
        name="left_cheek",
    )
    seat_collar.visual(
        Box((0.012, 0.060, 0.048)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0)),
        material=frame_gray,
        name="front_band",
    )
    seat_collar.visual(
        Box((0.012, 0.060, 0.048)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material=frame_gray,
        name="rear_band",
    )
    seat_collar.inertial = Inertial.from_geometry(
        Box((0.060, 0.070, 0.080)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
    )

    quick_release = model.part("quick_release")
    quick_release.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=frame_gray,
        name="pivot_hub",
    )
    quick_release.visual(
        Box((0.036, 0.004, 0.012)),
        origin=Origin(xyz=(0.014, 0.009, 0.0)),
        material=latch_red,
        name="lever",
    )
    quick_release.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(-0.003, -0.012, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=frame_gray,
        name="clamp_nut",
    )
    quick_release.inertial = Inertial.from_geometry(
        Box((0.050, 0.040, 0.020)),
        mass=0.08,
        origin=Origin(),
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=frame_gray,
        name="hinge_barrel",
    )
    stem.visual(
        Box((0.110, 0.040, 0.014)),
        origin=Origin(xyz=(0.055, 0.0, 0.010)),
        material=frame_gray,
        name="latch_plate",
    )
    stem.visual(
        Box((0.070, 0.070, 0.036)),
        origin=Origin(xyz=(-0.022, 0.0, 0.066)),
        material=frame_gray,
        name="fork_crown",
    )
    stem.visual(
        Box((0.056, 0.040, 0.030)),
        origin=Origin(xyz=(0.090, 0.0, 0.533)),
        material=frame_gray,
        name="bar_clamp",
    )
    _add_tube_member(
        stem,
        "neck_link",
        (0.000, 0.0, 0.004),
        (0.016, 0.0, 0.078),
        0.010,
        frame_gray,
    )
    _add_tube_member(
        stem,
        "stem_tube",
        (0.016, 0.0, 0.072),
        (0.090, 0.0, 0.533),
        0.016,
        frame_gray,
    )
    _add_tube_member(
        stem,
        "fork_right_leg",
        (-0.004, 0.027, 0.066),
        (FRONT_AXLE_X, 0.027, FRONT_AXLE_Z),
        0.009,
        frame_gray,
    )
    _add_tube_member(
        stem,
        "fork_left_leg",
        (-0.004, -0.027, 0.066),
        (FRONT_AXLE_X, -0.027, FRONT_AXLE_Z),
        0.009,
        frame_gray,
    )
    stem.visual(
        Box((0.020, 0.070, 0.018)),
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, FRONT_AXLE_Z)),
        material=frame_gray,
        name="axle_bridge",
    )
    _add_tube_member(
        stem,
        "handlebar",
        (0.090, -0.205, 0.545),
        (0.090, 0.205, 0.545),
        0.012,
        frame_gray,
    )
    stem.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(
            xyz=(0.090, 0.239, 0.545),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=rubber,
        name="right_grip",
    )
    stem.visual(
        Cylinder(radius=0.014, length=0.080),
        origin=Origin(
            xyz=(0.090, -0.239, 0.545),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=rubber,
        name="left_grip",
    )
    stem.inertial = Inertial.from_geometry(
        Box((0.22, 0.56, 0.62)),
        mass=2.6,
        origin=Origin(xyz=(0.040, 0.0, 0.285)),
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        Cylinder(radius=FRONT_WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=rubber,
        name="front_tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.056, length=0.016),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=matte_black,
        name="front_rim",
    )
    front_wheel.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.015, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=frame_gray,
        name="front_right_hub",
    )
    front_wheel.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=Origin(
            xyz=(0.0, -0.015, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=frame_gray,
        name="front_left_hub",
    )
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=FRONT_WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=1.4,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        Cylinder(radius=REAR_WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=rubber,
        name="rear_tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.032, length=0.016),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=matte_black,
        name="rear_rim",
    )
    rear_wheel.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.015, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=frame_gray,
        name="rear_right_hub",
    )
    rear_wheel.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(
            xyz=(0.0, -0.015, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=frame_gray,
        name="rear_left_hub",
    )
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=REAR_WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.9,
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
    )

    seat_post = model.part("seat_post")
    seat_post.visual(
        Cylinder(radius=SEAT_POST_RADIUS, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=frame_gray,
        name="seat_tube",
    )
    _add_tube_member(
        seat_post,
        "saddle_right_rail",
        (0.0, 0.017, 0.215),
        (0.022, 0.017, 0.265),
        0.005,
        frame_gray,
    )
    _add_tube_member(
        seat_post,
        "saddle_left_rail",
        (0.0, -0.017, 0.215),
        (0.022, -0.017, 0.265),
        0.005,
        frame_gray,
    )
    seat_post.visual(
        Box((0.125, 0.085, 0.022)),
        origin=Origin(xyz=(0.030, 0.0, 0.276)),
        material=saddle_black,
        name="saddle",
    )
    seat_post.visual(
        Box((0.070, 0.050, 0.018)),
        origin=Origin(xyz=(-0.055, 0.0, 0.276)),
        material=saddle_black,
        name="saddle_nose",
    )
    seat_post.inertial = Inertial.from_geometry(
        Box((0.24, 0.10, 0.44)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )

    model.articulation(
        "stem_fold",
        ArticulationType.REVOLUTE,
        parent=deck_frame,
        child=stem,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=0.0,
            upper=1.50,
        ),
    )
    model.articulation(
        "front_axle_mount",
        ArticulationType.FIXED,
        parent=stem,
        child=front_wheel,
        origin=Origin(xyz=(FRONT_AXLE_X, 0.0, FRONT_AXLE_Z)),
    )
    model.articulation(
        "rear_axle_mount",
        ArticulationType.FIXED,
        parent=deck_frame,
        child=rear_wheel,
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, REAR_WHEEL_RADIUS)),
    )
    model.articulation(
        "collar_mount",
        ArticulationType.FIXED,
        parent=deck_frame,
        child=seat_collar,
        origin=Origin(xyz=(SEAT_COLLAR_X, 0.0, SEAT_COLLAR_Z)),
    )
    model.articulation(
        "quick_release_mount",
        ArticulationType.FIXED,
        parent=seat_collar,
        child=quick_release,
        origin=Origin(xyz=(0.0, 0.037, 0.006)),
    )
    model.articulation(
        "seat_slide",
        ArticulationType.PRISMATIC,
        parent=seat_collar,
        child=seat_post,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=SEAT_POST_TRAVEL,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck_frame = object_model.get_part("deck_frame")
    seat_collar = object_model.get_part("seat_collar")
    quick_release = object_model.get_part("quick_release")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    seat_post = object_model.get_part("seat_post")
    stem_fold = object_model.get_articulation("stem_fold")
    seat_slide = object_model.get_articulation("seat_slide")

    deck_shell = deck_frame.get_visual("deck_shell")
    right_hinge_ear = deck_frame.get_visual("right_hinge_ear")
    latch_block = deck_frame.get_visual("latch_block")
    seat_mast_base = deck_frame.get_visual("seat_mast_base")
    rear_right_drop = deck_frame.get_visual("rear_right_drop")

    hinge_barrel = stem.get_visual("hinge_barrel")
    latch_plate = stem.get_visual("latch_plate")
    stem_tube = stem.get_visual("stem_tube")
    handlebar = stem.get_visual("handlebar")
    fork_right_leg = stem.get_visual("fork_right_leg")

    collar_mast = seat_collar.get_visual("collar_mast")
    right_cheek = seat_collar.get_visual("right_cheek")
    pivot_hub = quick_release.get_visual("pivot_hub")

    saddle = seat_post.get_visual("saddle")
    seat_tube = seat_post.get_visual("seat_tube")

    front_right_hub = front_wheel.get_visual("front_right_hub")
    rear_right_hub = rear_wheel.get_visual("rear_right_hub")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        deck_frame,
        stem,
        reason="the folding stem's hinge barrel and lower socket nest inside the deck-front hinge housing",
    )
    ctx.allow_overlap(
        seat_collar,
        seat_post,
        reason="the sliding seat post intentionally telescopes through the frame collar sleeve",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.expect_gap(
        deck_frame,
        stem,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=right_hinge_ear,
        negative_elem=hinge_barrel,
        name="stem_hinge_barrel_nests_between_deck_ears",
    )
    ctx.expect_contact(
        stem,
        deck_frame,
        elem_a=latch_plate,
        elem_b=latch_block,
        name="upright_stem_latches_at_deck_junction",
    )
    ctx.expect_within(
        seat_post,
        seat_collar,
        axes="xy",
        inner_elem=seat_tube,
        name="seat_post_runs_through_frame_collar",
    )
    ctx.expect_contact(
        seat_collar,
        deck_frame,
        elem_a=collar_mast,
        elem_b=seat_mast_base,
        name="frame_collar_is_seated_on_rear_frame_mast",
    )
    ctx.expect_contact(
        quick_release,
        seat_collar,
        elem_a=pivot_hub,
        elem_b=right_cheek,
        name="quick_release_lever_mounts_on_collar_cheek",
    )
    ctx.expect_gap(
        stem,
        front_wheel,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=fork_right_leg,
        negative_elem=front_right_hub,
        name="front_wheel_hub_seats_in_fork_dropouts",
    )
    ctx.expect_gap(
        deck_frame,
        rear_wheel,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rear_right_drop,
        negative_elem=rear_right_hub,
        name="rear_wheel_hub_seats_in_deck_dropouts",
    )
    ctx.expect_overlap(
        rear_wheel,
        deck_frame,
        axes="xy",
        min_overlap=0.002,
        name="rear_wheel_sits_under_deck_footprint",
    )
    ctx.expect_gap(
        stem,
        deck_frame,
        axis="z",
        min_gap=0.34,
        positive_elem=handlebar,
        negative_elem=deck_shell,
        name="upright_handlebar_sits_high_above_deck",
    )
    ctx.expect_gap(
        seat_post,
        deck_frame,
        axis="z",
        min_gap=0.28,
        positive_elem=saddle,
        negative_elem=deck_shell,
        name="saddle_is_raised_above_deck",
    )
    assert (
        FRONT_WHEEL_RADIUS > REAR_WHEEL_RADIUS + 0.03
    ), "front wheel should be visibly larger than rear wheel"

    with ctx.pose({stem_fold: 1.50}):
        ctx.expect_overlap(
            stem,
            deck_frame,
            axes="xy",
            min_overlap=0.020,
            elem_a=stem_tube,
            elem_b=deck_shell,
            name="folded_stem_swings_over_deck",
        )
        ctx.expect_gap(
            stem,
            deck_frame,
            axis="z",
            max_gap=0.080,
            max_penetration=0.0,
            positive_elem=stem_tube,
            negative_elem=deck_shell,
            name="folded_stem_stays_close_to_deck",
        )
        ctx.expect_gap(
            seat_collar,
            stem,
            axis="x",
            min_gap=0.01,
            positive_elem=collar_mast,
            negative_elem=stem_tube,
            name="folded_stem_clears_rear_seat_collar",
        )
        ctx.expect_gap(
            seat_post,
            stem,
            axis="x",
            min_gap=0.01,
            positive_elem=seat_tube,
            negative_elem=stem_tube,
            name="folded_stem_clears_sliding_seat_post",
        )

    with ctx.pose({seat_slide: SEAT_POST_TRAVEL}):
        ctx.expect_within(
            seat_post,
            seat_collar,
            axes="xy",
            inner_elem=seat_tube,
            name="raised_seat_post_still_tracks_in_collar",
        )
        ctx.expect_gap(
            seat_post,
            deck_frame,
            axis="z",
            min_gap=0.44,
            positive_elem=saddle,
            negative_elem=deck_shell,
            name="raised_saddle_gains_height_over_deck",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
