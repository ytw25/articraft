from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

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

SAFE_ASSET_ROOT = "/tmp"
SAFE_ASSETS = AssetContext(SAFE_ASSET_ROOT)
_ORIGINAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIGINAL_GETCWD()
    except FileNotFoundError:
        os.chdir(SAFE_ASSET_ROOT)
        return _ORIGINAL_GETCWD()


os.getcwd = _safe_getcwd

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir(SAFE_ASSET_ROOT)


def _segment_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-6:
        raise ValueError("cylinder segment length must be positive")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (
        Origin(
            xyz=(
                (start[0] + end[0]) / 2.0,
                (start[1] + end[1]) / 2.0,
                (start[2] + end[2]) / 2.0,
            ),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _add_segment(
    part,
    *,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
) -> None:
    origin, length = _segment_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dock_leveler", assets=SAFE_ASSETS)

    frame_steel = model.material("frame_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.42, 0.45, 0.48, 1.0))
    lip_steel = model.material("lip_steel", rgba=(0.37, 0.39, 0.42, 1.0))
    hydraulic = model.material("hydraulic", rgba=(0.10, 0.11, 0.12, 1.0))
    pin_metal = model.material("pin_metal", rgba=(0.70, 0.72, 0.74, 1.0))

    frame = model.part("pit_frame")
    frame.visual(
        Box((2.22, 0.10, 0.30)),
        origin=Origin(xyz=(1.00, 0.96, -0.18)),
        material=frame_steel,
        name="left_wall",
    )
    frame.visual(
        Box((2.22, 0.10, 0.30)),
        origin=Origin(xyz=(1.00, -0.96, -0.18)),
        material=frame_steel,
        name="right_wall",
    )
    frame.visual(
        Box((0.18, 2.02, 0.30)),
        origin=Origin(xyz=(-0.09, 0.00, -0.18)),
        material=frame_steel,
        name="rear_wall",
    )
    frame.visual(
        Box((0.08, 0.10, 0.30)),
        origin=Origin(xyz=(2.15, 0.96, -0.18)),
        material=frame_steel,
        name="left_front_corner",
    )
    frame.visual(
        Box((0.08, 0.10, 0.30)),
        origin=Origin(xyz=(2.15, -0.96, -0.18)),
        material=frame_steel,
        name="right_front_corner",
    )
    frame.visual(
        Box((1.72, 0.10, 0.04)),
        origin=Origin(xyz=(1.04, 0.86, -0.075)),
        material=frame_steel,
        name="left_ledger",
    )
    frame.visual(
        Box((1.72, 0.10, 0.04)),
        origin=Origin(xyz=(1.04, -0.86, -0.075)),
        material=frame_steel,
        name="right_ledger",
    )
    frame.visual(
        Box((0.40, 1.28, 0.10)),
        origin=Origin(xyz=(0.18, 0.00, -0.16)),
        material=frame_steel,
        name="actuator_support_beam",
    )
    frame.visual(
        Box((0.16, 0.18, 0.12)),
        origin=Origin(xyz=(0.16, 0.55, -0.15)),
        material=frame_steel,
        name="left_actuator_saddle",
    )
    frame.visual(
        Box((0.16, 0.18, 0.12)),
        origin=Origin(xyz=(0.16, -0.55, -0.15)),
        material=frame_steel,
        name="right_actuator_saddle",
    )
    _add_segment(
        frame,
        name="left_rear_hinge_barrel",
        start=(0.0, 0.33, 0.0),
        end=(0.0, 0.83, 0.0),
        radius=0.055,
        material=frame_steel,
    )
    _add_segment(
        frame,
        name="right_rear_hinge_barrel",
        start=(0.0, -0.83, 0.0),
        end=(0.0, -0.33, 0.0),
        radius=0.055,
        material=frame_steel,
    )
    _add_segment(
        frame,
        name="rear_hinge_pin",
        start=(0.0, -0.83, 0.0),
        end=(0.0, 0.83, 0.0),
        radius=0.018,
        material=pin_metal,
    )
    frame.inertial = Inertial.from_geometry(
        Box((2.32, 2.02, 0.30)),
        mass=900.0,
        origin=Origin(xyz=(1.00, 0.00, -0.18)),
    )

    deck = model.part("deck")
    deck.visual(
        Box((2.00, 1.82, 0.05)),
        origin=Origin(xyz=(1.00, 0.00, 0.00)),
        material=deck_steel,
        name="deck_plate",
    )
    deck.visual(
        Box((1.72, 0.10, 0.03)),
        origin=Origin(xyz=(1.04, 0.86, -0.040)),
        material=deck_steel,
        name="left_seat_pad",
    )
    deck.visual(
        Box((1.72, 0.10, 0.03)),
        origin=Origin(xyz=(1.04, -0.86, -0.040)),
        material=deck_steel,
        name="right_seat_pad",
    )
    deck.visual(
        Box((1.86, 0.06, 0.10)),
        origin=Origin(xyz=(0.98, 0.88, -0.075)),
        material=deck_steel,
        name="left_skirt",
    )
    deck.visual(
        Box((1.86, 0.06, 0.10)),
        origin=Origin(xyz=(0.98, -0.88, -0.075)),
        material=deck_steel,
        name="right_skirt",
    )
    deck.visual(
        Box((1.18, 0.14, 0.16)),
        origin=Origin(xyz=(1.13, 0.00, -0.105)),
        material=deck_steel,
        name="center_rib",
    )
    deck.visual(
        Box((0.98, 0.10, 0.12)),
        origin=Origin(xyz=(1.11, 0.44, -0.085)),
        material=deck_steel,
        name="left_rib",
    )
    deck.visual(
        Box((0.98, 0.10, 0.12)),
        origin=Origin(xyz=(1.11, -0.44, -0.085)),
        material=deck_steel,
        name="right_rib",
    )
    deck.visual(
        Box((0.14, 0.18, 0.06)),
        origin=Origin(xyz=(0.47, 0.55, -0.055)),
        material=deck_steel,
        name="left_actuator_mount",
    )
    deck.visual(
        Box((0.14, 0.18, 0.06)),
        origin=Origin(xyz=(0.47, -0.55, -0.055)),
        material=deck_steel,
        name="right_actuator_mount",
    )
    deck.visual(
        Box((0.08, 1.82, 0.09)),
        origin=Origin(xyz=(1.96, 0.00, -0.070)),
        material=deck_steel,
        name="front_nose",
    )
    _add_segment(
        deck,
        name="center_rear_hinge_barrel",
        start=(0.0, -0.29, 0.0),
        end=(0.0, 0.29, 0.0),
        radius=0.050,
        material=deck_steel,
    )
    _add_segment(
        deck,
        name="left_lip_hinge_barrel",
        start=(2.0, 0.36, 0.0),
        end=(2.0, 0.80, 0.0),
        radius=0.040,
        material=deck_steel,
    )
    _add_segment(
        deck,
        name="right_lip_hinge_barrel",
        start=(2.0, -0.80, 0.0),
        end=(2.0, -0.36, 0.0),
        radius=0.040,
        material=deck_steel,
    )
    _add_segment(
        deck,
        name="lip_hinge_pin",
        start=(2.0, -0.80, 0.0),
        end=(2.0, 0.80, 0.0),
        radius=0.013,
        material=pin_metal,
    )
    deck.inertial = Inertial.from_geometry(
        Box((2.00, 1.82, 0.18)),
        mass=650.0,
        origin=Origin(xyz=(1.00, 0.00, -0.05)),
    )

    lip = model.part("lip")
    lip.visual(
        Box((0.05, 1.74, 0.38)),
        origin=Origin(xyz=(0.025, 0.00, -0.19)),
        material=lip_steel,
        name="lip_plate",
    )
    lip.visual(
        Box((0.09, 1.40, 0.06)),
        origin=Origin(xyz=(0.045, 0.00, -0.06)),
        material=lip_steel,
        name="lip_stiffener",
    )
    lip.visual(
        Box((0.10, 1.60, 0.05)),
        origin=Origin(xyz=(0.05, 0.00, -0.39)),
        material=lip_steel,
        name="toe_bar",
    )
    _add_segment(
        lip,
        name="center_hinge_barrel",
        start=(0.0, -0.32, 0.0),
        end=(0.0, 0.32, 0.0),
        radius=0.036,
        material=lip_steel,
    )
    lip.inertial = Inertial.from_geometry(
        Box((0.10, 1.74, 0.42)),
        mass=120.0,
        origin=Origin(xyz=(0.05, 0.00, -0.21)),
    )

    left_actuator = model.part("left_actuator")
    left_actuator.visual(
        Box((0.10, 0.14, 0.08)),
        origin=Origin(xyz=(-0.35, 0.00, -0.065)),
        material=hydraulic,
        name="lower_clevis",
    )
    _add_segment(
        left_actuator,
        name="barrel",
        start=(-0.30, 0.00, -0.055),
        end=(-0.12, 0.00, -0.035),
        radius=0.050,
        material=hydraulic,
    )
    _add_segment(
        left_actuator,
        name="rod",
        start=(-0.12, 0.00, -0.035),
        end=(-0.01, 0.00, -0.015),
        radius=0.028,
        material=pin_metal,
    )
    left_actuator.visual(
        Box((0.10, 0.14, 0.05)),
        origin=Origin(xyz=(0.00, 0.00, -0.025)),
        material=hydraulic,
        name="upper_eye",
    )
    left_actuator.inertial = Inertial.from_geometry(
        Box((0.48, 0.14, 0.10)),
        mass=60.0,
        origin=Origin(xyz=(-0.19, 0.00, -0.035)),
    )

    right_actuator = model.part("right_actuator")
    right_actuator.visual(
        Box((0.10, 0.14, 0.08)),
        origin=Origin(xyz=(-0.35, 0.00, -0.065)),
        material=hydraulic,
        name="lower_clevis",
    )
    _add_segment(
        right_actuator,
        name="barrel",
        start=(-0.30, 0.00, -0.055),
        end=(-0.12, 0.00, -0.035),
        radius=0.050,
        material=hydraulic,
    )
    _add_segment(
        right_actuator,
        name="rod",
        start=(-0.12, 0.00, -0.035),
        end=(-0.01, 0.00, -0.015),
        radius=0.028,
        material=pin_metal,
    )
    right_actuator.visual(
        Box((0.10, 0.14, 0.05)),
        origin=Origin(xyz=(0.00, 0.00, -0.025)),
        material=hydraulic,
        name="upper_eye",
    )
    right_actuator.inertial = Inertial.from_geometry(
        Box((0.48, 0.14, 0.10)),
        mass=60.0,
        origin=Origin(xyz=(-0.19, 0.00, -0.035)),
    )

    model.articulation(
        "frame_to_deck",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=deck,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4500.0, velocity=0.8, lower=-0.35, upper=0.12),
    )
    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(2.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=2.0, lower=-1.45, upper=0.10),
    )
    model.articulation(
        "deck_to_left_actuator",
        ArticulationType.FIXED,
        parent=deck,
        child=left_actuator,
        origin=Origin(xyz=(0.47, 0.55, -0.085)),
    )
    model.articulation(
        "deck_to_right_actuator",
        ArticulationType.FIXED,
        parent=deck,
        child=right_actuator,
        origin=Origin(xyz=(0.47, -0.55, -0.085)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SAFE_ASSET_ROOT)
    frame = object_model.get_part("pit_frame")
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip")
    left_actuator = object_model.get_part("left_actuator")
    right_actuator = object_model.get_part("right_actuator")
    deck_hinge = object_model.get_articulation("frame_to_deck")
    lip_hinge = object_model.get_articulation("deck_to_lip")

    left_ledger = frame.get_visual("left_ledger")
    right_ledger = frame.get_visual("right_ledger")
    left_front_corner = frame.get_visual("left_front_corner")
    right_front_corner = frame.get_visual("right_front_corner")
    left_saddle = frame.get_visual("left_actuator_saddle")
    right_saddle = frame.get_visual("right_actuator_saddle")
    support_beam = frame.get_visual("actuator_support_beam")
    deck_plate = deck.get_visual("deck_plate")
    left_seat_pad = deck.get_visual("left_seat_pad")
    right_seat_pad = deck.get_visual("right_seat_pad")
    center_rib = deck.get_visual("center_rib")
    left_rib = deck.get_visual("left_rib")
    right_rib = deck.get_visual("right_rib")
    front_nose = deck.get_visual("front_nose")
    left_mount = deck.get_visual("left_actuator_mount")
    right_mount = deck.get_visual("right_actuator_mount")
    left_upper_eye = left_actuator.get_visual("upper_eye")
    right_upper_eye = right_actuator.get_visual("upper_eye")
    left_lower_clevis = left_actuator.get_visual("lower_clevis")
    right_lower_clevis = right_actuator.get_visual("lower_clevis")
    rear_hinge_pin = frame.get_visual("rear_hinge_pin")
    rear_hinge_barrel = deck.get_visual("center_rear_hinge_barrel")
    lip_hinge_pin = deck.get_visual("lip_hinge_pin")
    lip_center_barrel = lip.get_visual("center_hinge_barrel")
    toe_bar = lip.get_visual("toe_bar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap(deck, frame, reason="rear hinge pin runs through interleaved deck and frame barrels")
    ctx.allow_overlap(lip, deck, reason="front lip hinge pin runs through the deck nose hinge barrels")
    ctx.allow_overlap(left_actuator, frame, reason="left actuator clevis nests into the rear frame saddle as the deck lifts")
    ctx.allow_overlap(right_actuator, frame, reason="right actuator clevis nests into the rear frame saddle as the deck lifts")

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        deck,
        frame,
        axis="z",
        max_gap=0.002,
        max_penetration=0.003,
        positive_elem=left_seat_pad,
        negative_elem=left_ledger,
        name="left side of deck sits on the frame ledger",
    )
    ctx.expect_gap(
        deck,
        frame,
        axis="z",
        max_gap=0.002,
        max_penetration=0.003,
        positive_elem=right_seat_pad,
        negative_elem=right_ledger,
        name="right side of deck sits on the frame ledger",
    )
    ctx.expect_contact(
        left_actuator,
        frame,
        elem_a=left_lower_clevis,
        elem_b=left_saddle,
    )
    ctx.expect_contact(
        right_actuator,
        frame,
        elem_a=right_lower_clevis,
        elem_b=right_saddle,
    )
    ctx.expect_contact(
        deck,
        frame,
        elem_a=rear_hinge_barrel,
        elem_b=rear_hinge_pin,
    )
    ctx.expect_contact(
        lip,
        deck,
        elem_a=lip_center_barrel,
        elem_b=lip_hinge_pin,
    )
    ctx.expect_gap(
        deck,
        left_actuator,
        axis="z",
        max_gap=0.002,
        max_penetration=0.003,
        positive_elem=left_mount,
        negative_elem=left_upper_eye,
        name="left actuator reaches the deck underside mount",
    )
    ctx.expect_gap(
        deck,
        right_actuator,
        axis="z",
        max_gap=0.002,
        max_penetration=0.003,
        positive_elem=right_mount,
        negative_elem=right_upper_eye,
        name="right actuator reaches the deck underside mount",
    )
    ctx.expect_origin_distance(lip, deck, axes="y", max_dist=0.001)
    ctx.expect_gap(
        deck,
        lip,
        axis="z",
        min_gap=0.30,
        positive_elem=deck_plate,
        negative_elem=toe_bar,
        name="stored lip hangs below the deck nose",
    )
    ctx.expect_gap(
        deck,
        frame,
        axis="x",
        min_gap=0.12,
        positive_elem=center_rib,
        negative_elem=support_beam,
        name="center deck rib starts forward of the rear actuator beam",
    )
    ctx.expect_gap(
        deck,
        frame,
        axis="x",
        min_gap=0.20,
        positive_elem=left_rib,
        negative_elem=support_beam,
        name="left deck rib clears the rear actuator beam",
    )
    ctx.expect_gap(
        deck,
        frame,
        axis="x",
        min_gap=0.20,
        positive_elem=right_rib,
        negative_elem=support_beam,
        name="right deck rib clears the rear actuator beam",
    )

    with ctx.pose({deck_hinge: -0.28}):
        ctx.expect_gap(
            deck,
            frame,
            axis="z",
            min_gap=0.45,
            positive_elem=front_nose,
            negative_elem=left_front_corner,
            name="raised deck clears the left front edge of the pit frame",
        )
        ctx.expect_gap(
            deck,
            frame,
            axis="z",
            min_gap=0.45,
            positive_elem=front_nose,
            negative_elem=right_front_corner,
            name="raised deck clears the right front edge of the pit frame",
        )
        ctx.expect_contact(
            left_actuator,
            frame,
            elem_a=left_lower_clevis,
            elem_b=left_saddle,
        )
        ctx.expect_contact(
            right_actuator,
            frame,
            elem_a=right_lower_clevis,
            elem_b=right_saddle,
        )
        ctx.expect_gap(
            deck,
            frame,
            axis="z",
            min_gap=0.075,
            positive_elem=center_rib,
            negative_elem=support_beam,
            name="raised deck center rib lifts well above the rear actuator beam",
        )

    with ctx.pose({deck_hinge: -0.28, lip_hinge: -1.35}):
        ctx.expect_gap(
            lip,
            deck,
            axis="x",
            min_gap=0.18,
            positive_elem=toe_bar,
            negative_elem=front_nose,
            name="deployed lip projects out beyond the deck nose",
        )
        ctx.expect_origin_distance(lip, deck, axes="y", max_dist=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
