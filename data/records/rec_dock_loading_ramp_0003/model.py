from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)
SCRIPT_DIR = __file__.rpartition("/")[0] if "/" in __file__ else None
DECK_STOW_ANGLE = 0.82
LIP_FOLD_ANGLE = 1.05


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _shift_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_mount_plate_mesh():
    outer_profile = [
        (-0.37, -0.29),
        (0.37, -0.29),
        (0.37, 0.29),
        (-0.37, 0.29),
    ]
    bolt_hole = superellipse_profile(0.050, 0.050, exponent=2.0, segments=20)
    hole_profiles = [
        _shift_profile(bolt_hole, y_pos, z_pos)
        for z_pos in (-0.17, 0.17)
        for y_pos in (-0.27, -0.09, 0.09, 0.27)
    ]
    return _save_mesh(
        "edge_of_dock_mount_plate.obj",
        ExtrudeWithHolesGeometry(
            outer_profile,
            hole_profiles,
            height=0.022,
            center=True,
        ).rotate((1.0, 1.0, 1.0), 2.0 * pi / 3.0),
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="edge_of_dock_leveler", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.66, 0.67, 0.69, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.48, 0.50, 0.53, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.87, 0.72, 0.12, 1.0))

    dock = model.part("dock")
    dock.visual(
        Box((0.24, 1.60, 0.84)),
        origin=Origin(xyz=(-0.12, 0.0, -0.40)),
        material=concrete,
        name="dock_face",
    )
    dock.visual(
        Box((1.10, 1.60, 0.12)),
        origin=Origin(xyz=(-0.55, 0.0, -0.04)),
        material=concrete,
        name="dock_floor",
    )
    dock.visual(
        Box((1.02, 1.48, 0.004)),
        origin=Origin(xyz=(-0.51, 0.0, 0.018)),
        material=concrete,
        name="floor_surface",
    )
    dock.visual(
        Box((0.12, 1.06, 0.10)),
        origin=Origin(xyz=(-0.06, 0.0, -0.03)),
        material=dark_steel,
        name="edge_angle",
    )
    dock.inertial = Inertial.from_geometry(
        Box((1.10, 1.60, 0.96)),
        mass=2600.0,
        origin=Origin(xyz=(-0.43, 0.0, -0.26)),
    )

    frame = model.part("wall_frame")
    mount_plate_mesh = _build_mount_plate_mesh()
    frame.visual(
        mount_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=frame_steel,
        name="mount_plate",
    )
    frame.visual(
        Box((0.044, 0.74, 0.090)),
        origin=Origin(xyz=(0.022, 0.0, 0.330)),
        material=frame_steel,
        name="hinge_header",
    )
    frame.visual(
        Box((0.034, 0.68, 0.060)),
        origin=Origin(xyz=(0.017, 0.0, -0.215)),
        material=frame_steel,
        name="lower_stiffener",
    )
    frame.visual(
        Box((0.034, 0.085, 0.520)),
        origin=Origin(xyz=(0.017, 0.328, 0.020)),
        material=frame_steel,
        name="left_web",
    )
    frame.visual(
        Box((0.034, 0.085, 0.520)),
        origin=Origin(xyz=(0.017, -0.328, 0.020)),
        material=frame_steel,
        name="right_web",
    )
    frame.visual(
        Box((0.028, 0.090, 0.080)),
        origin=Origin(xyz=(0.050, 0.255, 0.330)),
        material=dark_steel,
        name="hinge_left",
    )
    frame.visual(
        Box((0.028, 0.090, 0.080)),
        origin=Origin(xyz=(0.050, -0.255, 0.330)),
        material=dark_steel,
        name="hinge_right",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.14, 1.02, 0.82)),
        mass=85.0,
        origin=Origin(xyz=(0.040, 0.0, -0.02)),
    )

    deck = model.part("deck")
    deck.visual(
        Box((0.024, 0.080, 0.052)),
        origin=Origin(xyz=(0.012, 0.255, -0.014)),
        material=dark_steel,
        name="deck_hinge_left",
    )
    deck.visual(
        Box((0.024, 0.080, 0.052)),
        origin=Origin(xyz=(0.012, -0.255, -0.014)),
        material=dark_steel,
        name="deck_hinge_right",
    )
    deck.visual(
        Box((0.56, 0.92, 0.022)),
        origin=Origin(xyz=(0.30, 0.0, -0.011)),
        material=deck_steel,
        name="deck_plate",
    )
    deck.visual(
        Box((0.54, 0.88, 0.002)),
        origin=Origin(xyz=(0.30, 0.0, 0.001)),
        material=deck_steel,
        name="deck_top_skin",
    )
    deck.visual(
        Box((0.030, 0.92, 0.032)),
        origin=Origin(xyz=(0.575, 0.0, -0.016)),
        material=frame_steel,
        name="nose_beam",
    )
    deck.visual(
        Box((0.42, 0.10, 0.07)),
        origin=Origin(xyz=(0.29, 0.24, -0.046)),
        material=frame_steel,
        name="stiffener_left",
    )
    deck.visual(
        Box((0.42, 0.10, 0.07)),
        origin=Origin(xyz=(0.29, -0.24, -0.046)),
        material=frame_steel,
        name="stiffener_right",
    )
    deck.visual(
        Box((0.32, 0.03, 0.09)),
        origin=Origin(xyz=(0.32, 0.445, -0.056)),
        material=frame_steel,
        name="side_rib_left",
    )
    deck.visual(
        Box((0.32, 0.03, 0.09)),
        origin=Origin(xyz=(0.32, -0.445, -0.056)),
        material=frame_steel,
        name="side_rib_right",
    )
    deck.visual(
        Box((0.15, 0.28, 0.010)),
        origin=Origin(xyz=(0.25, 0.0, -0.027)),
        material=frame_steel,
        name="handle_mount_pad",
    )
    for index, x_pos in enumerate((0.14, 0.27, 0.40, 0.51), start=1):
        deck.visual(
            Box((0.05, 0.76, 0.004)),
            origin=Origin(xyz=(x_pos, 0.0, 0.002)),
            material=safety_yellow,
            name=f"traction_bar_{index}",
        )
    deck.visual(
        Box((0.020, 0.060, 0.050)),
        origin=Origin(xyz=(0.570, 0.310, -0.022)),
        material=dark_steel,
        name="lip_hinge_left",
    )
    deck.visual(
        Box((0.020, 0.060, 0.050)),
        origin=Origin(xyz=(0.570, -0.310, -0.022)),
        material=dark_steel,
        name="lip_hinge_right",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.66, 0.96, 0.16)),
        mass=98.0,
        origin=Origin(xyz=(0.33, 0.0, -0.04)),
    )

    handle = model.part("lift_handle")
    handle.visual(
        Box((0.08, 0.03, 0.09)),
        origin=Origin(xyz=(0.0, 0.11, -0.045)),
        material=frame_steel,
        name="handle_cheek_right",
    )
    handle.visual(
        Box((0.08, 0.03, 0.09)),
        origin=Origin(xyz=(0.0, -0.11, -0.045)),
        material=frame_steel,
        name="handle_cheek_left",
    )
    handle.visual(
        Box((0.08, 0.22, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=frame_steel,
        name="handle_top_strap",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, -0.118), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handle_bar",
    )
    handle.visual(
        Box((0.022, 0.03, 0.090)),
        origin=Origin(xyz=(0.0, 0.11, -0.063)),
        material=dark_steel,
        name="handle_leg_right",
    )
    handle.visual(
        Box((0.022, 0.03, 0.090)),
        origin=Origin(xyz=(0.0, -0.11, -0.063)),
        material=dark_steel,
        name="handle_leg_left",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.14, 0.30, 0.16)),
        mass=8.0,
        origin=Origin(xyz=(0.02, 0.0, -0.07)),
    )

    lip = model.part("lip")
    lip.visual(
        Box((0.18, 0.84, 0.018)),
        origin=Origin(xyz=(0.10, 0.0, -0.009)),
        material=deck_steel,
        name="lip_plate",
    )
    lip.visual(
        Box((0.16, 0.80, 0.002)),
        origin=Origin(xyz=(0.10, 0.0, 0.001)),
        material=deck_steel,
        name="lip_top_skin",
    )
    lip.visual(
        Box((0.04, 0.84, 0.04)),
        origin=Origin(xyz=(0.18, 0.0, -0.020)),
        material=frame_steel,
        name="lip_front_angle",
    )
    lip.visual(
        Box((0.10, 0.08, 0.045)),
        origin=Origin(xyz=(0.11, 0.22, -0.024)),
        material=frame_steel,
        name="lip_rib_right",
    )
    lip.visual(
        Box((0.10, 0.08, 0.045)),
        origin=Origin(xyz=(0.11, -0.22, -0.024)),
        material=frame_steel,
        name="lip_rib_left",
    )
    lip.inertial = Inertial.from_geometry(
        Box((0.22, 0.90, 0.10)),
        mass=34.0,
        origin=Origin(xyz=(0.11, 0.0, -0.03)),
    )

    model.articulation(
        "frame_mount",
        ArticulationType.FIXED,
        parent=dock,
        child=frame,
        origin=Origin(xyz=(0.011, 0.0, -0.33)),
    )
    model.articulation(
        "deck_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=deck,
        origin=Origin(xyz=(0.038, 0.0, 0.350)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4500.0,
            velocity=1.4,
            lower=-DECK_STOW_ANGLE,
            upper=0.0,
        ),
    )
    model.articulation(
        "handle_mount",
        ArticulationType.FIXED,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(0.23, 0.0, -0.022)),
    )
    model.articulation(
        "lip_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip,
        origin=Origin(xyz=(0.580, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=1.8,
            lower=-LIP_FOLD_ANGLE,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SCRIPT_DIR) if SCRIPT_DIR else TestContext(object_model)
    dock = object_model.get_part("dock")
    frame = object_model.get_part("wall_frame")
    deck = object_model.get_part("deck")
    handle = object_model.get_part("lift_handle")
    lip = object_model.get_part("lip")
    deck_hinge = object_model.get_articulation("deck_hinge")
    lip_hinge = object_model.get_articulation("lip_hinge")
    dock_face = dock.get_visual("dock_face")
    floor_surface = dock.get_visual("floor_surface")
    mount_plate = frame.get_visual("mount_plate")
    frame_hinge_left = frame.get_visual("hinge_left")
    deck_hinge_left = deck.get_visual("deck_hinge_left")
    deck_plate = deck.get_visual("deck_plate")
    deck_top_skin = deck.get_visual("deck_top_skin")
    deck_lip_hinge_left = deck.get_visual("lip_hinge_left")
    nose_beam = deck.get_visual("nose_beam")
    handle_mount_pad = deck.get_visual("handle_mount_pad")
    handle_bar = handle.get_visual("handle_bar")
    handle_cheek_left = handle.get_visual("handle_cheek_left")
    handle_top_strap = handle.get_visual("handle_top_strap")
    lip_plate = lip.get_visual("lip_plate")
    lip_top_skin = lip.get_visual("lip_top_skin")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.02)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(deck, frame, reason="deck hinge tabs nest inside the wall-frame hinge brackets")
    ctx.allow_overlap(deck, lip, reason="lip rotates inside the nose hinge pockets at the deck front edge")
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
    ctx.expect_gap(
        frame,
        dock,
        axis="x",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem=mount_plate,
        negative_elem=dock_face,
        name="frame_plate_sits_flush_on_dock_face",
    )
    ctx.expect_overlap(
        frame,
        dock,
        axes="yz",
        min_overlap=0.45,
        name="frame_covers_dock_face_footprint",
    )
    ctx.expect_contact(
        deck,
        frame,
        elem_a=deck_hinge_left,
        elem_b=frame_hinge_left,
        name="deck_hinge_tab_seats_in_frame_bracket",
    )
    ctx.expect_gap(
        deck,
        dock,
        axis="x",
        min_gap=0.04,
        max_gap=0.25,
        positive_elem=deck_plate,
        negative_elem=dock_face,
        name="stored_deck_hangs_proud_of_wall",
    )
    ctx.expect_contact(
        handle,
        deck,
        elem_a=handle_cheek_left,
        elem_b=handle_mount_pad,
        name="lift_handle_bracket_is_bolted_to_deck_underside",
    )
    ctx.expect_gap(
        handle,
        handle,
        axis="z",
        min_gap=0.08,
        max_gap=0.12,
        positive_elem=handle_top_strap,
        negative_elem=handle_bar,
        name="lift_handle_bar_hangs_below_deck",
    )
    ctx.expect_gap(
        deck,
        dock,
        axis="z",
        max_gap=0.003,
        max_penetration=0.002,
        positive_elem=deck_top_skin,
        negative_elem=floor_surface,
        name="deployed_deck_runs_at_dock_floor_height",
    )
    ctx.expect_gap(
        lip,
        deck,
        axis="x",
        max_gap=0.010,
        max_penetration=0.005,
        positive_elem=lip_plate,
        negative_elem=nose_beam,
        name="lip_is_hinged_at_deck_nose",
    )
    ctx.expect_gap(
        lip,
        deck,
        axis="z",
        max_gap=0.004,
        max_penetration=0.004,
        positive_elem=lip_top_skin,
        negative_elem=deck_top_skin,
        name="extended_lip_continues_deck_top_surface",
    )
    with ctx.pose({deck_hinge: -DECK_STOW_ANGLE, lip_hinge: 0.0}):
        ctx.expect_gap(
            deck,
            dock,
            axis="x",
            min_gap=0.05,
            max_gap=0.20,
            positive_elem=deck_plate,
            negative_elem=dock_face,
            name="stowed_deck_hangs_proud_of_dock_face",
        )
    with ctx.pose({deck_hinge: -DECK_STOW_ANGLE, lip_hinge: -LIP_FOLD_ANGLE}):
        ctx.expect_gap(
            lip,
            deck,
            axis="z",
            min_gap=0.015,
            positive_elem=lip_top_skin,
            negative_elem=deck_top_skin,
            name="folded_lip_rises_clear_of_deck_surface",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
