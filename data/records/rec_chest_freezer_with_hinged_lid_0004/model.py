from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return _REAL_GETCWD()


os.getcwd = _safe_getcwd

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_chest_freezer")

    body_white = model.material("body_white", color=(0.93, 0.94, 0.95, 1.0))
    plinth_gray = model.material("plinth_gray", color=(0.26, 0.28, 0.30, 1.0))
    trim_gray = model.material("trim_gray", color=(0.56, 0.59, 0.62, 1.0))
    steel = model.material("steel", color=(0.69, 0.71, 0.73, 1.0))
    glass = model.material("glass", color=(0.73, 0.84, 0.92, 0.34))
    grille_black = model.material("grille_black", color=(0.15, 0.16, 0.17, 1.0))

    body_x = 1.60
    body_y = 0.92
    body_z = 0.85
    half_x = body_x / 2.0
    half_y = body_y / 2.0

    body = model.part("body")
    body.visual(
        Box((1.52, 0.84, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=plinth_gray,
        name="base_plinth",
    )
    body.visual(
        Box((1.44, 0.84, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=body_white,
        name="liner_floor",
    )
    body.visual(
        Box((0.04, body_y, 0.78)),
        origin=Origin(xyz=(-half_x + 0.02, 0.0, 0.39)),
        material=body_white,
        name="left_wall",
    )
    body.visual(
        Box((0.04, body_y, 0.78)),
        origin=Origin(xyz=(half_x - 0.02, 0.0, 0.39)),
        material=body_white,
        name="right_wall",
    )
    body.visual(
        Box((1.52, 0.04, 0.78)),
        origin=Origin(xyz=(0.0, -half_y + 0.02, 0.39)),
        material=body_white,
        name="front_wall",
    )
    body.visual(
        Box((1.52, 0.04, 0.78)),
        origin=Origin(xyz=(0.0, half_y - 0.02, 0.39)),
        material=body_white,
        name="back_wall",
    )
    body.visual(
        Box((1.40, 0.08, 0.07)),
        origin=Origin(xyz=(0.0, -0.42, 0.815)),
        material=trim_gray,
        name="front_rim",
    )
    body.visual(
        Box((1.40, 0.08, 0.07)),
        origin=Origin(xyz=(0.0, 0.42, 0.815)),
        material=trim_gray,
        name="rear_rim",
    )

    for side, sign in (("left", -1.0), ("right", 1.0)):
        body.visual(
            Box((0.108, 0.84, 0.03)),
            origin=Origin(xyz=(sign * 0.748, 0.0, 0.795)),
            material=trim_gray,
            name=f"{side}_side_deck",
        )
        body.visual(
            Box((0.026, 0.80, 0.045)),
            origin=Origin(xyz=(sign * 0.787, 0.0, 0.8275)),
            material=trim_gray,
            name=f"{side}_outer_rail",
        )
        body.visual(
            Box((0.090, 0.48, 0.014)),
            origin=Origin(xyz=(sign * 0.757, 0.0, 0.805)),
            material=steel,
            name=f"{side}_center_channel",
        )
        body.visual(
            Box((0.090, 0.16, 0.014)),
            origin=Origin(xyz=(sign * 0.757, -0.32, 0.805)),
            material=steel,
            name=f"{side}_front_release",
        )
        body.visual(
            Box((0.090, 0.16, 0.014)),
            origin=Origin(xyz=(sign * 0.757, 0.32, 0.805)),
            material=steel,
            name=f"{side}_rear_release",
        )
        body.visual(
            Box((0.020, 0.48, 0.028)),
            origin=Origin(xyz=(sign * 0.723, 0.0, 0.819)),
            material=trim_gray,
            name=f"{side}_inner_guide",
        )

    body.visual(
        Box((0.62, 0.008, 0.34)),
        origin=Origin(xyz=(0.0, 0.456, 0.49)),
        material=trim_gray,
        name="grille_mount_zone",
    )
    body.visual(
        Box((0.05, 0.018, 0.018)),
        origin=Origin(xyz=(-0.17, 0.481, 0.60)),
        material=grille_black,
        name="left_snap_tab",
    )
    body.visual(
        Box((0.018, 0.024, 0.030)),
        origin=Origin(xyz=(-0.17, 0.468, 0.60)),
        material=trim_gray,
        name="left_snap_stem",
    )
    body.visual(
        Box((0.05, 0.018, 0.018)),
        origin=Origin(xyz=(0.17, 0.481, 0.60)),
        material=grille_black,
        name="right_snap_tab",
    )
    body.visual(
        Box((0.018, 0.024, 0.030)),
        origin=Origin(xyz=(0.17, 0.468, 0.60)),
        material=trim_gray,
        name="right_snap_stem",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_x, body_y, body_z)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, body_z / 2.0)),
    )

    lid = model.part("lid_panel")
    lid_x_shift = 0.757
    lid_z_shift = 0.036
    lid.visual(
        Box((1.32, 0.50, 0.008)),
        origin=Origin(xyz=(lid_x_shift, 0.0, 0.006 + lid_z_shift)),
        material=glass,
        name="glass_pane",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        lid.visual(
            Box((0.032, 0.50, 0.050)),
            origin=Origin(xyz=(lid_x_shift + sign * 0.676, 0.0, 0.003 + lid_z_shift)),
            material=steel,
            name=f"{side}_end_bracket",
        )
        lid.visual(
            Box((0.084, 0.56, 0.014)),
            origin=Origin(xyz=(lid_x_shift + sign * 0.718, 0.0, 0.021 + lid_z_shift)),
            material=steel,
            name=f"{side}_top_carrier",
        )
        lid.visual(
            Box((0.012, 0.50, 0.046)),
            origin=Origin(xyz=(lid_x_shift + sign * 0.748, 0.0, -0.003 + lid_z_shift)),
            material=steel,
            name=f"{side}_slide_post",
        )
        lid.visual(
            Box((0.016, 0.50, 0.010)),
            origin=Origin(xyz=(lid_x_shift + sign * 0.752, 0.0, -0.031 + lid_z_shift)),
            material=steel,
            name=f"{side}_runner",
        )
    lid.inertial = Inertial.from_geometry(
        Box((1.36, 0.56, 0.08)),
        mass=11.0,
        origin=Origin(xyz=(lid_x_shift, 0.0, 0.032)),
    )

    grille = model.part("condenser_grille")
    grille.visual(
        Box((0.54, 0.012, 0.26)),
        origin=Origin(),
        material=grille_black,
        name="grille_panel",
    )
    for z in (-0.09, -0.05, -0.01, 0.03, 0.07):
        grille.visual(
            Box((0.46, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, 0.003, z)),
            material=steel,
            name=f"slat_{int((z + 0.1) * 1000):03d}",
        )
    grille.inertial = Inertial.from_geometry(
        Box((0.54, 0.012, 0.26)),
        mass=1.2,
        origin=Origin(),
    )

    model.articulation(
        "body_to_lid_panel",
        ArticulationType.PRISMATIC,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.757, 0.0, 0.812)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.5,
            lower=-0.12,
            upper=0.12,
        ),
    )
    model.articulation(
        "body_to_condenser_grille",
        ArticulationType.FIXED,
        parent=body,
        child=grille,
        origin=Origin(xyz=(0.0, 0.466, 0.49)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid_panel")
    grille = object_model.get_part("condenser_grille")
    lid_slide = object_model.get_articulation("body_to_lid_panel")

    left_runner = lid.get_visual("left_runner")
    right_runner = lid.get_visual("right_runner")
    glass_pane = lid.get_visual("glass_pane")
    left_center_channel = body.get_visual("left_center_channel")
    right_center_channel = body.get_visual("right_center_channel")
    left_front_release = body.get_visual("left_front_release")
    right_front_release = body.get_visual("right_front_release")
    left_rear_release = body.get_visual("left_rear_release")
    right_rear_release = body.get_visual("right_rear_release")
    left_outer_rail = body.get_visual("left_outer_rail")
    right_outer_rail = body.get_visual("right_outer_rail")
    front_rim = body.get_visual("front_rim")
    rear_rim = body.get_visual("rear_rim")
    grille_shell = grille.get_visual("grille_panel")
    grille_mount_zone = body.get_visual("grille_mount_zone")
    left_snap_tab = body.get_visual("left_snap_tab")
    right_snap_tab = body.get_visual("right_snap_tab")
    liner_floor = body.get_visual("liner_floor")
    left_end_bracket = lid.get_visual("left_end_bracket")
    right_end_bracket = lid.get_visual("right_end_bracket")

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

    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a=left_runner,
        elem_b=left_center_channel,
        min_overlap=0.010,
        name="left_runner_overlaps_center_slot",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a=right_runner,
        elem_b=right_center_channel,
        min_overlap=0.010,
        name="right_runner_overlaps_center_slot",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a=left_runner,
        elem_b=left_center_channel,
        name="left_runner_seated_on_center_slot",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a=right_runner,
        elem_b=right_center_channel,
        name="right_runner_seated_on_center_slot",
    )
    ctx.expect_within(
        lid,
        body,
        axes="x",
        inner_elem=left_runner,
        outer_elem=left_center_channel,
        name="left_runner_within_slot_width",
    )
    ctx.expect_within(
        lid,
        body,
        axes="x",
        inner_elem=right_runner,
        outer_elem=right_center_channel,
        name="right_runner_within_slot_width",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="y",
        elem_a=left_end_bracket,
        elem_b=left_outer_rail,
        min_overlap=0.40,
        name="left_end_bracket_runs_along_left_rail",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="y",
        elem_a=right_end_bracket,
        elem_b=right_outer_rail,
        min_overlap=0.40,
        name="right_end_bracket_runs_along_right_rail",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="x",
        positive_elem=left_end_bracket,
        negative_elem=left_outer_rail,
        min_gap=0.06,
        max_gap=0.10,
        name="left_end_bracket_stays_just_inboard_of_left_rail",
    )
    ctx.expect_gap(
        body,
        lid,
        axis="x",
        positive_elem=right_outer_rail,
        negative_elem=right_end_bracket,
        min_gap=0.06,
        max_gap=0.10,
        name="right_end_bracket_stays_just_inboard_of_right_rail",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a=glass_pane,
        elem_b=liner_floor,
        min_overlap=0.20,
        name="glass_panel_covers_top_opening",
    )

    with ctx.pose({lid_slide: -0.12}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=left_runner,
            elem_b=left_front_release,
            min_overlap=0.002,
            name="left_runner_reaches_front_release",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=right_runner,
            elem_b=right_front_release,
            min_overlap=0.002,
            name="right_runner_reaches_front_release",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a=left_runner,
            elem_b=left_front_release,
            name="left_runner_stays_seated_at_front_release",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a=right_runner,
            elem_b=right_front_release,
            name="right_runner_stays_seated_at_front_release",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            positive_elem=left_end_bracket,
            negative_elem=front_rim,
            min_gap=0.008,
            max_gap=0.02,
            name="left_end_bracket_clears_front_rim_for_lift_off",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            positive_elem=right_end_bracket,
            negative_elem=front_rim,
            min_gap=0.008,
            max_gap=0.02,
            name="right_end_bracket_clears_front_rim_for_lift_off",
        )

    with ctx.pose({lid_slide: 0.12}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=left_runner,
            elem_b=left_rear_release,
            min_overlap=0.002,
            name="left_runner_reaches_rear_release",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=right_runner,
            elem_b=right_rear_release,
            min_overlap=0.002,
            name="right_runner_reaches_rear_release",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a=left_runner,
            elem_b=left_rear_release,
            name="left_runner_stays_seated_at_rear_release",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a=right_runner,
            elem_b=right_rear_release,
            name="right_runner_stays_seated_at_rear_release",
        )
        ctx.expect_gap(
            body,
            lid,
            axis="y",
            positive_elem=rear_rim,
            negative_elem=left_end_bracket,
            min_gap=0.008,
            max_gap=0.02,
            name="left_end_bracket_clears_rear_rim_for_lift_off",
        )
        ctx.expect_gap(
            body,
            lid,
            axis="y",
            positive_elem=rear_rim,
            negative_elem=right_end_bracket,
            min_gap=0.008,
            max_gap=0.02,
            name="right_end_bracket_clears_rear_rim_for_lift_off",
        )

    ctx.expect_overlap(
        grille,
        body,
        axes="xz",
        elem_a=grille_shell,
        elem_b=grille_mount_zone,
        min_overlap=0.12,
        name="condenser_grille_covers_back_mount_zone",
    )
    ctx.expect_gap(
        grille,
        body,
        axis="y",
        positive_elem=grille_shell,
        negative_elem=grille_mount_zone,
        max_gap=0.001,
        max_penetration=0.0,
        name="condenser_grille_sits_flush_on_back_face",
    )
    ctx.expect_contact(
        grille,
        body,
        elem_a=grille_shell,
        elem_b=left_snap_tab,
        name="left_snap_tab_contacts_grille",
    )
    ctx.expect_contact(
        grille,
        body,
        elem_a=grille_shell,
        elem_b=right_snap_tab,
        name="right_snap_tab_contacts_grille",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
