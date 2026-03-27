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

CABINET_W = 0.598
CABINET_D = 0.555
CABINET_H = 0.806
SHELL_D = 0.515

FOOT_PAD_H = 0.009
FOOT_STEM_H = 0.021
BASE_Z = FOOT_PAD_H + FOOT_STEM_H
CABINET_TOP_Z = BASE_Z + CABINET_H

FRONT_PANEL_T = 0.042
SHELL_CENTER_Y = -((CABINET_D - SHELL_D) * 0.5)
SHELL_FRONT_Y = SHELL_CENTER_Y + (SHELL_D * 0.5)
FRONT_FRAME_CENTER_Y = SHELL_FRONT_Y + (FRONT_PANEL_T * 0.5)
DOOR_AXIS_Y = FRONT_FRAME_CENTER_Y + (FRONT_PANEL_T * 0.5)

DOOR_CENTER_Z = 0.405
OPENING_RADIUS = 0.188
DOOR_OUTER_RADIUS = 0.205
DOOR_GLASS_RADIUS = 0.151
HINGE_Z_OFFSET = 0.146

DOOR_AXIS_X = -((CABINET_W * 0.5) + 0.022)
DOOR_CENTER_OFFSET = -DOOR_AXIS_X

CONTROL_W = 0.548
CONTROL_D = 0.094
CONTROL_H = 0.022
CONTROL_CENTER_Y = 0.188

ASSETS = AssetContext.from_script(__file__)


def _add_ring_segments(
    part,
    *,
    center_x: float,
    center_z: float,
    inner_radius: float,
    outer_radius: float,
    depth: float,
    center_y: float,
    segments: int,
    material,
    name_prefix: str,
) -> None:
    radial_thickness = outer_radius - inner_radius
    mid_radius = inner_radius + (radial_thickness * 0.5)
    tangential_length = ((2.0 * math.pi * mid_radius) / segments) * 1.04
    for index in range(segments):
        theta = (2.0 * math.pi * index) / segments
        part.visual(
            Box((tangential_length, depth, radial_thickness)),
            origin=Origin(
                xyz=(
                    center_x + (mid_radius * math.cos(theta)),
                    center_y,
                    center_z + (mid_radius * math.sin(theta)),
                ),
                rpy=(0.0, -(theta + (math.pi * 0.5)), 0.0),
            ),
            material=material,
            name=f"{name_prefix}_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stackable_dryer", assets=ASSETS)

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.95, 0.96, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.76, 0.78, 0.81, 1.0))
    control_dark = model.material("control_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    knob_dark = model.material("knob_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    button_grey = model.material("button_grey", rgba=(0.80, 0.82, 0.84, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.28, 0.34, 0.39, 0.42))
    cavity_dark = model.material("cavity_dark", rgba=(0.11, 0.12, 0.13, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    foot_black = model.material("foot_black", rgba=(0.09, 0.09, 0.10, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CABINET_W, SHELL_D, CABINET_H)),
        origin=Origin(xyz=(0.0, SHELL_CENTER_Y, BASE_Z + (CABINET_H * 0.5))),
        material=cabinet_white,
        name="cabinet_shell",
    )

    front_outer_w = 0.548
    front_outer_h = 0.695
    top_bottom_frame_h = (front_outer_h - (2.0 * OPENING_RADIUS)) * 0.5
    side_frame_w = (front_outer_w - (2.0 * OPENING_RADIUS)) * 0.5

    cabinet.visual(
        Box((front_outer_w, FRONT_PANEL_T, top_bottom_frame_h)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_FRAME_CENTER_Y,
                DOOR_CENTER_Z + OPENING_RADIUS + (top_bottom_frame_h * 0.5),
            ),
        ),
        material=cabinet_white,
        name="front_top_frame",
    )
    cabinet.visual(
        Box((front_outer_w, FRONT_PANEL_T, top_bottom_frame_h)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_FRAME_CENTER_Y,
                DOOR_CENTER_Z - OPENING_RADIUS - (top_bottom_frame_h * 0.5),
            ),
        ),
        material=cabinet_white,
        name="front_bottom_frame",
    )
    cabinet.visual(
        Box((side_frame_w, FRONT_PANEL_T, OPENING_RADIUS * 2.0)),
        origin=Origin(
            xyz=(
                OPENING_RADIUS + (side_frame_w * 0.5),
                FRONT_FRAME_CENTER_Y,
                DOOR_CENTER_Z,
            ),
        ),
        material=cabinet_white,
        name="front_right_frame",
    )
    cabinet.visual(
        Box((side_frame_w, FRONT_PANEL_T, OPENING_RADIUS * 2.0)),
        origin=Origin(
            xyz=(
                -(OPENING_RADIUS + (side_frame_w * 0.5)),
                FRONT_FRAME_CENTER_Y,
                DOOR_CENTER_Z,
            ),
        ),
        material=cabinet_white,
        name="front_left_frame",
    )
    cabinet.visual(
        Cylinder(radius=0.178, length=0.032),
        origin=Origin(
            xyz=(0.0, SHELL_FRONT_Y + 0.016, DOOR_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_silver,
        name="drum_rim",
    )
    cabinet.visual(
        Cylinder(radius=0.144, length=0.150),
        origin=Origin(
            xyz=(0.0, SHELL_FRONT_Y - 0.075, DOOR_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=cavity_dark,
        name="drum_cavity",
    )
    cabinet.visual(
        Box((0.178, 0.012, 0.058)),
        origin=Origin(
            xyz=(DOOR_AXIS_X + 0.089, DOOR_AXIS_Y - 0.006, DOOR_CENTER_Z + HINGE_Z_OFFSET),
        ),
        material=hinge_dark,
        name="upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.178, 0.012, 0.058)),
        origin=Origin(
            xyz=(DOOR_AXIS_X + 0.089, DOOR_AXIS_Y - 0.006, DOOR_CENTER_Z - HINGE_Z_OFFSET),
        ),
        material=hinge_dark,
        name="lower_hinge_mount",
    )
    cabinet.visual(
        Box((0.040, 0.012, 0.048)),
        origin=Origin(xyz=(0.188, DOOR_AXIS_Y - 0.006, DOOR_CENTER_Z)),
        material=hinge_dark,
        name="latch_striker",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.320),
        origin=Origin(
            xyz=(DOOR_AXIS_X + 0.012, DOOR_AXIS_Y - 0.007, DOOR_CENTER_Z),
        ),
        material=hinge_dark,
        name="hinge_backbone",
    )

    foot_positions = {
        "front_left": (-0.255, 0.228),
        "front_right": (0.255, 0.228),
        "rear_left": (-0.255, -0.228),
        "rear_right": (0.255, -0.228),
    }
    for label, (x_pos, y_pos) in foot_positions.items():
        cabinet.visual(
            Cylinder(radius=0.018, length=FOOT_PAD_H),
            origin=Origin(xyz=(x_pos, y_pos, FOOT_PAD_H * 0.5)),
            material=foot_black,
            name=f"{label}_foot_pad",
        )
        cabinet.visual(
            Cylinder(radius=0.007, length=FOOT_STEM_H),
            origin=Origin(xyz=(x_pos, y_pos, FOOT_PAD_H + (FOOT_STEM_H * 0.5))),
            material=foot_black,
            name=f"{label}_foot_stem",
        )

    cabinet.visual(
        Box((CONTROL_W, CONTROL_D, CONTROL_H)),
        origin=Origin(xyz=(0.0, CONTROL_CENTER_Y, CABINET_TOP_Z + (CONTROL_H * 0.5))),
        material=control_dark,
        name="control_strip",
    )
    cabinet.visual(
        Box((CONTROL_W, 0.016, 0.008)),
        origin=Origin(
            xyz=(
                0.0,
                CONTROL_CENTER_Y + (CONTROL_D * 0.5) - 0.008,
                CABINET_TOP_Z + 0.004,
            ),
        ),
        material=trim_silver,
        name="control_front_lip",
    )
    cabinet.visual(
        Cylinder(radius=0.028, length=0.028),
        origin=Origin(
            xyz=(-0.165, CONTROL_CENTER_Y, CABINET_TOP_Z + CONTROL_H + 0.014),
        ),
        material=knob_dark,
        name="program_knob",
    )
    cabinet.visual(
        Cylinder(radius=0.020, length=0.030),
        origin=Origin(
            xyz=(-0.165, CONTROL_CENTER_Y, CABINET_TOP_Z + CONTROL_H + 0.015),
        ),
        material=trim_silver,
        name="program_knob_cap",
    )
    cabinet.visual(
        Box((0.004, 0.012, 0.003)),
        origin=Origin(
            xyz=(-0.165, CONTROL_CENTER_Y + 0.020, CABINET_TOP_Z + CONTROL_H + 0.0295),
        ),
        material=button_grey,
        name="program_knob_marker",
    )
    for name, x_pos in (
        ("start_button", 0.118),
        ("dryness_button", 0.172),
        ("options_button", 0.226),
    ):
        cabinet.visual(
            Box((0.032, 0.016, 0.007)),
            origin=Origin(
                xyz=(x_pos, CONTROL_CENTER_Y, CABINET_TOP_Z + CONTROL_H + 0.0035),
            ),
            material=button_grey,
            name=name,
        )

    cabinet.inertial = Inertial.from_geometry(
        Box((CABINET_W, CABINET_D, CABINET_H + BASE_Z)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, (CABINET_H + BASE_Z) * 0.5)),
    )

    door = model.part("door")
    _add_ring_segments(
        door,
        center_x=DOOR_CENTER_OFFSET,
        center_z=0.0,
        inner_radius=0.153,
        outer_radius=DOOR_OUTER_RADIUS,
        depth=0.036,
        center_y=0.018,
        segments=16,
        material=trim_silver,
        name_prefix="door_outer_ring",
    )
    _add_ring_segments(
        door,
        center_x=DOOR_CENTER_OFFSET,
        center_z=0.0,
        inner_radius=0.150,
        outer_radius=0.184,
        depth=0.020,
        center_y=0.010,
        segments=12,
        material=control_dark,
        name_prefix="door_inner_ring",
    )
    door.visual(
        Cylinder(radius=DOOR_GLASS_RADIUS, length=0.012),
        origin=Origin(
            xyz=(DOOR_CENTER_OFFSET, 0.012, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=glass_smoke,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.066),
        origin=Origin(xyz=(0.0, 0.011, HINGE_Z_OFFSET)),
        material=hinge_dark,
        name="upper_knuckle",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.066),
        origin=Origin(xyz=(0.0, 0.011, -HINGE_Z_OFFSET)),
        material=hinge_dark,
        name="lower_knuckle",
    )
    door.visual(
        Cylinder(radius=0.005, length=0.320),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=trim_silver,
        name="hinge_pin",
    )
    door.visual(
        Box((0.178, 0.012, 0.058)),
        origin=Origin(xyz=(0.089, 0.006, HINGE_Z_OFFSET)),
        material=trim_silver,
        name="upper_leaf",
    )
    door.visual(
        Box((0.178, 0.012, 0.058)),
        origin=Origin(xyz=(0.089, 0.006, -HINGE_Z_OFFSET)),
        material=trim_silver,
        name="lower_leaf",
    )
    door.visual(
        Box((0.038, 0.012, 0.044)),
        origin=Origin(
            xyz=(DOOR_CENTER_OFFSET + DOOR_OUTER_RADIUS - 0.018, 0.006, 0.0),
        ),
        material=hinge_dark,
        name="latch_catch",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_CENTER_OFFSET + DOOR_OUTER_RADIUS, 0.040, 0.470)),
        mass=5.8,
        origin=Origin(
            xyz=((DOOR_CENTER_OFFSET + DOOR_OUTER_RADIUS) * 0.5, 0.020, 0.0),
        ),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(DOOR_AXIS_X, DOOR_AXIS_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("cabinet_to_door")

    cabinet_shell = cabinet.get_visual("cabinet_shell")
    front_top_frame = cabinet.get_visual("front_top_frame")
    upper_mount = cabinet.get_visual("upper_hinge_mount")
    lower_mount = cabinet.get_visual("lower_hinge_mount")
    striker = cabinet.get_visual("latch_striker")
    drum_rim = cabinet.get_visual("drum_rim")
    control_strip = cabinet.get_visual("control_strip")
    knob_body = cabinet.get_visual("program_knob")
    start_button = cabinet.get_visual("start_button")
    dryness_button = cabinet.get_visual("dryness_button")
    options_button = cabinet.get_visual("options_button")
    front_left_foot = cabinet.get_visual("front_left_foot_pad")
    front_right_foot = cabinet.get_visual("front_right_foot_pad")
    rear_left_foot = cabinet.get_visual("rear_left_foot_pad")
    rear_right_foot = cabinet.get_visual("rear_right_foot_pad")

    door_top_ring = door.get_visual("door_outer_ring_4")
    door_glass = door.get_visual("door_glass")
    upper_knuckle = door.get_visual("upper_knuckle")
    lower_knuckle = door.get_visual("lower_knuckle")
    upper_leaf = door.get_visual("upper_leaf")
    lower_leaf = door.get_visual("lower_leaf")
    latch_catch = door.get_visual("latch_catch")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(
        door,
        cabinet,
        axes="xz",
        inner_elem=door_glass,
        outer_elem=drum_rim,
        name="door_glass_centers_over_round_porthole",
    )
    ctx.expect_gap(
        door,
        cabinet,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=door_top_ring,
        negative_elem=front_top_frame,
        name="door_sits_flush_to_front_frame",
    )
    ctx.expect_within(
        door,
        door,
        axes="xz",
        inner_elem=door_glass,
        name="glass_window_sits_within_round_door",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a=upper_knuckle,
        elem_b=upper_mount,
        name="upper_exposed_knuckle_seats_at_left_hinge",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a=lower_knuckle,
        elem_b=lower_mount,
        name="lower_exposed_knuckle_seats_at_left_hinge",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a=upper_leaf,
        elem_b=upper_mount,
        name="upper_hinge_leaf_contacts_mount",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a=lower_leaf,
        elem_b=lower_mount,
        name="lower_hinge_leaf_contacts_mount",
    )
    ctx.expect_contact(
        door,
        cabinet,
        elem_a=latch_catch,
        elem_b=striker,
        name="single_right_side_catch_meets_striker",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=control_strip,
        negative_elem=cabinet_shell,
        name="control_strip_sits_on_top_face",
    )
    ctx.expect_within(
        cabinet,
        cabinet,
        axes="xy",
        inner_elem=control_strip,
        outer_elem=cabinet_shell,
        name="control_strip_stays_within_cabinet_plan",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=knob_body,
        negative_elem=control_strip,
        name="program_knob_sits_on_control_strip",
    )
    ctx.expect_within(
        cabinet,
        cabinet,
        axes="xy",
        inner_elem=knob_body,
        outer_elem=control_strip,
        name="program_knob_stays_on_strip",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=start_button,
        negative_elem=control_strip,
        name="start_button_sits_on_control_strip",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=dryness_button,
        negative_elem=control_strip,
        name="dryness_button_sits_on_control_strip",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=options_button,
        negative_elem=control_strip,
        name="options_button_sits_on_control_strip",
    )
    ctx.expect_within(
        cabinet,
        cabinet,
        axes="xy",
        inner_elem=start_button,
        outer_elem=control_strip,
        name="start_button_within_strip",
    )
    ctx.expect_within(
        cabinet,
        cabinet,
        axes="xy",
        inner_elem=dryness_button,
        outer_elem=control_strip,
        name="dryness_button_within_strip",
    )
    ctx.expect_within(
        cabinet,
        cabinet,
        axes="xy",
        inner_elem=options_button,
        outer_elem=control_strip,
        name="options_button_within_strip",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        min_gap=0.018,
        positive_elem=cabinet_shell,
        negative_elem=front_left_foot,
        name="front_left_foot_drops_below_cabinet_shell",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        min_gap=0.018,
        positive_elem=cabinet_shell,
        negative_elem=front_right_foot,
        name="front_right_foot_drops_below_cabinet_shell",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        min_gap=0.018,
        positive_elem=cabinet_shell,
        negative_elem=rear_left_foot,
        name="rear_left_foot_drops_below_cabinet_shell",
    )
    ctx.expect_gap(
        cabinet,
        cabinet,
        axis="z",
        min_gap=0.018,
        positive_elem=cabinet_shell,
        negative_elem=rear_right_foot,
        name="rear_right_foot_drops_below_cabinet_shell",
    )

    with ctx.pose({door_hinge: 1.2}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            min_gap=0.14,
            positive_elem=latch_catch,
            negative_elem=striker,
            name="door_latch_swings_clear_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
