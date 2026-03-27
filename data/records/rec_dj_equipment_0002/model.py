from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
from math import pi

try:
    os.chdir("/")
except FileNotFoundError:
    pass


def _ensure_safe_cwd() -> None:
    try:
        os.getcwd()
    except FileNotFoundError:
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


def _build_platter_part(
    model: ArticulatedObject,
    name: str,
    platter_metal,
    slipmat_dark,
    spindle_metal,
):
    platter = model.part(name)
    platter.visual(
        Cylinder(radius=0.128, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
        material=platter_metal,
        name="platter_disk",
    )
    platter.visual(
        Cylinder(radius=0.114, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=slipmat_dark,
        name="slip_mat",
    )
    platter.visual(
        Cylinder(radius=0.004, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=spindle_metal,
        name="spindle_hub",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.128, length=0.020),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    return platter


def _build_tonearm_part(
    model: ArticulatedObject,
    name: str,
    arm_metal,
    counterweight_metal,
    stylus_metal,
):
    tonearm = model.part(name)
    tonearm.visual(
        Box((0.006, 0.018, 0.010)),
        origin=Origin(xyz=(-0.009, 0.006, 0.0136)),
        material=arm_metal,
        name="hub_left_cheek",
    )
    tonearm.visual(
        Box((0.006, 0.018, 0.010)),
        origin=Origin(xyz=(0.009, 0.006, 0.0136)),
        material=arm_metal,
        name="hub_right_cheek",
    )
    tonearm.visual(
        Box((0.024, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.015, 0.0136)),
        material=arm_metal,
        name="hub",
    )
    tonearm.visual(
        Cylinder(radius=0.0032, length=0.026),
        origin=Origin(xyz=(0.0, 0.031, 0.016), rpy=(pi / 2.0, 0.0, 0.0)),
        material=arm_metal,
        name="rear_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.047, 0.016), rpy=(pi / 2.0, 0.0, 0.0)),
        material=counterweight_metal,
        name="counterweight",
    )
    tonearm.visual(
        Cylinder(radius=0.0042, length=0.128),
        origin=Origin(xyz=(0.0, -0.048, 0.020), rpy=(pi / 2.0, 0.0, 0.0)),
        material=arm_metal,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.010, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, -0.126, 0.0225), rpy=(0.24, 0.0, 0.0)),
        material=arm_metal,
        name="arm_tube_front",
    )
    tonearm.visual(
        Box((0.008, 0.020, 0.005)),
        origin=Origin(xyz=(0.0, -0.137, 0.0240), rpy=(0.32, 0.0, 0.0)),
        material=arm_metal,
        name="headshell_neck",
    )
    tonearm.visual(
        Box((0.018, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, -0.147, 0.0255), rpy=(0.40, 0.0, 0.0)),
        material=arm_metal,
        name="headshell",
    )
    tonearm.visual(
        Cylinder(radius=0.0014, length=0.006),
        origin=Origin(xyz=(0.0, -0.158, 0.023)),
        material=stylus_metal,
        name="stylus",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.055, 0.230, 0.050)),
        mass=0.25,
        origin=Origin(xyz=(0.0, -0.070, 0.024)),
    )
    return tonearm


def _build_fader_part(
    model: ArticulatedObject,
    name: str,
    cap_size: tuple[float, float, float],
    stem_size: tuple[float, float, float],
    cap_material,
    stem_material,
):
    fader = model.part(name)
    fader.visual(
        Box(stem_size),
        origin=Origin(xyz=(0.0, 0.0, stem_size[2] * 0.5)),
        material=stem_material,
        name="stem",
    )
    fader.visual(
        Box(cap_size),
        origin=Origin(xyz=(0.0, 0.0, stem_size[2] + cap_size[2] * 0.5)),
        material=cap_material,
        name="cap",
    )
    fader.inertial = Inertial.from_geometry(
        Box(
            (
                max(cap_size[0], stem_size[0]),
                max(cap_size[1], stem_size[1]),
                stem_size[2] + cap_size[2],
            )
        ),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, (stem_size[2] + cap_size[2]) * 0.5)),
    )
    return fader


def build_object_model() -> ArticulatedObject:
    _ensure_safe_cwd()
    model = ArticulatedObject(name="club_standard_dj_setup")

    plinth_black = model.material("plinth_black", rgba=(0.08, 0.09, 0.10, 1.0))
    deck_black = model.material("deck_black", rgba=(0.12, 0.13, 0.14, 1.0))
    mixer_black = model.material("mixer_black", rgba=(0.10, 0.10, 0.11, 1.0))
    slot_black = model.material("slot_black", rgba=(0.03, 0.03, 0.03, 1.0))
    platter_metal = model.material("platter_metal", rgba=(0.56, 0.58, 0.60, 1.0))
    spindle_metal = model.material("spindle_metal", rgba=(0.80, 0.81, 0.84, 1.0))
    slipmat_dark = model.material("slipmat_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    arm_metal = model.material("arm_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    counterweight_metal = model.material("counterweight_metal", rgba=(0.36, 0.37, 0.40, 1.0))
    control_metal = model.material("control_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    fader_cap = model.material("fader_cap", rgba=(0.76, 0.78, 0.80, 1.0))
    white_mark = model.material("white_mark", rgba=(0.92, 0.93, 0.94, 1.0))

    console = model.part("console")
    console.visual(
        Box((1.22, 0.43, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=plinth_black,
        name="base_plinth",
    )
    console.visual(
        Box((0.36, 0.37, 0.015)),
        origin=Origin(xyz=(-0.355, 0.0, 0.0375)),
        material=deck_black,
        name="left_deck_block",
    )
    console.visual(
        Box((0.36, 0.37, 0.015)),
        origin=Origin(xyz=(0.355, 0.0, 0.0375)),
        material=deck_black,
        name="right_deck_block",
    )
    console.visual(
        Box((0.348, 0.358, 0.002)),
        origin=Origin(xyz=(-0.355, 0.0, 0.044)),
        material=deck_black,
        name="left_deck_surface",
    )
    console.visual(
        Box((0.348, 0.358, 0.002)),
        origin=Origin(xyz=(0.355, 0.0, 0.044)),
        material=deck_black,
        name="right_deck_surface",
    )
    console.visual(
        Cylinder(radius=0.138, length=0.003),
        origin=Origin(xyz=(-0.355, 0.0, 0.0465)),
        material=slot_black,
        name="left_platter_well",
    )
    console.visual(
        Cylinder(radius=0.138, length=0.003),
        origin=Origin(xyz=(0.355, 0.0, 0.0465)),
        material=slot_black,
        name="right_platter_well",
    )
    console.visual(
        Box((0.26, 0.37, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=mixer_black,
        name="mixer_block",
    )
    console.visual(
        Box((0.24, 0.35, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=mixer_black,
        name="mixer_top",
    )
    console.visual(
        Box((0.164, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, -0.134, 0.0615)),
        material=slot_black,
        name="crossfader_channel",
    )

    channel_x_positions = (-0.072, -0.024, 0.024, 0.072)
    channel_cap_y_positions = (-0.026, -0.006, 0.014, -0.016)
    for index, x_pos in enumerate(channel_x_positions):
        console.visual(
            Box((0.012, 0.156, 0.003)),
            origin=Origin(xyz=(x_pos, 0.014, 0.0615)),
            material=slot_black,
            name=f"channel_slot_{index}",
        )
        console.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(x_pos, 0.150, 0.071)),
            material=control_metal,
            name=f"eq_knob_upper_{index}",
        )
        console.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(x_pos, 0.120, 0.071)),
            material=control_metal,
            name=f"eq_knob_mid_{index}",
        )
        console.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(x_pos, 0.090, 0.071)),
            material=control_metal,
            name=f"eq_knob_low_{index}",
        )
        console.visual(
            Box((0.006, 0.169, 0.0015)),
            origin=Origin(xyz=(x_pos, 0.014, 0.06475)),
            material=white_mark,
            name=f"channel_scale_{index}",
        )
        console.visual(
            Box((0.012, 0.008, 0.003)),
            origin=Origin(xyz=(x_pos, channel_cap_y_positions[index], 0.0615)),
            material=slot_black,
            name=f"channel_cap_shadow_{index}",
        )

    console.visual(
        Box((0.190, 0.006, 0.0015)),
        origin=Origin(xyz=(0.0, -0.134, 0.06475)),
        material=white_mark,
        name="crossfader_scale",
    )

    for side_name, pivot_x in (("left", -0.210), ("right", 0.505)):
        console.visual(
            Cylinder(radius=0.016, length=0.008),
            origin=Origin(xyz=(pivot_x, 0.112, 0.049)),
            material=counterweight_metal,
            name=f"{side_name}_tonearm_bearing",
        )
        console.visual(
            Cylinder(radius=0.0035, length=0.008),
            origin=Origin(xyz=(pivot_x, 0.112, 0.057)),
            material=arm_metal,
            name=f"{side_name}_tonearm_pin",
        )
        console.visual(
            Box((0.010, 0.020, 0.010)),
            origin=Origin(xyz=(pivot_x + 0.018, 0.076, 0.050)),
            material=control_metal,
            name=f"{side_name}_armrest",
        )

    console.inertial = Inertial.from_geometry(
        Box((1.22, 0.43, 0.08)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    left_platter = _build_platter_part(model, "left_platter", platter_metal, slipmat_dark, spindle_metal)
    right_platter = _build_platter_part(model, "right_platter", platter_metal, slipmat_dark, spindle_metal)
    _build_tonearm_part(
        model,
        "left_tonearm",
        arm_metal,
        counterweight_metal,
        spindle_metal,
    )
    _build_tonearm_part(
        model,
        "right_tonearm",
        arm_metal,
        counterweight_metal,
        spindle_metal,
    )

    _build_fader_part(
        model,
        "crossfader",
        cap_size=(0.026, 0.018, 0.005),
        stem_size=(0.006, 0.010, 0.004),
        cap_material=fader_cap,
        stem_material=control_metal,
    )

    for index in range(4):
        _build_fader_part(
            model,
            f"channel_fader_{index}",
            cap_size=(0.018, 0.026, 0.005),
            stem_size=(0.006, 0.010, 0.004),
            cap_material=fader_cap,
            stem_material=control_metal,
        )

    model.articulation(
        "left_platter_spin",
        ArticulationType.CONTINUOUS,
        parent=console,
        child=left_platter,
        origin=Origin(xyz=(-0.355, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "right_platter_spin",
        ArticulationType.CONTINUOUS,
        parent=console,
        child=right_platter,
        origin=Origin(xyz=(0.355, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "left_tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=console,
        child="left_tonearm",
        origin=Origin(xyz=(-0.210, 0.112, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.5, lower=-0.90, upper=0.10),
    )
    model.articulation(
        "right_tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=console,
        child="right_tonearm",
        origin=Origin(xyz=(0.505, 0.112, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.5, lower=-0.90, upper=0.10),
    )
    model.articulation(
        "crossfader_slide",
        ArticulationType.PRISMATIC,
        parent=console,
        child="crossfader",
        origin=Origin(xyz=(0.0, -0.134, 0.063)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.4, lower=-0.055, upper=0.055),
    )

    for index, x_pos in enumerate(channel_x_positions):
        model.articulation(
            f"channel_fader_slide_{index}",
            ArticulationType.PRISMATIC,
            parent=console,
            child=f"channel_fader_{index}",
            origin=Origin(xyz=(x_pos, channel_cap_y_positions[index], 0.063)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.4, lower=-0.052, upper=0.052),
        )

    return model


def run_tests() -> TestReport:
    _ensure_safe_cwd()
    ctx = TestContext(object_model, asset_root="/")
    console = object_model.get_part("console")
    left_platter = object_model.get_part("left_platter")
    right_platter = object_model.get_part("right_platter")
    left_tonearm = object_model.get_part("left_tonearm")
    right_tonearm = object_model.get_part("right_tonearm")
    crossfader = object_model.get_part("crossfader")
    channel_faders = [object_model.get_part(f"channel_fader_{index}") for index in range(4)]

    left_platter_spin = object_model.get_articulation("left_platter_spin")
    right_platter_spin = object_model.get_articulation("right_platter_spin")
    left_tonearm_swing = object_model.get_articulation("left_tonearm_swing")
    right_tonearm_swing = object_model.get_articulation("right_tonearm_swing")
    crossfader_slide = object_model.get_articulation("crossfader_slide")
    channel_fader_slides = [
        object_model.get_articulation(f"channel_fader_slide_{index}") for index in range(4)
    ]

    left_deck_surface = console.get_visual("left_deck_surface")
    right_deck_surface = console.get_visual("right_deck_surface")
    crossfader_channel = console.get_visual("crossfader_channel")
    left_tonearm_bearing = console.get_visual("left_tonearm_bearing")
    left_tonearm_pin = console.get_visual("left_tonearm_pin")
    right_tonearm_bearing = console.get_visual("right_tonearm_bearing")
    right_tonearm_pin = console.get_visual("right_tonearm_pin")
    channel_slots = [console.get_visual(f"channel_slot_{index}") for index in range(4)]

    left_platter_disk = left_platter.get_visual("platter_disk")
    right_platter_disk = right_platter.get_visual("platter_disk")
    left_slip_mat = left_platter.get_visual("slip_mat")
    right_slip_mat = right_platter.get_visual("slip_mat")
    left_spindle = left_platter.get_visual("spindle_hub")
    right_spindle = right_platter.get_visual("spindle_hub")

    left_tonearm_hub = left_tonearm.get_visual("hub")
    right_tonearm_hub = right_tonearm.get_visual("hub")
    left_hub_left_cheek = left_tonearm.get_visual("hub_left_cheek")
    left_hub_right_cheek = left_tonearm.get_visual("hub_right_cheek")
    right_hub_left_cheek = right_tonearm.get_visual("hub_left_cheek")
    right_hub_right_cheek = right_tonearm.get_visual("hub_right_cheek")
    left_headshell = left_tonearm.get_visual("headshell")
    right_headshell = right_tonearm.get_visual("headshell")
    left_stylus = left_tonearm.get_visual("stylus")
    right_stylus = right_tonearm.get_visual("stylus")

    crossfader_stem = crossfader.get_visual("stem")
    channel_stems = [fader.get_visual("stem") for fader in channel_faders]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        left_platter,
        console,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_platter_disk,
        negative_elem=left_deck_surface,
        name="left_platter_seated_on_deck",
    )
    ctx.expect_gap(
        right_platter,
        console,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_platter_disk,
        negative_elem=right_deck_surface,
        name="right_platter_seated_on_deck",
    )
    ctx.expect_within(
        left_platter,
        left_platter,
        axes="xy",
        inner_elem=left_spindle,
        outer_elem=left_slip_mat,
        name="left_spindle_centered_on_slipmat",
    )
    ctx.expect_within(
        right_platter,
        right_platter,
        axes="xy",
        inner_elem=right_spindle,
        outer_elem=right_slip_mat,
        name="right_spindle_centered_on_slipmat",
    )
    ctx.expect_origin_distance(
        left_platter,
        right_platter,
        axes="y",
        max_dist=0.001,
        name="platters_share_a_common_row",
    )
    ctx.expect_gap(
        right_platter,
        left_platter,
        axis="x",
        min_gap=0.20,
        positive_elem=right_platter_disk,
        negative_elem=left_platter_disk,
        name="mixer_space_between_platters",
    )

    ctx.expect_within(
        console,
        console,
        axes="xy",
        inner_elem=left_tonearm_pin,
        outer_elem=left_tonearm_bearing,
        name="left_tonearm_pin_centered_in_bearing",
    )
    ctx.expect_within(
        console,
        console,
        axes="xy",
        inner_elem=right_tonearm_pin,
        outer_elem=right_tonearm_bearing,
        name="right_tonearm_pin_centered_in_bearing",
    )
    ctx.expect_gap(
        left_tonearm,
        console,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_tonearm_hub,
        negative_elem=left_tonearm_bearing,
        name="left_tonearm_base_seated_on_bearing",
    )
    ctx.expect_gap(
        right_tonearm,
        console,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_tonearm_hub,
        negative_elem=right_tonearm_bearing,
        name="right_tonearm_base_seated_on_bearing",
    )
    ctx.expect_gap(
        console,
        left_tonearm,
        axis="x",
        min_gap=0.002,
        positive_elem=left_tonearm_pin,
        negative_elem=left_hub_left_cheek,
        name="left_tonearm_pin_clears_left_cheek",
    )
    ctx.expect_gap(
        left_tonearm,
        console,
        axis="x",
        min_gap=0.002,
        positive_elem=left_hub_right_cheek,
        negative_elem=left_tonearm_pin,
        name="left_tonearm_pin_clears_right_cheek",
    )
    ctx.expect_gap(
        console,
        right_tonearm,
        axis="x",
        min_gap=0.002,
        positive_elem=right_tonearm_pin,
        negative_elem=right_hub_left_cheek,
        name="right_tonearm_pin_clears_left_cheek",
    )
    ctx.expect_gap(
        right_tonearm,
        console,
        axis="x",
        min_gap=0.002,
        positive_elem=right_hub_right_cheek,
        negative_elem=right_tonearm_pin,
        name="right_tonearm_pin_clears_right_cheek",
    )
    ctx.expect_gap(
        left_tonearm,
        left_platter,
        axis="x",
        min_gap=0.008,
        positive_elem=left_headshell,
        negative_elem=left_slip_mat,
        name="left_tonearm_parked_to_the_side",
    )
    ctx.expect_gap(
        right_tonearm,
        right_platter,
        axis="x",
        min_gap=0.008,
        positive_elem=right_headshell,
        negative_elem=right_slip_mat,
        name="right_tonearm_parked_to_the_side",
    )

    ctx.expect_gap(
        crossfader,
        console,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=crossfader_stem,
        negative_elem=crossfader_channel,
        name="crossfader_stem_seated_in_channel",
    )
    ctx.expect_within(
        crossfader,
        console,
        axes="xy",
        inner_elem=crossfader_stem,
        outer_elem=crossfader_channel,
        name="crossfader_runs_inside_horizontal_channel",
    )

    for index, fader in enumerate(channel_faders):
        ctx.expect_gap(
            fader,
            console,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=channel_stems[index],
            negative_elem=channel_slots[index],
            name=f"channel_fader_{index}_stem_seated_in_slot",
        )
        ctx.expect_within(
            fader,
            console,
            axes="xy",
            inner_elem=channel_stems[index],
            outer_elem=channel_slots[index],
            name=f"channel_fader_{index}_runs_in_vertical_slot",
        )

    with ctx.pose(
        {
            left_tonearm_swing: -0.78,
            right_tonearm_swing: -0.78,
            left_platter_spin: 1.8,
            right_platter_spin: -1.4,
        }
    ):
        ctx.expect_overlap(
            left_tonearm,
            left_platter,
            axes="xy",
            min_overlap=0.010,
            elem_a=left_headshell,
            elem_b=left_slip_mat,
            name="left_headshell_swings_over_record",
        )
        ctx.expect_overlap(
            right_tonearm,
            right_platter,
            axes="xy",
            min_overlap=0.010,
            elem_a=right_headshell,
            elem_b=right_slip_mat,
            name="right_headshell_swings_over_record",
        )
        ctx.expect_gap(
            left_tonearm,
            left_platter,
            axis="z",
            max_gap=0.003,
            max_penetration=0.0,
            positive_elem=left_stylus,
            negative_elem=left_slip_mat,
            name="left_stylus_hovers_just_above_record",
        )
        ctx.expect_gap(
            right_tonearm,
            right_platter,
            axis="z",
            max_gap=0.003,
            max_penetration=0.0,
            positive_elem=right_stylus,
            negative_elem=right_slip_mat,
            name="right_stylus_hovers_just_above_record",
        )

    with ctx.pose(
        {
            crossfader_slide: 0.055,
            channel_fader_slides[0]: 0.052,
            channel_fader_slides[1]: -0.052,
            channel_fader_slides[2]: 0.040,
            channel_fader_slides[3]: -0.040,
        }
    ):
        ctx.expect_within(
            crossfader,
            console,
            axes="x",
            inner_elem=crossfader_stem,
            outer_elem=crossfader_channel,
            name="crossfader_reaches_end_without_leaving_channel",
        )
        for index, fader in enumerate(channel_faders):
            ctx.expect_within(
                fader,
                console,
                axes="y",
                inner_elem=channel_stems[index],
                outer_elem=channel_slots[index],
                name=f"channel_fader_{index}_stays_in_slot_at_travel_limit",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
