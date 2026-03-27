from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

_SCRIPT_DIR = os.path.dirname(__file__) if "__file__" in globals() else "/tmp"
for _candidate_cwd in (_SCRIPT_DIR, "/tmp", "/"):
    try:
        os.chdir(_candidate_cwd)
        break
    except OSError:
        continue

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

CABINET_W = 0.62
CABINET_H = 0.92
CABINET_D = 0.22
BACK_T = 0.018
STILE_W = 0.075
RAIL_H = 0.09

OPENING_W = CABINET_W - 2.0 * STILE_W
OPENING_H = CABINET_H - 2.0 * RAIL_H

DOOR_GAP = 0.004
DOOR_W = OPENING_W - 2.0 * DOOR_GAP
DOOR_H = OPENING_H - 2.0 * DOOR_GAP
DOOR_T = 0.026

HINGE_THROW_X = 0.008
HINGE_THROW_Y = 0.010
HINGE_X = -OPENING_W * 0.5 + DOOR_GAP - HINGE_THROW_X
HINGE_Y = HINGE_THROW_Y

DOOR_CENTER_X = DOOR_W * 0.5 + HINGE_THROW_X
DOOR_CENTER_Y = -(DOOR_T * 0.5 + HINGE_THROW_Y)
DOOR_RIGHT_EDGE_X = DOOR_W + HINGE_THROW_X

HINGE_Z = 0.235
HANDLE_X = DOOR_RIGHT_EDGE_X - 0.095
HANDLE_Z = 0.0
KEY_Z = -0.112
BOLT_X = DOOR_RIGHT_EDGE_X - 0.022
TOP_BOLT_Z = DOOR_H * 0.5 - 0.020
BOTTOM_BOLT_Z = -DOOR_H * 0.5 + 0.020
STOP_Y = -DOOR_T - 0.002
PLATE_Y = -0.053
LOCK_CENTER_Y = DOOR_CENTER_Y
HALF_PI = math.pi * 0.5


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_mount_recessed_cabinet_door_set")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.76, 0.77, 0.75, 1.0))
    door_paint = model.material("door_paint", rgba=(0.66, 0.68, 0.70, 1.0))
    shadow = model.material("shadow", rgba=(0.15, 0.15, 0.16, 1.0))
    hardware = model.material("hardware", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.31, 0.32, 0.34, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.63, 0.34, 1.0))

    cabinet_frame = model.part("cabinet_frame")
    cabinet_frame.inertial = Inertial.from_geometry(
        Box((CABINET_W, CABINET_D, CABINET_H)),
        mass=26.0,
        origin=Origin(xyz=(0.0, -CABINET_D * 0.5, 0.0)),
    )
    cabinet_frame.visual(
        Box((STILE_W, CABINET_D, CABINET_H)),
        origin=Origin(xyz=(-CABINET_W * 0.5 + STILE_W * 0.5, -CABINET_D * 0.5, 0.0)),
        material=cabinet_paint,
        name="left_stile",
    )
    cabinet_frame.visual(
        Box((STILE_W, CABINET_D, CABINET_H)),
        origin=Origin(xyz=(CABINET_W * 0.5 - STILE_W * 0.5, -CABINET_D * 0.5, 0.0)),
        material=cabinet_paint,
        name="right_stile",
    )
    cabinet_frame.visual(
        Box((CABINET_W, CABINET_D, RAIL_H)),
        origin=Origin(xyz=(0.0, -CABINET_D * 0.5, CABINET_H * 0.5 - RAIL_H * 0.5)),
        material=cabinet_paint,
        name="top_rail",
    )
    cabinet_frame.visual(
        Box((CABINET_W, CABINET_D, RAIL_H)),
        origin=Origin(xyz=(0.0, -CABINET_D * 0.5, -CABINET_H * 0.5 + RAIL_H * 0.5)),
        material=cabinet_paint,
        name="bottom_rail",
    )
    cabinet_frame.visual(
        Box((CABINET_W - 0.020, BACK_T, CABINET_H - 0.020)),
        origin=Origin(xyz=(0.0, -CABINET_D + BACK_T * 0.5, 0.0)),
        material=shadow,
        name="back_panel",
    )
    cabinet_frame.visual(
        Box((0.008, 0.004, DOOR_H - 0.080)),
        origin=Origin(xyz=(OPENING_W * 0.5 - 0.004, STOP_Y, 0.0)),
        material=shadow,
        name="latch_side_stop",
    )
    cabinet_frame.visual(
        Box((DOOR_W - 0.060, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, STOP_Y, DOOR_H * 0.5 + 0.004)),
        material=shadow,
        name="top_stop",
    )
    cabinet_frame.visual(
        Box((DOOR_W - 0.060, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, STOP_Y, -DOOR_H * 0.5 - 0.004)),
        material=shadow,
        name="bottom_stop",
    )
    cabinet_frame.visual(
        Box((0.022, 0.018, 0.008)),
        origin=Origin(xyz=(HINGE_X + BOLT_X, LOCK_CENTER_Y, OPENING_H * 0.5 + 0.008)),
        material=dark_hardware,
        name="top_keeper",
    )
    cabinet_frame.visual(
        Box((0.008, 0.018, 0.052)),
        origin=Origin(xyz=(OPENING_W * 0.5 - 0.004, LOCK_CENTER_Y, 0.0)),
        material=dark_hardware,
        name="side_keeper",
    )
    cabinet_frame.visual(
        Box((0.022, 0.018, 0.008)),
        origin=Origin(xyz=(HINGE_X + BOLT_X, LOCK_CENTER_Y, -OPENING_H * 0.5 - 0.008)),
        material=dark_hardware,
        name="bottom_keeper",
    )
    cabinet_frame.visual(
        Box((0.010, 0.044, 0.060)),
        origin=Origin(xyz=(HINGE_X - 0.005, PLATE_Y, HINGE_Z)),
        material=dark_hardware,
        name="upper_hinge_plate",
    )
    cabinet_frame.visual(
        Box((0.010, 0.044, 0.060)),
        origin=Origin(xyz=(HINGE_X - 0.005, PLATE_Y, -HINGE_Z)),
        material=dark_hardware,
        name="lower_hinge_plate",
    )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        mass=18.0,
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_CENTER_Y, 0.0)),
    )
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_CENTER_Y, 0.0)),
        material=door_paint,
        name="door_shell",
    )
    door.visual(
        Box((DOOR_W, 0.004, DOOR_H - 0.018)),
        origin=Origin(xyz=(DOOR_CENTER_X, -HINGE_THROW_Y - 0.002, 0.0)),
        material=door_paint,
        name="exterior_face",
    )
    door.visual(
        Box((DOOR_W, 0.004, DOOR_H - 0.030)),
        origin=Origin(xyz=(DOOR_CENTER_X, -DOOR_T - HINGE_THROW_Y + 0.002, 0.0)),
        material=shadow,
        name="interior_face",
    )
    door.visual(
        Box((0.018, DOOR_T - 0.004, DOOR_H - 0.060)),
        origin=Origin(xyz=(DOOR_RIGHT_EDGE_X - 0.009, LOCK_CENTER_Y, 0.0)),
        material=shadow,
        name="latch_stile",
    )
    door.visual(
        Box((0.004, DOOR_T - 0.006, 0.430)),
        origin=Origin(xyz=(DOOR_RIGHT_EDGE_X - 0.002, LOCK_CENTER_Y, 0.0)),
        material=hardware,
        name="latch_faceplate",
    )
    door.visual(
        Box((0.020, DOOR_T - 0.004, 0.220)),
        origin=Origin(xyz=(BOLT_X - 0.010, LOCK_CENTER_Y, -0.030)),
        material=dark_hardware,
        name="lock_case",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(HANDLE_X, -HINGE_THROW_Y + 0.004, KEY_Z), rpy=(HALF_PI, 0.0, 0.0)),
        material=dark_hardware,
        name="key_rose",
    )
    door.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(HANDLE_X, -HINGE_THROW_Y + 0.007, KEY_Z), rpy=(HALF_PI, 0.0, 0.0)),
        material=brass,
        name="key_cylinder",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.046, -0.016, HINGE_Z), rpy=(HALF_PI, 0.0, 0.0)),
        material=dark_hardware,
        name="upper_hinge_cup",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.046, -0.016, -HINGE_Z), rpy=(HALF_PI, 0.0, 0.0)),
        material=dark_hardware,
        name="lower_hinge_cup",
    )
    door.visual(
        Box((0.053, 0.022, 0.024)),
        origin=Origin(xyz=(0.0265, -0.030, HINGE_Z)),
        material=hardware,
        name="upper_hinge_arm",
    )
    door.visual(
        Box((0.053, 0.022, 0.024)),
        origin=Origin(xyz=(0.0265, -0.030, -HINGE_Z)),
        material=hardware,
        name="lower_hinge_arm",
    )
    door.visual(
        Box((0.022, 0.018, 0.018)),
        origin=Origin(xyz=(BOLT_X, LOCK_CENTER_Y, TOP_BOLT_Z - 0.012)),
        material=dark_hardware,
        name="top_bolt_guide",
    )
    door.visual(
        Box((0.022, 0.018, 0.018)),
        origin=Origin(xyz=(BOLT_X, LOCK_CENTER_Y, BOTTOM_BOLT_Z + 0.012)),
        material=dark_hardware,
        name="bottom_bolt_guide",
    )
    door.visual(
        Box((0.022, 0.018, 0.040)),
        origin=Origin(xyz=(BOLT_X - 0.010, LOCK_CENTER_Y, 0.0)),
        material=dark_hardware,
        name="side_bolt_guide",
    )

    lever_handle = model.part("lever_handle")
    lever_handle.inertial = Inertial.from_geometry(
        Box((0.120, 0.028, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(0.056, 0.012, 0.0)),
    )
    lever_handle.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(HALF_PI, 0.0, 0.0)),
        material=hardware,
        name="rosette",
    )
    lever_handle.visual(
        Cylinder(radius=0.009, length=0.026),
        origin=Origin(xyz=(0.016, 0.010, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=hardware,
        name="spindle_hub",
    )
    lever_handle.visual(
        Cylinder(radius=0.007, length=0.105),
        origin=Origin(xyz=(0.060, 0.014, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=hardware,
        name="lever_bar",
    )
    lever_handle.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.109, 0.014, -0.010)),
        material=hardware,
        name="lever_drop",
    )

    top_bolt = model.part("top_bolt")
    top_bolt.inertial = Inertial.from_geometry(
        Box((0.018, 0.018, 0.048)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    top_bolt.visual(
        Box((0.012, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=hardware,
        name="top_rod",
    )
    top_bolt.visual(
        Box((0.018, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=dark_hardware,
        name="top_tip",
    )

    side_bolt = model.part("side_bolt")
    side_bolt.inertial = Inertial.from_geometry(
        Box((0.040, 0.018, 0.018)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    side_bolt.visual(
        Box((0.032, 0.012, 0.012)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=hardware,
        name="side_rod",
    )
    side_bolt.visual(
        Box((0.004, 0.018, 0.018)),
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
        material=dark_hardware,
        name="side_tip",
    )

    bottom_bolt = model.part("bottom_bolt")
    bottom_bolt.inertial = Inertial.from_geometry(
        Box((0.018, 0.018, 0.048)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )
    bottom_bolt.visual(
        Box((0.012, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=hardware,
        name="bottom_rod",
    )
    bottom_bolt.visual(
        Box((0.018, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=dark_hardware,
        name="bottom_tip",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet_frame,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.4, lower=0.0, upper=1.18),
    )
    model.articulation(
        "handle_turn",
        ArticulationType.REVOLUTE,
        parent=door,
        child=lever_handle,
        origin=Origin(xyz=(HANDLE_X, -HINGE_THROW_Y, HANDLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "top_bolt_slide",
        ArticulationType.PRISMATIC,
        parent=door,
        child=top_bolt,
        origin=Origin(xyz=(BOLT_X, LOCK_CENTER_Y, TOP_BOLT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.2, lower=0.0, upper=0.016),
    )
    model.articulation(
        "side_bolt_slide",
        ArticulationType.PRISMATIC,
        parent=door,
        child=side_bolt,
        origin=Origin(xyz=(BOLT_X, LOCK_CENTER_Y, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.2, lower=0.0, upper=0.016),
    )
    model.articulation(
        "bottom_bolt_slide",
        ArticulationType.PRISMATIC,
        parent=door,
        child=bottom_bolt,
        origin=Origin(xyz=(BOLT_X, LOCK_CENTER_Y, BOTTOM_BOLT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.2, lower=0.0, upper=0.016),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet_frame = object_model.get_part("cabinet_frame")
    door = object_model.get_part("door")
    lever_handle = object_model.get_part("lever_handle")
    top_bolt = object_model.get_part("top_bolt")
    side_bolt = object_model.get_part("side_bolt")
    bottom_bolt = object_model.get_part("bottom_bolt")

    door_hinge = object_model.get_articulation("door_hinge")
    handle_turn = object_model.get_articulation("handle_turn")
    top_bolt_slide = object_model.get_articulation("top_bolt_slide")
    side_bolt_slide = object_model.get_articulation("side_bolt_slide")
    bottom_bolt_slide = object_model.get_articulation("bottom_bolt_slide")

    left_stile = cabinet_frame.get_visual("left_stile")
    right_stile = cabinet_frame.get_visual("right_stile")
    top_rail = cabinet_frame.get_visual("top_rail")
    bottom_rail = cabinet_frame.get_visual("bottom_rail")
    latch_side_stop = cabinet_frame.get_visual("latch_side_stop")
    top_keeper = cabinet_frame.get_visual("top_keeper")
    side_keeper = cabinet_frame.get_visual("side_keeper")
    bottom_keeper = cabinet_frame.get_visual("bottom_keeper")
    upper_hinge_plate = cabinet_frame.get_visual("upper_hinge_plate")
    lower_hinge_plate = cabinet_frame.get_visual("lower_hinge_plate")

    door_shell = door.get_visual("door_shell")
    exterior_face = door.get_visual("exterior_face")
    interior_face = door.get_visual("interior_face")
    latch_stile = door.get_visual("latch_stile")
    key_cylinder = door.get_visual("key_cylinder")
    upper_hinge_arm = door.get_visual("upper_hinge_arm")
    lower_hinge_arm = door.get_visual("lower_hinge_arm")

    rosette = lever_handle.get_visual("rosette")
    top_tip = top_bolt.get_visual("top_tip")
    side_tip = side_bolt.get_visual("side_tip")
    bottom_tip = bottom_bolt.get_visual("bottom_tip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(top_bolt, door, reason="top shoot bolt retracts into the door edge guide")
    ctx.allow_overlap(side_bolt, door, reason="side latch tongue retracts into the lock case in the door stile")
    ctx.allow_overlap(bottom_bolt, door, reason="bottom shoot bolt retracts into the door edge guide")
    ctx.allow_overlap(lever_handle, door, reason="lever rosette seats tightly against the exterior face of the door")
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

    ctx.expect_gap(
        door,
        cabinet_frame,
        axis="x",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=door_shell,
        negative_elem=left_stile,
        name="hinge-side reveal stays narrow in the inset frame",
    )
    ctx.expect_gap(
        cabinet_frame,
        door,
        axis="x",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=right_stile,
        negative_elem=door_shell,
        name="latch-side reveal stays narrow in the inset frame",
    )
    ctx.expect_gap(
        cabinet_frame,
        door,
        axis="z",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=top_rail,
        negative_elem=door_shell,
        name="top reveal stays narrow in the inset frame",
    )
    ctx.expect_gap(
        door,
        cabinet_frame,
        axis="z",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=door_shell,
        negative_elem=bottom_rail,
        name="bottom reveal stays narrow in the inset frame",
    )
    ctx.expect_contact(
        door,
        cabinet_frame,
        elem_a=interior_face,
        elem_b=latch_side_stop,
        name="door seats against the recessed frame stop",
    )
    ctx.expect_contact(
        door,
        cabinet_frame,
        elem_a=upper_hinge_arm,
        elem_b=upper_hinge_plate,
        name="upper concealed hinge arm meets its mounting plate",
    )
    ctx.expect_contact(
        door,
        cabinet_frame,
        elem_a=lower_hinge_arm,
        elem_b=lower_hinge_plate,
        name="lower concealed hinge arm meets its mounting plate",
    )
    ctx.expect_contact(
        lever_handle,
        door,
        elem_a=rosette,
        elem_b=exterior_face,
        name="lever handle mounts on the exterior face",
    )
    ctx.expect_gap(
        lever_handle,
        door,
        axis="z",
        min_gap=0.060,
        positive_elem=rosette,
        negative_elem=key_cylinder,
        name="key cylinder sits below the lever handle",
    )

    with ctx.pose(
        {
            handle_turn: 0.55,
            top_bolt_slide: 0.0,
            side_bolt_slide: 0.0,
            bottom_bolt_slide: 0.0,
        }
    ):
        ctx.expect_contact(top_bolt, cabinet_frame, elem_a=top_tip, elem_b=top_keeper, name="top bolt engages the frame")
        ctx.expect_contact(side_bolt, cabinet_frame, elem_a=side_tip, elem_b=side_keeper, name="side bolt engages the frame")
        ctx.expect_contact(
            bottom_bolt,
            cabinet_frame,
            elem_a=bottom_tip,
            elem_b=bottom_keeper,
            name="bottom bolt engages the frame",
        )

    with ctx.pose(
        {
            handle_turn: -0.55,
            top_bolt_slide: 0.016,
            side_bolt_slide: 0.016,
            bottom_bolt_slide: 0.016,
        }
    ):
        ctx.expect_gap(
            cabinet_frame,
            top_bolt,
            axis="z",
            min_gap=0.012,
            positive_elem=top_keeper,
            negative_elem=top_tip,
            name="top bolt retracts clear when the lever rotates",
        )
        ctx.expect_gap(
            cabinet_frame,
            side_bolt,
            axis="x",
            min_gap=0.012,
            positive_elem=side_keeper,
            negative_elem=side_tip,
            name="side bolt retracts clear when the lever rotates",
        )
        ctx.expect_gap(
            bottom_bolt,
            cabinet_frame,
            axis="z",
            min_gap=0.012,
            positive_elem=bottom_tip,
            negative_elem=bottom_keeper,
            name="bottom bolt retracts clear when the lever rotates",
        )

    with ctx.pose(
        {
            door_hinge: 1.05,
            handle_turn: -0.55,
            top_bolt_slide: 0.016,
            side_bolt_slide: 0.016,
            bottom_bolt_slide: 0.016,
        }
    ):
        ctx.expect_gap(
            door,
            cabinet_frame,
            axis="y",
            min_gap=0.080,
            positive_elem=latch_stile,
            negative_elem=right_stile,
            name="door swings outward clear of the deep frame when unlatched",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
