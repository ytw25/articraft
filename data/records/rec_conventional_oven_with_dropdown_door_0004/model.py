from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
import math

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return _ORIG_GETCWD()


os.getcwd = _safe_getcwd
os.chdir(_safe_getcwd())

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_drop_front_oven")

    body_color = model.material("body_enamel", rgba=(0.17, 0.17, 0.18, 1.0))
    trim_color = model.material("trim_metal", rgba=(0.73, 0.74, 0.76, 1.0))
    glass_color = model.material("smoked_glass", rgba=(0.58, 0.68, 0.75, 0.35))
    rubber_color = model.material("rubber_black", rgba=(0.08, 0.08, 0.09, 1.0))
    panel_color = model.material("panel_gray", rgba=(0.46, 0.48, 0.50, 1.0))

    width = 0.40
    depth = 0.30
    foot_height = 0.012
    body_height = 0.236
    shell_t = 0.012
    front_t = 0.012
    rear_t = 0.010

    body_bottom_z = foot_height
    body_top_z = foot_height + body_height
    front_frame_y = depth / 2.0 - front_t / 2.0
    rear_frame_y = -depth / 2.0 + rear_t / 2.0

    door_width = 0.326
    door_height = 0.142
    door_t = 0.016
    door_frame_w = 0.024
    door_rail_h = 0.020
    hinge_r = 0.005
    hinge_len = 0.044
    door_hinge_z = 0.054
    door_top_z = door_hinge_z + door_height
    side_stile_w = (width - door_width) / 2.0

    tray_width = 0.288
    tray_depth = 0.190
    tray_floor_t = 0.003
    tray_wall_t = 0.004
    tray_wall_h = 0.014
    tray_lip_t = 0.006
    tray_lip_h = 0.016
    runner_t = 0.004
    runner_w = 0.016
    runner_len = 0.172
    runner_center_x = 0.122
    ledge_len = 0.205
    ledge_w = 0.080
    ledge_center_z = 0.02325
    tray_origin_y = 0.039
    tray_origin_z = 0.0307

    rear_opening_w = 0.304
    rear_opening_z_min = 0.050
    rear_opening_z_max = 0.202
    rear_opening_h = rear_opening_z_max - rear_opening_z_min
    rear_opening_center_z = rear_opening_z_min + rear_opening_h / 2.0
    rear_side_w = (width - rear_opening_w) / 2.0
    panel_t = 0.006
    panel_ear_w = 0.030
    panel_ear_h = 0.018
    panel_ear_x = 0.110
    panel_ear_center_z = rear_opening_h / 2.0 + panel_ear_h / 2.0 - 0.001
    tab_w = 0.024
    tab_t = 0.010
    tab_h = 0.016
    tab_center_y = -0.005
    tab_locked_z = rear_opening_z_min + rear_opening_h / 2.0 + panel_ear_center_z

    body = model.part("body")
    body.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom_z + shell_t / 2.0)),
        material=body_color,
        name="bottom_shell",
    )
    body.visual(
        Box((width, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_top_z - shell_t / 2.0)),
        material=body_color,
        name="top_shell",
    )
    side_wall_h = body_height - 2.0 * shell_t
    side_wall_z = body_bottom_z + shell_t + side_wall_h / 2.0
    body.visual(
        Box((shell_t, depth, side_wall_h)),
        origin=Origin(xyz=(-width / 2.0 + shell_t / 2.0, 0.0, side_wall_z)),
        material=body_color,
        name="left_wall",
    )
    body.visual(
        Box((shell_t, depth, side_wall_h)),
        origin=Origin(xyz=(width / 2.0 - shell_t / 2.0, 0.0, side_wall_z)),
        material=body_color,
        name="right_wall",
    )

    front_post_h = body_top_z - (body_bottom_z + 0.010)
    front_post_z = body_bottom_z + 0.010 + front_post_h / 2.0
    body.visual(
        Box((side_stile_w, front_t, front_post_h)),
        origin=Origin(xyz=(-width / 2.0 + side_stile_w / 2.0, front_frame_y, front_post_z)),
        material=body_color,
        name="front_left_post",
    )
    body.visual(
        Box((side_stile_w, front_t, front_post_h)),
        origin=Origin(xyz=(width / 2.0 - side_stile_w / 2.0, front_frame_y, front_post_z)),
        material=body_color,
        name="front_right_post",
    )
    body.visual(
        Box((width, front_t, 0.010)),
        origin=Origin(xyz=(0.0, front_frame_y, body_bottom_z + 0.005)),
        material=body_color,
        name="lower_skirt",
    )
    body.visual(
        Box((door_width, front_t, 0.010)),
        origin=Origin(xyz=(0.0, front_frame_y, 0.043)),
        material=body_color,
        name="tray_slot_header",
    )
    body.visual(
        Box((door_width, front_t, body_top_z - door_top_z)),
        origin=Origin(xyz=(0.0, front_frame_y, door_top_z + (body_top_z - door_top_z) / 2.0)),
        material=body_color,
        name="front_lintel",
    )
    hinge_x = door_width / 2.0 - hinge_len / 2.0 - 0.016
    body.visual(
        Box((hinge_len, 0.010, 0.012)),
        origin=Origin(xyz=(-hinge_x, 0.1395, door_hinge_z - 0.002)),
        material=trim_color,
        name="left_hinge_leaf",
    )
    body.visual(
        Box((hinge_len, 0.010, 0.012)),
        origin=Origin(xyz=(hinge_x, 0.1395, door_hinge_z - 0.002)),
        material=trim_color,
        name="right_hinge_leaf",
    )
    body.visual(
        Box((ledge_w, ledge_len, runner_t)),
        origin=Origin(xyz=(-0.154, 0.004, ledge_center_z)),
        material=trim_color,
        name="left_tray_ledge",
    )
    body.visual(
        Box((ledge_w, ledge_len, runner_t)),
        origin=Origin(xyz=(0.154, 0.004, ledge_center_z)),
        material=trim_color,
        name="right_tray_ledge",
    )

    body.visual(
        Box((rear_side_w, rear_t, rear_opening_h)),
        origin=Origin(
            xyz=(-width / 2.0 + rear_side_w / 2.0, rear_frame_y, rear_opening_center_z)
        ),
        material=body_color,
        name="rear_left_rail",
    )
    body.visual(
        Box((rear_side_w, rear_t, rear_opening_h)),
        origin=Origin(
            xyz=(width / 2.0 - rear_side_w / 2.0, rear_frame_y, rear_opening_center_z)
        ),
        material=body_color,
        name="rear_right_rail",
    )
    body.visual(
        Box((rear_opening_w, rear_t, body_top_z - rear_opening_z_max)),
        origin=Origin(
            xyz=(0.0, rear_frame_y, rear_opening_z_max + (body_top_z - rear_opening_z_max) / 2.0)
        ),
        material=body_color,
        name="rear_top_rail",
    )
    body.visual(
        Box((rear_opening_w, rear_t, rear_opening_z_min - body_bottom_z)),
        origin=Origin(
            xyz=(0.0, rear_frame_y, body_bottom_z + (rear_opening_z_min - body_bottom_z) / 2.0)
        ),
        material=body_color,
        name="rear_bottom_rail",
    )
    body.visual(
        Box((0.034, rear_t, 0.022)),
        origin=Origin(xyz=(-panel_ear_x, rear_frame_y, rear_opening_z_max + 0.011)),
        material=trim_color,
        name="left_tab_guide",
    )
    body.visual(
        Box((0.034, rear_t, 0.022)),
        origin=Origin(xyz=(panel_ear_x, rear_frame_y, rear_opening_z_max + 0.011)),
        material=trim_color,
        name="right_tab_guide",
    )

    foot_r = 0.016
    foot_x = 0.145
    foot_y = 0.095
    for name, x, y in (
        ("front_left_foot", -foot_x, foot_y),
        ("front_right_foot", foot_x, foot_y),
        ("rear_left_foot", -foot_x, -foot_y),
        ("rear_right_foot", foot_x, -foot_y),
    ):
        body.visual(
            Cylinder(radius=foot_r, length=foot_height),
            origin=Origin(xyz=(x, y, foot_height / 2.0)),
            material=rubber_color,
            name=name,
        )

    door = model.part("door")
    stile_h = door_height - door_rail_h
    stile_center_z = door_rail_h / 2.0 + stile_h / 2.0
    door.visual(
        Box((door_frame_w, door_t, stile_h)),
        origin=Origin(xyz=(-door_width / 2.0 + door_frame_w / 2.0, door_t / 2.0, stile_center_z)),
        material=body_color,
        name="left_stile",
    )
    door.visual(
        Box((door_frame_w, door_t, stile_h)),
        origin=Origin(xyz=(door_width / 2.0 - door_frame_w / 2.0, door_t / 2.0, stile_center_z)),
        material=body_color,
        name="right_stile",
    )
    door.visual(
        Box((door_width, door_t, door_rail_h)),
        origin=Origin(xyz=(0.0, door_t / 2.0, door_rail_h / 2.0)),
        material=body_color,
        name="bottom_rail",
    )
    door.visual(
        Box((door_width, door_t, door_rail_h)),
        origin=Origin(xyz=(0.0, door_t / 2.0, door_height - door_rail_h / 2.0)),
        material=body_color,
        name="top_rail",
    )
    door.visual(
        Box((0.170, 0.004, 0.064)),
        origin=Origin(xyz=(0.0, 0.004, 0.052)),
        material=glass_color,
        name="window_pane",
    )
    handle_z = 0.108
    door.visual(
        Box((0.012, 0.014, 0.032)),
        origin=Origin(xyz=(-0.082, 0.016, handle_z)),
        material=trim_color,
        name="left_handle_post",
    )
    door.visual(
        Box((0.012, 0.014, 0.032)),
        origin=Origin(xyz=(0.082, 0.016, handle_z)),
        material=trim_color,
        name="right_handle_post",
    )
    door.visual(
        Cylinder(radius=0.005, length=0.180),
        origin=Origin(xyz=(0.0, 0.024, handle_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_color,
        name="pull_bar",
    )
    door.visual(
        Cylinder(radius=hinge_r, length=hinge_len),
        origin=Origin(xyz=(-hinge_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_color,
        name="left_hinge_barrel",
    )
    door.visual(
        Cylinder(radius=hinge_r, length=hinge_len),
        origin=Origin(xyz=(hinge_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_color,
        name="right_hinge_barrel",
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        Box((tray_width, tray_depth, tray_floor_t)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=panel_color,
        name="tray_floor",
    )
    side_wall_len = 0.096
    side_wall_center_y = -0.060
    crumb_tray.visual(
        Box((tray_wall_t, side_wall_len, tray_wall_h)),
        origin=Origin(
            xyz=(-(tray_width - tray_wall_t) / 2.0, side_wall_center_y, tray_wall_h / 2.0)
        ),
        material=panel_color,
        name="left_wall",
    )
    crumb_tray.visual(
        Box((tray_wall_t, side_wall_len, tray_wall_h)),
        origin=Origin(
            xyz=((tray_width - tray_wall_t) / 2.0, side_wall_center_y, tray_wall_h / 2.0)
        ),
        material=panel_color,
        name="right_wall",
    )
    crumb_tray.visual(
        Box((tray_width, tray_wall_t, tray_wall_h)),
        origin=Origin(xyz=(0.0, -(tray_depth - tray_wall_t) / 2.0, tray_wall_h / 2.0)),
        material=panel_color,
        name="rear_wall",
    )
    crumb_tray.visual(
        Box((tray_width, tray_lip_t, tray_lip_h)),
        origin=Origin(xyz=(0.0, tray_depth / 2.0 - tray_lip_t / 2.0, tray_lip_h / 2.0 - 0.001)),
        material=trim_color,
        name="front_lip",
    )
    crumb_tray.visual(
        Box((runner_w, runner_len, runner_t)),
        origin=Origin(xyz=(-runner_center_x, -0.005, -0.0035)),
        material=trim_color,
        name="left_runner",
    )
    crumb_tray.visual(
        Box((runner_w, runner_len, runner_t)),
        origin=Origin(xyz=(runner_center_x, -0.005, -0.0035)),
        material=trim_color,
        name="right_runner",
    )

    back_panel = model.part("back_panel")
    back_panel.visual(
        Box((rear_opening_w, panel_t, rear_opening_h)),
        origin=Origin(xyz=(0.0, -panel_t / 2.0, 0.0)),
        material=panel_color,
        name="panel_plate",
    )
    back_panel.visual(
        Box((panel_ear_w, panel_t, panel_ear_h)),
        origin=Origin(xyz=(-panel_ear_x, -panel_t / 2.0, panel_ear_center_z)),
        material=trim_color,
        name="left_ear",
    )
    back_panel.visual(
        Box((panel_ear_w, panel_t, panel_ear_h)),
        origin=Origin(xyz=(panel_ear_x, -panel_t / 2.0, panel_ear_center_z)),
        material=trim_color,
        name="right_ear",
    )

    left_tab = model.part("left_tab")
    left_tab.visual(
        Box((tab_w, tab_t, tab_h)),
        origin=Origin(xyz=(0.0, tab_center_y, 0.0)),
        material=trim_color,
        name="tab_block",
    )

    right_tab = model.part("right_tab")
    right_tab.visual(
        Box((tab_w, tab_t, tab_h)),
        origin=Origin(xyz=(0.0, tab_center_y, 0.0)),
        material=trim_color,
        name="tab_block",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, depth / 2.0, door_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.10, upper=0.0),
    )
    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=crumb_tray,
        origin=Origin(xyz=(0.0, tray_origin_y, tray_origin_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.30, lower=0.0, upper=0.075),
    )
    model.articulation(
        "back_panel_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=back_panel,
        origin=Origin(xyz=(0.0, -depth / 2.0, rear_opening_center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.25, lower=0.0, upper=0.045),
    )
    model.articulation(
        "left_tab_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_tab,
        origin=Origin(xyz=(-panel_ear_x, -depth / 2.0 - panel_t, tab_locked_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.12, lower=0.0, upper=0.024),
    )
    model.articulation(
        "right_tab_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_tab,
        origin=Origin(xyz=(panel_ear_x, -depth / 2.0 - panel_t, tab_locked_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.12, lower=0.0, upper=0.024),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    crumb_tray = object_model.get_part("crumb_tray")
    back_panel = object_model.get_part("back_panel")
    left_tab = object_model.get_part("left_tab")
    right_tab = object_model.get_part("right_tab")

    door_hinge = object_model.get_articulation("door_hinge")
    tray_slide = object_model.get_articulation("tray_slide")
    back_panel_slide = object_model.get_articulation("back_panel_slide")
    left_tab_slide = object_model.get_articulation("left_tab_slide")
    right_tab_slide = object_model.get_articulation("right_tab_slide")

    left_hinge_barrel = door.get_visual("left_hinge_barrel")
    right_hinge_barrel = door.get_visual("right_hinge_barrel")
    top_rail = door.get_visual("top_rail")
    pull_bar = door.get_visual("pull_bar")
    window_pane = door.get_visual("window_pane")

    front_lintel = body.get_visual("front_lintel")
    tray_slot_header = body.get_visual("tray_slot_header")
    left_hinge_leaf = body.get_visual("left_hinge_leaf")
    right_hinge_leaf = body.get_visual("right_hinge_leaf")
    left_tray_ledge = body.get_visual("left_tray_ledge")
    right_tray_ledge = body.get_visual("right_tray_ledge")
    lower_skirt = body.get_visual("lower_skirt")
    rear_left_rail = body.get_visual("rear_left_rail")
    rear_right_rail = body.get_visual("rear_right_rail")

    left_runner = crumb_tray.get_visual("left_runner")
    right_runner = crumb_tray.get_visual("right_runner")
    front_lip = crumb_tray.get_visual("front_lip")

    panel_plate = back_panel.get_visual("panel_plate")
    left_ear = back_panel.get_visual("left_ear")
    right_ear = back_panel.get_visual("right_ear")
    left_tab_block = left_tab.get_visual("tab_block")
    right_tab_block = right_tab.get_visual("tab_block")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.14)
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem=left_hinge_barrel,
        negative_elem=left_hinge_leaf,
        name="left_piano_hinge_section_seated",
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem=right_hinge_barrel,
        negative_elem=right_hinge_leaf,
        name="right_piano_hinge_section_seated",
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=top_rail,
        negative_elem=front_lintel,
        name="door_face_sits_flush_with_front_frame",
    )
    ctx.expect_within(
        door,
        door,
        axes="xz",
        inner_elem=window_pane,
        name="window_is_nested_within_door_frame",
    )
    ctx.expect_gap(
        door,
        door,
        axis="y",
        min_gap=0.0015,
        positive_elem=pull_bar,
        negative_elem=top_rail,
        name="pull_bar_projects_proud_of_door_face",
    )

    ctx.expect_gap(
        crumb_tray,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0004,
        positive_elem=left_runner,
        negative_elem=left_tray_ledge,
        name="left_runner_rides_left_guide_ledge",
    )
    ctx.expect_gap(
        crumb_tray,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0004,
        positive_elem=right_runner,
        negative_elem=right_tray_ledge,
        name="right_runner_rides_right_guide_ledge",
    )
    ctx.expect_overlap(
        crumb_tray,
        body,
        axes="x",
        min_overlap=0.012,
        elem_a=left_runner,
        elem_b=left_tray_ledge,
        name="left_runner_stays_over_its_ledge",
    )
    ctx.expect_overlap(
        crumb_tray,
        body,
        axes="x",
        min_overlap=0.012,
        elem_a=right_runner,
        elem_b=right_tray_ledge,
        name="right_runner_stays_over_its_ledge",
    )
    ctx.expect_gap(
        body,
        crumb_tray,
        axis="y",
        min_gap=0.002,
        max_gap=0.020,
        positive_elem=tray_slot_header,
        negative_elem=front_lip,
        name="crumb_tray_front_lip_sits_just_inside_bottom_slot",
    )

    ctx.expect_gap(
        body,
        back_panel,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rear_left_rail,
        negative_elem=panel_plate,
        name="rear_panel_seats_against_rear_frame",
    )
    ctx.expect_within(back_panel, body, axes="xz", inner_elem=panel_plate, name="rear_panel_fits_opening")
    ctx.expect_contact(
        left_tab,
        back_panel,
        elem_a=left_tab_block,
        elem_b=left_ear,
        name="left_slide_tab_locks_panel_ear",
    )
    ctx.expect_contact(
        right_tab,
        back_panel,
        elem_a=right_tab_block,
        elem_b=right_ear,
        name="right_slide_tab_locks_panel_ear",
    )

    with ctx.pose({door_hinge: -1.05}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            min_gap=0.090,
            positive_elem=pull_bar,
            negative_elem=front_lintel,
            name="drop_front_door_swings_forward_and_down",
        )
        ctx.expect_overlap(door, body, axes="x", min_overlap=0.20)

    with ctx.pose({tray_slide: 0.075}):
        ctx.expect_gap(
            crumb_tray,
            body,
            axis="y",
            min_gap=0.050,
            positive_elem=front_lip,
            negative_elem=lower_skirt,
            name="crumb_tray_pulls_out_the_front",
        )

    with ctx.pose({left_tab_slide: 0.022, right_tab_slide: 0.022}):
        ctx.expect_gap(
            left_tab,
            back_panel,
            axis="z",
            min_gap=0.004,
            positive_elem=left_tab_block,
            negative_elem=left_ear,
            name="left_tab_retracts_above_panel_ear",
        )
        ctx.expect_gap(
            right_tab,
            back_panel,
            axis="z",
            min_gap=0.004,
            positive_elem=right_tab_block,
            negative_elem=right_ear,
            name="right_tab_retracts_above_panel_ear",
        )
        with ctx.pose({back_panel_slide: 0.040}):
            ctx.expect_gap(
                body,
                back_panel,
                axis="y",
                min_gap=0.030,
                positive_elem=rear_right_rail,
                negative_elem=panel_plate,
                name="rear_panel_can_slide_out_once_tabs_are_raised",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
