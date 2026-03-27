from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

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

HERE = Path(__file__).parent if os.path.isabs(__file__) else Path("/tmp")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gas_dryer")

    enamel = model.material("dryer_enamel", rgba=(0.94, 0.95, 0.95, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.20, 0.22, 0.24, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.16, 0.19, 0.22, 0.72))
    hinge_gray = model.material("hinge_gray", rgba=(0.64, 0.67, 0.70, 1.0))
    control_gray = model.material("control_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    knob_black = model.material("knob_black", rgba=(0.13, 0.13, 0.13, 1.0))
    duct_metal = model.material("duct_metal", rgba=(0.70, 0.72, 0.74, 1.0))

    width = 0.69
    depth = 0.62
    height = 0.92
    shell_t = 0.025
    front_t = 0.025
    stile_w = 0.05
    top_rail_h = 0.085
    mid_rail_h = 0.032

    body = model.part("body")
    body.visual(
        Box((shell_t, depth, height)),
        origin=Origin(xyz=(-width / 2 + shell_t / 2, 0.0, height / 2)),
        material=enamel,
        name="left_side",
    )
    body.visual(
        Box((shell_t, depth, height)),
        origin=Origin(xyz=(width / 2 - shell_t / 2, 0.0, height / 2)),
        material=enamel,
        name="right_side",
    )
    body.visual(
        Box((width - 2 * shell_t, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t / 2)),
        material=enamel,
        name="bottom_panel",
    )
    body.visual(
        Box((width - 2 * shell_t, depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, height - shell_t / 2)),
        material=enamel,
        name="top_panel",
    )
    body.visual(
        Box((width - 2 * shell_t, shell_t, height - 2 * shell_t)),
        origin=Origin(xyz=(0.0, -depth / 2 + shell_t / 2, height / 2)),
        material=enamel,
        name="back_panel",
    )
    body.visual(
        Box((stile_w, front_t, height - 2 * shell_t)),
        origin=Origin(
            xyz=(-width / 2 + stile_w / 2, depth / 2 - front_t / 2, height / 2)
        ),
        material=enamel,
        name="left_stile",
    )
    body.visual(
        Box((stile_w, front_t, height - 2 * shell_t)),
        origin=Origin(
            xyz=(width / 2 - stile_w / 2, depth / 2 - front_t / 2, height / 2)
        ),
        material=enamel,
        name="right_stile",
    )
    body.visual(
        Box((width - 2 * stile_w, front_t, top_rail_h)),
        origin=Origin(
            xyz=(
                0.0,
                depth / 2 - front_t / 2,
                height - shell_t - top_rail_h / 2,
            )
        ),
        material=enamel,
        name="top_rail",
    )
    mid_rail_z = 0.234
    body.visual(
        Box((width - 2 * stile_w, front_t, mid_rail_h)),
        origin=Origin(xyz=(0.0, depth / 2 - front_t / 2, mid_rail_z)),
        material=enamel,
        name="mid_rail",
    )

    door_hinge_y = depth / 2 + 0.003
    door_center_z = 0.53
    top_hinge_z = door_center_z + 0.115
    bottom_hinge_z = door_center_z - 0.115
    body_leaf_size = (0.10, 0.008, 0.05)
    body_leaf_center_x = -0.245

    body.visual(
        Box(body_leaf_size),
        origin=Origin(xyz=(body_leaf_center_x, depth / 2 - 0.009, top_hinge_z)),
        material=hinge_gray,
        name="top_door_hinge_plate",
    )
    body.visual(
        Box(body_leaf_size),
        origin=Origin(xyz=(body_leaf_center_x, depth / 2 - 0.009, bottom_hinge_z)),
        material=hinge_gray,
        name="bottom_door_hinge_plate",
    )

    bracket_size = (0.032, 0.014, 0.018)
    panel_hinge_y = depth / 2 + 0.003
    panel_axis_z = 0.032
    body.visual(
        Box(bracket_size),
        origin=Origin(xyz=(-0.283, panel_hinge_y - 0.007, panel_axis_z)),
        material=hinge_gray,
        name="left_panel_bracket",
    )
    body.visual(
        Box(bracket_size),
        origin=Origin(xyz=(0.283, panel_hinge_y - 0.007, panel_axis_z)),
        material=hinge_gray,
        name="right_panel_bracket",
    )

    door = model.part("door")
    door_radius = 0.192
    door_thickness = 0.040
    hinge_axis_x = -0.195
    door.visual(
        Cylinder(radius=door_radius, length=door_thickness),
        origin=Origin(
            xyz=(0.195, 0.0225, 0.0),
            rpy=(math.pi / 2, 0.0, 0.0),
        ),
        material=enamel,
        name="door_shell",
    )
    door.visual(
        Cylinder(radius=0.145, length=0.020),
        origin=Origin(
            xyz=(0.195, 0.025, 0.0),
            rpy=(math.pi / 2, 0.0, 0.0),
        ),
        material=smoked_glass,
        name="door_glass",
    )
    door.visual(
        Box((0.040, 0.008, 0.048)),
        origin=Origin(xyz=(0.320, 0.031, 0.0)),
        material=dark_trim,
        name="door_handle",
    )
    door.visual(
        Box((0.060, 0.016, 0.050)),
        origin=Origin(xyz=(0.030, 0.000, 0.115)),
        material=hinge_gray,
        name="top_hinge_tab",
    )
    door.visual(
        Box((0.060, 0.016, 0.050)),
        origin=Origin(xyz=(0.030, 0.000, -0.115)),
        material=hinge_gray,
        name="bottom_hinge_tab",
    )

    service_panel = model.part("service_panel")
    service_panel.visual(
        Box((0.50, 0.012, 0.150)),
        origin=Origin(xyz=(0.0, 0.006, 0.111)),
        material=enamel,
        name="panel_shell",
    )
    service_panel.visual(
        Box((0.47, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.009, 0.020)),
        material=enamel,
        name="panel_lower_lip",
    )
    service_panel.visual(
        Box((0.070, 0.012, 0.014)),
        origin=Origin(xyz=(-0.262, 0.006, 0.007)),
        material=hinge_gray,
        name="left_hinge_ear",
    )
    service_panel.visual(
        Box((0.070, 0.012, 0.014)),
        origin=Origin(xyz=(0.262, 0.006, 0.007)),
        material=hinge_gray,
        name="right_hinge_ear",
    )
    service_panel.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(
            xyz=(-0.283, 0.0, 0.0),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=hinge_gray,
        name="left_hinge_pin",
    )
    service_panel.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(
            xyz=(0.283, 0.0, 0.0),
            rpy=(0.0, math.pi / 2, 0.0),
        ),
        material=hinge_gray,
        name="right_hinge_pin",
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        Box((0.56, 0.14, 0.035)),
        material=control_gray,
        name="strip_shell",
    )

    selector_left = model.part("selector_left")
    selector_left.visual(
        Cylinder(radius=0.021, length=0.026),
        origin=Origin(rpy=(0.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_shell",
    )

    selector_center = model.part("selector_center")
    selector_center.visual(
        Cylinder(radius=0.021, length=0.026),
        origin=Origin(rpy=(0.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_shell",
    )

    timer_knob = model.part("timer_knob")
    timer_knob.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(rpy=(0.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_shell",
    )

    exhaust = model.part("exhaust_collar")
    exhaust.visual(
        Cylinder(radius=0.050, length=0.055),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=duct_metal,
        name="collar_shell",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_axis_x, door_hinge_y, door_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "service_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_panel,
        origin=Origin(xyz=(0.0, panel_hinge_y, panel_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_control_strip",
        ArticulationType.FIXED,
        parent=body,
        child=control_strip,
        origin=Origin(xyz=(0.0, -0.055, height + 0.0175)),
    )
    model.articulation(
        "control_strip_to_selector_left",
        ArticulationType.FIXED,
        parent=control_strip,
        child=selector_left,
        origin=Origin(xyz=(-0.155, 0.014, 0.0305)),
    )
    model.articulation(
        "control_strip_to_selector_center",
        ArticulationType.FIXED,
        parent=control_strip,
        child=selector_center,
        origin=Origin(xyz=(-0.020, 0.014, 0.0305)),
    )
    model.articulation(
        "control_strip_to_timer_knob",
        ArticulationType.FIXED,
        parent=control_strip,
        child=timer_knob,
        origin=Origin(xyz=(0.185, 0.014, 0.0325)),
    )
    model.articulation(
        "body_to_exhaust",
        ArticulationType.FIXED,
        parent=body,
        child=exhaust,
        origin=Origin(xyz=(0.0, -depth / 2 - 0.0275, 0.58)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    service_panel = object_model.get_part("service_panel")
    control_strip = object_model.get_part("control_strip")
    selector_left = object_model.get_part("selector_left")
    selector_center = object_model.get_part("selector_center")
    timer_knob = object_model.get_part("timer_knob")
    exhaust = object_model.get_part("exhaust_collar")

    door_hinge = object_model.get_articulation("door_hinge")
    service_panel_hinge = object_model.get_articulation("service_panel_hinge")

    top_panel = body.get_visual("top_panel")
    back_panel = body.get_visual("back_panel")
    mid_rail = body.get_visual("mid_rail")
    top_body_hinge = body.get_visual("top_door_hinge_plate")
    bottom_body_hinge = body.get_visual("bottom_door_hinge_plate")
    left_panel_bracket = body.get_visual("left_panel_bracket")
    right_panel_bracket = body.get_visual("right_panel_bracket")

    door_shell = door.get_visual("door_shell")
    door_handle = door.get_visual("door_handle")

    panel_shell = service_panel.get_visual("panel_shell")
    panel_lower_lip = service_panel.get_visual("panel_lower_lip")
    left_hinge_pin = service_panel.get_visual("left_hinge_pin")
    right_hinge_pin = service_panel.get_visual("right_hinge_pin")

    strip_shell = control_strip.get_visual("strip_shell")
    left_knob = selector_left.get_visual("knob_shell")
    center_knob = selector_center.get_visual("knob_shell")
    timer_knob_shell = timer_knob.get_visual("knob_shell")
    exhaust_shell = exhaust.get_visual("collar_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        service_panel,
        body,
        reason="service-panel hinge pins are intentionally captured inside the two front brackets",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        control_strip,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=strip_shell,
        negative_elem=top_panel,
        name="control_strip_sits_on_top_panel",
    )
    ctx.expect_within(
        control_strip,
        body,
        axes="xy",
        inner_elem=strip_shell,
        outer_elem=top_panel,
        name="control_strip_stays_within_top_footprint",
    )
    ctx.expect_gap(
        selector_left,
        control_strip,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_knob,
        negative_elem=strip_shell,
        name="left_selector_mounts_to_strip",
    )
    ctx.expect_gap(
        selector_center,
        control_strip,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=center_knob,
        negative_elem=strip_shell,
        name="center_selector_mounts_to_strip",
    )
    ctx.expect_gap(
        timer_knob,
        control_strip,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=timer_knob_shell,
        negative_elem=strip_shell,
        name="timer_knob_mounts_to_strip",
    )
    ctx.expect_gap(
        selector_center,
        selector_left,
        axis="x",
        min_gap=0.07,
        name="selector_knobs_are_distinct",
    )
    ctx.expect_gap(
        timer_knob,
        selector_center,
        axis="x",
        min_gap=0.12,
        name="timer_knob_sits_to_the_right",
    )

    ctx.expect_gap(
        body,
        exhaust,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=back_panel,
        negative_elem=exhaust_shell,
        name="rear_exhaust_stub_seats_on_back_panel",
    )

    ctx.expect_gap(
        door,
        body,
        axis="x",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem=door_shell,
        negative_elem=top_body_hinge,
        name="upper_hinge_plate_sits_beside_door_edge",
    )
    ctx.expect_gap(
        door,
        body,
        axis="x",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem=door_shell,
        negative_elem=bottom_body_hinge,
        name="lower_hinge_plate_sits_beside_door_edge",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="z",
        elem_a=door_shell,
        elem_b=top_body_hinge,
        min_overlap=0.04,
        name="upper_hinge_plate_aligns_with_door_height",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="z",
        elem_a=door_shell,
        elem_b=bottom_body_hinge,
        min_overlap=0.04,
        name="lower_hinge_plate_aligns_with_door_height",
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.012,
        max_penetration=0.0,
        positive_elem=door_shell,
        negative_elem=top_body_hinge,
        name="upper_hinge_plate_stays_on_front_face",
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.012,
        max_penetration=0.0,
        positive_elem=door_shell,
        negative_elem=bottom_body_hinge,
        name="lower_hinge_plate_stays_on_front_face",
    )
    ctx.expect_gap(
        door,
        service_panel,
        axis="z",
        min_gap=0.11,
        positive_elem=door_shell,
        negative_elem=panel_shell,
        name="door_sits_above_service_panel",
    )

    ctx.expect_gap(
        body,
        service_panel,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=mid_rail,
        negative_elem=panel_shell,
        name="service_panel_closes_under_mid_rail",
    )
    ctx.expect_within(
        service_panel,
        body,
        axes="xz",
        inner_elem=left_hinge_pin,
        outer_elem=left_panel_bracket,
        name="left_service_hinge_pin_sits_in_bracket",
    )
    ctx.expect_within(
        service_panel,
        body,
        axes="xz",
        inner_elem=right_hinge_pin,
        outer_elem=right_panel_bracket,
        name="right_service_hinge_pin_sits_in_bracket",
    )

    with ctx.pose({door_hinge: 1.1}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            min_gap=0.15,
            positive_elem=door_handle,
            name="door_swings_out_from_front",
        )
    with ctx.pose({service_panel_hinge: 1.2}):
        ctx.expect_gap(
            service_panel,
            body,
            axis="y",
            min_gap=0.03,
            positive_elem=panel_shell,
            name="service_panel_swings_outward_when_open",
        )
        ctx.expect_gap(
            body,
            service_panel,
            axis="z",
            min_gap=0.09,
            positive_elem=mid_rail,
            negative_elem=panel_shell,
            name="service_panel_folds_down_below_opening",
        )
        ctx.expect_within(
            service_panel,
            body,
            axes="xz",
            inner_elem=left_hinge_pin,
            outer_elem=left_panel_bracket,
            name="left_panel_pin_stays_captured_when_open",
        )
        ctx.expect_within(
            service_panel,
            body,
            axes="xz",
            inner_elem=right_hinge_pin,
            outer_elem=right_panel_bracket,
            name="right_panel_pin_stays_captured_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
