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
    Sphere,
    TestContext,
    TestReport,
)

CUBE_SIZE = 0.074
HALF = CUBE_SIZE / 2.0
PANEL_SPAN = 0.048
PANEL_THICK = 0.0025


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fidget_cube")

    shell = model.material("shell", rgba=(0.16, 0.17, 0.20, 1.0))
    panel = model.material("panel", rgba=(0.26, 0.27, 0.31, 1.0))
    metal = model.material("metal", rgba=(0.74, 0.76, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.11, 0.12, 1.0))
    teal = model.material("teal", rgba=(0.20, 0.60, 0.65, 1.0))
    amber = model.material("amber", rgba=(0.74, 0.58, 0.24, 1.0))
    red = model.material("red", rgba=(0.73, 0.20, 0.18, 1.0))
    ivory = model.material("ivory", rgba=(0.88, 0.86, 0.80, 1.0))

    body = model.part("body")
    body.visual(Box((CUBE_SIZE, CUBE_SIZE, CUBE_SIZE)), material=shell, name="shell")
    body.visual(
        Box((PANEL_SPAN, PANEL_SPAN, PANEL_THICK)),
        origin=Origin(xyz=(0.0, 0.0, HALF - PANEL_THICK / 2.0)),
        material=panel,
        name="top_panel",
    )
    body.visual(
        Box((PANEL_SPAN, PANEL_SPAN, PANEL_THICK)),
        origin=Origin(xyz=(0.0, 0.0, -HALF + PANEL_THICK / 2.0)),
        material=panel,
        name="bottom_panel",
    )
    body.visual(
        Box((PANEL_THICK, PANEL_SPAN, PANEL_SPAN)),
        origin=Origin(xyz=(HALF - PANEL_THICK / 2.0, 0.0, 0.0)),
        material=panel,
        name="right_panel",
    )
    body.visual(
        Box((PANEL_THICK, PANEL_SPAN, PANEL_SPAN)),
        origin=Origin(xyz=(-HALF + PANEL_THICK / 2.0, 0.0, 0.0)),
        material=panel,
        name="left_panel",
    )
    body.visual(
        Box((PANEL_SPAN, PANEL_THICK, PANEL_SPAN)),
        origin=Origin(xyz=(0.0, HALF - PANEL_THICK / 2.0, 0.0)),
        material=panel,
        name="front_panel",
    )
    body.visual(
        Box((PANEL_SPAN, PANEL_THICK, PANEL_SPAN)),
        origin=Origin(xyz=(0.0, -HALF + PANEL_THICK / 2.0, 0.0)),
        material=panel,
        name="rear_panel",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, HALF - 0.00125)),
        material=metal,
        name="button_seat",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.0025),
        origin=Origin(xyz=(HALF - 0.00125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="dial_seat",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(-HALF + 0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="joystick_socket",
    )
    body.visual(
        Box((0.028, 0.0025, 0.020)),
        origin=Origin(xyz=(0.0, HALF - 0.00125, 0.0)),
        material=metal,
        name="toggle_plate",
    )
    body.visual(
        Box((0.052, 0.0025, 0.020)),
        origin=Origin(xyz=(0.0, -HALF + 0.00125, 0.0)),
        material=metal,
        name="slider_track",
    )
    body.visual(
        Box((0.0035, 0.004, 0.018)),
        origin=Origin(xyz=(-0.0185, -HALF + 0.002, 0.0)),
        material=metal,
        name="slider_left_rail",
    )
    body.visual(
        Box((0.0035, 0.004, 0.018)),
        origin=Origin(xyz=(0.0185, -HALF + 0.002, 0.0)),
        material=metal,
        name="slider_right_rail",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.0025),
        origin=Origin(xyz=(0.0, 0.0, -HALF + 0.00125)),
        material=metal,
        name="gear_seat",
    )
    body.inertial = Inertial.from_geometry(Box((CUBE_SIZE, CUBE_SIZE, CUBE_SIZE)), mass=0.50)

    button = model.part("button")
    button.visual(
        Cylinder(radius=0.0105, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=teal,
        name="button_plunger",
    )
    button.visual(
        Cylinder(radius=0.0095, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=teal,
        name="button_cap",
    )
    button.visual(
        Sphere(radius=0.0068),
        origin=Origin(xyz=(0.0, 0.0, 0.0058)),
        material=teal,
        name="button_dome",
    )
    button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.016),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.0130, length=0.003),
        origin=Origin(xyz=(0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="dial_collar",
    )
    dial.visual(
        Cylinder(radius=0.0108, length=0.007),
        origin=Origin(xyz=(0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="dial_knob",
    )
    dial.visual(
        Box((0.0016, 0.0025, 0.008)),
        origin=Origin(xyz=(0.0088, 0.0, 0.0)),
        material=ivory,
        name="dial_pointer",
    )
    for index in range(8):
        angle = 2.0 * math.pi * index / 8.0
        dial.visual(
            Box((0.003, 0.0016, 0.004)),
            origin=Origin(
                xyz=(0.005, 0.0098 * math.cos(angle), 0.0098 * math.sin(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=metal,
            name=f"dial_ridge_{index}",
        )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0135, length=0.015),
        mass=0.035,
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.0098, length=0.003),
        origin=Origin(xyz=(-0.0015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="joystick_skirt",
    )
    joystick.visual(
        Sphere(radius=0.004),
        origin=Origin(xyz=(-0.0015, 0.0, 0.0)),
        material=metal,
        name="joystick_gimbal",
    )
    joystick.visual(
        Cylinder(radius=0.0030, length=0.008),
        origin=Origin(xyz=(-0.0045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="joystick_stem",
    )
    joystick.visual(
        Sphere(radius=0.0060),
        origin=Origin(xyz=(-0.0085, 0.0, 0.0)),
        material=amber,
        name="joystick_cap",
    )
    joystick.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0065, length=0.020),
        mass=0.025,
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    toggle = model.part("toggle")
    toggle.visual(
        Box((0.012, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.0015, 0.0)),
        material=metal,
        name="toggle_base",
    )
    toggle.visual(
        Cylinder(radius=0.0018, length=0.010),
        origin=Origin(xyz=(0.0, 0.006, 0.0030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="toggle_stem",
    )
    toggle.visual(
        Box((0.0055, 0.0055, 0.0055)),
        origin=Origin(xyz=(0.0, 0.0105, 0.0055)),
        material=red,
        name="toggle_tip",
    )
    toggle.inertial = Inertial.from_geometry(
        Box((0.012, 0.022, 0.016)),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.011, 0.004)),
    )

    slider = model.part("slider")
    slider.visual(
        Box((0.016, 0.009, 0.012)),
        origin=Origin(xyz=(0.0, -0.0045, 0.0)),
        material=ivory,
        name="slider_thumb",
    )
    slider.visual(
        Box((0.010, 0.003, 0.008)),
        origin=Origin(xyz=(0.0, -0.0010, 0.0)),
        material=metal,
        name="slider_guide",
    )
    slider.visual(
        Box((0.011, 0.0025, 0.003)),
        origin=Origin(xyz=(0.0, -0.0085, 0.004)),
        material=metal,
        name="slider_rib",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.016, 0.010, 0.012)),
        mass=0.020,
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
    )

    gear = model.part("gear")
    gear.visual(
        Cylinder(radius=0.004, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=metal,
        name="gear_axle",
    )
    gear.visual(
        Cylinder(radius=0.0075, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=metal,
        name="gear_hub",
    )
    gear.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=amber,
        name="gear_rim",
    )
    for index in range(10):
        angle = 2.0 * math.pi * index / 10.0
        gear.visual(
            Box((0.005, 0.0026, 0.004)),
            origin=Origin(
                xyz=(0.0168 * math.cos(angle), 0.0168 * math.sin(angle), -0.004),
                rpy=(0.0, 0.0, angle),
            ),
            material=amber,
            name=f"gear_tooth_{index}",
        )
    gear.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0165, length=0.007),
        mass=0.030,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
    )

    model.articulation(
        "button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.0, 0.0, HALF)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=0.2, lower=-0.0025, upper=0.0),
    )
    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(HALF, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "joystick_rock",
        ArticulationType.REVOLUTE,
        parent=body,
        child=joystick,
        origin=Origin(xyz=(-HALF, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "toggle_flip",
        ArticulationType.REVOLUTE,
        parent=body,
        child=toggle,
        origin=Origin(xyz=(0.0, HALF, -0.001)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.65, upper=0.65),
    )
    model.articulation(
        "slider_move",
        ArticulationType.PRISMATIC,
        parent=body,
        child=slider,
        origin=Origin(xyz=(0.0, -HALF, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.25, lower=-0.011, upper=0.011),
    )
    model.articulation(
        "gear_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=gear,
        origin=Origin(xyz=(0.0, 0.0, -HALF)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=10.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    button = object_model.get_part("button")
    dial = object_model.get_part("dial")
    joystick = object_model.get_part("joystick")
    toggle = object_model.get_part("toggle")
    slider = object_model.get_part("slider")
    gear = object_model.get_part("gear")

    button_press = object_model.get_articulation("button_press")
    dial_turn = object_model.get_articulation("dial_turn")
    joystick_rock = object_model.get_articulation("joystick_rock")
    toggle_flip = object_model.get_articulation("toggle_flip")
    slider_move = object_model.get_articulation("slider_move")
    gear_spin = object_model.get_articulation("gear_spin")

    top_panel = body.get_visual("top_panel")
    bottom_panel = body.get_visual("bottom_panel")
    right_panel = body.get_visual("right_panel")
    left_panel = body.get_visual("left_panel")
    front_panel = body.get_visual("front_panel")
    rear_panel = body.get_visual("rear_panel")
    button_seat = body.get_visual("button_seat")
    dial_seat = body.get_visual("dial_seat")
    joystick_socket = body.get_visual("joystick_socket")
    toggle_plate = body.get_visual("toggle_plate")
    slider_track = body.get_visual("slider_track")
    gear_seat = body.get_visual("gear_seat")

    button_plunger = button.get_visual("button_plunger")
    button_cap = button.get_visual("button_cap")
    button_dome = button.get_visual("button_dome")
    dial_collar = dial.get_visual("dial_collar")
    dial_knob = dial.get_visual("dial_knob")
    joystick_skirt = joystick.get_visual("joystick_skirt")
    joystick_cap = joystick.get_visual("joystick_cap")
    toggle_base = toggle.get_visual("toggle_base")
    toggle_tip = toggle.get_visual("toggle_tip")
    slider_thumb = slider.get_visual("slider_thumb")
    slider_guide = slider.get_visual("slider_guide")
    gear_axle = gear.get_visual("gear_axle")
    gear_rim = gear.get_visual("gear_rim")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(button, body, reason="button plunger nests into the top seat")
    ctx.allow_overlap(dial, body, reason="dial collar sleeves into the right-side bezel")
    ctx.allow_overlap(joystick, body, reason="joystick skirt nests into the rubber socket")
    ctx.allow_overlap(toggle, body, reason="toggle base is seated into the face plate")
    ctx.allow_overlap(slider, body, reason="slider guide rides inside the rear track")
    ctx.allow_overlap(gear, body, reason="gear axle seats into the bottom bearing pad")
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.0015,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.0015,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(button, body, axes="xy", max_dist=0.001)
    ctx.expect_within(button, body, axes="xy", inner_elem=button_cap, outer_elem=top_panel)
    ctx.expect_gap(
        button,
        body,
        axis="z",
        positive_elem=button_plunger,
        negative_elem=button_seat,
        max_gap=0.001,
        max_penetration=0.0015,
    )
    ctx.expect_overlap(button, body, axes="xy", elem_a=button_dome, elem_b=top_panel, min_overlap=0.0001)

    ctx.expect_origin_distance(dial, body, axes="yz", max_dist=0.001)
    ctx.expect_within(dial, body, axes="yz", inner_elem=dial_knob, outer_elem=right_panel)
    ctx.expect_gap(
        dial,
        body,
        axis="x",
        positive_elem=dial_collar,
        negative_elem=dial_seat,
        max_gap=0.001,
        max_penetration=0.0015,
    )

    ctx.expect_origin_distance(joystick, body, axes="yz", max_dist=0.001)
    ctx.expect_within(joystick, body, axes="yz", inner_elem=joystick_cap, outer_elem=left_panel)
    ctx.expect_gap(
        body,
        joystick,
        axis="x",
        positive_elem=joystick_socket,
        negative_elem=joystick_skirt,
        max_gap=0.001,
        max_penetration=0.0015,
    )

    ctx.expect_origin_distance(toggle, body, axes="xz", max_dist=0.001)
    ctx.expect_within(toggle, body, axes="xz", inner_elem=toggle_tip, outer_elem=front_panel)
    ctx.expect_gap(
        toggle,
        body,
        axis="y",
        positive_elem=toggle_base,
        negative_elem=toggle_plate,
        max_gap=0.001,
        max_penetration=0.0015,
    )

    ctx.expect_origin_distance(slider, body, axes="xz", max_dist=0.001)
    ctx.expect_within(slider, body, axes="xz", inner_elem=slider_thumb, outer_elem=rear_panel)
    ctx.expect_gap(
        body,
        slider,
        axis="y",
        positive_elem=slider_track,
        negative_elem=slider_guide,
        max_gap=0.001,
        max_penetration=0.001,
    )

    ctx.expect_origin_distance(gear, body, axes="xy", max_dist=0.001)
    ctx.expect_within(gear, body, axes="xy", inner_elem=gear_rim, outer_elem=bottom_panel)
    ctx.expect_gap(
        body,
        gear,
        axis="z",
        positive_elem=gear_seat,
        negative_elem=gear_axle,
        max_gap=0.001,
        max_penetration=0.002,
    )

    with ctx.pose({button_press: -0.0025}):
        ctx.expect_within(button, body, axes="xy", inner_elem=button_cap, outer_elem=top_panel)
        ctx.expect_gap(
            button,
            body,
            axis="z",
            positive_elem=button_plunger,
            negative_elem=button_seat,
            max_gap=0.001,
            max_penetration=0.004,
        )

    with ctx.pose({dial_turn: 1.1}):
        ctx.expect_within(dial, body, axes="yz", inner_elem=dial_knob, outer_elem=right_panel)

    with ctx.pose({joystick_rock: 0.28}):
        ctx.expect_within(joystick, body, axes="yz", inner_elem=joystick_cap, outer_elem=left_panel)

    with ctx.pose({toggle_flip: 0.55}):
        ctx.expect_overlap(toggle, body, axes="xz", elem_a=toggle_tip, elem_b=front_panel, min_overlap=0.0001)
    with ctx.pose({toggle_flip: -0.55}):
        ctx.expect_overlap(toggle, body, axes="xz", elem_a=toggle_tip, elem_b=front_panel, min_overlap=0.0001)

    with ctx.pose({slider_move: 0.011}):
        ctx.expect_within(slider, body, axes="xz", inner_elem=slider_thumb, outer_elem=rear_panel)
    with ctx.pose({slider_move: -0.011}):
        ctx.expect_within(slider, body, axes="xz", inner_elem=slider_thumb, outer_elem=rear_panel)

    with ctx.pose({gear_spin: 0.8}):
        ctx.expect_within(gear, body, axes="xy", inner_elem=gear_rim, outer_elem=bottom_panel)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
