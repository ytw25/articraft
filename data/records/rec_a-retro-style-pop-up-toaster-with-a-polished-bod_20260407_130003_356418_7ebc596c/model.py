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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_popup_toaster")

    body_w = 0.312
    body_d = 0.184
    body_h = 0.212
    wall = 0.003
    shell_top = 0.144
    shoulder_radius = 0.021
    shoulder_y = 0.071
    shoulder_z = 0.165
    deck_top = 0.190
    deck_thickness = 0.004

    slot_center_x = 0.045
    slot_width = 0.034
    slot_length = 0.100
    slot_liner_depth = 0.016

    chrome = model.material("chrome_body", rgba=(0.84, 0.85, 0.87, 1.0))
    black = model.material("bakelite_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel_trim", rgba=(0.71, 0.73, 0.75, 1.0))
    ivory = model.material("ivory_selector", rgba=(0.94, 0.91, 0.82, 1.0))
    red = model.material("cancel_red", rgba=(0.75, 0.10, 0.08, 1.0))

    chassis = model.part("chassis")

    # Lower base skirt.
    chassis.visual(
        Box((body_w - 0.008, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -(body_d / 2.0 + 0.002), 0.010)),
        material=black,
        name="front_base_strip",
    )
    chassis.visual(
        Box((0.094, 0.010, 0.020)),
        origin=Origin(xyz=(-0.109, body_d / 2.0 + 0.002, 0.010)),
        material=black,
        name="rear_base_strip_left",
    )
    chassis.visual(
        Box((0.094, 0.010, 0.020)),
        origin=Origin(xyz=(0.109, body_d / 2.0 + 0.002, 0.010)),
        material=black,
        name="rear_base_strip_right",
    )
    chassis.visual(
        Box((0.010, body_d, 0.020)),
        origin=Origin(xyz=(-(body_w / 2.0 + 0.002), 0.0, 0.010)),
        material=black,
        name="left_base_strip",
    )
    chassis.visual(
        Box((0.010, body_d, 0.020)),
        origin=Origin(xyz=(body_w / 2.0 + 0.002, 0.0, 0.010)),
        material=black,
        name="right_base_strip",
    )

    # Chrome shell walls.
    chassis.visual(
        Box((wall, body_d, shell_top - 0.020)),
        origin=Origin(xyz=(-(body_w / 2.0 - wall / 2.0), 0.0, (shell_top + 0.020) / 2.0)),
        material=chrome,
        name="left_wall",
    )
    chassis.visual(
        Box((wall, body_d, shell_top - 0.020)),
        origin=Origin(xyz=((body_w / 2.0 - wall / 2.0), 0.0, (shell_top + 0.020) / 2.0)),
        material=chrome,
        name="right_wall",
    )
    chassis.visual(
        Box((body_w - 2.0 * wall, wall, shell_top - 0.020)),
        origin=Origin(xyz=(0.0, -(body_d / 2.0 - wall / 2.0), (shell_top + 0.020) / 2.0)),
        material=chrome,
        name="front_wall",
    )
    chassis.visual(
        Box((body_w - 2.0 * wall, wall, shell_top - 0.045)),
        origin=Origin(xyz=(0.0, body_d / 2.0 - wall / 2.0, (shell_top + 0.045) / 2.0)),
        material=chrome,
        name="rear_upper_wall",
    )

    # Rounded upper shoulders.
    chassis.visual(
        Cylinder(radius=shoulder_radius, length=body_w - 2.0 * wall),
        origin=Origin(
            xyz=(0.0, -shoulder_y, shoulder_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="front_shoulder",
    )
    chassis.visual(
        Cylinder(radius=shoulder_radius, length=body_w - 2.0 * wall),
        origin=Origin(
            xyz=(0.0, shoulder_y, shoulder_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="rear_shoulder",
    )

    # Top deck around the bread slots.
    chassis.visual(
        Box((0.068, slot_length, deck_thickness)),
        origin=Origin(xyz=(-0.095, 0.0, deck_top - deck_thickness / 2.0)),
        material=chrome,
        name="left_top_deck",
    )
    chassis.visual(
        Box((0.056, slot_length, deck_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_top - deck_thickness / 2.0)),
        material=chrome,
        name="center_top_bridge",
    )
    chassis.visual(
        Box((0.068, slot_length, deck_thickness)),
        origin=Origin(xyz=(0.095, 0.0, deck_top - deck_thickness / 2.0)),
        material=chrome,
        name="right_top_deck",
    )

    # Dark slot liners so the openings read clearly.
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        slot_x = x_sign * slot_center_x
        chassis.visual(
            Box((0.002, slot_length, slot_liner_depth)),
            origin=Origin(
                xyz=(
                    slot_x - slot_width / 2.0,
                    0.0,
                    deck_top - deck_thickness - slot_liner_depth / 2.0,
                )
            ),
            material=black,
            name=f"{side_name}_slot_left_liner",
        )
        chassis.visual(
            Box((0.002, slot_length, slot_liner_depth)),
            origin=Origin(
                xyz=(
                    slot_x + slot_width / 2.0,
                    0.0,
                    deck_top - deck_thickness - slot_liner_depth / 2.0,
                )
            ),
            material=black,
            name=f"{side_name}_slot_right_liner",
        )
        chassis.visual(
            Box((slot_width, 0.002, slot_liner_depth)),
            origin=Origin(
                xyz=(
                    slot_x,
                    -slot_length / 2.0,
                    deck_top - deck_thickness - slot_liner_depth / 2.0,
                )
            ),
            material=black,
            name=f"{side_name}_slot_front_liner",
        )
        chassis.visual(
            Box((slot_width, 0.002, slot_liner_depth)),
            origin=Origin(
                xyz=(
                    slot_x,
                    slot_length / 2.0,
                    deck_top - deck_thickness - slot_liner_depth / 2.0,
                )
            ),
            material=black,
            name=f"{side_name}_slot_rear_liner",
        )

    # Front control pod.
    chassis.visual(
        Box((0.116, 0.018, 0.102)),
        origin=Origin(xyz=(0.067, -(body_d / 2.0 + 0.009), 0.071)),
        material=black,
        name="control_pod",
    )

    # Side lever guide.
    chassis.visual(
        Box((0.007, 0.052, 0.150)),
        origin=Origin(xyz=(body_w / 2.0 + 0.0045, 0.040, 0.095)),
        material=black,
        name="lever_backplate",
    )
    chassis.visual(
        Box((0.013, 0.008, 0.150)),
        origin=Origin(xyz=(body_w / 2.0 + 0.0105, 0.018, 0.095)),
        material=black,
        name="lever_front_rail",
    )
    chassis.visual(
        Box((0.013, 0.008, 0.150)),
        origin=Origin(xyz=(body_w / 2.0 + 0.0105, 0.062, 0.095)),
        material=black,
        name="lever_rear_rail",
    )

    chassis.inertial = Inertial.from_geometry(
        Box((body_w + 0.020, body_d + 0.020, body_h)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.064, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="cross_bridge",
    )
    carriage.visual(
        Box((0.120, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.030, 0.004)),
        material=steel,
        name="rear_bridge",
    )
    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        basket_x = x_sign * slot_center_x
        carriage.visual(
            Box((0.030, 0.080, 0.003)),
            origin=Origin(xyz=(basket_x, 0.0, 0.0015)),
            material=steel,
            name=f"{side_name}_basket_floor",
        )
        carriage.visual(
            Box((0.002, 0.080, 0.082)),
            origin=Origin(xyz=(basket_x - 0.015, 0.0, 0.041)),
            material=steel,
            name=f"{side_name}_basket_left_wall",
        )
        carriage.visual(
            Box((0.002, 0.080, 0.082)),
            origin=Origin(xyz=(basket_x + 0.015, 0.0, 0.041)),
            material=steel,
            name=f"{side_name}_basket_right_wall",
        )
        carriage.visual(
            Box((0.032, 0.002, 0.082)),
            origin=Origin(xyz=(basket_x, -0.039, 0.041)),
            material=steel,
            name=f"{side_name}_basket_front_wall",
        )
        carriage.visual(
            Box((0.032, 0.002, 0.082)),
            origin=Origin(xyz=(basket_x, 0.039, 0.041)),
            material=steel,
            name=f"{side_name}_basket_back_wall",
        )
    carriage.inertial = Inertial.from_geometry(
        Box((0.120, 0.082, 0.082)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    model.articulation(
        "chassis_to_carriage",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.12,
            lower=0.0,
            upper=0.058,
        ),
    )

    lever = model.part("lever")
    lever.visual(
        Box((0.013, 0.008, 0.092)),
        origin=Origin(xyz=(-0.0045, 0.0, -0.020)),
        material=steel,
        name="lever_stem",
    )
    lever.visual(
        Box((0.024, 0.014, 0.060)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=black,
        name="lever_paddle",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.030, 0.016, 0.100)),
        mass=0.08,
        origin=Origin(xyz=(0.010, 0.0, -0.010)),
    )

    model.articulation(
        "chassis_to_lever",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=lever,
        origin=Origin(xyz=(body_w / 2.0 + 0.019, 0.040, 0.130)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.12,
            lower=0.0,
            upper=0.058,
        ),
    )

    selector = model.part("shade_selector")
    selector.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ivory,
        name="selector_knob",
    )
    selector.visual(
        Box((0.004, 0.003, 0.018)),
        origin=Origin(xyz=(0.0, -0.0215, 0.013)),
        material=red,
        name="selector_pointer",
    )
    selector.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=0.020),
        mass=0.09,
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
    )

    model.articulation(
        "chassis_to_selector",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=selector,
        origin=Origin(xyz=(0.067, -(body_d / 2.0 + 0.018), 0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=3.0,
            lower=-2.2,
            upper=2.2,
        ),
    )

    cancel_button = model.part("cancel_button")
    cancel_button.visual(
        Box((0.024, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=red,
        name="button_cap",
    )
    cancel_button.visual(
        Box((0.010, 0.003, 0.010)),
        origin=Origin(xyz=(0.0, -0.0015, 0.0)),
        material=black,
        name="button_plunger",
    )
    cancel_button.inertial = Inertial.from_geometry(
        Box((0.024, 0.006, 0.016)),
        mass=0.02,
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
    )

    model.articulation(
        "chassis_to_cancel_button",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=cancel_button,
        origin=Origin(xyz=(0.067, -(body_d / 2.0 + 0.018), 0.102)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        Box((0.170, 0.120, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=steel,
        name="tray_bottom",
    )
    crumb_tray.visual(
        Box((0.002, 0.120, 0.010)),
        origin=Origin(xyz=(-0.084, 0.0, 0.005)),
        material=steel,
        name="tray_left_wall",
    )
    crumb_tray.visual(
        Box((0.002, 0.120, 0.010)),
        origin=Origin(xyz=(0.084, 0.0, 0.005)),
        material=steel,
        name="tray_right_wall",
    )
    crumb_tray.visual(
        Box((0.170, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, -0.059, 0.005)),
        material=steel,
        name="tray_front_wall",
    )
    crumb_tray.visual(
        Box((0.060, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.062, 0.008)),
        material=black,
        name="tray_pull",
    )
    crumb_tray.inertial = Inertial.from_geometry(
        Box((0.172, 0.124, 0.018)),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    model.articulation(
        "chassis_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=crumb_tray,
        origin=Origin(xyz=(0.0, 0.038, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=0.070,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    carriage = object_model.get_part("carriage")
    lever = object_model.get_part("lever")
    selector = object_model.get_part("shade_selector")
    cancel_button = object_model.get_part("cancel_button")
    crumb_tray = object_model.get_part("crumb_tray")

    carriage_joint = object_model.get_articulation("chassis_to_carriage")
    lever_joint = object_model.get_articulation("chassis_to_lever")
    selector_joint = object_model.get_articulation("chassis_to_selector")
    button_joint = object_model.get_articulation("chassis_to_cancel_button")
    tray_joint = object_model.get_articulation("chassis_to_crumb_tray")

    ctx.check(
        "selector rotates about front axis",
        abs(selector_joint.axis[1]) > 0.99,
        details=f"axis={selector_joint.axis}",
    )
    ctx.check(
        "cancel button plunges inward from the front",
        button_joint.axis[1] > 0.99,
        details=f"axis={button_joint.axis}",
    )
    ctx.check(
        "shade selector has broad rotary travel",
        selector_joint.motion_limits is not None
        and selector_joint.motion_limits.lower is not None
        and selector_joint.motion_limits.upper is not None
        and selector_joint.motion_limits.lower < -1.5
        and selector_joint.motion_limits.upper > 1.5,
        details=str(selector_joint.motion_limits),
    )

    rest_carriage = ctx.part_world_position(carriage)
    rest_lever = ctx.part_world_position(lever)
    rest_button = ctx.part_world_position(cancel_button)
    rest_tray = ctx.part_world_position(crumb_tray)
    rest_selector = ctx.part_world_position(selector)

    with ctx.pose(
        {
            carriage_joint: carriage_joint.motion_limits.upper,
            lever_joint: lever_joint.motion_limits.upper,
        }
    ):
        depressed_carriage = ctx.part_world_position(carriage)
        depressed_lever = ctx.part_world_position(lever)

    ctx.check(
        "bread carriage lowers into the shell",
        rest_carriage is not None
        and depressed_carriage is not None
        and depressed_carriage[2] < rest_carriage[2] - 0.05,
        details=f"rest={rest_carriage}, depressed={depressed_carriage}",
    )
    ctx.check(
        "side lever drops with the carriage",
        rest_carriage is not None
        and depressed_carriage is not None
        and rest_lever is not None
        and depressed_lever is not None
        and depressed_lever[2] < rest_lever[2] - 0.05
        and abs((rest_lever[2] - rest_carriage[2]) - (depressed_lever[2] - depressed_carriage[2])) < 0.004,
        details=(
            f"rest_carriage={rest_carriage}, depressed_carriage={depressed_carriage}, "
            f"rest_lever={rest_lever}, depressed_lever={depressed_lever}"
        ),
    )

    with ctx.pose({button_joint: button_joint.motion_limits.upper}):
        pressed_button = ctx.part_world_position(cancel_button)

    ctx.check(
        "cancel button presses inward",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[1] > rest_button[1] + 0.002,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    with ctx.pose({tray_joint: tray_joint.motion_limits.upper}):
        extended_tray = ctx.part_world_position(crumb_tray)
        ctx.expect_overlap(
            crumb_tray,
            chassis,
            axes="x",
            min_overlap=0.10,
            elem_a="tray_bottom",
            name="crumb tray stays centered beneath the toaster body",
        )

    ctx.check(
        "crumb tray slides out from the shell",
        rest_tray is not None
        and extended_tray is not None
        and extended_tray[1] > rest_tray[1] + 0.05,
        details=f"rest={rest_tray}, extended={extended_tray}",
    )

    with ctx.pose({selector_joint: 1.4}):
        rotated_selector = ctx.part_world_position(selector)

    ctx.check(
        "selector turns in place",
        rest_selector is not None
        and rotated_selector is not None
        and abs(rotated_selector[0] - rest_selector[0]) < 1e-6
        and abs(rotated_selector[1] - rest_selector[1]) < 1e-6
        and abs(rotated_selector[2] - rest_selector[2]) < 1e-6,
        details=f"rest={rest_selector}, rotated={rotated_selector}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
