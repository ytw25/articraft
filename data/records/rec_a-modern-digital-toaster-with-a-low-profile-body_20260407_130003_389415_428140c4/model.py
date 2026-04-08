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
    model = ArticulatedObject(name="digital_toaster")

    shell_metal = model.material("shell_metal", rgba=(0.79, 0.80, 0.82, 1.0))
    top_trim = model.material("top_trim", rgba=(0.64, 0.65, 0.68, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.11, 0.12, 0.13, 1.0))
    knob_plastic = model.material("knob_plastic", rgba=(0.14, 0.14, 0.15, 1.0))
    button_grey = model.material("button_grey", rgba=(0.30, 0.31, 0.33, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.67, 0.68, 0.70, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    carriage_steel = model.material("carriage_steel", rgba=(0.58, 0.60, 0.62, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.328, 0.168, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=charcoal,
        name="bottom_base",
    )
    body.visual(
        Box((0.008, 0.180, 0.174)),
        origin=Origin(xyz=(-0.166, 0.0, 0.097)),
        material=shell_metal,
        name="left_end_wall",
    )
    body.visual(
        Box((0.328, 0.008, 0.174)),
        origin=Origin(xyz=(0.0, -0.086, 0.097)),
        material=shell_metal,
        name="back_wall",
    )
    body.visual(
        Box((0.328, 0.008, 0.146)),
        origin=Origin(xyz=(0.0, 0.086, 0.117)),
        material=shell_metal,
        name="front_upper_wall",
    )
    body.visual(
        Box((0.040, 0.008, 0.044)),
        origin=Origin(xyz=(-0.144, 0.086, 0.022)),
        material=shell_metal,
        name="front_lower_left",
    )
    body.visual(
        Box((0.040, 0.008, 0.044)),
        origin=Origin(xyz=(0.144, 0.086, 0.022)),
        material=shell_metal,
        name="front_lower_right",
    )
    body.visual(
        Box((0.328, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, 0.079, 0.186)),
        material=top_trim,
        name="top_front_strip",
    )
    body.visual(
        Box((0.328, 0.022, 0.008)),
        origin=Origin(xyz=(0.0, -0.079, 0.186)),
        material=top_trim,
        name="top_back_strip",
    )
    body.visual(
        Box((0.104, 0.136, 0.008)),
        origin=Origin(xyz=(-0.112, 0.0, 0.186)),
        material=top_trim,
        name="top_left_strip",
    )
    body.visual(
        Box((0.104, 0.136, 0.008)),
        origin=Origin(xyz=(0.112, 0.0, 0.186)),
        material=top_trim,
        name="top_right_strip",
    )
    body.visual(
        Box((0.062, 0.136, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.186)),
        material=top_trim,
        name="top_center_bridge",
    )

    for prefix, slot_x in (("left", -0.047), ("right", 0.047)):
        body.visual(
            Box((0.004, 0.135, 0.068)),
            origin=Origin(xyz=(slot_x - 0.016, 0.0, 0.150)),
            material=dark_plastic,
            name=f"{prefix}_slot_left_wall",
        )
        body.visual(
            Box((0.004, 0.135, 0.068)),
            origin=Origin(xyz=(slot_x + 0.016, 0.0, 0.150)),
            material=dark_plastic,
            name=f"{prefix}_slot_right_wall",
        )
        body.visual(
            Box((0.028, 0.004, 0.068)),
            origin=Origin(xyz=(slot_x, 0.066, 0.150)),
            material=dark_plastic,
            name=f"{prefix}_slot_front_wall",
        )
        body.visual(
            Box((0.028, 0.004, 0.068)),
            origin=Origin(xyz=(slot_x, -0.066, 0.150)),
            material=dark_plastic,
            name=f"{prefix}_slot_back_wall",
        )

    body.visual(
        Box((0.008, 0.180, 0.026)),
        origin=Origin(xyz=(0.166, 0.0, 0.177)),
        material=shell_metal,
        name="control_end_top",
    )
    body.visual(
        Box((0.008, 0.180, 0.050)),
        origin=Origin(xyz=(0.166, 0.0, 0.025)),
        material=shell_metal,
        name="control_end_bottom",
    )
    body.visual(
        Box((0.008, 0.038, 0.112)),
        origin=Origin(xyz=(0.166, -0.015, 0.099)),
        material=charcoal,
        name="end_center_rib",
    )
    body.visual(
        Box((0.008, 0.080, 0.020)),
        origin=Origin(xyz=(0.166, 0.036, 0.155)),
        material=charcoal,
        name="panel_bezel_top",
    )
    body.visual(
        Box((0.008, 0.080, 0.020)),
        origin=Origin(xyz=(0.166, 0.036, 0.045)),
        material=charcoal,
        name="panel_bezel_bottom",
    )
    body.visual(
        Box((0.008, 0.012, 0.112)),
        origin=Origin(xyz=(0.166, 0.002, 0.100)),
        material=charcoal,
        name="panel_bezel_back",
    )
    body.visual(
        Box((0.008, 0.012, 0.112)),
        origin=Origin(xyz=(0.166, 0.070, 0.100)),
        material=charcoal,
        name="panel_bezel_front",
    )
    body.visual(
        Box((0.008, 0.030, 0.022)),
        origin=Origin(xyz=(0.166, -0.048, 0.154)),
        material=charcoal,
        name="lever_frame_top",
    )
    body.visual(
        Box((0.008, 0.030, 0.024)),
        origin=Origin(xyz=(0.166, -0.048, 0.055)),
        material=charcoal,
        name="lever_frame_bottom",
    )
    body.visual(
        Box((0.008, 0.008, 0.088)),
        origin=Origin(xyz=(0.166, -0.059, 0.104)),
        material=charcoal,
        name="lever_frame_back",
    )
    body.visual(
        Box((0.008, 0.008, 0.088)),
        origin=Origin(xyz=(0.166, -0.037, 0.104)),
        material=charcoal,
        name="lever_frame_front",
    )
    body.visual(
        Box((0.008, 0.006, 0.068)),
        origin=Origin(xyz=(0.166, 0.023, 0.111)),
        material=charcoal,
        name="button_guide_left",
    )
    body.visual(
        Box((0.008, 0.006, 0.068)),
        origin=Origin(xyz=(0.166, 0.049, 0.111)),
        material=charcoal,
        name="button_guide_right",
    )
    body.visual(
        Box((0.008, 0.032, 0.004)),
        origin=Origin(xyz=(0.166, 0.036, 0.147)),
        material=charcoal,
        name="button_guide_top",
    )
    body.visual(
        Box((0.008, 0.032, 0.004)),
        origin=Origin(xyz=(0.166, 0.036, 0.123)),
        material=charcoal,
        name="button_guide_separator_upper",
    )
    body.visual(
        Box((0.008, 0.032, 0.004)),
        origin=Origin(xyz=(0.166, 0.036, 0.098)),
        material=charcoal,
        name="button_guide_separator_lower",
    )
    body.visual(
        Box((0.008, 0.032, 0.024)),
        origin=Origin(xyz=(0.166, 0.036, 0.064)),
        material=charcoal,
        name="button_guide_bottom",
    )
    body.visual(
        Box((0.008, 0.004, 0.034)),
        origin=Origin(xyz=(0.166, 0.020, 0.062)),
        material=charcoal,
        name="selector_guide_left",
    )
    body.visual(
        Box((0.008, 0.004, 0.034)),
        origin=Origin(xyz=(0.166, 0.052, 0.062)),
        material=charcoal,
        name="selector_guide_right",
    )
    body.visual(
        Box((0.008, 0.036, 0.004)),
        origin=Origin(xyz=(0.166, 0.036, 0.078)),
        material=charcoal,
        name="selector_guide_top",
    )
    body.visual(
        Box((0.008, 0.036, 0.004)),
        origin=Origin(xyz=(0.166, 0.036, 0.046)),
        material=charcoal,
        name="selector_guide_bottom",
    )

    for foot_index, foot_center in enumerate(
        ((-0.126, -0.062, -0.004), (0.126, -0.062, -0.004), (-0.126, 0.062, -0.004), (0.126, 0.062, -0.004)),
        start=1,
    ):
        body.visual(
            Box((0.030, 0.030, 0.008)),
            origin=Origin(xyz=foot_center),
            material=rubber,
            name=f"foot_{foot_index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.340, 0.180, 0.190)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.128, 0.040, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=carriage_steel,
        name="carriage_frame",
    )
    carriage.visual(
        Box((0.010, 0.012, 0.060)),
        origin=Origin(xyz=(-0.047, 0.0, -0.044)),
        material=carriage_steel,
        name="left_post",
    )
    carriage.visual(
        Box((0.010, 0.012, 0.060)),
        origin=Origin(xyz=(0.047, 0.0, -0.044)),
        material=carriage_steel,
        name="right_post",
    )
    carriage.visual(
        Box((0.026, 0.110, 0.004)),
        origin=Origin(xyz=(-0.047, 0.0, -0.012)),
        material=carriage_steel,
        name="left_lift_plate",
    )
    carriage.visual(
        Box((0.026, 0.110, 0.004)),
        origin=Origin(xyz=(0.047, 0.0, -0.012)),
        material=carriage_steel,
        name="right_lift_plate",
    )
    carriage.visual(
        Box((0.028, 0.084, 0.010)),
        origin=Origin(xyz=(0.061, -0.042, -0.044)),
        material=carriage_steel,
        name="lever_crossbar",
    )
    carriage.visual(
        Box((0.172, 0.012, 0.010)),
        origin=Origin(xyz=(0.084, -0.048, -0.044)),
        material=carriage_steel,
        name="lever_rod",
    )
    carriage.visual(
        Box((0.010, 0.014, 0.042)),
        origin=Origin(xyz=(0.173, -0.048, -0.044)),
        material=dark_plastic,
        name="lever_handle",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.180, 0.110, 0.100)),
        mass=0.32,
        origin=Origin(xyz=(0.060, -0.020, -0.045)),
    )
    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.18,
            lower=0.0,
            upper=0.045,
        ),
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        Box((0.248, 0.135, 0.003)),
        origin=Origin(xyz=(0.0, -0.0675, 0.0015)),
        material=tray_metal,
        name="tray_bottom",
    )
    crumb_tray.visual(
        Box((0.003, 0.135, 0.014)),
        origin=Origin(xyz=(-0.1225, -0.0675, 0.008)),
        material=tray_metal,
        name="tray_left_wall",
    )
    crumb_tray.visual(
        Box((0.003, 0.135, 0.014)),
        origin=Origin(xyz=(0.1225, -0.0675, 0.008)),
        material=tray_metal,
        name="tray_right_wall",
    )
    crumb_tray.visual(
        Box((0.248, 0.003, 0.014)),
        origin=Origin(xyz=(0.0, -0.134, 0.008)),
        material=tray_metal,
        name="tray_back_wall",
    )
    crumb_tray.visual(
        Box((0.248, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=tray_metal,
        name="tray_front",
    )
    crumb_tray.visual(
        Box((0.060, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.004, 0.012)),
        material=dark_plastic,
        name="tray_pull",
    )
    crumb_tray.inertial = Inertial.from_geometry(
        Box((0.248, 0.135, 0.020)),
        mass=0.16,
        origin=Origin(xyz=(0.0, -0.067, 0.010)),
    )
    model.articulation(
        "body_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=crumb_tray,
        origin=Origin(xyz=(0.0, 0.088, 0.013)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.20,
            lower=0.0,
            upper=0.060,
        ),
    )

    button_positions = (0.136, 0.111, 0.086)
    for button_index, button_z in enumerate(button_positions, start=1):
        button = model.part(f"button_{button_index}")
        button.visual(
            Box((0.014, 0.020, 0.008)),
            material=button_grey,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.010, 0.012)),
            origin=Origin(xyz=(-0.011, 0.0, 0.0)),
            material=dark_plastic,
            name="button_plunger",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.024, 0.020, 0.012)),
            mass=0.018,
            origin=Origin(xyz=(-0.005, 0.0, 0.0)),
        )
        model.articulation(
            f"body_to_button_{button_index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.177, 0.036, button_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=0.005,
            ),
        )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.005, length=0.016),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="selector_shaft",
    )
    selector.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_plastic,
        name="selector_knob",
    )
    selector.visual(
        Box((0.004, 0.003, 0.010)),
        origin=Origin(xyz=(0.023, 0.0, 0.008)),
        material=top_trim,
        name="selector_pointer",
    )
    selector.inertial = Inertial.from_geometry(
        Box((0.030, 0.030, 0.030)),
        mass=0.045,
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.171, 0.036, 0.062)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=3.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    carriage_joint = object_model.get_articulation("body_to_carriage")
    crumb_tray = object_model.get_part("crumb_tray")
    crumb_tray_joint = object_model.get_articulation("body_to_crumb_tray")
    selector = object_model.get_part("selector")
    selector_joint = object_model.get_articulation("body_to_selector")

    carriage_rest = ctx.part_world_position(carriage)
    rest_handle_aabb = ctx.part_element_world_aabb(carriage, elem="lever_handle")
    rest_plate_aabb = ctx.part_element_world_aabb(carriage, elem="left_lift_plate")
    with ctx.pose({carriage_joint: carriage_joint.motion_limits.upper}):
        carriage_pressed = ctx.part_world_position(carriage)
        pressed_handle_aabb = ctx.part_element_world_aabb(carriage, elem="lever_handle")
        pressed_plate_aabb = ctx.part_element_world_aabb(carriage, elem="left_lift_plate")

    ctx.check(
        "carriage lowers into the shell",
        carriage_rest is not None
        and carriage_pressed is not None
        and carriage_pressed[2] < carriage_rest[2] - 0.035,
        details=f"rest={carriage_rest}, pressed={carriage_pressed}",
    )
    ctx.check(
        "lever follows the carriage on the same vertical path",
        rest_handle_aabb is not None
        and pressed_handle_aabb is not None
        and rest_plate_aabb is not None
        and pressed_plate_aabb is not None
        and abs(
            (pressed_handle_aabb[0][2] - rest_handle_aabb[0][2])
            - (pressed_plate_aabb[0][2] - rest_plate_aabb[0][2])
        )
        <= 0.001,
        details=(
            f"handle_rest={rest_handle_aabb}, handle_pressed={pressed_handle_aabb}, "
            f"plate_rest={rest_plate_aabb}, plate_pressed={pressed_plate_aabb}"
        ),
    )

    tray_rest = ctx.part_world_position(crumb_tray)
    with ctx.pose({crumb_tray_joint: crumb_tray_joint.motion_limits.upper}):
        tray_extended = ctx.part_world_position(crumb_tray)
    ctx.check(
        "crumb tray slides out from the lower side",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] > tray_rest[1] + 0.050,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    for button_index in range(1, 4):
        button = object_model.get_part(f"button_{button_index}")
        button_joint = object_model.get_articulation(f"body_to_button_{button_index}")
        button_rest = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            button_pressed = ctx.part_world_position(button)
        ctx.check(
            f"button {button_index} plunges into the side panel",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[0] < button_rest[0] - 0.004,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    selector_rest = ctx.part_world_position(selector)
    with ctx.pose({selector_joint: 1.2}):
        selector_rotated = ctx.part_world_position(selector)
    selector_limits = selector_joint.motion_limits
    ctx.check(
        "selector is a rotary control on a short shaft",
        selector_joint.axis == (1.0, 0.0, 0.0)
        and selector_limits is not None
        and selector_limits.lower is not None
        and selector_limits.upper is not None
        and selector_limits.upper - selector_limits.lower >= 4.5
        and selector_rest == selector_rotated,
        details=(
            f"axis={selector_joint.axis}, limits={selector_limits}, "
            f"rest={selector_rest}, rotated={selector_rotated}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
