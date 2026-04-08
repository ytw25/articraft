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
    model = ArticulatedObject(name="compact_countertop_toaster")

    brushed_steel = model.material("brushed_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    shadow_black = model.material("shadow_black", rgba=(0.15, 0.16, 0.17, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    warm_wire = model.material("warm_wire", rgba=(0.55, 0.57, 0.60, 1.0))

    housing = model.part("housing")

    housing.visual(
        Box((0.270, 0.180, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.003)),
        material=shadow_black,
        name="base_plate",
    )
    housing.visual(
        Box((0.004, 0.135, 0.189)),
        origin=Origin(xyz=(-0.133, 0.0225, 0.1005)),
        material=brushed_steel,
        name="left_wall_rear",
    )
    housing.visual(
        Box((0.004, 0.045, 0.044)),
        origin=Origin(xyz=(-0.133, -0.0675, 0.028)),
        material=brushed_steel,
        name="left_wall_front_lower",
    )
    housing.visual(
        Box((0.004, 0.045, 0.090)),
        origin=Origin(xyz=(-0.133, -0.0675, 0.150)),
        material=brushed_steel,
        name="left_wall_front_upper",
    )
    housing.visual(
        Box((0.004, 0.180, 0.189)),
        origin=Origin(xyz=(0.133, 0.000, 0.1005)),
        material=brushed_steel,
        name="right_wall",
    )
    housing.visual(
        Box((0.262, 0.004, 0.189)),
        origin=Origin(xyz=(0.000, 0.088, 0.1005)),
        material=brushed_steel,
        name="back_wall",
    )
    housing.visual(
        Box((0.262, 0.004, 0.016)),
        origin=Origin(xyz=(0.000, -0.088, 0.014)),
        material=brushed_steel,
        name="front_lower_rail",
    )
    housing.visual(
        Box((0.038, 0.004, 0.033)),
        origin=Origin(xyz=(-0.112, -0.088, 0.0385)),
        material=brushed_steel,
        name="front_left_jamb",
    )
    housing.visual(
        Box((0.038, 0.004, 0.033)),
        origin=Origin(xyz=(0.112, -0.088, 0.0385)),
        material=brushed_steel,
        name="front_right_jamb",
    )
    housing.visual(
        Box((0.198, 0.004, 0.140)),
        origin=Origin(xyz=(-0.032, -0.088, 0.125)),
        material=brushed_steel,
        name="front_upper_left_panel",
    )
    housing.visual(
        Box((0.013, 0.004, 0.140)),
        origin=Origin(xyz=(0.0735, -0.088, 0.125)),
        material=brushed_steel,
        name="control_left_stile",
    )
    housing.visual(
        Box((0.017, 0.004, 0.140)),
        origin=Origin(xyz=(0.1225, -0.088, 0.125)),
        material=brushed_steel,
        name="control_right_stile",
    )
    housing.visual(
        Box((0.064, 0.004, 0.039)),
        origin=Origin(xyz=(0.099, -0.088, 0.1755)),
        material=brushed_steel,
        name="control_top_strip",
    )
    housing.visual(
        Box((0.064, 0.004, 0.012)),
        origin=Origin(xyz=(0.099, -0.088, 0.132)),
        material=brushed_steel,
        name="control_button_divider",
    )
    housing.visual(
        Box((0.064, 0.004, 0.022)),
        origin=Origin(xyz=(0.099, -0.088, 0.097)),
        material=brushed_steel,
        name="control_dial_header",
    )
    housing.visual(
        Box((0.064, 0.004, 0.007)),
        origin=Origin(xyz=(0.099, -0.088, 0.0585)),
        material=brushed_steel,
        name="control_bottom_strip",
    )
    housing.visual(
        Box((0.003, 0.012, 0.018)),
        origin=Origin(xyz=(0.0865, -0.084, 0.147)),
        material=shadow_black,
        name="upper_button_left_guide",
    )
    housing.visual(
        Box((0.003, 0.012, 0.018)),
        origin=Origin(xyz=(0.1075, -0.084, 0.147)),
        material=shadow_black,
        name="upper_button_right_guide",
    )
    housing.visual(
        Box((0.003, 0.012, 0.018)),
        origin=Origin(xyz=(0.0865, -0.084, 0.117)),
        material=shadow_black,
        name="lower_button_left_guide",
    )
    housing.visual(
        Box((0.003, 0.012, 0.018)),
        origin=Origin(xyz=(0.1075, -0.084, 0.117)),
        material=shadow_black,
        name="lower_button_right_guide",
    )
    housing.visual(
        Box((0.262, 0.030, 0.004)),
        origin=Origin(xyz=(0.000, -0.073, 0.193)),
        material=brushed_steel,
        name="top_front_bridge",
    )
    housing.visual(
        Box((0.262, 0.030, 0.004)),
        origin=Origin(xyz=(0.000, 0.073, 0.193)),
        material=brushed_steel,
        name="top_rear_bridge",
    )
    housing.visual(
        Box((0.078, 0.120, 0.004)),
        origin=Origin(xyz=(-0.092, 0.000, 0.193)),
        material=brushed_steel,
        name="top_left_deck",
    )
    housing.visual(
        Box((0.030, 0.120, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.193)),
        material=brushed_steel,
        name="top_center_bridge",
    )
    housing.visual(
        Box((0.078, 0.120, 0.004)),
        origin=Origin(xyz=(0.092, 0.000, 0.193)),
        material=brushed_steel,
        name="top_right_deck",
    )
    housing.visual(
        Box((0.052, 0.124, 0.026)),
        origin=Origin(xyz=(-0.0335, 0.000, 0.178)),
        material=shadow_black,
        name="left_slot_liner",
    )
    housing.visual(
        Box((0.052, 0.124, 0.026)),
        origin=Origin(xyz=(0.0335, 0.000, 0.178)),
        material=shadow_black,
        name="right_slot_liner",
    )
    housing.visual(
        Box((0.006, 0.044, 0.138)),
        origin=Origin(xyz=(-0.011, 0.000, 0.075)),
        material=shadow_black,
        name="left_carriage_guide",
    )
    housing.visual(
        Box((0.006, 0.044, 0.138)),
        origin=Origin(xyz=(0.011, 0.000, 0.075)),
        material=shadow_black,
        name="right_carriage_guide",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.270, 0.180, 0.195)),
        mass=2.4,
        origin=Origin(xyz=(0.000, 0.000, 0.0975)),
    )

    carriage = model.part("bread_carriage")
    carriage.visual(
        Box((0.120, 0.136, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, 0.008)),
        material=warm_wire,
        name="carriage_floor",
    )
    carriage.visual(
        Box((0.040, 0.126, 0.003)),
        origin=Origin(xyz=(-0.034, 0.000, 0.024)),
        material=warm_wire,
        name="left_bread_support",
    )
    carriage.visual(
        Box((0.040, 0.126, 0.003)),
        origin=Origin(xyz=(0.034, 0.000, 0.024)),
        material=warm_wire,
        name="right_bread_support",
    )
    for side_name, x_pos in (("left", -0.052), ("left_inner", -0.016), ("right_inner", 0.016), ("right", 0.052)):
        carriage.visual(
            Box((0.003, 0.126, 0.110)),
            origin=Origin(xyz=(x_pos, 0.000, 0.063)),
            material=warm_wire,
            name=f"{side_name}_upright",
        )
    carriage.visual(
        Box((0.108, 0.020, 0.010)),
        origin=Origin(xyz=(0.000, -0.055, 0.020)),
        material=warm_wire,
        name="front_crosshead",
    )
    carriage.visual(
        Box((0.108, 0.020, 0.010)),
        origin=Origin(xyz=(0.000, 0.055, 0.020)),
        material=warm_wire,
        name="rear_crosshead",
    )
    carriage.visual(
        Box((0.006, 0.044, 0.110)),
        origin=Origin(xyz=(-0.017, 0.000, 0.055)),
        material=shadow_black,
        name="left_guide_shoe",
    )
    carriage.visual(
        Box((0.006, 0.044, 0.110)),
        origin=Origin(xyz=(0.017, 0.000, 0.055)),
        material=shadow_black,
        name="right_guide_shoe",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.120, 0.136, 0.122)),
        mass=0.28,
        origin=Origin(xyz=(0.000, 0.000, 0.061)),
    )

    model.articulation(
        "housing_to_bread_carriage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=carriage,
        origin=Origin(xyz=(0.000, 0.000, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.12,
            lower=0.0,
            upper=0.042,
        ),
    )

    lever = model.part("carriage_lever")
    lever.visual(
        Box((0.014, 0.018, 0.052)),
        origin=Origin(xyz=(-0.008, 0.000, 0.026)),
        material=plastic_black,
        name="lever_paddle",
    )
    lever.visual(
        Box((0.020, 0.010, 0.012)),
        origin=Origin(xyz=(-0.015, 0.000, 0.010)),
        material=plastic_black,
        name="lever_thumb_lip",
    )
    lever.visual(
        Box((0.024, 0.008, 0.040)),
        origin=Origin(xyz=(0.006, 0.000, 0.020)),
        material=plastic_black,
        name="lever_stem",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.034, 0.018, 0.052)),
        mass=0.05,
        origin=Origin(xyz=(-0.004, 0.000, 0.026)),
    )
    model.articulation(
        "bread_carriage_to_lever",
        ArticulationType.FIXED,
        parent=carriage,
        child=lever,
        origin=Origin(xyz=(-0.129, -0.0675, 0.028)),
    )

    upper_button = model.part("upper_button")
    upper_button.visual(
        Box((0.026, 0.010, 0.014)),
        origin=Origin(xyz=(0.000, -0.005, 0.000)),
        material=plastic_black,
        name="button_cap",
    )
    upper_button.visual(
        Box((0.018, 0.012, 0.012)),
        origin=Origin(xyz=(0.000, 0.006, 0.000)),
        material=shadow_black,
        name="button_plunger",
    )
    upper_button.inertial = Inertial.from_geometry(
        Box((0.026, 0.022, 0.014)),
        mass=0.02,
        origin=Origin(xyz=(0.000, 0.001, 0.000)),
    )
    model.articulation(
        "housing_to_upper_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=upper_button,
        origin=Origin(xyz=(0.097, -0.090, 0.147)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )

    lower_button = model.part("lower_button")
    lower_button.visual(
        Box((0.026, 0.010, 0.014)),
        origin=Origin(xyz=(0.000, -0.005, 0.000)),
        material=plastic_black,
        name="button_cap",
    )
    lower_button.visual(
        Box((0.018, 0.012, 0.012)),
        origin=Origin(xyz=(0.000, 0.006, 0.000)),
        material=shadow_black,
        name="button_plunger",
    )
    lower_button.inertial = Inertial.from_geometry(
        Box((0.026, 0.022, 0.014)),
        mass=0.02,
        origin=Origin(xyz=(0.000, 0.001, 0.000)),
    )
    model.articulation(
        "housing_to_lower_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=lower_button,
        origin=Origin(xyz=(0.097, -0.090, 0.117)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(0.000, 0.007, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shadow_black,
        name="dial_shaft",
    )
    timer_dial.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(xyz=(0.000, -0.007, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="dial_knob",
    )
    timer_dial.visual(
        Box((0.004, 0.004, 0.014)),
        origin=Origin(xyz=(0.000, -0.017, 0.009)),
        material=tray_metal,
        name="dial_pointer",
    )
    timer_dial.inertial = Inertial.from_geometry(
        Box((0.032, 0.032, 0.018)),
        mass=0.03,
        origin=Origin(xyz=(0.000, -0.007, 0.000)),
    )
    model.articulation(
        "housing_to_timer_dial",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=timer_dial,
        origin=Origin(xyz=(0.097, -0.090, 0.074)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(240.0),
        ),
    )

    crumb_tray = model.part("crumb_tray")
    crumb_tray.visual(
        Box((0.176, 0.112, 0.002)),
        origin=Origin(xyz=(0.000, 0.056, 0.003)),
        material=tray_metal,
        name="tray_floor",
    )
    crumb_tray.visual(
        Box((0.002, 0.112, 0.014)),
        origin=Origin(xyz=(-0.087, 0.056, 0.009)),
        material=tray_metal,
        name="left_tray_wall",
    )
    crumb_tray.visual(
        Box((0.002, 0.112, 0.014)),
        origin=Origin(xyz=(0.087, 0.056, 0.009)),
        material=tray_metal,
        name="right_tray_wall",
    )
    crumb_tray.visual(
        Box((0.176, 0.002, 0.014)),
        origin=Origin(xyz=(0.000, 0.111, 0.009)),
        material=tray_metal,
        name="rear_tray_wall",
    )
    crumb_tray.visual(
        Box((0.184, 0.006, 0.030)),
        origin=Origin(xyz=(0.000, -0.003, 0.015)),
        material=tray_metal,
        name="tray_front",
    )
    crumb_tray.visual(
        Box((0.040, 0.008, 0.012)),
        origin=Origin(xyz=(0.000, -0.010, 0.020)),
        material=plastic_black,
        name="tray_pull",
    )
    crumb_tray.inertial = Inertial.from_geometry(
        Box((0.184, 0.122, 0.030)),
        mass=0.10,
        origin=Origin(xyz=(0.000, 0.050, 0.015)),
    )
    model.articulation(
        "housing_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=crumb_tray,
        origin=Origin(xyz=(0.000, -0.090, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.065,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    housing = object_model.get_part("housing")
    carriage = object_model.get_part("bread_carriage")
    lever = object_model.get_part("carriage_lever")
    upper_button = object_model.get_part("upper_button")
    lower_button = object_model.get_part("lower_button")
    timer_dial = object_model.get_part("timer_dial")
    crumb_tray = object_model.get_part("crumb_tray")
    lift_joint = object_model.get_articulation("housing_to_bread_carriage")
    upper_button_joint = object_model.get_articulation("housing_to_upper_button")
    lower_button_joint = object_model.get_articulation("housing_to_lower_button")
    dial_joint = object_model.get_articulation("housing_to_timer_dial")
    tray_joint = object_model.get_articulation("housing_to_crumb_tray")

    ctx.expect_overlap(
        carriage,
        housing,
        axes="xy",
        min_overlap=0.10,
        elem_a="carriage_floor",
        elem_b="base_plate",
        name="carriage stays centered within toaster footprint",
    )

    rest_carriage_z = ctx.part_world_position(carriage)
    rest_lever_z = ctx.part_world_position(lever)
    with ctx.pose({lift_joint: lift_joint.motion_limits.upper}):
        raised_carriage_z = ctx.part_world_position(carriage)
        raised_lever_z = ctx.part_world_position(lever)

    ctx.check(
        "bread carriage rises upward",
        rest_carriage_z is not None
        and raised_carriage_z is not None
        and raised_carriage_z[2] > rest_carriage_z[2] + 0.035,
        details=f"rest={rest_carriage_z}, raised={raised_carriage_z}",
    )
    ctx.check(
        "lever follows carriage upward travel",
        rest_lever_z is not None
        and raised_lever_z is not None
        and raised_lever_z[2] > rest_lever_z[2] + 0.035,
        details=f"rest={rest_lever_z}, raised={raised_lever_z}",
    )

    rest_upper_button = ctx.part_world_position(upper_button)
    with ctx.pose({upper_button_joint: upper_button_joint.motion_limits.upper}):
        pressed_upper_button = ctx.part_world_position(upper_button)
    ctx.check(
        "upper button presses into front panel",
        rest_upper_button is not None
        and pressed_upper_button is not None
        and pressed_upper_button[1] > rest_upper_button[1] + 0.004,
        details=f"rest={rest_upper_button}, pressed={pressed_upper_button}",
    )

    rest_lower_button = ctx.part_world_position(lower_button)
    with ctx.pose({lower_button_joint: lower_button_joint.motion_limits.upper}):
        pressed_lower_button = ctx.part_world_position(lower_button)
    ctx.check(
        "lower button presses into front panel",
        rest_lower_button is not None
        and pressed_lower_button is not None
        and pressed_lower_button[1] > rest_lower_button[1] + 0.004,
        details=f"rest={rest_lower_button}, pressed={pressed_lower_button}",
    )

    rest_tray = ctx.part_world_position(crumb_tray)
    with ctx.pose({tray_joint: tray_joint.motion_limits.upper}):
        extended_tray = ctx.part_world_position(crumb_tray)
    ctx.check(
        "crumb tray slides outward",
        rest_tray is not None
        and extended_tray is not None
        and extended_tray[1] < rest_tray[1] - 0.050,
        details=f"rest={rest_tray}, extended={extended_tray}",
    )

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_pointer = aabb_center(ctx.part_element_world_aabb(timer_dial, elem="dial_pointer"))
    with ctx.pose({dial_joint: dial_joint.motion_limits.upper}):
        turned_pointer = aabb_center(ctx.part_element_world_aabb(timer_dial, elem="dial_pointer"))
    ctx.check(
        "timer dial pointer sweeps through an arc",
        rest_pointer is not None
        and turned_pointer is not None
        and (
            abs(turned_pointer[0] - rest_pointer[0]) > 0.006
            or abs(turned_pointer[2] - rest_pointer[2]) > 0.006
        ),
        details=f"rest={rest_pointer}, turned={turned_pointer}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
