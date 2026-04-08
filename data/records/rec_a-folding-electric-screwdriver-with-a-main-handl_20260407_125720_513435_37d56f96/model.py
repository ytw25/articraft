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
    mesh_from_geometry,
    place_on_surface,
    rounded_rect_profile,
    section_loft,
)


def _xy_section(
    *,
    width_x: float,
    depth_y: float,
    radius: float,
    z: float,
    x_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_center, y + y_center, z)
        for x, y in rounded_rect_profile(width_x, depth_y, radius)
    ]


def _yz_section(
    *,
    width_y: float,
    height_z: float,
    radius: float,
    x: float,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_center, z + z_center)
        for y, z in rounded_rect_profile(width_y, height_z, radius)
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_electric_screwdriver")

    body_dark = model.material("body_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    body_mid = model.material("body_mid", rgba=(0.28, 0.30, 0.33, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_red = model.material("accent_red", rgba=(0.83, 0.24, 0.16, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.36, 0.39, 0.42, 1.0))

    handle_body = model.part("handle_body")
    handle_shell = section_loft(
        [
            _xy_section(width_x=0.044, depth_y=0.036, radius=0.010, z=-0.020, x_center=-0.015),
            _xy_section(width_x=0.050, depth_y=0.040, radius=0.012, z=-0.085, x_center=-0.020),
            _xy_section(width_x=0.055, depth_y=0.042, radius=0.012, z=-0.150, x_center=-0.020),
            _xy_section(width_x=0.048, depth_y=0.038, radius=0.010, z=-0.215, x_center=-0.015),
        ]
    )
    handle_body.visual(
        _mesh("screwdriver_handle_shell", handle_shell),
        material=body_dark,
        name="handle_shell",
    )
    handle_body.visual(
        Box((0.016, 0.058, 0.030)),
        origin=Origin(xyz=(-0.032, 0.0, -0.006)),
        material=body_mid,
        name="pivot_bridge",
    )
    handle_body.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=body_mid,
        name="left_hinge_cheek",
    )
    handle_body.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, 0.018, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=body_mid,
        name="right_hinge_cheek",
    )
    handle_body.visual(
        Box((0.028, 0.032, 0.022)),
        origin=Origin(xyz=(-0.014, 0.0, -0.225)),
        material=grip_black,
        name="battery_cap",
    )
    handle_body.inertial = Inertial.from_geometry(
        Box((0.080, 0.070, 0.260)),
        mass=0.58,
        origin=Origin(xyz=(-0.015, 0.0, -0.110)),
    )

    driver_section = model.part("driver_section")
    driver_shell = section_loft(
        [
            _yz_section(width_y=0.028, height_z=0.030, radius=0.007, x=0.000),
            _yz_section(width_y=0.030, height_z=0.030, radius=0.007, x=0.050),
            _yz_section(width_y=0.028, height_z=0.026, radius=0.006, x=0.105),
            _yz_section(width_y=0.020, height_z=0.020, radius=0.004, x=0.150),
        ]
    )
    driver_section.visual(
        _mesh("screwdriver_driver_shell", driver_shell),
        material=body_mid,
        name="driver_shell",
    )
    driver_section.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=body_mid,
        name="hinge_knuckle",
    )
    driver_section.visual(
        Cylinder(radius=0.016, length=0.058),
        origin=Origin(xyz=(0.178, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="nose_collar",
    )
    driver_section.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(xyz=(0.125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="rubber_ring",
    )
    driver_section.inertial = Inertial.from_geometry(
        Box((0.230, 0.045, 0.045)),
        mass=0.25,
        origin=Origin(xyz=(0.110, 0.0, 0.0)),
    )

    chuck = model.part("chuck")
    chuck.visual(
        Cylinder(radius=0.0085, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="chuck_collar",
    )
    chuck.visual(
        Cylinder(radius=0.0065, length=0.016),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="bit_holder",
    )
    chuck.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="socket_tip",
    )
    chuck.inertial = Inertial.from_geometry(
        Cylinder(radius=0.009, length=0.048),
        mass=0.02,
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.012, 0.018, 0.024)),
        origin=Origin(xyz=(0.0058, 0.0, 0.0)),
        material=accent_red,
        name="trigger_pad",
    )
    trigger.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.0058, 0.0, 0.010), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="trigger_crown",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.014, 0.020, 0.026)),
        mass=0.012,
        origin=Origin(xyz=(0.0068, 0.0, 0.004)),
    )

    mode_button = model.part("mode_button")
    mode_button.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=accent_red,
        name="mode_cap",
    )
    mode_button.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_steel,
        name="mode_plunger",
    )
    mode_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.006, length=0.004),
        mass=0.004,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    model.articulation(
        "handle_to_driver",
        ArticulationType.REVOLUTE,
        parent=handle_body,
        child=driver_section,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "driver_to_chuck",
        ArticulationType.CONTINUOUS,
        parent=driver_section,
        child=chuck,
        origin=Origin(xyz=(0.206, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=18.0),
    )
    model.articulation(
        "handle_to_trigger",
        ArticulationType.PRISMATIC,
        parent=handle_body,
        child=trigger,
        origin=place_on_surface(
            trigger,
            handle_body,
            point_hint=(0.014, 0.0, -0.112),
            child_axis="+x",
            clearance=-0.0002,
            prefer_collisions=False,
            child_prefer_collisions=False,
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.12,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "handle_to_mode_button",
        ArticulationType.PRISMATIC,
        parent=handle_body,
        child=mode_button,
        origin=place_on_surface(
            mode_button,
            handle_body,
            point_hint=(-0.028, 0.0, 0.012),
            child_axis="+z",
            prefer_collisions=False,
            child_prefer_collisions=False,
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.10,
            lower=0.0,
            upper=0.0025,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle_body = object_model.get_part("handle_body")
    driver_section = object_model.get_part("driver_section")
    chuck = object_model.get_part("chuck")
    trigger = object_model.get_part("trigger")
    mode_button = object_model.get_part("mode_button")

    fold_joint = object_model.get_articulation("handle_to_driver")
    chuck_joint = object_model.get_articulation("driver_to_chuck")
    trigger_joint = object_model.get_articulation("handle_to_trigger")
    button_joint = object_model.get_articulation("handle_to_mode_button")

    ctx.expect_overlap(
        chuck,
        driver_section,
        axes="yz",
        min_overlap=0.012,
        name="chuck remains coaxial with the driver nose",
    )
    ctx.expect_overlap(
        trigger,
        handle_body,
        axes="yz",
        min_overlap=0.016,
        name="trigger sits within the grip face footprint",
    )
    ctx.expect_overlap(
        mode_button,
        handle_body,
        axes="xy",
        min_overlap=0.010,
        name="mode button sits on the pivot housing footprint",
    )

    fold_upper = fold_joint.motion_limits.upper if fold_joint.motion_limits is not None else None
    trigger_upper = trigger_joint.motion_limits.upper if trigger_joint.motion_limits is not None else None
    button_upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None

    rest_chuck_pos = ctx.part_world_position(chuck)
    folded_chuck_pos = None
    if fold_upper is not None:
        with ctx.pose({fold_joint: fold_upper}):
            folded_chuck_pos = ctx.part_world_position(chuck)
    ctx.check(
        "front section swings toward a straight inline configuration",
        rest_chuck_pos is not None
        and folded_chuck_pos is not None
        and folded_chuck_pos[2] < rest_chuck_pos[2] - 0.18
        and folded_chuck_pos[0] < rest_chuck_pos[0] - 0.15,
        details=f"rest={rest_chuck_pos}, folded={folded_chuck_pos}",
    )

    rest_trigger_pos = ctx.part_world_position(trigger)
    pressed_trigger_pos = None
    if trigger_upper is not None:
        with ctx.pose({trigger_joint: trigger_upper}):
            pressed_trigger_pos = ctx.part_world_position(trigger)
    ctx.check(
        "trigger presses inward along its short guide",
        rest_trigger_pos is not None
        and pressed_trigger_pos is not None
        and pressed_trigger_pos[0] < rest_trigger_pos[0] - 0.0045,
        details=f"rest={rest_trigger_pos}, pressed={pressed_trigger_pos}",
    )

    rest_button_pos = ctx.part_world_position(mode_button)
    pressed_button_pos = None
    if button_upper is not None:
        with ctx.pose({button_joint: button_upper}):
            pressed_button_pos = ctx.part_world_position(mode_button)
    ctx.check(
        "mode button plunges down toward the pivot housing",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.0018,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    ctx.check(
        "articulation types and axes match the screwdriver mechanisms",
        fold_joint.articulation_type == ArticulationType.REVOLUTE
        and chuck_joint.articulation_type == ArticulationType.CONTINUOUS
        and trigger_joint.articulation_type == ArticulationType.PRISMATIC
        and button_joint.articulation_type == ArticulationType.PRISMATIC
        and math.isclose(fold_joint.axis[1], 1.0, abs_tol=1e-9)
        and math.isclose(chuck_joint.axis[0], 1.0, abs_tol=1e-9)
        and math.isclose(trigger_joint.axis[0], -1.0, abs_tol=1e-9)
        and math.isclose(button_joint.axis[2], -1.0, abs_tol=1e-9),
        details=(
            f"fold={fold_joint.axis}, chuck={chuck_joint.axis}, "
            f"trigger={trigger_joint.axis}, button={button_joint.axis}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
