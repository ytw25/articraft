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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _translate_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cordless_screwdriver")

    body_green = model.material("body_green", rgba=(0.16, 0.47, 0.24, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.11, 0.11, 0.12, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    metal = model.material("metal", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.35, 0.37, 0.40, 1.0))

    body = model.part("body")

    outer_profile = [
        (-0.078, 0.118),
        (-0.075, 0.148),
        (-0.020, 0.160),
        (0.045, 0.156),
        (0.070, 0.145),
        (0.082, 0.131),
        (0.086, 0.114),
        (0.084, 0.098),
        (0.074, 0.086),
        (0.060, 0.078),
        (0.034, 0.068),
        (0.020, 0.040),
        (0.016, -0.010),
        (0.020, -0.084),
        (0.012, -0.112),
        (-0.016, -0.126),
        (-0.040, -0.128),
        (-0.060, -0.116),
        (-0.071, -0.082),
        (-0.074, -0.010),
        (-0.069, 0.054),
        (-0.072, 0.090),
    ]
    opening_profile = [
        (-0.034, 0.072),
        (-0.024, 0.094),
        (-0.006, 0.100),
        (0.010, 0.094),
        (0.011, 0.078),
        (0.009, 0.050),
        (0.010, 0.012),
        (0.003, -0.052),
        (-0.014, -0.070),
        (-0.026, -0.060),
        (-0.034, -0.022),
        (-0.038, 0.028),
    ]
    switch_slot = _translate_profile(
        rounded_rect_profile(0.030, 0.016, 0.003, corner_segments=4),
        dx=-0.004,
        dy=0.093,
    )
    body_geom = (
        ExtrudeWithHolesGeometry(
            outer_profile,
            [opening_profile, switch_slot],
            height=0.066,
            center=True,
        ).rotate_x(math.pi / 2.0)
    )
    body.visual(
        mesh_from_geometry(body_geom, "screwdriver_body_shell"),
        material=body_green,
        name="body_shell",
    )
    body.visual(
        Box((0.074, 0.056, 0.060)),
        origin=Origin(xyz=(-0.018, 0.0, 0.118)),
        material=dark_rubber,
        name="rear_overmold",
    )
    body.visual(
        Box((0.034, 0.060, 0.062)),
        origin=Origin(xyz=(-0.042, 0.0, -0.010)),
        material=dark_rubber,
        name="grip_overmold",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.036),
        origin=Origin(xyz=(0.093, 0.0, 0.106), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_collar",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.112, 0.0, 0.106), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="nose_spigot",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.202, 0.066, 0.290)),
        mass=1.15,
        origin=Origin(xyz=(0.020, 0.0, 0.015)),
    )

    torque_ring = model.part("torque_ring")
    ring_outer = [
        (0.033, -0.009),
        (0.035, -0.007),
        (0.036, -0.003),
        (0.036, 0.003),
        (0.035, 0.007),
        (0.033, 0.009),
    ]
    ring_inner = [
        (0.025, -0.009),
        (0.025, 0.009),
    ]
    ring_geom = LatheGeometry.from_shell_profiles(ring_outer, ring_inner, segments=64).rotate_y(
        math.pi / 2.0
    )
    torque_ring.visual(
        mesh_from_geometry(ring_geom, "screwdriver_torque_ring"),
        material=black_plastic,
        name="ring_shell",
    )
    torque_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.024),
        mass=0.08,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    chuck = model.part("chuck")
    chuck.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_sleeve",
    )
    chuck.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.034, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_collar",
    )
    chuck.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.053, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="bit_holder",
    )
    chuck.inertial = Inertial.from_geometry(
        Box((0.076, 0.032, 0.032)),
        mass=0.12,
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.016, 0.044, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black_plastic,
        name="guide_block",
    )
    trigger.visual(
        Box((0.022, 0.044, 0.052)),
        origin=Origin(xyz=(-0.004, 0.0, -0.026)),
        material=black_plastic,
        name="trigger_paddle",
    )
    trigger.visual(
        Cylinder(radius=0.015, length=0.044),
        origin=Origin(xyz=(-0.008, 0.0, -0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="trigger_curve",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.030, 0.044, 0.082)),
        mass=0.05,
        origin=Origin(xyz=(-0.006, 0.0, -0.032)),
    )

    direction_switch = model.part("direction_switch")
    direction_switch.visual(
        Box((0.008, 0.066, 0.004)),
        material=black_plastic,
        name="switch_bar",
    )
    direction_switch.visual(
        Box((0.008, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=black_plastic,
        name="left_stem",
    )
    direction_switch.visual(
        Box((0.016, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.047, 0.0)),
        material=black_plastic,
        name="left_paddle",
    )
    direction_switch.visual(
        Box((0.008, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.040, 0.0)),
        material=black_plastic,
        name="right_stem",
    )
    direction_switch.visual(
        Box((0.016, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.047, 0.0)),
        material=black_plastic,
        name="right_paddle",
    )
    direction_switch.inertial = Inertial.from_geometry(
        Box((0.016, 0.104, 0.010)),
        mass=0.03,
    )

    model.articulation(
        "body_to_torque_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=torque_ring,
        origin=Origin(xyz=(0.094, 0.0, 0.106)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.0),
    )
    model.articulation(
        "body_to_chuck",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=chuck,
        origin=Origin(xyz=(0.121, 0.0, 0.106)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.8, velocity=25.0),
    )
    model.articulation(
        "body_to_trigger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(-0.004, 0.0, 0.060)),
        axis=(-0.42, 0.0, -0.91),
        motion_limits=MotionLimits(effort=10.0, velocity=0.08, lower=0.0, upper=0.012),
    )
    model.articulation(
        "body_to_direction_switch",
        ArticulationType.PRISMATIC,
        parent=body,
        child=direction_switch,
        origin=Origin(xyz=(-0.004, 0.0, 0.093)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=-0.007, upper=0.007),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    chuck = object_model.get_part("chuck")
    torque_ring = object_model.get_part("torque_ring")
    trigger = object_model.get_part("trigger")
    direction_switch = object_model.get_part("direction_switch")

    trigger_joint = object_model.get_articulation("body_to_trigger")
    switch_joint = object_model.get_articulation("body_to_direction_switch")

    ctx.expect_origin_distance(
        torque_ring,
        chuck,
        axes="yz",
        max_dist=0.001,
        name="torque ring and chuck share the tool axis",
    )
    ctx.expect_origin_gap(
        direction_switch,
        trigger,
        axis="z",
        min_gap=0.02,
        name="forward-reverse switch stays above the trigger",
    )

    trigger_rest = ctx.part_world_position(trigger)
    with ctx.pose({trigger_joint: 0.012}):
        trigger_pressed = ctx.part_world_position(trigger)
    ctx.check(
        "trigger presses inward into the handle",
        trigger_rest is not None
        and trigger_pressed is not None
        and trigger_pressed[0] < trigger_rest[0] - 0.004
        and trigger_pressed[2] < trigger_rest[2] - 0.008,
        details=f"rest={trigger_rest}, pressed={trigger_pressed}",
    )

    with ctx.pose({switch_joint: -0.007}):
        switch_left = ctx.part_world_position(direction_switch)
    with ctx.pose({switch_joint: 0.007}):
        switch_right = ctx.part_world_position(direction_switch)
    ctx.check(
        "forward-reverse switch slides side to side",
        switch_left is not None
        and switch_right is not None
        and switch_right[1] > switch_left[1] + 0.012
        and abs(switch_right[0] - switch_left[0]) < 1e-6
        and abs(switch_right[2] - switch_left[2]) < 1e-6,
        details=f"left={switch_left}, right={switch_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
