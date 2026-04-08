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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="straight_body_electric_screwdriver")

    housing_dark = model.material("housing_dark", rgba=(0.20, 0.22, 0.24, 1.0))
    housing_light = model.material("housing_light", rgba=(0.34, 0.37, 0.40, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.83, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    control_red = model.material("control_red", rgba=(0.74, 0.15, 0.11, 1.0))

    torque_ring_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.0180, -0.0060),
                (0.0195, -0.0035),
                (0.0195, 0.0035),
                (0.0180, 0.0060),
            ],
            inner_profile=[
                (0.0144, -0.0058),
                (0.0144, 0.0058),
            ],
            segments=56,
        ).rotate_y(math.pi / 2.0),
        "torque_ring",
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.019, length=0.150),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_dark,
        name="main_housing",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.020),
        origin=Origin(xyz=(-0.085, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="rear_cap",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_light,
        name="front_neck",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.090, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="nose_bushing",
    )
    body.visual(
        Box((0.055, 0.005, 0.005)),
        origin=Origin(xyz=(-0.004, 0.0075, 0.0175)),
        material=housing_light,
        name="top_rail_left",
    )
    body.visual(
        Box((0.055, 0.005, 0.005)),
        origin=Origin(xyz=(-0.004, -0.0075, 0.0175)),
        material=housing_light,
        name="top_rail_right",
    )
    body.visual(
        Box((0.055, 0.008, 0.0025)),
        origin=Origin(xyz=(-0.004, 0.0, 0.01775)),
        material=housing_dark,
        name="top_slot_bed",
    )
    body.visual(
        Box((0.016, 0.003, 0.014)),
        origin=Origin(xyz=(0.058, 0.0165, 0.0)),
        material=housing_light,
        name="switch_backer",
    )
    body.visual(
        Box((0.016, 0.006, 0.002)),
        origin=Origin(xyz=(0.058, 0.0205, 0.0045)),
        material=housing_light,
        name="switch_lip_upper",
    )
    body.visual(
        Box((0.016, 0.006, 0.002)),
        origin=Origin(xyz=(0.058, 0.0205, -0.0045)),
        material=housing_light,
        name="switch_lip_lower",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.190, 0.042, 0.042)),
        mass=0.55,
        origin=Origin(xyz=(-0.001, 0.0, 0.0)),
    )

    bit_holder = model.part("bit_holder")
    bit_holder.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="drive_shaft",
    )
    bit_holder.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="bit_socket",
    )
    bit_holder.visual(
        Cylinder(radius=0.0030, length=0.016),
        origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hex_stub",
    )
    bit_holder.inertial = Inertial.from_geometry(
        Box((0.045, 0.013, 0.013)),
        mass=0.06,
        origin=Origin(xyz=(0.0225, 0.0, 0.0)),
    )

    power_slider = model.part("power_slider")
    power_slider.visual(
        Box((0.010, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=black_rubber,
        name="slider_shoe",
    )
    power_slider.visual(
        Box((0.018, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=control_red,
        name="slider_tab",
    )
    power_slider.visual(
        Cylinder(radius=0.003, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_red,
        name="slider_grip",
    )
    power_slider.inertial = Inertial.from_geometry(
        Box((0.020, 0.012, 0.014)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    direction_switch = model.part("direction_switch")
    direction_switch.visual(
        Box((0.008, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=black_rubber,
        name="switch_carriage",
    )
    direction_switch.visual(
        Box((0.012, 0.006, 0.007)),
        origin=Origin(xyz=(0.0, 0.005, 0.0)),
        material=control_red,
        name="switch_paddle",
    )
    direction_switch.inertial = Inertial.from_geometry(
        Box((0.014, 0.008, 0.008)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.004, 0.0)),
    )

    torque_ring = model.part("torque_ring")
    torque_ring.visual(
        torque_ring_mesh,
        material=black_rubber,
        name="selector_ring_shell",
    )
    torque_ring.visual(
        Box((0.004, 0.007, 0.004)),
        origin=Origin(xyz=(0.0, 0.0165, 0.0)),
        material=control_red,
        name="selector_index_lug",
    )
    torque_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0195, length=0.012),
        mass=0.03,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_bit_holder",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=bit_holder,
        origin=Origin(xyz=(0.094, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )
    model.articulation(
        "body_to_power_slider",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_slider,
        origin=Origin(xyz=(-0.004, 0.0, 0.0190)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=-0.016, upper=0.016),
    )
    model.articulation(
        "body_to_direction_switch",
        ArticulationType.PRISMATIC,
        parent=body,
        child=direction_switch,
        origin=Origin(xyz=(0.058, 0.0195, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=0.04, lower=-0.0015, upper=0.0015),
    )
    model.articulation(
        "body_to_torque_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=torque_ring,
        origin=Origin(xyz=(0.0765, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    bit_holder = object_model.get_part("bit_holder")
    power_slider = object_model.get_part("power_slider")
    direction_switch = object_model.get_part("direction_switch")
    torque_ring = object_model.get_part("torque_ring")
    spin = object_model.get_articulation("body_to_bit_holder")
    slider_joint = object_model.get_articulation("body_to_power_slider")
    direction_joint = object_model.get_articulation("body_to_direction_switch")
    ring_joint = object_model.get_articulation("body_to_torque_ring")

    ctx.expect_gap(
        bit_holder,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="drive_shaft",
        negative_elem="nose_bushing",
        name="bit holder seats against the nose bushing",
    )
    ctx.expect_overlap(
        bit_holder,
        body,
        axes="yz",
        min_overlap=0.008,
        elem_a="drive_shaft",
        elem_b="nose_bushing",
        name="bit holder stays centered on the tool axis",
    )
    ctx.expect_gap(
        power_slider,
        body,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="slider_shoe",
        negative_elem="top_slot_bed",
        name="power slider rides just above the top slot bed",
    )
    ctx.expect_overlap(
        power_slider,
        body,
        axes="xy",
        min_overlap=0.006,
        elem_a="slider_shoe",
        elem_b="top_slot_bed",
        name="power slider stays captured over the top slot",
    )
    ctx.expect_gap(
        direction_switch,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.003,
        positive_elem="switch_carriage",
        negative_elem="switch_backer",
        name="direction switch sits on the side guide without sinking into the housing",
    )
    ctx.expect_overlap(
        torque_ring,
        body,
        axes="yz",
        min_overlap=0.024,
        elem_a="selector_ring_shell",
        elem_b="front_neck",
        name="torque ring remains concentric with the front neck",
    )

    rest_pos = ctx.part_world_position(bit_holder)
    with ctx.pose({spin: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(bit_holder)

    ctx.check(
        "bit holder rotates in place about the main axis",
        rest_pos is not None
        and spun_pos is not None
        and abs(rest_pos[0] - spun_pos[0]) < 1e-6
        and abs(rest_pos[1] - spun_pos[1]) < 1e-6
        and abs(rest_pos[2] - spun_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    slider_rest = ctx.part_world_position(power_slider)
    with ctx.pose({slider_joint: slider_joint.motion_limits.upper}):
        slider_forward = ctx.part_world_position(power_slider)
    ctx.check(
        "power slider moves forward along the housing",
        slider_rest is not None
        and slider_forward is not None
        and slider_forward[0] > slider_rest[0] + 0.010,
        details=f"rest={slider_rest}, forward={slider_forward}",
    )

    switch_rest = ctx.part_world_position(direction_switch)
    with ctx.pose({direction_joint: direction_joint.motion_limits.upper}):
        switch_outer = ctx.part_world_position(direction_switch)
    with ctx.pose({direction_joint: direction_joint.motion_limits.lower}):
        switch_inner = ctx.part_world_position(direction_switch)
    ctx.check(
        "direction switch slides transversely across the side guide",
        switch_rest is not None
        and switch_outer is not None
        and switch_inner is not None
        and switch_outer[1] > switch_rest[1]
        and switch_inner[1] < switch_rest[1],
        details=f"inner={switch_inner}, rest={switch_rest}, outer={switch_outer}",
    )

    ring_rest = ctx.part_world_position(torque_ring)
    with ctx.pose({ring_joint: math.pi / 3.0}):
        ring_rotated = ctx.part_world_position(torque_ring)
    ctx.check(
        "torque ring rotates in place around the nose",
        ring_rest is not None
        and ring_rotated is not None
        and abs(ring_rest[0] - ring_rotated[0]) < 1e-6
        and abs(ring_rest[1] - ring_rotated[1]) < 1e-6
        and abs(ring_rest[2] - ring_rotated[2]) < 1e-6,
        details=f"rest={ring_rest}, rotated={ring_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
