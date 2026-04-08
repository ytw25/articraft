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
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    *,
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for z, y in rounded_rect_profile(height_z, width_y, radius)
    ]


def _build_upper_housing_mesh():
    return section_loft(
        [
            _yz_section(
                x=-0.135,
                width_y=0.086,
                height_z=0.072,
                radius=0.014,
                z_center=0.084,
            ),
            _yz_section(
                x=-0.070,
                width_y=0.114,
                height_z=0.088,
                radius=0.020,
                z_center=0.087,
            ),
            _yz_section(
                x=0.010,
                width_y=0.126,
                height_z=0.096,
                radius=0.023,
                z_center=0.088,
            ),
            _yz_section(
                x=0.082,
                width_y=0.118,
                height_z=0.090,
                radius=0.021,
                z_center=0.086,
            ),
            _yz_section(
                x=0.145,
                width_y=0.080,
                height_z=0.066,
                radius=0.015,
                z_center=0.078,
            ),
        ]
    )


def _build_trigger_guard_mesh():
    return tube_from_spline_points(
        [
            (0.032, 0.0, 0.024),
            (0.026, 0.0, 0.004),
            (0.014, 0.0, -0.026),
            (0.002, 0.0, -0.058),
            (-0.004, 0.0, -0.082),
        ],
        radius=0.0065,
        samples_per_segment=18,
        radial_segments=18,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_electric_screwdriver")

    housing_yellow = model.material(
        "housing_yellow",
        rgba=(0.86, 0.73, 0.16, 1.0),
    )
    dark_polymer = model.material(
        "dark_polymer",
        rgba=(0.16, 0.17, 0.18, 1.0),
    )
    black_rubber = model.material(
        "black_rubber",
        rgba=(0.08, 0.08, 0.09, 1.0),
    )
    steel = model.material(
        "steel",
        rgba=(0.72, 0.74, 0.77, 1.0),
    )
    gunmetal = model.material(
        "gunmetal",
        rgba=(0.33, 0.35, 0.38, 1.0),
    )

    body = model.part("body")
    body.visual(
        _save_mesh("screwdriver_upper_housing", _build_upper_housing_mesh()),
        material=housing_yellow,
        name="upper_housing",
    )
    body.visual(
        Box((0.030, 0.082, 0.060)),
        origin=Origin(xyz=(-0.148, 0.0, 0.080)),
        material=dark_polymer,
        name="rear_cap",
    )
    body.visual(
        Box((0.110, 0.020, 0.018)),
        origin=Origin(xyz=(-0.020, 0.043, 0.051)),
        material=housing_yellow,
        name="left_selector_cheek",
    )
    body.visual(
        Box((0.110, 0.020, 0.018)),
        origin=Origin(xyz=(-0.020, -0.043, 0.051)),
        material=housing_yellow,
        name="right_selector_cheek",
    )
    body.visual(
        Box((0.072, 0.020, 0.016)),
        origin=Origin(xyz=(-0.042, 0.043, 0.020)),
        material=dark_polymer,
        name="left_lower_selector_mount",
    )
    body.visual(
        Box((0.072, 0.020, 0.016)),
        origin=Origin(xyz=(-0.042, -0.043, 0.020)),
        material=dark_polymer,
        name="right_lower_selector_mount",
    )
    body.visual(
        Box((0.090, 0.070, 0.026)),
        origin=Origin(xyz=(-0.082, 0.0, 0.033)),
        material=dark_polymer,
        name="rear_lower_bridge",
    )
    body.visual(
        Box((0.100, 0.068, 0.028)),
        origin=Origin(xyz=(0.105, 0.0, 0.046)),
        material=dark_polymer,
        name="front_support_block",
    )
    body.visual(
        Box((0.080, 0.010, 0.010)),
        origin=Origin(xyz=(0.008, 0.017, 0.132)),
        material=dark_polymer,
        name="top_slot_left_rail",
    )
    body.visual(
        Box((0.080, 0.010, 0.010)),
        origin=Origin(xyz=(0.008, -0.017, 0.132)),
        material=dark_polymer,
        name="top_slot_right_rail",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.042),
        origin=Origin(
            xyz=(0.170, 0.0, 0.074),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=gunmetal,
        name="nose_collar",
    )

    grip_pitch = -0.30
    body.visual(
        Box((0.058, 0.014, 0.160)),
        origin=Origin(xyz=(-0.040, 0.022, -0.046), rpy=(0.0, grip_pitch, 0.0)),
        material=black_rubber,
        name="left_grip_panel",
    )
    body.visual(
        Box((0.058, 0.014, 0.160)),
        origin=Origin(xyz=(-0.040, -0.022, -0.046), rpy=(0.0, grip_pitch, 0.0)),
        material=black_rubber,
        name="right_grip_panel",
    )
    body.visual(
        Box((0.014, 0.052, 0.158)),
        origin=Origin(xyz=(-0.064, 0.0, -0.046), rpy=(0.0, grip_pitch, 0.0)),
        material=black_rubber,
        name="backstrap",
    )
    body.visual(
        Box((0.050, 0.050, 0.020)),
        origin=Origin(xyz=(-0.058, 0.0, -0.127), rpy=(0.0, grip_pitch, 0.0)),
        material=dark_polymer,
        name="grip_base",
    )
    body.visual(
        Box((0.016, 0.048, 0.044)),
        origin=Origin(xyz=(-0.010, 0.0, -0.088), rpy=(0.0, -0.18, 0.0)),
        material=dark_polymer,
        name="lower_front_spine",
    )
    body.visual(
        _save_mesh("screwdriver_trigger_guard", _build_trigger_guard_mesh()),
        material=dark_polymer,
        name="trigger_guard",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.360, 0.130, 0.280)),
        mass=2.2,
        origin=Origin(xyz=(0.010, 0.0, 0.005)),
    )

    chuck = model.part("chuck")
    chuck.visual(
        Cylinder(radius=0.017, length=0.036),
        origin=Origin(
            xyz=(0.018, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="chuck_body",
    )
    chuck.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(
            xyz=(0.018, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=gunmetal,
        name="quick_release_sleeve",
    )
    chuck.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(
            xyz=(0.043, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="bit_receiver",
    )
    chuck.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(
            xyz=(0.053, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=gunmetal,
        name="retainer_nose",
    )
    chuck.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.060),
        mass=0.18,
        origin=Origin(
            xyz=(0.028, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.018, 0.030, 0.046)),
        origin=Origin(xyz=(0.000, 0.0, -0.008)),
        material=dark_polymer,
        name="trigger_paddle",
    )
    trigger.visual(
        Box((0.012, 0.030, 0.022)),
        origin=Origin(xyz=(0.004, 0.0, 0.026)),
        material=dark_polymer,
        name="trigger_top_shoe",
    )
    trigger.visual(
        Box((0.016, 0.016, 0.020)),
        origin=Origin(xyz=(-0.010, 0.0, 0.024)),
        material=dark_polymer,
        name="trigger_guide",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.032, 0.042, 0.066)),
        mass=0.06,
        origin=Origin(xyz=(-0.002, 0.0, 0.004)),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Box((0.022, 0.024, 0.010)),
        origin=Origin(xyz=(0.000, 0.0, 0.009)),
        material=dark_polymer,
        name="selector_knob",
    )
    speed_selector.visual(
        Box((0.016, 0.014, 0.008)),
        origin=Origin(xyz=(0.000, 0.0, 0.005)),
        material=dark_polymer,
        name="selector_stem",
    )
    speed_selector.visual(
        Box((0.030, 0.012, 0.008)),
        origin=Origin(xyz=(0.000, 0.0, 0.002)),
        material=dark_polymer,
        name="selector_tongue",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.030, 0.024, 0.020)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    direction_selector = model.part("direction_selector")
    direction_selector.visual(
        Box((0.016, 0.058, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_polymer,
        name="crossbar",
    )
    direction_selector.visual(
        Box((0.022, 0.016, 0.014)),
        origin=Origin(xyz=(-0.002, 0.037, 0.0)),
        material=dark_polymer,
        name="left_paddle",
    )
    direction_selector.visual(
        Box((0.022, 0.016, 0.014)),
        origin=Origin(xyz=(-0.002, -0.037, 0.0)),
        material=dark_polymer,
        name="right_paddle",
    )
    direction_selector.inertial = Inertial.from_geometry(
        Box((0.022, 0.086, 0.016)),
        mass=0.035,
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_chuck",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=chuck,
        origin=Origin(xyz=(0.191, 0.0, 0.074)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=45.0),
    )
    model.articulation(
        "body_to_trigger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(-0.014, 0.0, -0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.16, lower=0.0, upper=0.013),
    )
    model.articulation(
        "body_to_speed_selector",
        ArticulationType.PRISMATIC,
        parent=body,
        child=speed_selector,
        origin=Origin(xyz=(-0.006, 0.0, 0.136)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.014),
    )
    model.articulation(
        "body_to_direction_selector",
        ArticulationType.PRISMATIC,
        parent=body,
        child=direction_selector,
        origin=Origin(xyz=(-0.004, 0.0, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=-0.004, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    chuck = object_model.get_part("chuck")
    trigger = object_model.get_part("trigger")
    speed_selector = object_model.get_part("speed_selector")
    direction_selector = object_model.get_part("direction_selector")

    chuck_spin = object_model.get_articulation("body_to_chuck")
    trigger_slide = object_model.get_articulation("body_to_trigger")
    speed_slide = object_model.get_articulation("body_to_speed_selector")
    direction_slide = object_model.get_articulation("body_to_direction_selector")

    ctx.expect_gap(
        chuck,
        body,
        axis="x",
        min_gap=0.0,
        max_gap=0.006,
        name="chuck seats at the nose",
    )
    ctx.expect_overlap(
        chuck,
        body,
        axes="yz",
        min_overlap=0.030,
        name="chuck stays aligned with the front gearbox axis",
    )

    trigger_rest = ctx.part_world_position(trigger)
    with ctx.pose({trigger_slide: trigger_slide.motion_limits.upper}):
        trigger_pressed = ctx.part_world_position(trigger)
    ctx.check(
        "trigger pulls rearward into the grip",
        trigger_rest is not None
        and trigger_pressed is not None
        and trigger_pressed[0] < trigger_rest[0] - 0.010,
        details=f"rest={trigger_rest}, pressed={trigger_pressed}",
    )

    speed_rest = ctx.part_world_position(speed_selector)
    with ctx.pose({speed_slide: speed_slide.motion_limits.upper}):
        speed_forward = ctx.part_world_position(speed_selector)
    ctx.check(
        "speed selector slides forward along the top slot",
        speed_rest is not None
        and speed_forward is not None
        and speed_forward[0] > speed_rest[0] + 0.010,
        details=f"rest={speed_rest}, forward={speed_forward}",
    )

    direction_rest = ctx.part_world_position(direction_selector)
    with ctx.pose({direction_slide: direction_slide.motion_limits.upper}):
        direction_shifted = ctx.part_world_position(direction_selector)
    ctx.check(
        "direction selector moves laterally through the housing",
        direction_rest is not None
        and direction_shifted is not None
        and direction_shifted[1] > direction_rest[1] + 0.0025,
        details=f"rest={direction_rest}, shifted={direction_shifted}",
    )

    with ctx.pose({chuck_spin: math.pi / 2.0}):
        ctx.expect_gap(
            chuck,
            body,
            axis="x",
            min_gap=0.0,
            max_gap=0.006,
            name="rotated chuck remains seated at the nose",
        )
        ctx.expect_overlap(
            chuck,
            body,
            axes="yz",
            min_overlap=0.020,
            name="rotated chuck remains on the output axis",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
