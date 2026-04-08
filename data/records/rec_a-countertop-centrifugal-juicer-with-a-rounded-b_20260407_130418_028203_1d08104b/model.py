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
    rounded_rect_profile,
    section_loft,
)


def _rect_loop(
    width: float,
    depth: float,
    corner_radius: float,
    z: float,
    *,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, corner_radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_centrifugal_juicer")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.93, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.86, 0.90, 0.93, 0.34))
    basket_metal = model.material("basket_metal", rgba=(0.84, 0.85, 0.86, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base_housing")

    body_mesh = mesh_from_geometry(
        section_loft(
            [
                _rect_loop(0.250, 0.205, 0.060, 0.000),
                _rect_loop(0.290, 0.225, 0.070, 0.038, y_shift=0.003),
                _rect_loop(0.278, 0.212, 0.067, 0.110, y_shift=0.006),
                _rect_loop(0.228, 0.176, 0.052, 0.152, y_shift=0.004),
            ]
        ),
        "juicer_body_shell",
    )
    base.visual(body_mesh, material=body_white, name="body_shell")

    chamber_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.090, 0.000), (0.100, 0.024), (0.103, 0.055)],
            inner_profile=[(0.080, 0.006), (0.089, 0.024), (0.093, 0.051)],
            segments=56,
        ),
        "juicer_chamber_shell",
    )
    base.visual(
        chamber_shell,
        origin=Origin(xyz=(0.0, 0.010, 0.150)),
        material=charcoal,
        name="upper_chamber_shell",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.11318, 0.084), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="selector_bezel",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.010, 0.148), rpy=(0.0, 0.0, 0.0)),
        material=charcoal,
        name="drive_coupler",
    )
    base.visual(
        Box((0.080, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.099, 0.191)),
        material=charcoal,
        name="rear_hinge_bridge",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.290, 0.225, 0.205)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.003, 0.1025)),
    )

    lid = model.part("lid")
    lid_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.046, 0.038), (0.079, 0.051), (0.116, 0.019), (0.121, 0.004)],
            inner_profile=[(0.041, 0.036), (0.074, 0.045), (0.110, 0.016), (0.115, 0.002)],
            segments=60,
        ),
        "juicer_lid_shell",
    )
    lid.visual(
        lid_shell,
        origin=Origin(xyz=(0.0, 0.105, 0.0)),
        material=clear_smoke,
        name="lid_shell",
    )

    feed_chute_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.047, 0.000), (0.047, 0.102)],
            inner_profile=[(0.040, 0.000), (0.040, 0.102)],
            segments=48,
        ),
        "juicer_feed_chute_shell",
    )
    lid.visual(
        feed_chute_shell,
        origin=Origin(xyz=(0.0, 0.105, 0.034)),
        material=clear_smoke,
        name="feed_chute_shell",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.062),
        origin=Origin(xyz=(0.0, -0.010, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.245, 0.185, 0.145)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.095, 0.050)),
    )

    pusher = model.part("feed_pusher")
    pusher.visual(
        Cylinder(radius=0.034, length=0.152),
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
        material=knob_black,
        name="pusher_stem",
    )
    pusher.visual(
        Cylinder(radius=0.046, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=knob_black,
        name="pusher_cap",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.039, length=0.172),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
    )

    speed_selector = model.part("speed_selector")
    speed_selector.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="selector_shaft",
    )
    speed_selector.visual(
        Cylinder(radius=0.031, length=0.020),
        origin=Origin(xyz=(0.0, 0.024, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="selector_knob",
    )
    speed_selector.visual(
        Box((0.007, 0.004, 0.015)),
        origin=Origin(xyz=(0.018, 0.034, 0.0)),
        material=body_white,
        name="selector_indicator",
    )
    speed_selector.inertial = Inertial.from_geometry(
        Box((0.065, 0.040, 0.065)),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
    )

    basket = model.part("filter_basket")
    bottom_ring = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.077, 0.000), (0.077, 0.006)],
            inner_profile=[(0.070, 0.000), (0.070, 0.006)],
            segments=48,
        ),
        "juicer_basket_bottom_ring",
    )
    top_ring = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile=[(0.077, 0.000), (0.077, 0.006)],
            inner_profile=[(0.070, 0.000), (0.070, 0.006)],
            segments=48,
        ),
        "juicer_basket_top_ring",
    )
    basket.visual(bottom_ring, material=basket_metal, name="basket_bottom_ring")
    basket.visual(
        top_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=basket_metal,
        name="basket_top_ring",
    )
    basket.visual(
        Cylinder(radius=0.012, length=0.043),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=charcoal,
        name="basket_spindle",
    )
    basket.visual(
        Cylinder(radius=0.050, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0405)),
        material=basket_metal,
        name="cutter_disk",
    )
    slat_count = 14
    slat_radius = 0.073
    for index in range(slat_count):
        angle = 2.0 * math.pi * index / slat_count
        basket.visual(
            Box((0.002, 0.013, 0.036)),
            origin=Origin(
                xyz=(slat_radius * math.cos(angle), slat_radius * math.sin(angle), 0.023),
                rpy=(0.0, 0.0, angle),
            ),
            material=basket_metal,
            name=f"basket_slat_{index}",
        )
    for angle in (0.0, math.pi / 2.0):
        basket.visual(
            Box((0.146, 0.004, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.004), rpy=(0.0, 0.0, angle)),
            material=basket_metal,
            name=f"basket_lower_spoke_{int(angle == 0.0)}",
        )
        basket.visual(
            Box((0.146, 0.004, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.039), rpy=(0.0, 0.0, angle)),
            material=basket_metal,
            name=f"basket_upper_spoke_{int(angle == 0.0)}",
        )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.077, length=0.043),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, -0.095, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )

    model.articulation(
        "lid_to_feed_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, 0.105, 0.136)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.14,
            lower=0.0,
            upper=0.075,
        ),
    )

    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(0.0, 0.11618, 0.084)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=3.5,
            lower=-2.35,
            upper=2.35,
        ),
    )

    model.articulation(
        "base_to_filter_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.010, 0.153)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=120.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_housing")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("feed_pusher")
    selector = object_model.get_part("speed_selector")
    basket = object_model.get_part("filter_basket")

    lid_joint = object_model.get_articulation("base_to_lid")
    pusher_joint = object_model.get_articulation("lid_to_feed_pusher")
    selector_joint = object_model.get_articulation("base_to_speed_selector")
    basket_joint = object_model.get_articulation("base_to_filter_basket")

    with ctx.pose({lid_joint: 0.0, pusher_joint: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="upper_chamber_shell",
            max_gap=0.012,
            max_penetration=0.0,
            name="closed lid sits on the upper chamber rim",
        )
        ctx.expect_overlap(
            basket,
            base,
            axes="xy",
            min_overlap=0.140,
            elem_b="upper_chamber_shell",
            name="filter basket stays centered inside the chamber footprint",
        )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    rest_pusher_pos = ctx.part_world_position(pusher)
    rest_indicator_aabb = ctx.part_element_world_aabb(selector, elem="selector_indicator")

    with ctx.pose({lid_joint: lid_joint.motion_limits.upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({pusher_joint: pusher_joint.motion_limits.upper}):
        raised_pusher_pos = ctx.part_world_position(pusher)

    with ctx.pose({selector_joint: 1.1}):
        turned_indicator_aabb = ctx.part_element_world_aabb(selector, elem="selector_indicator")

    ctx.check(
        "lid opens upward from the rear hinge",
        rest_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.060,
        details=f"rest={rest_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "feed pusher raises vertically through the chute",
        rest_pusher_pos is not None
        and raised_pusher_pos is not None
        and raised_pusher_pos[2] > rest_pusher_pos[2] + 0.060,
        details=f"rest={rest_pusher_pos}, raised={raised_pusher_pos}",
    )
    ctx.check(
        "speed selector indicator swings around its shaft",
        rest_indicator_aabb is not None
        and turned_indicator_aabb is not None
        and abs(turned_indicator_aabb[0][0] - rest_indicator_aabb[0][0]) > 0.006,
        details=f"rest={rest_indicator_aabb}, turned={turned_indicator_aabb}",
    )
    ctx.check(
        "basket articulation is a vertical continuous spin",
        basket_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in basket_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={basket_joint.articulation_type}, axis={basket_joint.axis}",
    )
    ctx.check(
        "selector articulation is a front-mounted rotary control",
        selector_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 6) for value in selector_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={selector_joint.articulation_type}, axis={selector_joint.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
