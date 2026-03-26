from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _write_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _ring_shell(
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    *,
    upper_outer: float | None = None,
    upper_inner: float | None = None,
):
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, z0),
            (outer_radius if upper_outer is None else upper_outer, z1),
        ],
        [
            (inner_radius, z0),
            (inner_radius if upper_inner is None else upper_inner, z1),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )


def _build_outer_tube_mesh():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.0166, 0.000),
            (0.0166, 0.220),
            (0.0162, 0.295),
        ],
        [
            (0.0146, 0.000),
            (0.0146, 0.220),
            (0.0142, 0.295),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    routing_slot = BoxGeometry((0.011, 0.022, 0.080)).translate(0.0, -0.0182, 0.056)
    return boolean_difference(shell, routing_slot)


def _build_lower_sleeve_mesh():
    return _ring_shell(
        0.0188,
        0.01655,
        -0.056,
        0.056,
        upper_outer=0.0182,
        upper_inner=0.01645,
    )


def _build_guide_collar_mesh():
    collar = _ring_shell(
        0.0186,
        0.0167,
        -0.016,
        0.016,
        upper_outer=0.0180,
        upper_inner=0.0165,
    )
    stop_washer = _ring_shell(0.0167, 0.0129, -0.019, -0.013)
    collar.merge(stop_washer)
    return collar


def _build_cartridge_collar_mesh():
    return _ring_shell(0.0175, 0.0128, 0.000, 0.028)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dropper_seatpost", assets=ASSETS)

    outer_black = model.material("outer_black", rgba=(0.09, 0.09, 0.10, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.16, 0.17, 0.18, 1.0))
    stanchion_black = model.material("stanchion_black", rgba=(0.20, 0.21, 0.23, 1.0))
    hardware = model.material("hardware", rgba=(0.63, 0.65, 0.68, 1.0))

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        _write_mesh(_build_outer_tube_mesh(), "dropper_outer_tube.obj"),
        material=outer_black,
        name="main_shell",
    )
    outer_tube.visual(
        Box((0.0086, 0.0018, 0.078)),
        origin=Origin(xyz=(0.000, -0.0137, 0.056)),
        material=anodized_black,
        name="routing_channel_floor",
    )
    outer_tube.visual(
        Box((0.0016, 0.0020, 0.078)),
        origin=Origin(xyz=(-0.0043, -0.0136, 0.056)),
        material=anodized_black,
        name="routing_channel_left_wall",
    )
    outer_tube.visual(
        Box((0.0016, 0.0020, 0.078)),
        origin=Origin(xyz=(0.0043, -0.0136, 0.056)),
        material=anodized_black,
        name="routing_channel_right_wall",
    )
    outer_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.295),
        mass=0.72,
        origin=Origin(xyz=(0.000, 0.000, 0.1475)),
    )

    lower_sleeve = model.part("lower_clamp_sleeve")
    lower_sleeve.visual(
        _write_mesh(_build_lower_sleeve_mesh(), "dropper_lower_sleeve.obj"),
        material=anodized_black,
        name="sleeve_shell",
    )
    lower_sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.112),
        mass=0.16,
        origin=Origin(),
    )

    actuator_boss = model.part("actuator_boss")
    actuator_boss.visual(
        Box((0.012, 0.010, 0.016)),
        origin=Origin(xyz=(0.000, 0.003, 0.000)),
        material=anodized_black,
        name="boss_seat",
    )
    actuator_boss.visual(
        Box((0.020, 0.018, 0.026)),
        origin=Origin(xyz=(0.000, -0.004, 0.000)),
        material=anodized_black,
        name="boss_base",
    )
    actuator_boss.visual(
        Cylinder(radius=0.0105, length=0.024),
        origin=Origin(xyz=(0.000, -0.014, 0.002), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=anodized_black,
        name="pivot_housing",
    )
    actuator_boss.visual(
        Cylinder(radius=0.0030, length=0.028),
        origin=Origin(xyz=(0.000, -0.014, 0.002), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=hardware,
        name="pivot_pin",
    )
    actuator_boss.visual(
        Cylinder(radius=0.0040, length=0.016),
        origin=Origin(xyz=(0.000, -0.015, -0.012), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=hardware,
        name="routing_ferrule",
    )
    actuator_boss.inertial = Inertial.from_geometry(
        Box((0.024, 0.034, 0.032)),
        mass=0.10,
        origin=Origin(xyz=(0.000, -0.004, 0.000)),
    )

    guide_collar = model.part("guide_collar")
    guide_collar.visual(
        _write_mesh(_build_guide_collar_mesh(), "dropper_guide_collar.obj"),
        material=anodized_black,
        name="guide_shell",
    )
    guide_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.019, length=0.038),
        mass=0.09,
        origin=Origin(),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0125, length=0.360),
        origin=Origin(xyz=(0.000, 0.000, 0.180)),
        material=stanchion_black,
        name="stanchion",
    )
    inner_post.visual(
        Cylinder(radius=0.0136, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.085)),
        material=anodized_black,
        name="travel_stop_ring",
    )
    inner_post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.360),
        mass=0.34,
        origin=Origin(xyz=(0.000, 0.000, 0.180)),
    )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        _write_mesh(_build_cartridge_collar_mesh(), "dropper_cartridge_collar.obj"),
        material=anodized_black,
        name="cartridge_collar",
    )
    saddle_clamp.visual(
        Box((0.016, 0.020, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.033)),
        material=anodized_black,
        name="support_neck",
    )
    saddle_clamp.visual(
        Box((0.050, 0.018, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.046)),
        material=anodized_black,
        name="lower_rail_cradle",
    )
    saddle_clamp.visual(
        Box((0.040, 0.014, 0.005)),
        origin=Origin(xyz=(0.000, 0.000, 0.060)),
        material=anodized_black,
        name="upper_rail_cap",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0026, length=0.022),
        origin=Origin(xyz=(0.000, -0.008, 0.051), rpy=(0.000, 0.000, 0.000)),
        material=hardware,
        name="front_clamp_bolt",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0026, length=0.022),
        origin=Origin(xyz=(0.000, 0.008, 0.051), rpy=(0.000, 0.000, 0.000)),
        material=hardware,
        name="rear_clamp_bolt",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0028, length=0.014),
        origin=Origin(xyz=(-0.016, 0.000, 0.014), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=hardware,
        name="left_pinch_bolt",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0028, length=0.014),
        origin=Origin(xyz=(0.016, 0.000, 0.014), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=hardware,
        name="right_pinch_bolt",
    )
    saddle_clamp.inertial = Inertial.from_geometry(
        Box((0.052, 0.026, 0.066)),
        mass=0.12,
        origin=Origin(xyz=(0.000, 0.000, 0.033)),
    )

    model.articulation(
        "outer_to_lower_sleeve",
        ArticulationType.FIXED,
        parent=outer_tube,
        child=lower_sleeve,
        origin=Origin(xyz=(0.000, 0.000, 0.116)),
    )
    model.articulation(
        "outer_to_actuator_boss",
        ArticulationType.FIXED,
        parent=outer_tube,
        child=actuator_boss,
        origin=Origin(xyz=(0.000, -0.021, 0.043)),
    )
    model.articulation(
        "outer_to_guide_collar",
        ArticulationType.FIXED,
        parent=outer_tube,
        child=guide_collar,
        origin=Origin(xyz=(0.000, 0.000, 0.304)),
    )
    model.articulation(
        "outer_to_inner_post",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.000, 0.000, 0.070)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.25,
            lower=0.000,
            upper=0.125,
        ),
    )
    model.articulation(
        "inner_to_saddle_clamp",
        ArticulationType.FIXED,
        parent=inner_post,
        child=saddle_clamp,
        origin=Origin(xyz=(0.000, 0.000, 0.348)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    outer_tube = object_model.get_part("outer_tube")
    lower_sleeve = object_model.get_part("lower_clamp_sleeve")
    actuator_boss = object_model.get_part("actuator_boss")
    guide_collar = object_model.get_part("guide_collar")
    inner_post = object_model.get_part("inner_post")
    saddle_clamp = object_model.get_part("saddle_clamp")
    dropper_travel = object_model.get_articulation("outer_to_inner_post")

    main_shell = outer_tube.get_visual("main_shell")
    routing_floor = outer_tube.get_visual("routing_channel_floor")
    sleeve_shell = lower_sleeve.get_visual("sleeve_shell")
    boss_seat = actuator_boss.get_visual("boss_seat")
    pivot_housing = actuator_boss.get_visual("pivot_housing")
    guide_shell = guide_collar.get_visual("guide_shell")
    stanchion = inner_post.get_visual("stanchion")
    travel_stop_ring = inner_post.get_visual("travel_stop_ring")
    cartridge_collar = saddle_clamp.get_visual("cartridge_collar")
    front_clamp_bolt = saddle_clamp.get_visual("front_clamp_bolt")
    rear_clamp_bolt = saddle_clamp.get_visual("rear_clamp_bolt")
    left_pinch_bolt = saddle_clamp.get_visual("left_pinch_bolt")
    right_pinch_bolt = saddle_clamp.get_visual("right_pinch_bolt")
    lower_rail_cradle = saddle_clamp.get_visual("lower_rail_cradle")
    upper_rail_cap = saddle_clamp.get_visual("upper_rail_cap")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.017)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(
        outer_tube,
        lower_sleeve,
        axes="xy",
        inner_elem=main_shell,
        outer_elem=sleeve_shell,
        name="lower_clamp_sleeve_wraps_outer_body",
    )
    ctx.expect_contact(
        actuator_boss,
        outer_tube,
        elem_a=boss_seat,
        elem_b=main_shell,
        name="actuator_boss_is_housed_on_outer_tube",
    )
    ctx.expect_contact(
        actuator_boss,
        outer_tube,
        elem_a=boss_seat,
        elem_b=routing_floor,
        name="boss_feeds_internal_routing_channel",
    )
    ctx.expect_gap(
        lower_sleeve,
        actuator_boss,
        axis="z",
        min_gap=0.006,
        positive_elem=sleeve_shell,
        negative_elem=boss_seat,
        name="actuator_boss_sits_below_lower_clamp_sleeve",
    )
    ctx.expect_within(
        inner_post,
        guide_collar,
        axes="xy",
        inner_elem=stanchion,
        outer_elem=guide_shell,
        name="inner_post_runs_through_guide_collar",
    )
    ctx.expect_gap(
        guide_collar,
        lower_sleeve,
        axis="z",
        min_gap=0.110,
        positive_elem=guide_shell,
        negative_elem=sleeve_shell,
        name="guide_collar_sits_above_lower_clamp_sleeve",
    )
    ctx.expect_overlap(
        inner_post,
        outer_tube,
        axes="xy",
        min_overlap=0.0002,
        name="inner_post_stays_coaxial_with_outer_tube",
    )
    ctx.expect_gap(
        guide_collar,
        inner_post,
        axis="z",
        min_gap=0.115,
        positive_elem=guide_shell,
        negative_elem=travel_stop_ring,
        name="travel_stop_ring_sits_below_collar_at_rest",
    )
    ctx.expect_within(
        inner_post,
        saddle_clamp,
        axes="xy",
        inner_elem=stanchion,
        outer_elem=cartridge_collar,
        name="cartridge_collar_wraps_inner_post_tip",
    )
    ctx.expect_gap(
        saddle_clamp,
        guide_collar,
        axis="z",
        min_gap=0.060,
        positive_elem=cartridge_collar,
        negative_elem=guide_shell,
        name="saddle_clamp_stays_above_guide_collar",
    )
    ctx.expect_gap(
        saddle_clamp,
        saddle_clamp,
        axis="z",
        min_gap=0.006,
        positive_elem=upper_rail_cap,
        negative_elem=lower_rail_cradle,
        name="rail_cradle_has_visible_two_piece_gap",
    )
    ctx.expect_overlap(
        saddle_clamp,
        saddle_clamp,
        axes="xy",
        min_overlap=0.0001,
        elem_a=front_clamp_bolt,
        elem_b=lower_rail_cradle,
        name="front_clamp_bolt_aligns_with_rail_cradle",
    )
    ctx.expect_overlap(
        saddle_clamp,
        saddle_clamp,
        axes="xy",
        min_overlap=0.0001,
        elem_a=rear_clamp_bolt,
        elem_b=lower_rail_cradle,
        name="rear_clamp_bolt_aligns_with_rail_cradle",
    )
    ctx.expect_overlap(
        saddle_clamp,
        saddle_clamp,
        axes="yz",
        min_overlap=0.00005,
        elem_a=left_pinch_bolt,
        elem_b=cartridge_collar,
        name="left_side_pinch_bolt_seats_in_cartridge_collar",
    )
    ctx.expect_overlap(
        saddle_clamp,
        saddle_clamp,
        axes="yz",
        min_overlap=0.00005,
        elem_a=right_pinch_bolt,
        elem_b=cartridge_collar,
        name="right_side_pinch_bolt_seats_in_cartridge_collar",
    )
    with ctx.pose({dropper_travel: 0.125}):
        ctx.expect_contact(
            guide_collar,
            inner_post,
            elem_a=guide_shell,
            elem_b=travel_stop_ring,
            name="travel_stop_ring_tops_out_under_guide_collar",
        )
        ctx.expect_gap(
            saddle_clamp,
            guide_collar,
            axis="z",
            min_gap=0.180,
            positive_elem=cartridge_collar,
            negative_elem=guide_shell,
            name="extended_post_lifts_saddle_clamp_clear_of_outer_body",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
