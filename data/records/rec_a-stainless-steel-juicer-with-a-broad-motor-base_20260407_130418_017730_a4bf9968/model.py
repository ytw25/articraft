from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
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


def _section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    dx: float = 0.0,
    dy: float = 0.0,
):
    return tuple((x + dx, y + dy, z) for x, y in rounded_rect_profile(width, depth, radius))


def _build_base_shell():
    geom = section_loft(
        [
            _section(0.300, 0.240, 0.048, 0.000),
            _section(0.292, 0.232, 0.045, 0.050, dy=0.004),
            _section(0.254, 0.204, 0.038, 0.118, dy=0.010),
            _section(0.212, 0.176, 0.032, 0.166, dy=0.015),
        ]
    )
    geom.merge(CylinderGeometry(radius=0.112, height=0.010, radial_segments=56).translate(0.0, 0.0, 0.171))
    geom.merge(BoxGeometry((0.150, 0.018, 0.060)).translate(0.0, -0.112, 0.078))
    return geom


def _build_chamber_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.056, 0.000),
            (0.088, 0.018),
            (0.094, 0.074),
            (0.096, 0.126),
            (0.098, 0.152),
        ],
        [
            (0.044, 0.000),
            (0.080, 0.018),
            (0.086, 0.074),
            (0.089, 0.126),
            (0.091, 0.152),
        ],
        segments=64,
    )


def _build_lid_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.104, 0.000),
            (0.090, 0.008),
            (0.062, 0.014),
            (0.046, 0.016),
        ],
        [
            (0.098, 0.002),
            (0.084, 0.009),
            (0.056, 0.013),
            (0.040, 0.014),
        ],
        segments=64,
    )


def _build_feed_tube_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.044, 0.000),
            (0.044, 0.094),
        ],
        [
            (0.032, 0.000),
            (0.032, 0.094),
        ],
        segments=48,
    )


def _build_ring_shell(outer_radius: float, inner_radius: float, length: float):
    return LatheGeometry.from_shell_profiles(
        [
            (outer_radius, 0.000),
            (outer_radius, length),
        ],
        [
            (inner_radius, 0.000),
            (inner_radius, length),
        ],
        segments=40,
    )


def _build_basket_geometry():
    return CylinderGeometry(radius=0.074, height=0.100, radial_segments=40, closed=False).translate(
        0.0, 0.0, 0.056
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_steel_juicer")

    stainless = model.material("stainless", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.15, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.76, 0.85, 0.90, 0.30))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    control_grey = model.material("control_grey", rgba=(0.28, 0.29, 0.31, 1.0))

    base_housing = model.part("base_housing")
    base_housing.visual(
        mesh_from_geometry(_build_base_shell(), "juicer_base_shell"),
        material=stainless,
        name="base_shell",
    )
    base_housing.visual(
        Box((0.268, 0.206, 0.018)),
        origin=Origin(xyz=(0.0, 0.003, 0.009)),
        material=dark_trim,
        name="lower_trim",
    )
    base_housing.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.181)),
        material=dark_trim,
        name="drive_coupler",
    )
    base_housing.visual(
        mesh_from_geometry(_build_ring_shell(0.038, 0.031, 0.006), "juicer_dial_bezel"),
        origin=Origin(xyz=(-0.034, -0.121, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="dial_bezel",
    )
    base_housing.visual(
        mesh_from_geometry(_build_ring_shell(0.018, 0.010, 0.014), "juicer_button_bezel"),
        origin=Origin(xyz=(0.056, -0.121, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="button_bezel",
    )
    base_housing.inertial = Inertial.from_geometry(
        Box((0.300, 0.240, 0.176)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
    )

    chamber = model.part("chamber")
    chamber.visual(
        mesh_from_geometry(_build_chamber_shell(), "juicer_chamber_shell"),
        material=smoked_clear,
        name="chamber_shell",
    )
    chamber.visual(
        Box((0.050, 0.040, 0.026)),
        origin=Origin(xyz=(0.0, -0.110, 0.060)),
        material=smoked_clear,
        name="juice_spout",
    )
    chamber.visual(
        Box((0.032, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, -0.138, 0.058)),
        material=smoked_clear,
        name="spout_lip",
    )
    chamber.visual(
        Box((0.084, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.089, 0.146)),
        material=dark_trim,
        name="hinge_bridge",
    )
    chamber.inertial = Inertial.from_geometry(
        Box((0.220, 0.300, 0.170)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
    )
    model.articulation(
        "base_to_chamber",
        ArticulationType.FIXED,
        parent=base_housing,
        child=chamber,
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_shell(), "juicer_lid_shell"),
        origin=Origin(xyz=(0.0, -0.090, 0.0)),
        material=smoked_clear,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_geometry(_build_feed_tube_shell(), "juicer_feed_tube_shell"),
        origin=Origin(xyz=(0.0, -0.090, 0.012)),
        material=smoked_clear,
        name="feed_tube_shell",
    )
    lid.visual(
        Box((0.084, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.008, 0.006)),
        material=dark_trim,
        name="lid_hinge_bridge",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.090),
        origin=Origin(xyz=(0.0, -0.187, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="front_grip",
    )
    lid.visual(
        Box((0.040, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, -0.179, 0.016)),
        material=dark_trim,
        name="front_grip_bridge",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.220, 0.220, 0.110)),
        mass=0.65,
        origin=Origin(xyz=(0.0, -0.090, 0.050)),
    )
    model.articulation(
        "chamber_to_lid",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=lid,
        origin=Origin(xyz=(0.0, 0.090, 0.158)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.029, length=0.176),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=black_plastic,
        name="pusher_body",
    )
    plunger.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
        material=black_plastic,
        name="upper_guide_collar",
    )
    plunger.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=black_plastic,
        name="lower_guide_collar",
    )
    plunger.visual(
        Cylinder(radius=0.042, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=control_grey,
        name="plunger_stop_flange",
    )
    plunger.visual(
        Cylinder(radius=0.040, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        material=control_grey,
        name="pusher_cap",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.196),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )
    model.articulation(
        "lid_to_plunger",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=plunger,
        origin=Origin(xyz=(0.0, -0.090, 0.106)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.095,
        ),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_geometry(_build_basket_geometry(), "juicer_basket"),
        material=stainless,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.074, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=stainless,
        name="grater_disc",
    )
    basket.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_trim,
        name="basket_hub",
    )
    for blade_index in range(6):
        basket.visual(
            Box((0.046, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, 0.022), rpy=(0.0, 0.0, blade_index * math.pi / 3.0)),
            material=stainless,
            name=f"grater_blade_{blade_index}",
        )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.110),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )
    model.articulation(
        "chamber_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base_housing,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.186)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=40.0),
    )

    selector_dial = model.part("selector_dial")
    selector_dial.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="dial_stem",
    )
    selector_dial.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_grey,
        name="dial_knob",
    )
    selector_dial.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="dial_rim",
    )
    selector_dial.visual(
        Box((0.004, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, -0.015, 0.012)),
        material=stainless,
        name="dial_indicator",
    )
    selector_dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=0.026),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "base_to_selector_dial",
        ArticulationType.REVOLUTE,
        parent=base_housing,
        child=selector_dial,
        origin=Origin(xyz=(-0.034, -0.123, 0.078)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    pulse_button = model.part("pulse_button")
    pulse_button.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="pulse_stem",
    )
    pulse_button.visual(
        Cylinder(radius=0.009, length=0.008),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=control_grey,
        name="pulse_cap",
    )
    pulse_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.009, length=0.020),
        mass=0.02,
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "base_to_pulse_button",
        ArticulationType.PRISMATIC,
        parent=base_housing,
        child=pulse_button,
        origin=Origin(xyz=(0.056, -0.121, 0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.08,
            lower=0.0,
            upper=0.005,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    plunger = object_model.get_part("plunger")
    basket = object_model.get_part("basket")
    pulse_button = object_model.get_part("pulse_button")

    lid_joint = object_model.get_articulation("chamber_to_lid")
    plunger_joint = object_model.get_articulation("lid_to_plunger")
    dial_joint = object_model.get_articulation("base_to_selector_dial")
    button_joint = object_model.get_articulation("base_to_pulse_button")
    basket_joint = object_model.get_articulation("chamber_to_basket")

    with ctx.pose({lid_joint: 0.0, plunger_joint: 0.0}):
        ctx.expect_gap(
            lid,
            chamber,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="chamber_shell",
            max_gap=0.008,
            max_penetration=0.0,
            name="closed lid sits just above the chamber rim",
        )
        ctx.expect_overlap(
            lid,
            chamber,
            axes="xy",
            elem_a="lid_shell",
            elem_b="chamber_shell",
            min_overlap=0.160,
            name="lid covers the chamber opening",
        )
        ctx.expect_within(
            plunger,
            lid,
            axes="xy",
            inner_elem="pusher_body",
            outer_elem="feed_tube_shell",
            margin=0.004,
            name="plunger stays centered in the feed tube",
        )
        ctx.expect_within(
            basket,
            chamber,
            axes="xy",
            inner_elem="basket_shell",
            outer_elem="chamber_shell",
            margin=0.012,
            name="basket remains inside the clear chamber footprint",
        )

    rest_plunger_position = ctx.part_world_position(plunger)
    with ctx.pose({plunger_joint: 0.095}):
        raised_plunger_position = ctx.part_world_position(plunger)
    ctx.check(
        "plunger lifts vertically from the lid",
        rest_plunger_position is not None
        and raised_plunger_position is not None
        and raised_plunger_position[2] > rest_plunger_position[2] + 0.070,
        details=f"rest={rest_plunger_position}, raised={raised_plunger_position}",
    )

    with ctx.pose({lid_joint: math.radians(60.0)}):
        ctx.expect_gap(
            lid,
            chamber,
            axis="z",
            positive_elem="feed_tube_shell",
            negative_elem="chamber_shell",
            min_gap=0.040,
            name="opened lid lifts the feed tube clear of the chamber",
        )

    rest_button_position = ctx.part_world_position(pulse_button)
    with ctx.pose({button_joint: 0.005}):
        pressed_button_position = ctx.part_world_position(pulse_button)
    ctx.check(
        "pulse button presses inward",
        rest_button_position is not None
        and pressed_button_position is not None
        and pressed_button_position[1] > rest_button_position[1] + 0.003,
        details=f"rest={rest_button_position}, pressed={pressed_button_position}",
    )

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "selector dial is a front-mounted rotary control",
        dial_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(dial_joint.axis[1]) > 0.99
        and dial_limits is not None
        and dial_limits.lower is not None
        and dial_limits.upper is not None,
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}, limits={dial_limits}",
    )

    basket_limits = basket_joint.motion_limits
    ctx.check(
        "grater basket spins continuously on the vertical drive axis",
        basket_joint.articulation_type == ArticulationType.CONTINUOUS
        and abs(basket_joint.axis[2]) > 0.99
        and basket_limits is not None
        and basket_limits.lower is None
        and basket_limits.upper is None,
        details=f"type={basket_joint.articulation_type}, axis={basket_joint.axis}, limits={basket_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
