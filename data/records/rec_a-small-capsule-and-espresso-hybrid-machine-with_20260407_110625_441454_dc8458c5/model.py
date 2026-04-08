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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _yz_section(
    *,
    x: float,
    width: float,
    height: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    profile = rounded_rect_profile(height, width, radius)
    return [(x, y, z + height * 0.5) for z, y in profile]


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="capsule_espresso_hybrid_machine")

    body_metal = model.material("body_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.09, 0.10, 1.0))
    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.82, 1.0))
    warm_black = model.material("warm_black", rgba=(0.14, 0.12, 0.11, 1.0))
    tray_metal = model.material("tray_metal", rgba=(0.62, 0.64, 0.66, 1.0))
    reservoir_smoke = model.material("reservoir_smoke", rgba=(0.42, 0.47, 0.52, 0.45))

    body = model.part("body")

    shell_geom = section_loft(
        [
            _yz_section(x=0.150, width=0.138, height=0.245, radius=0.026),
            _yz_section(x=0.055, width=0.152, height=0.276, radius=0.034),
            _yz_section(x=-0.055, width=0.150, height=0.248, radius=0.032),
            _yz_section(x=-0.160, width=0.136, height=0.220, radius=0.028),
        ]
    )
    body.visual(
        mesh_from_geometry(shell_geom, "espresso_body_shell"),
        material=body_metal,
        name="body_shell",
    )
    body.visual(
        Box((0.044, 0.116, 0.090)),
        origin=Origin(xyz=(0.120, 0.0, 0.096)),
        material=body_metal,
        name="front_fascia",
    )
    body.visual(
        Box((0.026, 0.086, 0.060)),
        origin=Origin(xyz=(0.145, 0.0, 0.206)),
        material=satin_black,
        name="group_mount",
    )
    body.visual(
        Cylinder(radius=0.034, length=0.055),
        origin=Origin(xyz=(0.176, 0.0, 0.216), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=chrome,
        name="group_head_body",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.204, 0.0, 0.216), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=chrome,
        name="group_head_face",
    )
    body.visual(
        Box((0.020, 0.094, 0.022)),
        origin=Origin(xyz=(0.145, 0.0, 0.072)),
        material=satin_black,
        name="platform_hinge_mount",
    )
    body.visual(
        Box((0.115, 0.128, 0.156)),
        origin=Origin(xyz=(-0.072, 0.0, 0.222)),
        material=reservoir_smoke,
        name="reservoir_shell",
    )
    body.visual(
        Box((0.122, 0.135, 0.010)),
        origin=Origin(xyz=(-0.072, 0.0, 0.298)),
        material=satin_black,
        name="reservoir_rim",
    )
    body.visual(
        Box((0.028, 0.015, 0.046)),
        origin=Origin(xyz=(0.112, 0.0675, 0.203)),
        material=satin_black,
        name="wand_support_block",
    )
    body.visual(
        Box((0.028, 0.022, 0.044)),
        origin=Origin(xyz=(-0.010, -0.082, 0.180)),
        material=satin_black,
        name="selector_boss",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.340, 0.170, 0.320)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.031, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=chrome,
        name="portafilter_ring",
    )
    portafilter.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=chrome,
        name="portafilter_basket",
    )
    portafilter.visual(
        Box((0.026, 0.018, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, -0.030)),
        material=chrome,
        name="spout_block",
    )
    portafilter.visual(
        Cylinder(radius=0.010, length=0.112),
        origin=Origin(xyz=(0.074, 0.0, -0.030), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=warm_black,
        name="portafilter_handle",
    )
    portafilter.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.132, 0.0, -0.030)),
        material=warm_black,
        name="handle_cap",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.150, 0.062, 0.050)),
        mass=0.55,
        origin=Origin(xyz=(0.060, 0.0, -0.020)),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.009, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=chrome,
        name="wand_pivot",
    )
    steam_wand.visual(
        Box((0.016, 0.018, 0.024)),
        origin=Origin(xyz=(0.004, 0.0, -0.012)),
        material=chrome,
        name="wand_collar",
    )
    steam_wand.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.005, 0.0, -0.006),
                    (0.012, 0.0, -0.026),
                    (0.018, 0.0, -0.082),
                    (0.034, 0.0, -0.152),
                ],
                radius=0.004,
                samples_per_segment=18,
                radial_segments=18,
            ),
            "steam_wand_tube",
        ),
        material=chrome,
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.0026, length=0.018),
        origin=Origin(xyz=(0.035, 0.0, -0.152), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=chrome,
        name="wand_tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.180)),
        mass=0.16,
        origin=Origin(xyz=(0.022, 0.0, -0.078)),
    )

    reservoir_lid = model.part("reservoir_lid")
    reservoir_lid.visual(
        Cylinder(radius=0.0045, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=chrome,
        name="lid_hinge_barrel",
    )
    reservoir_lid.visual(
        Box((0.118, 0.132, 0.008)),
        origin=Origin(xyz=(0.059, 0.0, 0.0085)),
        material=satin_black,
        name="lid_panel",
    )
    reservoir_lid.visual(
        Box((0.016, 0.052, 0.008)),
        origin=Origin(xyz=(0.114, 0.0, 0.014)),
        material=satin_black,
        name="lid_tab",
    )
    reservoir_lid.inertial = Inertial.from_geometry(
        Box((0.122, 0.132, 0.020)),
        mass=0.20,
        origin=Origin(xyz=(0.058, 0.0, 0.010)),
    )

    cup_platform = model.part("cup_platform")
    cup_platform.visual(
        Cylinder(radius=0.005, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=chrome,
        name="platform_hinge_barrel",
    )
    cup_platform.visual(
        Box((0.092, 0.090, 0.006)),
        origin=Origin(xyz=(0.046, 0.0, 0.008)),
        material=tray_metal,
        name="platform_surface",
    )
    for index, y_pos in enumerate((-0.024, 0.0, 0.024)):
        cup_platform.visual(
            Box((0.078, 0.010, 0.004)),
            origin=Origin(xyz=(0.046, y_pos, 0.013)),
            material=chrome,
            name=f"platform_rib_{index}",
        )
    cup_platform.inertial = Inertial.from_geometry(
        Box((0.095, 0.095, 0.020)),
        mass=0.22,
        origin=Origin(xyz=(0.046, 0.0, 0.010)),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=chrome,
        name="selector_hub",
    )
    selector_knob.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=warm_black,
        name="selector_knob_body",
    )
    selector_knob.visual(
        Box((0.006, 0.008, 0.008)),
        origin=Origin(xyz=(0.015, -0.020, 0.0)),
        material=chrome,
        name="selector_indicator",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Box((0.045, 0.028, 0.045)),
        mass=0.08,
        origin=Origin(xyz=(0.0, -0.016, 0.0)),
    )

    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.190, 0.0, 0.184)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.65,
            upper=0.10,
        ),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.124, 0.090, 0.203)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.2,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "body_to_reservoir_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=reservoir_lid,
        origin=Origin(xyz=(-0.134, 0.0, 0.300)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "body_to_cup_platform",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cup_platform,
        origin=Origin(xyz=(0.160, 0.0, 0.072)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "body_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(-0.010, -0.093, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")
    reservoir_lid = object_model.get_part("reservoir_lid")
    cup_platform = object_model.get_part("cup_platform")
    selector_knob = object_model.get_part("selector_knob")

    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    steam_joint = object_model.get_articulation("body_to_steam_wand")
    lid_joint = object_model.get_articulation("body_to_reservoir_lid")
    platform_joint = object_model.get_articulation("body_to_cup_platform")
    selector_joint = object_model.get_articulation("body_to_selector_knob")

    def elem_center(part, elem):
        return _aabb_center(ctx.part_element_world_aabb(part, elem=elem))

    ctx.expect_overlap(
        reservoir_lid,
        body,
        axes="xy",
        elem_a="lid_panel",
        elem_b="reservoir_rim",
        min_overlap=0.10,
        name="lid covers the reservoir opening footprint",
    )
    ctx.expect_gap(
        reservoir_lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="reservoir_rim",
        min_gap=0.0005,
        max_gap=0.006,
        name="lid sits just above the reservoir rim when closed",
    )
    ctx.expect_gap(
        body,
        cup_platform,
        axis="z",
        positive_elem="group_head_face",
        negative_elem="platform_surface",
        min_gap=0.085,
        max_gap=0.140,
        name="cup platform stays below the group head",
    )

    closed_lid_center = elem_center(reservoir_lid, "lid_panel")
    with ctx.pose({lid_joint: 1.10}):
        opened_lid_center = elem_center(reservoir_lid, "lid_panel")
    ctx.check(
        "reservoir lid flips upward",
        closed_lid_center is not None
        and opened_lid_center is not None
        and opened_lid_center[2] > closed_lid_center[2] + 0.045,
        details=f"closed={closed_lid_center}, opened={opened_lid_center}",
    )

    rest_platform_center = elem_center(cup_platform, "platform_surface")
    with ctx.pose({platform_joint: 1.30}):
        folded_platform_center = elem_center(cup_platform, "platform_surface")
    ctx.check(
        "cup platform folds downward",
        rest_platform_center is not None
        and folded_platform_center is not None
        and folded_platform_center[2] < rest_platform_center[2] - 0.040,
        details=f"rest={rest_platform_center}, folded={folded_platform_center}",
    )

    locked_handle_center = elem_center(portafilter, "portafilter_handle")
    with ctx.pose({portafilter_joint: -0.55}):
        unlocked_handle_center = elem_center(portafilter, "portafilter_handle")
    ctx.check(
        "portafilter rotates into the centered lock position",
        locked_handle_center is not None
        and unlocked_handle_center is not None
        and abs(locked_handle_center[1]) < 0.010
        and unlocked_handle_center[1] < -0.030,
        details=f"locked={locked_handle_center}, unlocked={unlocked_handle_center}",
    )

    rest_wand_tip = elem_center(steam_wand, "wand_tip")
    with ctx.pose({steam_joint: 0.85}):
        swung_wand_tip = elem_center(steam_wand, "wand_tip")
    ctx.check(
        "steam wand swings forward and upward",
        rest_wand_tip is not None
        and swung_wand_tip is not None
        and swung_wand_tip[0] > rest_wand_tip[0] + 0.070
        and swung_wand_tip[2] > rest_wand_tip[2] + 0.060,
        details=f"rest={rest_wand_tip}, swung={swung_wand_tip}",
    )

    rest_indicator = elem_center(selector_knob, "selector_indicator")
    with ctx.pose({selector_joint: 1.20}):
        turned_indicator = elem_center(selector_knob, "selector_indicator")
    ctx.check(
        "selector knob turns on its shaft",
        rest_indicator is not None
        and turned_indicator is not None
        and turned_indicator[2] > rest_indicator[2] + 0.010,
        details=f"rest={rest_indicator}, turned={turned_indicator}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
