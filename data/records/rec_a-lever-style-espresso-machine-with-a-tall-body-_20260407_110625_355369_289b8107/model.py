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
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lever_espresso_machine")

    polished_steel = model.material("polished_steel", rgba=(0.80, 0.82, 0.84, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    handle_black = model.material("handle_black", rgba=(0.14, 0.12, 0.10, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.50, 0.58, 0.62, 0.35))

    def midpoint(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

    def distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def rpy_for_cylinder(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        length_xy = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(length_xy, dz)
        return (0.0, pitch, yaw)

    def add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
        part.visual(
            Cylinder(radius=radius, length=distance(a, b)),
            origin=Origin(xyz=midpoint(a, b), rpy=rpy_for_cylinder(a, b)),
            material=material,
            name=name,
        )

    cup_platform_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.20, 0.22),
            0.004,
            slot_size=(0.030, 0.006),
            pitch=(0.040, 0.018),
            frame=0.012,
            corner_radius=0.006,
            stagger=True,
        ),
        "espresso_cup_platform",
    )
    tray_grate_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.186, 0.158),
            0.003,
            slot_size=(0.026, 0.005),
            pitch=(0.036, 0.016),
            frame=0.010,
            corner_radius=0.004,
            stagger=True,
        ),
        "espresso_tray_grate",
    )

    body = model.part("body")
    body.visual(
        Box((0.28, 0.04, 0.10)),
        origin=Origin(xyz=(0.0, -0.115, 0.05)),
        material=polished_steel,
        name="left_base_rail",
    )
    body.visual(
        Box((0.28, 0.04, 0.10)),
        origin=Origin(xyz=(0.0, 0.115, 0.05)),
        material=polished_steel,
        name="right_base_rail",
    )
    body.visual(
        Box((0.09, 0.04, 0.05)),
        origin=Origin(xyz=(-0.09, -0.107, 0.125)),
        material=polished_steel,
        name="left_rear_riser",
    )
    body.visual(
        Box((0.09, 0.04, 0.05)),
        origin=Origin(xyz=(-0.09, 0.107, 0.125)),
        material=polished_steel,
        name="right_rear_riser",
    )
    body.visual(
        Box((0.11, 0.18, 0.018)),
        origin=Origin(xyz=(-0.08, 0.0, 0.143)),
        material=polished_steel,
        name="rear_deck",
    )
    body.visual(
        Box((0.03, 0.194, 0.03)),
        origin=Origin(xyz=(0.125, 0.0, 0.105)),
        material=polished_steel,
        name="front_cross_beam",
    )
    body.visual(
        Box((0.03, 0.03, 0.05)),
        origin=Origin(xyz=(0.02, -0.112, 0.12)),
        material=polished_steel,
        name="left_platform_post",
    )
    body.visual(
        Box((0.03, 0.03, 0.05)),
        origin=Origin(xyz=(0.02, 0.112, 0.12)),
        material=polished_steel,
        name="right_platform_post",
    )
    body.visual(
        cup_platform_mesh,
        origin=Origin(xyz=(0.06, 0.0, 0.143)),
        material=dark_steel,
        name="cup_platform",
    )
    body.visual(
        Box((0.16, 0.19, 0.348)),
        origin=Origin(xyz=(-0.025, 0.0, 0.326)),
        material=polished_steel,
        name="rear_column",
    )
    body.visual(
        Box((0.055, 0.16, 0.13)),
        origin=Origin(xyz=(0.0675, 0.0, 0.34)),
        material=polished_steel,
        name="front_fascia",
    )
    body.visual(
        Box((0.024, 0.022, 0.044)),
        origin=Origin(xyz=(0.093, -0.053, 0.425)),
        material=dark_steel,
        name="left_lever_mount",
    )
    body.visual(
        Box((0.024, 0.022, 0.044)),
        origin=Origin(xyz=(0.093, 0.053, 0.425)),
        material=dark_steel,
        name="right_lever_mount",
    )
    body.visual(
        Box((0.14, 0.19, 0.10)),
        origin=Origin(xyz=(-0.01, 0.0, 0.55)),
        material=polished_steel,
        name="upper_tank",
    )
    body.visual(
        Box((0.11, 0.12, 0.012)),
        origin=Origin(xyz=(0.005, 0.0, 0.606)),
        material=dark_steel,
        name="top_fill_collar",
    )
    body.visual(
        Box((0.09, 0.12, 0.004)),
        origin=Origin(xyz=(0.005, 0.0, 0.6085)),
        material=smoked_glass,
        name="water_window",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.32, 0.27, 0.612)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.306)),
    )

    group_head = model.part("group_head")
    group_head.visual(
        Box((0.04, 0.10, 0.08)),
        origin=Origin(xyz=(0.02, 0.0, -0.005)),
        material=dark_steel,
        name="head_block",
    )
    group_head.visual(
        Cylinder(radius=0.036, length=0.072),
        origin=Origin(xyz=(0.056, 0.0, -0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="brew_chamber",
    )
    group_head.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.084, 0.0, -0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="lower_collar",
    )
    group_head.visual(
        Box((0.022, 0.060, 0.024)),
        origin=Origin(xyz=(0.066, 0.0, -0.053)),
        material=dark_steel,
        name="spout_block",
    )
    group_head.inertial = Inertial.from_geometry(
        Box((0.095, 0.10, 0.10)),
        mass=1.4,
        origin=Origin(xyz=(0.048, 0.0, -0.015)),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.034, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=polished_steel,
        name="basket",
    )
    portafilter.visual(
        Box((0.036, 0.020, 0.015)),
        origin=Origin(xyz=(0.018, 0.0, -0.014)),
        material=polished_steel,
        name="handle_neck",
    )
    portafilter.visual(
        Box((0.06, 0.028, 0.022)),
        origin=Origin(xyz=(0.062, 0.0, -0.018)),
        material=handle_black,
        name="handle_grip",
    )
    portafilter.visual(
        Cylinder(radius=0.012, length=0.03),
        origin=Origin(xyz=(0.105, 0.0, -0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_black,
        name="handle_cap",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.12, 0.07, 0.05)),
        mass=0.45,
        origin=Origin(xyz=(0.05, 0.0, -0.018)),
    )

    brew_lever = model.part("brew_lever")
    brew_lever.visual(
        Cylinder(radius=0.014, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_hub",
    )
    add_member(
        brew_lever,
        (0.0, -0.020, 0.0),
        (0.070, -0.038, 0.162),
        0.010,
        dark_steel,
        name="left_arm",
    )
    add_member(
        brew_lever,
        (0.0, 0.020, 0.0),
        (0.070, 0.038, 0.162),
        0.010,
        dark_steel,
        name="right_arm",
    )
    add_member(
        brew_lever,
        (0.0, 0.0, 0.0),
        (0.058, 0.0, 0.148),
        0.008,
        dark_steel,
        name="center_link",
    )
    brew_lever.visual(
        Cylinder(radius=0.014, length=0.115),
        origin=Origin(xyz=(0.072, 0.0, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_black,
        name="grip_bar",
    )
    brew_lever.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 0.20)),
        mass=0.55,
        origin=Origin(xyz=(0.04, 0.0, 0.10)),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_collar",
    )
    steam_wand.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.010, 0.0, -0.002),
                    (0.026, 0.0, -0.020),
                    (0.030, 0.0, -0.090),
                    (0.020, 0.0, -0.172),
                ],
                radius=0.005,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
                up_hint=(0.0, 1.0, 0.0),
            ),
            "espresso_steam_wand",
        ),
        material=polished_steel,
        name="wand_tube",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.04, 0.03, 0.19)),
        mass=0.18,
        origin=Origin(xyz=(0.02, 0.0, -0.09)),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.20, 0.17, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_steel,
        name="tray_floor",
    )
    drip_tray.visual(
        Box((0.20, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -0.081, 0.014)),
        material=polished_steel,
        name="left_wall",
    )
    drip_tray.visual(
        Box((0.20, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.081, 0.014)),
        material=polished_steel,
        name="right_wall",
    )
    drip_tray.visual(
        Box((0.008, 0.17, 0.024)),
        origin=Origin(xyz=(-0.096, 0.0, 0.012)),
        material=polished_steel,
        name="rear_wall",
    )
    drip_tray.visual(
        Box((0.008, 0.13, 0.028)),
        origin=Origin(xyz=(0.096, 0.0, 0.014)),
        material=polished_steel,
        name="front_wall",
    )
    drip_tray.visual(
        Box((0.020, 0.055, 0.020)),
        origin=Origin(xyz=(0.108, 0.0, 0.020)),
        material=handle_black,
        name="tray_pull",
    )
    drip_tray.visual(
        tray_grate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0295)),
        material=dark_steel,
        name="grate",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, 0.034)),
        mass=0.7,
        origin=Origin(xyz=(0.01, 0.0, 0.017)),
    )

    water_lid = model.part("water_lid")
    water_lid.visual(
        Box((0.11, 0.12, 0.006)),
        origin=Origin(xyz=(0.055, 0.0, 0.003)),
        material=dark_steel,
        name="lid_panel",
    )
    water_lid.visual(
        Box((0.018, 0.05, 0.010)),
        origin=Origin(xyz=(0.104, 0.0, 0.006)),
        material=handle_black,
        name="lid_tab",
    )
    water_lid.inertial = Inertial.from_geometry(
        Box((0.11, 0.12, 0.012)),
        mass=0.15,
        origin=Origin(xyz=(0.055, 0.0, 0.006)),
    )

    model.articulation(
        "body_to_group_head",
        ArticulationType.FIXED,
        parent=body,
        child=group_head,
        origin=Origin(xyz=(0.095, 0.0, 0.315)),
    )
    model.articulation(
        "group_head_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=group_head,
        child=portafilter,
        origin=Origin(xyz=(0.084, 0.0, -0.062)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-0.85,
            upper=0.15,
        ),
    )
    model.articulation(
        "body_to_brew_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=brew_lever,
        origin=Origin(xyz=(0.095, 0.0, 0.425)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=1.18,
        ),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.025, 0.107, 0.345)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.6,
            lower=-0.55,
            upper=0.90,
        ),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(-0.005, 0.0, 0.10)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.25,
            lower=0.0,
            upper=0.12,
        ),
    )
    model.articulation(
        "body_to_water_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=water_lid,
        origin=Origin(xyz=(-0.05, 0.0, 0.612)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.6,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    group_head = object_model.get_part("group_head")
    portafilter = object_model.get_part("portafilter")
    lever = object_model.get_part("brew_lever")
    steam_wand = object_model.get_part("steam_wand")
    tray = object_model.get_part("drip_tray")
    lid = object_model.get_part("water_lid")

    tray_slide = object_model.get_articulation("body_to_drip_tray")
    lever_joint = object_model.get_articulation("body_to_brew_lever")
    portafilter_joint = object_model.get_articulation("group_head_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_steam_wand")
    lid_joint = object_model.get_articulation("body_to_water_lid")

    def elem_center_z(part_obj, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    def elem_center_y(part_obj, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    def elem_center_x(part_obj, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) * 0.5

    with ctx.pose({tray_slide: 0.0}):
        ctx.expect_overlap(
            body,
            tray,
            axes="xy",
            elem_a="cup_platform",
            elem_b="grate",
            min_overlap=0.12,
            name="cup platform covers the inserted drip tray",
        )
        ctx.expect_gap(
            body,
            tray,
            axis="z",
            positive_elem="cup_platform",
            negative_elem="grate",
            min_gap=0.008,
            max_gap=0.020,
            name="tray sits just below the cup platform",
        )

    tray_rest = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.12}):
        tray_extended = ctx.part_world_position(tray)
        ctx.expect_overlap(
            body,
            tray,
            axes="y",
            elem_a="cup_platform",
            elem_b="grate",
            min_overlap=0.14,
            name="extended tray stays centered between the side rails",
        )
    ctx.check(
        "drip tray slides out forward",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[0] > tray_rest[0] + 0.10,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    with ctx.pose({portafilter_joint: 0.0}):
        ctx.expect_gap(
            group_head,
            portafilter,
            axis="z",
            positive_elem="lower_collar",
            negative_elem="basket",
            max_gap=0.001,
            max_penetration=0.0,
            name="portafilter seats under the group head collar",
        )
        ctx.expect_overlap(
            group_head,
            portafilter,
            axes="xy",
            elem_a="lower_collar",
            elem_b="basket",
            min_overlap=0.020,
            name="portafilter stays centered on the brew axis",
        )

    pf_locked_y = elem_center_y(portafilter, "handle_grip")
    with ctx.pose({portafilter_joint: -0.75}):
        pf_unlocked_y = elem_center_y(portafilter, "handle_grip")
    ctx.check(
        "portafilter handle twists to unlock",
        pf_locked_y is not None
        and pf_unlocked_y is not None
        and abs(pf_unlocked_y - pf_locked_y) > 0.035,
        details=f"locked_y={pf_locked_y}, unlocked_y={pf_unlocked_y}",
    )

    ctx.expect_origin_gap(
        lever,
        group_head,
        axis="z",
        min_gap=0.09,
        name="lever pivot sits above the group head",
    )
    lever_rest_z = elem_center_z(lever, "grip_bar")
    with ctx.pose({lever_joint: 1.10}):
        lever_pulled_z = elem_center_z(lever, "grip_bar")
    ctx.check(
        "brew lever pulls downward through its stroke",
        lever_rest_z is not None
        and lever_pulled_z is not None
        and lever_pulled_z < lever_rest_z - 0.12,
        details=f"rest_z={lever_rest_z}, pulled_z={lever_pulled_z}",
    )

    wand_rest_x = elem_center_x(steam_wand, "wand_tube")
    with ctx.pose({wand_joint: 0.75}):
        wand_swung_x = elem_center_x(steam_wand, "wand_tube")
    ctx.check(
        "steam wand swings away from its tucked pose",
        wand_rest_x is not None
        and wand_swung_x is not None
        and abs(wand_swung_x - wand_rest_x) > 0.03,
        details=f"rest_x={wand_rest_x}, swung_x={wand_swung_x}",
    )

    lid_rest_z = elem_center_z(lid, "lid_panel")
    with ctx.pose({lid_joint: 1.0}):
        lid_open_z = elem_center_z(lid, "lid_panel")
    ctx.check(
        "water lid opens upward for top filling",
        lid_rest_z is not None and lid_open_z is not None and lid_open_z > lid_rest_z + 0.03,
        details=f"rest_z={lid_rest_z}, open_z={lid_open_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
