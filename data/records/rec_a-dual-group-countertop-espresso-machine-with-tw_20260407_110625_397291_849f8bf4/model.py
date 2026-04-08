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
    Inertial,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_group_espresso_machine")

    body_metal = model.material("body_metal", rgba=(0.79, 0.80, 0.82, 1.0))
    satin_dark = model.material("satin_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.10, 0.11, 1.0))
    grate_steel = model.material("grate_steel", rgba=(0.72, 0.73, 0.75, 1.0))
    water_tank_plastic = model.material("water_tank_plastic", rgba=(0.74, 0.81, 0.88, 0.40))
    handle_black = model.material("handle_black", rgba=(0.16, 0.14, 0.12, 1.0))
    wand_steel = model.material("wand_steel", rgba=(0.69, 0.70, 0.72, 1.0))

    lid_panel_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.44, 0.16, 0.016, corner_segments=8),
            0.012,
            center=True,
        ),
        "service_lid_panel",
    )
    grate_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.56, 0.17),
            0.004,
            slot_size=(0.040, 0.006),
            pitch=(0.055, 0.016),
            frame=0.014,
            corner_radius=0.008,
            slot_angle_deg=0.0,
            stagger=True,
            center=True,
        ),
        "espresso_tray_grate",
    )

    chassis = model.part("chassis")

    foot_positions = (
        (-0.305, -0.215),
        (0.305, -0.215),
        (-0.305, 0.215),
        (0.305, 0.215),
    )
    for index, (x_pos, y_pos) in enumerate(foot_positions):
        chassis.visual(
            Box((0.055, 0.055, 0.018)),
            origin=Origin(xyz=(x_pos, y_pos, 0.009)),
            material=trim_black,
            name=f"foot_{index}",
        )

    chassis.visual(
        Box((0.020, 0.500, 0.030)),
        origin=Origin(xyz=(-0.360, 0.000, 0.033)),
        material=satin_dark,
        name="left_base_rail",
    )
    chassis.visual(
        Box((0.020, 0.500, 0.030)),
        origin=Origin(xyz=(0.360, 0.000, 0.033)),
        material=satin_dark,
        name="right_base_rail",
    )
    chassis.visual(
        Box((0.680, 0.040, 0.030)),
        origin=Origin(xyz=(0.000, -0.230, 0.033)),
        material=satin_dark,
        name="front_base_bridge",
    )
    chassis.visual(
        Box((0.680, 0.060, 0.030)),
        origin=Origin(xyz=(0.000, 0.220, 0.033)),
        material=satin_dark,
        name="rear_base_bridge",
    )
    chassis.visual(
        Box((0.020, 0.520, 0.461)),
        origin=Origin(xyz=(-0.360, 0.000, 0.2785)),
        material=body_metal,
        name="left_side_shell",
    )
    chassis.visual(
        Box((0.020, 0.520, 0.461)),
        origin=Origin(xyz=(0.360, 0.000, 0.2785)),
        material=body_metal,
        name="right_side_shell",
    )
    chassis.visual(
        Box((0.700, 0.016, 0.461)),
        origin=Origin(xyz=(0.000, 0.252, 0.2785)),
        material=body_metal,
        name="rear_wall",
    )
    chassis.visual(
        Box((0.700, 0.270, 0.018)),
        origin=Origin(xyz=(0.000, 0.117, 0.150)),
        material=satin_dark,
        name="internal_shelf",
    )
    chassis.visual(
        Box((0.700, 0.160, 0.040)),
        origin=Origin(xyz=(0.000, -0.180, 0.292)),
        material=satin_dark,
        name="brew_deck",
    )
    chassis.visual(
        Box((0.700, 0.030, 0.168)),
        origin=Origin(xyz=(0.000, -0.245, 0.396)),
        material=body_metal,
        name="front_fascia",
    )
    chassis.visual(
        Box((0.700, 0.040, 0.040)),
        origin=Origin(xyz=(0.000, -0.220, 0.105)),
        material=satin_dark,
        name="tray_opening_lip",
    )
    chassis.visual(
        Box((0.030, 0.200, 0.070)),
        origin=Origin(xyz=(-0.335, -0.110, 0.083)),
        material=satin_dark,
        name="left_tray_cheek",
    )
    chassis.visual(
        Box((0.030, 0.200, 0.070)),
        origin=Origin(xyz=(0.335, -0.110, 0.083)),
        material=satin_dark,
        name="right_tray_cheek",
    )
    chassis.visual(
        Box((0.700, 0.300, 0.018)),
        origin=Origin(xyz=(0.000, -0.050, 0.518)),
        material=body_metal,
        name="top_front_deck",
    )
    chassis.visual(
        Box((0.090, 0.160, 0.018)),
        origin=Origin(xyz=(-0.265, 0.180, 0.518)),
        material=body_metal,
        name="left_lid_rail",
    )
    chassis.visual(
        Box((0.090, 0.160, 0.018)),
        origin=Origin(xyz=(0.265, 0.180, 0.518)),
        material=body_metal,
        name="right_lid_rail",
    )
    chassis.visual(
        Box((0.700, 0.020, 0.018)),
        origin=Origin(xyz=(0.000, 0.250, 0.518)),
        material=body_metal,
        name="rear_top_beam",
    )
    chassis.visual(
        Box((0.560, 0.014, 0.024)),
        origin=Origin(xyz=(0.000, -0.245, 0.468)),
        material=trim_black,
        name="control_band",
    )
    chassis.visual(
        Box((0.034, 0.034, 0.004)),
        origin=Origin(xyz=(-0.372, -0.130, 0.292)),
        material=trim_black,
        name="left_wand_mount_plate",
    )
    chassis.visual(
        Box((0.034, 0.034, 0.004)),
        origin=Origin(xyz=(0.372, -0.130, 0.292)),
        material=trim_black,
        name="right_wand_mount_plate",
    )

    def add_group_head(
        *,
        x_pos: float,
        body_name: str,
        face_name: str,
        collar_name: str,
    ) -> None:
        chassis.visual(
            Box((0.110, 0.110, 0.065)),
            origin=Origin(xyz=(x_pos, -0.175, 0.2395)),
            material=satin_dark,
            name=body_name,
        )
        chassis.visual(
            Cylinder(radius=0.034, length=0.026),
            origin=Origin(
                xyz=(x_pos, -0.243, 0.238),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=trim_black,
            name=face_name,
        )
        chassis.visual(
            Cylinder(radius=0.039, length=0.018),
            origin=Origin(xyz=(x_pos, -0.180, 0.198)),
            material=trim_black,
            name=collar_name,
        )

    add_group_head(
        x_pos=-0.160,
        body_name="left_group_body",
        face_name="left_group_face",
        collar_name="left_group_collar",
    )
    add_group_head(
        x_pos=0.160,
        body_name="right_group_body",
        face_name="right_group_face",
        collar_name="right_group_collar",
    )

    chassis.inertial = Inertial.from_geometry(
        Box((0.740, 0.540, 0.545)),
        mass=28.0,
        origin=Origin(xyz=(0.000, 0.000, 0.2725)),
    )

    water_tank = model.part("water_tank")
    water_tank.visual(
        Box((0.310, 0.160, 0.210)),
        origin=Origin(xyz=(0.000, 0.000, 0.105)),
        material=water_tank_plastic,
        name="tank_body",
    )
    water_tank.visual(
        Box((0.330, 0.180, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.214)),
        material=trim_black,
        name="tank_cap",
    )
    water_tank.visual(
        Box((0.060, 0.012, 0.026)),
        origin=Origin(xyz=(0.000, -0.050, 0.227)),
        material=trim_black,
        name="tank_handle",
    )
    water_tank.inertial = Inertial.from_geometry(
        Box((0.330, 0.180, 0.240)),
        mass=2.0,
        origin=Origin(xyz=(0.000, 0.000, 0.120)),
    )
    model.articulation(
        "chassis_to_water_tank",
        ArticulationType.FIXED,
        parent=chassis,
        child=water_tank,
        origin=Origin(xyz=(0.000, 0.150, 0.159)),
    )

    service_lid = model.part("service_lid")
    service_lid.visual(
        lid_panel_mesh,
        origin=Origin(xyz=(0.000, -0.080, 0.006)),
        material=body_metal,
        name="lid_panel",
    )
    service_lid.visual(
        Box((0.012, 0.160, 0.014)),
        origin=Origin(xyz=(-0.214, -0.080, 0.009)),
        material=body_metal,
        name="left_lid_skirt",
    )
    service_lid.visual(
        Box((0.012, 0.160, 0.014)),
        origin=Origin(xyz=(0.214, -0.080, 0.009)),
        material=body_metal,
        name="right_lid_skirt",
    )
    service_lid.visual(
        Box((0.120, 0.018, 0.016)),
        origin=Origin(xyz=(0.000, -0.145, 0.014)),
        material=trim_black,
        name="lid_pull",
    )
    service_lid.inertial = Inertial.from_geometry(
        Box((0.440, 0.160, 0.022)),
        mass=1.1,
        origin=Origin(xyz=(0.000, -0.080, 0.008)),
    )
    model.articulation(
        "chassis_to_service_lid",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=service_lid,
        origin=Origin(xyz=(0.000, 0.240, 0.527)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    def add_portafilter(name: str, x_pos: float) -> None:
        portafilter = model.part(name)
        portafilter.visual(
            Cylinder(radius=0.031, length=0.016),
            origin=Origin(xyz=(0.000, 0.000, -0.008)),
            material=wand_steel,
            name="locking_ring",
        )
        portafilter.visual(
            Cylinder(radius=0.043, length=0.028),
            origin=Origin(xyz=(0.000, 0.000, -0.030)),
            material=wand_steel,
            name="basket_body",
        )
        portafilter.visual(
            Box((0.020, 0.016, 0.012)),
            origin=Origin(xyz=(0.000, -0.010, -0.050)),
            material=wand_steel,
            name="spout_bridge",
        )
        portafilter.visual(
            Cylinder(radius=0.006, length=0.014),
            origin=Origin(xyz=(0.000, -0.010, -0.063)),
            material=wand_steel,
            name="spout_body",
        )
        portafilter.visual(
            Cylinder(radius=0.014, length=0.038),
            origin=Origin(
                xyz=(0.000, -0.031, -0.038),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=trim_black,
            name="handle_root",
        )
        portafilter.visual(
            Cylinder(radius=0.015, length=0.160),
            origin=Origin(
                xyz=(0.000, -0.110, -0.040),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=handle_black,
            name="handle",
        )
        portafilter.visual(
            Cylinder(radius=0.017, length=0.028),
            origin=Origin(
                xyz=(0.000, -0.190, -0.040),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=handle_black,
            name="handle_end",
        )
        portafilter.inertial = Inertial.from_geometry(
            Box((0.100, 0.210, 0.080)),
            mass=0.75,
            origin=Origin(xyz=(0.000, -0.085, -0.030)),
        )
        model.articulation(
            f"chassis_to_{name}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=portafilter,
            origin=Origin(xyz=(x_pos, -0.180, 0.189)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.0,
                lower=-0.60,
                upper=0.60,
            ),
        )

    add_portafilter("left_portafilter", -0.160)
    add_portafilter("right_portafilter", 0.160)

    def add_steam_wand(
        *,
        name: str,
        origin_xyz: tuple[float, float, float],
        outward_sign: float,
        axis: tuple[float, float, float],
    ) -> None:
        wand = model.part(name)
        wand.visual(
            Cylinder(radius=0.009, length=0.018),
            origin=Origin(
                xyz=(outward_sign * 0.009, 0.000, 0.000),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=wand_steel,
            name="pivot_collar",
        )
        wand.visual(
            Cylinder(radius=0.006, length=0.205),
            origin=Origin(xyz=(outward_sign * 0.018, 0.000, -0.1025)),
            material=wand_steel,
            name="wand_tube",
        )
        wand.visual(
            Cylinder(radius=0.0072, length=0.034),
            origin=Origin(xyz=(outward_sign * 0.018, 0.000, -0.150)),
            material=trim_black,
            name="wand_grip",
        )
        wand.visual(
            Cylinder(radius=0.004, length=0.030),
            origin=Origin(
                xyz=(outward_sign * 0.018, -0.015, -0.205),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=wand_steel,
            name="wand_tip",
        )
        wand.inertial = Inertial.from_geometry(
            Box((0.050, 0.040, 0.220)),
            mass=0.24,
            origin=Origin(xyz=(outward_sign * 0.018, 0.000, -0.102)),
        )
        model.articulation(
            f"chassis_to_{name}",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=wand,
            origin=Origin(xyz=origin_xyz),
            axis=axis,
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.5,
                lower=-0.65,
                upper=0.55,
            ),
        )

    add_steam_wand(
        name="left_steam_wand",
        origin_xyz=(-0.374, -0.130, 0.292),
        outward_sign=-1.0,
        axis=(0.0, 1.0, 0.0),
    )
    add_steam_wand(
        name="right_steam_wand",
        origin_xyz=(0.374, -0.130, 0.292),
        outward_sign=1.0,
        axis=(0.0, -1.0, 0.0),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.604, 0.220, 0.004)),
        origin=Origin(xyz=(0.000, -0.050, 0.002)),
        material=satin_dark,
        name="tray_floor",
    )
    drip_tray.visual(
        Box((0.604, 0.008, 0.040)),
        origin=Origin(xyz=(0.000, -0.156, 0.020)),
        material=satin_dark,
        name="tray_front_wall",
    )
    drip_tray.visual(
        Box((0.604, 0.008, 0.040)),
        origin=Origin(xyz=(0.000, 0.056, 0.020)),
        material=satin_dark,
        name="tray_rear_wall",
    )
    drip_tray.visual(
        Box((0.008, 0.204, 0.040)),
        origin=Origin(xyz=(-0.298, -0.050, 0.020)),
        material=satin_dark,
        name="tray_left_wall",
    )
    drip_tray.visual(
        Box((0.008, 0.204, 0.040)),
        origin=Origin(xyz=(0.298, -0.050, 0.020)),
        material=satin_dark,
        name="tray_right_wall",
    )
    drip_tray.visual(
        Box((0.420, 0.014, 0.020)),
        origin=Origin(xyz=(0.000, -0.167, 0.026)),
        material=trim_black,
        name="tray_handle",
    )
    drip_tray.visual(
        Box((0.018, 0.180, 0.024)),
        origin=Origin(xyz=(-0.272, 0.035, 0.016)),
        material=trim_black,
        name="left_runner",
    )
    drip_tray.visual(
        Box((0.018, 0.180, 0.024)),
        origin=Origin(xyz=(0.272, 0.035, 0.016)),
        material=trim_black,
        name="right_runner",
    )
    for x_pos in (-0.250, 0.250):
        for y_pos in (-0.120, 0.000):
            drip_tray.visual(
                Box((0.016, 0.016, 0.032)),
                origin=Origin(xyz=(x_pos, y_pos, 0.020)),
                material=grate_steel,
                name=f"grate_post_{'l' if x_pos < 0 else 'r'}_{'f' if y_pos < -0.05 else 'r'}",
            )
    drip_tray.visual(
        grate_mesh,
        origin=Origin(xyz=(0.000, -0.058, 0.038)),
        material=grate_steel,
        name="tray_grate",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.620, 0.240, 0.050)),
        mass=1.2,
        origin=Origin(xyz=(0.000, -0.050, 0.025)),
    )
    model.articulation(
        "chassis_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=drip_tray,
        origin=Origin(xyz=(0.000, -0.100, 0.044)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.20,
            lower=0.0,
            upper=0.120,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    service_lid = object_model.get_part("service_lid")
    water_tank = object_model.get_part("water_tank")
    left_portafilter = object_model.get_part("left_portafilter")
    right_portafilter = object_model.get_part("right_portafilter")
    left_steam_wand = object_model.get_part("left_steam_wand")
    right_steam_wand = object_model.get_part("right_steam_wand")
    drip_tray = object_model.get_part("drip_tray")

    lid_hinge = object_model.get_articulation("chassis_to_service_lid")
    tray_slide = object_model.get_articulation("chassis_to_drip_tray")
    left_pf_joint = object_model.get_articulation("chassis_to_left_portafilter")
    right_pf_joint = object_model.get_articulation("chassis_to_right_portafilter")
    left_wand_joint = object_model.get_articulation("chassis_to_left_steam_wand")
    right_wand_joint = object_model.get_articulation("chassis_to_right_steam_wand")

    def elem_center(part_name, elem_name):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        chassis,
        left_portafilter,
        axis="z",
        positive_elem="left_group_collar",
        negative_elem="locking_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="left portafilter seats against left group collar",
    )
    ctx.expect_gap(
        chassis,
        right_portafilter,
        axis="z",
        positive_elem="right_group_collar",
        negative_elem="locking_ring",
        max_gap=0.001,
        max_penetration=0.0,
        name="right portafilter seats against right group collar",
    )
    ctx.expect_overlap(
        service_lid,
        water_tank,
        axes="xy",
        min_overlap=0.12,
        elem_a="lid_panel",
        elem_b="tank_body",
        name="service lid covers the water tank opening",
    )
    ctx.expect_gap(
        chassis,
        drip_tray,
        axis="z",
        positive_elem="left_group_collar",
        negative_elem="tray_grate",
        min_gap=0.090,
        name="drip tray grate sits below the group heads",
    )

    closed_lid_pull = elem_center("service_lid", "lid_pull")
    with ctx.pose({lid_hinge: 1.0}):
        open_lid_pull = elem_center("service_lid", "lid_pull")
    ctx.check(
        "service lid opens upward",
        closed_lid_pull is not None
        and open_lid_pull is not None
        and open_lid_pull[2] > closed_lid_pull[2] + 0.08,
        details=f"closed={closed_lid_pull}, open={open_lid_pull}",
    )

    tray_rest = elem_center("drip_tray", "tray_handle")
    with ctx.pose({tray_slide: 0.12}):
        tray_extended = elem_center("drip_tray", "tray_handle")
    ctx.check(
        "drip tray slides out from the front",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[1] < tray_rest[1] - 0.08,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    with ctx.pose({tray_slide: 0.12}):
        ctx.expect_within(
            drip_tray,
            chassis,
            axes="x",
            margin=0.03,
            name="extended drip tray remains laterally within the machine footprint",
        )

    left_pf_low = elem_center("left_portafilter", "handle_end")
    with ctx.pose({left_pf_joint: 0.45}):
        left_pf_rotated = elem_center("left_portafilter", "handle_end")
    ctx.check(
        "left portafilter rotates around the brew axis",
        left_pf_low is not None
        and left_pf_rotated is not None
        and abs(left_pf_rotated[0] - left_pf_low[0]) > 0.03,
        details=f"rest={left_pf_low}, rotated={left_pf_rotated}",
    )

    right_pf_low = elem_center("right_portafilter", "handle_end")
    with ctx.pose({right_pf_joint: -0.45}):
        right_pf_rotated = elem_center("right_portafilter", "handle_end")
    ctx.check(
        "right portafilter rotates around the brew axis",
        right_pf_low is not None
        and right_pf_rotated is not None
        and abs(right_pf_rotated[0] - right_pf_low[0]) > 0.03,
        details=f"rest={right_pf_low}, rotated={right_pf_rotated}",
    )

    left_tip_rest = elem_center("left_steam_wand", "wand_tip")
    with ctx.pose({left_wand_joint: 0.45}):
        left_tip_swung = elem_center("left_steam_wand", "wand_tip")
    ctx.check(
        "left steam wand swings outward",
        left_tip_rest is not None
        and left_tip_swung is not None
        and left_tip_swung[0] < left_tip_rest[0] - 0.02,
        details=f"rest={left_tip_rest}, swung={left_tip_swung}",
    )

    right_tip_rest = elem_center("right_steam_wand", "wand_tip")
    with ctx.pose({right_wand_joint: 0.45}):
        right_tip_swung = elem_center("right_steam_wand", "wand_tip")
    ctx.check(
        "right steam wand swings outward",
        right_tip_rest is not None
        and right_tip_swung is not None
        and right_tip_swung[0] > right_tip_rest[0] + 0.02,
        details=f"rest={right_tip_rest}, swung={right_tip_swung}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
