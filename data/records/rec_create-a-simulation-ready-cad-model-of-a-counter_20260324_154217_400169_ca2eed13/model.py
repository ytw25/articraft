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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bean_to_cup_coffee_machine", assets=ASSETS)

    body_dark = model.material("body_dark", rgba=(0.16, 0.16, 0.17, 1.0))
    body_graphite = model.material("body_graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    metal = model.material("metal", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.07, 0.08, 0.09, 0.96))
    display_blue = model.material("display_blue", rgba=(0.20, 0.53, 0.86, 1.0))
    smoke = model.material("smoke", rgba=(0.38, 0.40, 0.43, 0.55))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.28, 0.31, 0.24)),
        origin=Origin(xyz=(0.0, -0.005, 0.12)),
        material=body_graphite,
        name="lower_shell",
    )
    chassis.visual(
        Box((0.28, 0.19, 0.13)),
        origin=Origin(xyz=(0.0, -0.055, 0.305)),
        material=body_graphite,
        name="upper_shell",
    )
    chassis.visual(
        Box((0.24, 0.12, 0.05)),
        origin=Origin(xyz=(0.0, 0.092, 0.295), rpy=(-0.55, 0.0, 0.0)),
        material=body_dark,
        name="control_bezel",
    )
    chassis.visual(
        Box((0.115, 0.055, 0.006)),
        origin=Origin(xyz=(0.0, 0.122, 0.304), rpy=(-0.55, 0.0, 0.0)),
        material=dark_glass,
        name="display_glass",
    )
    chassis.visual(
        Box((0.090, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.123, 0.305), rpy=(-0.55, 0.0, 0.0)),
        material=display_blue,
        name="display_strip",
    )
    chassis.visual(
        Box((0.018, 0.018, 0.006)),
        origin=Origin(xyz=(-0.072, 0.139, 0.287), rpy=(-0.55, 0.0, 0.0)),
        material=metal,
        name="left_button",
    )
    chassis.visual(
        Box((0.018, 0.018, 0.006)),
        origin=Origin(xyz=(0.072, 0.139, 0.287), rpy=(-0.55, 0.0, 0.0)),
        material=metal,
        name="right_button",
    )
    chassis.visual(
        Box((0.004, 0.22, 0.22)),
        origin=Origin(xyz=(0.138, 0.01, 0.19)),
        material=body_dark,
        name="service_frame",
    )
    chassis.visual(
        Box((0.028, 0.02, 0.03)),
        origin=Origin(xyz=(-0.095, 0.14, 0.235)),
        material=metal,
        name="wand_mount_block",
    )
    for i, (x, y) in enumerate(((-0.105, -0.11), (0.105, -0.11), (-0.105, 0.09), (0.105, 0.09)), start=1):
        chassis.visual(
            Cylinder(radius=0.013, length=0.01),
            origin=Origin(xyz=(x, y, 0.005)),
            material=body_dark,
            name=f"foot_{i}",
        )
    chassis.inertial = Inertial.from_geometry(
        Box((0.28, 0.31, 0.37)),
        mass=8.5,
        origin=Origin(xyz=(0.0, -0.03, 0.185)),
    )

    water_tank = model.part("water_tank")
    water_tank.visual(
        Box((0.09, 0.05, 0.22)),
        origin=Origin(xyz=(0.0, -0.025, 0.11)),
        material=smoke,
        name="tank_body",
    )
    water_tank.visual(
        Box((0.095, 0.055, 0.015)),
        origin=Origin(xyz=(0.0, -0.0275, 0.2275)),
        material=body_dark,
        name="tank_cap",
    )
    water_tank.inertial = Inertial.from_geometry(
        Box((0.09, 0.05, 0.22)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.025, 0.11)),
    )

    bean_hopper = model.part("bean_hopper")
    bean_hopper.visual(
        Box((0.15, 0.15, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=body_dark,
        name="hopper_collar",
    )
    bean_hopper.visual(
        Box((0.13, 0.13, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=smoke,
        name="hopper_bin",
    )
    bean_hopper.inertial = Inertial.from_geometry(
        Box((0.15, 0.15, 0.12)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    bean_hopper_lid = model.part("bean_hopper_lid")
    bean_hopper_lid.visual(
        Box((0.145, 0.135, 0.015)),
        origin=Origin(xyz=(0.0, 0.0675, 0.0075)),
        material=body_dark,
        name="lid_panel",
    )
    bean_hopper_lid.visual(
        Box((0.05, 0.02, 0.02)),
        origin=Origin(xyz=(0.0, 0.118, 0.022)),
        material=metal,
        name="lid_handle",
    )
    bean_hopper_lid.inertial = Inertial.from_geometry(
        Box((0.145, 0.135, 0.015)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0675, 0.0075)),
    )

    spout = model.part("spout")
    spout.visual(
        Box((0.07, 0.04, 0.06)),
        origin=Origin(xyz=(0.0, 0.02, 0.03)),
        material=metal,
        name="spout_mount",
    )
    spout.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(-0.016, 0.052, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="left_nozzle",
    )
    spout.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.016, 0.052, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="right_nozzle",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.07, 0.05, 0.06)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.025, 0.03)),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.20, 0.12, 0.03)),
        origin=Origin(xyz=(0.0, 0.06, 0.015)),
        material=body_dark,
        name="tray_shell",
    )
    drip_tray.visual(
        Box((0.175, 0.10, 0.004)),
        origin=Origin(xyz=(0.0, 0.06, 0.032)),
        material=metal,
        name="tray_grate",
    )
    drip_tray.visual(
        Box((0.20, 0.012, 0.035)),
        origin=Origin(xyz=(0.0, 0.114, 0.0175)),
        material=body_dark,
        name="tray_lip",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.20, 0.12, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.06, 0.015)),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.008, 0.22, 0.22)),
        origin=Origin(xyz=(0.004, 0.11, 0.0)),
        material=body_dark,
        name="door_panel",
    )
    service_door.visual(
        Box((0.003, 0.16, 0.14)),
        origin=Origin(xyz=(0.0055, 0.11, 0.0)),
        material=dark_glass,
        name="door_inset",
    )
    service_door.visual(
        Box((0.012, 0.04, 0.05)),
        origin=Origin(xyz=(0.014, 0.17, 0.0)),
        material=metal,
        name="door_handle",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.008, 0.22, 0.22)),
        mass=0.5,
        origin=Origin(xyz=(0.004, 0.11, 0.0)),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="pivot_pin",
    )
    steam_wand.visual(
        Cylinder(radius=0.007, length=0.02),
        origin=Origin(xyz=(0.0, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="wand_collar",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.15),
        origin=Origin(xyz=(0.0, 0.016, -0.075)),
        material=metal,
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.02),
        origin=Origin(xyz=(0.0, 0.016, -0.155)),
        material=metal,
        name="wand_tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.005, length=0.17),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.016, -0.085)),
    )

    model.articulation(
        "chassis_to_water_tank",
        ArticulationType.FIXED,
        parent=chassis,
        child=water_tank,
        origin=Origin(xyz=(0.085, -0.16, 0.13)),
    )
    model.articulation(
        "chassis_to_bean_hopper",
        ArticulationType.FIXED,
        parent=chassis,
        child=bean_hopper,
        origin=Origin(xyz=(-0.045, -0.055, 0.37)),
    )
    model.articulation(
        "bean_hopper_to_lid",
        ArticulationType.REVOLUTE,
        parent=bean_hopper,
        child=bean_hopper_lid,
        origin=Origin(xyz=(0.0, -0.065, 0.12)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "chassis_to_spout",
        ArticulationType.FIXED,
        parent=chassis,
        child=spout,
        origin=Origin(xyz=(0.0, 0.15, 0.145)),
    )
    model.articulation(
        "chassis_to_drip_tray",
        ArticulationType.FIXED,
        parent=chassis,
        child=drip_tray,
        origin=Origin(xyz=(0.0, 0.15, 0.0)),
    )
    model.articulation(
        "chassis_to_service_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=service_door,
        origin=Origin(xyz=(0.14, -0.10, 0.19)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-1.35, upper=0.0),
    )
    model.articulation(
        "chassis_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=steam_wand,
        origin=Origin(xyz=(-0.095, 0.156, 0.235)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    water_tank = object_model.get_part("water_tank")
    bean_hopper = object_model.get_part("bean_hopper")
    bean_hopper_lid = object_model.get_part("bean_hopper_lid")
    spout = object_model.get_part("spout")
    drip_tray = object_model.get_part("drip_tray")
    service_door = object_model.get_part("service_door")
    steam_wand = object_model.get_part("steam_wand")

    hopper_lid_hinge = object_model.get_articulation("bean_hopper_to_lid")
    service_door_hinge = object_model.get_articulation("chassis_to_service_door")
    steam_wand_pivot = object_model.get_articulation("chassis_to_steam_wand")

    lower_shell = chassis.get_visual("lower_shell")
    upper_shell = chassis.get_visual("upper_shell")
    control_bezel = chassis.get_visual("control_bezel")
    display_glass = chassis.get_visual("display_glass")
    display_strip = chassis.get_visual("display_strip")
    left_button = chassis.get_visual("left_button")
    right_button = chassis.get_visual("right_button")
    service_frame = chassis.get_visual("service_frame")
    wand_mount_block = chassis.get_visual("wand_mount_block")
    hopper_collar = bean_hopper.get_visual("hopper_collar")
    hopper_bin = bean_hopper.get_visual("hopper_bin")
    tank_body = water_tank.get_visual("tank_body")
    lid_panel = bean_hopper_lid.get_visual("lid_panel")
    lid_handle = bean_hopper_lid.get_visual("lid_handle")
    left_nozzle = spout.get_visual("left_nozzle")
    tray_shell = drip_tray.get_visual("tray_shell")
    tray_grate = drip_tray.get_visual("tray_grate")
    door_panel = service_door.get_visual("door_panel")
    door_handle = service_door.get_visual("door_handle")
    pivot_pin = steam_wand.get_visual("pivot_pin")
    wand_collar = steam_wand.get_visual("wand_collar")
    wand_tip = steam_wand.get_visual("wand_tip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        bean_hopper,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=hopper_collar,
        negative_elem=upper_shell,
    )
    ctx.expect_overlap(bean_hopper, chassis, axes="xy", min_overlap=0.01)
    ctx.expect_within(chassis, chassis, axes="xz", inner_elem=display_glass, outer_elem=control_bezel)
    ctx.expect_within(chassis, chassis, axes="xz", inner_elem=display_strip, outer_elem=display_glass)
    ctx.expect_within(chassis, chassis, axes="xz", inner_elem=left_button, outer_elem=control_bezel)
    ctx.expect_within(chassis, chassis, axes="xz", inner_elem=right_button, outer_elem=control_bezel)
    ctx.expect_gap(
        bean_hopper_lid,
        bean_hopper,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=lid_panel,
        negative_elem=hopper_bin,
    )
    ctx.expect_overlap(bean_hopper_lid, bean_hopper, axes="xy", min_overlap=0.012)

    ctx.expect_gap(
        service_door,
        chassis,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=door_panel,
        negative_elem=service_frame,
    )
    ctx.expect_overlap(service_door, chassis, axes="yz", min_overlap=0.03)

    ctx.expect_gap(
        drip_tray,
        chassis,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tray_shell,
        negative_elem=lower_shell,
    )
    ctx.expect_overlap(drip_tray, chassis, axes="xz", min_overlap=0.03)

    ctx.expect_gap(
        chassis,
        water_tank,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lower_shell,
        negative_elem=tank_body,
    )
    ctx.expect_overlap(water_tank, chassis, axes="xz", min_overlap=0.01)

    ctx.expect_within(spout, drip_tray, axes="x")
    ctx.expect_overlap(spout, drip_tray, axes="xy", min_overlap=0.005)
    ctx.expect_origin_distance(spout, drip_tray, axes="x", max_dist=0.02)
    ctx.expect_gap(
        spout,
        drip_tray,
        axis="z",
        min_gap=0.10,
        max_gap=0.18,
        positive_elem=left_nozzle,
        negative_elem=tray_grate,
    )

    ctx.expect_gap(
        steam_wand,
        chassis,
        axis="y",
        min_gap=0.004,
        max_gap=0.015,
        positive_elem=wand_collar,
        negative_elem=wand_mount_block,
    )
    ctx.expect_gap(
        steam_wand,
        chassis,
        axis="y",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=pivot_pin,
        negative_elem=lower_shell,
    )

    with ctx.pose({hopper_lid_hinge: 1.05}):
        ctx.expect_gap(
            bean_hopper_lid,
            bean_hopper,
            axis="z",
            min_gap=0.06,
            positive_elem=lid_handle,
            negative_elem=hopper_bin,
        )

    with ctx.pose({service_door_hinge: -1.05}):
        ctx.expect_gap(
            service_door,
            chassis,
            axis="x",
            min_gap=0.08,
            positive_elem=door_handle,
            negative_elem=service_frame,
        )

    with ctx.pose({steam_wand_pivot: 0.85}):
        ctx.expect_overlap(steam_wand, drip_tray, axes="xy", min_overlap=0.003)
        ctx.expect_gap(
            steam_wand,
            drip_tray,
            axis="z",
            min_gap=0.05,
            max_gap=0.16,
            positive_elem=wand_tip,
            negative_elem=tray_grate,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
