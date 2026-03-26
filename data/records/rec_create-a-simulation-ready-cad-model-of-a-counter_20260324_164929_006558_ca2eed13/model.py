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


def _add_box(
    part,
    size,
    xyz,
    *,
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    radius,
    length,
    xyz,
    *,
    rpy=(0.0, 0.0, 0.0),
    material=None,
    name=None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bean_to_cup_coffee_machine", assets=ASSETS)

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    silver = model.material("silver", rgba=(0.77, 0.79, 0.81, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.05, 0.10, 0.13, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.28, 0.30, 0.33, 0.45))
    water_clear = model.material("water_clear", rgba=(0.74, 0.86, 0.95, 0.38))
    accent_dark = model.material("accent_dark", rgba=(0.32, 0.34, 0.37, 1.0))

    front_tilt = (-math.radians(17.0), 0.0, 0.0)

    chassis = model.part("chassis")
    _add_box(chassis, (0.25, 0.23, 0.03), (0.0, -0.03, 0.015), material=matte_black, name="base_plinth")
    _add_box(chassis, (0.23, 0.21, 0.30), (0.0, -0.045, 0.18), material=graphite, name="main_body")
    _add_box(chassis, (0.20, 0.16, 0.024), (0.0, -0.045, 0.342), material=accent_dark, name="top_cover")
    _add_box(chassis, (0.024, 0.040, 0.17), (-0.100, 0.072, 0.115), material=graphite, name="left_front_leg")
    _add_box(chassis, (0.024, 0.040, 0.17), (0.100, 0.072, 0.115), material=graphite, name="right_front_leg")
    _add_box(chassis, (0.186, 0.042, 0.13), (0.0, 0.094, 0.248), rpy=front_tilt, material=graphite, name="front_bridge")
    _add_box(chassis, (0.11, 0.10, 0.006), (0.0, -0.04, 0.327), material=accent_dark, name="hopper_seat")
    _add_box(chassis, (0.08, 0.004, 0.05), (0.0, 0.047, 0.186), material=accent_dark, name="brew_mount_face")
    _add_box(chassis, (0.18, 0.006, 0.024), (0.0, 0.057, 0.020), material=accent_dark, name="tray_slot_face")
    _add_box(chassis, (0.003, 0.13, 0.19), (0.1135, 0.01, 0.165), material=accent_dark, name="service_frame")
    _add_box(chassis, (0.006, 0.12, 0.25), (-0.051, -0.04, 0.18), material=accent_dark, name="tank_bay")
    _add_box(chassis, (0.020, 0.018, 0.120), (-0.078, 0.069, 0.126), material=graphite, name="left_bay_cheek")
    _add_box(chassis, (0.020, 0.018, 0.120), (0.078, 0.069, 0.126), material=graphite, name="right_bay_cheek")
    _add_box(chassis, (0.112, 0.018, 0.120), (0.0, 0.051, 0.126), material=matte_black, name="cup_bay_back")
    _add_box(chassis, (0.132, 0.016, 0.040), (0.0, 0.105, 0.214), rpy=front_tilt, material=accent_dark, name="control_panel")
    _add_box(chassis, (0.110, 0.010, 0.054), (0.0, 0.110, 0.262), rpy=front_tilt, material=screen_glass, name="display")
    _add_box(chassis, (0.024, 0.010, 0.014), (-0.042, 0.112, 0.210), rpy=front_tilt, material=silver, name="button_left")
    _add_box(chassis, (0.024, 0.012, 0.014), (0.0, 0.111, 0.210), rpy=front_tilt, material=silver, name="button_center")
    _add_box(chassis, (0.024, 0.010, 0.014), (0.042, 0.112, 0.210), rpy=front_tilt, material=silver, name="button_right")
    chassis.inertial = Inertial.from_geometry(
        Box((0.25, 0.23, 0.35)),
        mass=8.5,
        origin=Origin(xyz=(0.0, -0.03, 0.175)),
    )

    water_tank = model.part("water_tank")
    _add_box(water_tank, (0.06, 0.12, 0.25), (0.0, 0.0, 0.0), material=water_clear, name="tank_body")
    _add_box(water_tank, (0.062, 0.10, 0.016), (0.0, 0.0, 0.133), material=accent_dark, name="tank_cap")
    _add_box(water_tank, (0.006, 0.10, 0.22), (0.033, 0.0, 0.0), material=accent_dark, name="mount_face")
    water_tank.inertial = Inertial.from_geometry(
        Box((0.06, 0.12, 0.25)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    bean_hopper = model.part("bean_hopper")
    _add_box(bean_hopper, (0.10, 0.09, 0.018), (0.0, 0.0, 0.009), material=matte_black, name="hopper_collar")
    _add_box(bean_hopper, (0.11, 0.10, 0.085), (0.0, 0.0, 0.060), material=smoke_clear, name="hopper_body")
    _add_box(bean_hopper, (0.112, 0.102, 0.004), (0.0, 0.0, 0.102), material=matte_black, name="lid_rim")
    bean_hopper.inertial = Inertial.from_geometry(
        Box((0.11, 0.10, 0.108)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
    )

    hopper_lid = model.part("hopper_lid")
    _add_box(hopper_lid, (0.116, 0.108, 0.008), (0.0, 0.054, 0.004), material=matte_black, name="lid_panel")
    _add_box(hopper_lid, (0.05, 0.010, 0.014), (0.0, 0.103, 0.009), material=silver, name="lid_handle")
    hopper_lid.inertial = Inertial.from_geometry(
        Box((0.116, 0.108, 0.014)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.054, 0.007)),
    )

    brew_head = model.part("brew_head")
    _add_box(brew_head, (0.082, 0.05, 0.055), (0.0, 0.0, 0.0), material=matte_black, name="brew_block")
    _add_box(brew_head, (0.07, 0.008, 0.04), (0.0, -0.025, 0.0), material=accent_dark, name="mount_pad")
    _add_box(brew_head, (0.055, 0.028, 0.018), (0.0, 0.012, -0.020), material=accent_dark, name="spout_block")
    _add_cylinder(brew_head, 0.005, 0.040, (-0.018, 0.010, -0.040), material=silver, name="left_nozzle")
    _add_cylinder(brew_head, 0.005, 0.040, (0.018, 0.010, -0.040), material=silver, name="right_nozzle")
    brew_head.inertial = Inertial.from_geometry(
        Box((0.082, 0.05, 0.055)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    service_door = model.part("service_door")
    _add_box(service_door, (0.004, 0.12, 0.19), (0.002, 0.06, 0.0), material=accent_dark, name="door_panel")
    _add_box(service_door, (0.010, 0.028, 0.08), (0.009, 0.09, 0.0), material=silver, name="door_handle")
    service_door.inertial = Inertial.from_geometry(
        Box((0.010, 0.12, 0.19)),
        mass=0.28,
        origin=Origin(xyz=(0.005, 0.06, 0.0)),
    )

    drip_tray = model.part("drip_tray")
    _add_box(drip_tray, (0.168, 0.118, 0.026), (0.0, 0.059, 0.013), material=matte_black, name="tray_body")
    _add_box(drip_tray, (0.146, 0.088, 0.004), (0.0, 0.059, 0.028), material=silver, name="grate")
    _add_box(drip_tray, (0.148, 0.006, 0.020), (0.0, 0.003, 0.010), material=accent_dark, name="rear_stop")
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.168, 0.118, 0.030)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    model.articulation(
        "chassis_to_water_tank",
        ArticulationType.FIXED,
        parent=chassis,
        child=water_tank,
        origin=Origin(xyz=(-0.078, -0.04, 0.18)),
    )
    model.articulation(
        "chassis_to_bean_hopper",
        ArticulationType.FIXED,
        parent=chassis,
        child=bean_hopper,
        origin=Origin(xyz=(0.0, -0.04, 0.33)),
    )
    model.articulation(
        "bean_hopper_to_hopper_lid",
        ArticulationType.REVOLUTE,
        parent=bean_hopper,
        child=hopper_lid,
        origin=Origin(xyz=(0.0, -0.05, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "chassis_to_brew_head",
        ArticulationType.FIXED,
        parent=chassis,
        child=brew_head,
        origin=Origin(xyz=(0.0, 0.078, 0.205)),
    )
    model.articulation(
        "chassis_to_service_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=service_door,
        origin=Origin(xyz=(0.115, -0.05, 0.165)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.5),
    )
    model.articulation(
        "chassis_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=drip_tray,
        origin=Origin(xyz=(0.0, 0.060, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.25, lower=0.0, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    water_tank = object_model.get_part("water_tank")
    bean_hopper = object_model.get_part("bean_hopper")
    hopper_lid = object_model.get_part("hopper_lid")
    brew_head = object_model.get_part("brew_head")
    service_door = object_model.get_part("service_door")
    drip_tray = object_model.get_part("drip_tray")

    hopper_hinge = object_model.get_articulation("bean_hopper_to_hopper_lid")
    service_hinge = object_model.get_articulation("chassis_to_service_door")
    tray_slide = object_model.get_articulation("chassis_to_drip_tray")

    hopper_seat = chassis.get_visual("hopper_seat")
    tank_bay = chassis.get_visual("tank_bay")
    brew_mount_face = chassis.get_visual("brew_mount_face")
    service_frame = chassis.get_visual("service_frame")
    tray_slot_face = chassis.get_visual("tray_slot_face")

    hopper_collar = bean_hopper.get_visual("hopper_collar")
    hopper_rim = bean_hopper.get_visual("lid_rim")
    lid_handle = hopper_lid.get_visual("lid_handle")
    tank_mount = water_tank.get_visual("mount_face")
    mount_pad = brew_head.get_visual("mount_pad")
    left_nozzle = brew_head.get_visual("left_nozzle")
    door_panel = service_door.get_visual("door_panel")
    door_handle = service_door.get_visual("door_handle")
    tray_rear_stop = drip_tray.get_visual("rear_stop")
    tray_grate = drip_tray.get_visual("grate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
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
        negative_elem=hopper_seat,
        name="bean hopper seats on the top deck",
    )
    ctx.expect_contact(
        water_tank,
        chassis,
        elem_a=tank_mount,
        elem_b=tank_bay,
    )
    ctx.expect_gap(
        brew_head,
        chassis,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=mount_pad,
        negative_elem=brew_mount_face,
        name="brew head mounts to the front bridge",
    )
    ctx.expect_gap(
        service_door,
        chassis,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=door_panel,
        negative_elem=service_frame,
        name="service door closes flush with the side panel",
    )
    ctx.expect_contact(
        drip_tray,
        chassis,
        elem_a=tray_rear_stop,
        elem_b=tray_slot_face,
    )
    ctx.expect_overlap(drip_tray, brew_head, axes="x", min_overlap=0.08)
    ctx.expect_origin_distance(brew_head, drip_tray, axes="x", max_dist=0.01)
    ctx.expect_gap(
        brew_head,
        drip_tray,
        axis="z",
        min_gap=0.084,
        positive_elem=left_nozzle,
        negative_elem=tray_grate,
        name="brew nozzles clear the drip tray for a cup",
    )

    with ctx.pose({hopper_hinge: 1.1}):
        ctx.expect_gap(
            hopper_lid,
            bean_hopper,
            axis="z",
            min_gap=0.05,
            positive_elem=lid_handle,
            negative_elem=hopper_rim,
            name="hopper lid lifts clear for bean refill",
        )

    with ctx.pose({service_hinge: 1.2}):
        ctx.expect_gap(
            service_door,
            chassis,
            axis="x",
            min_gap=0.05,
            positive_elem=door_handle,
            negative_elem=service_frame,
            name="service door swings outward for maintenance access",
        )

    with ctx.pose({tray_slide: 0.10}):
        ctx.expect_gap(
            drip_tray,
            chassis,
            axis="y",
            min_gap=0.095,
            positive_elem=tray_rear_stop,
            negative_elem=tray_slot_face,
            name="drip tray pulls forward for cleaning",
        )
        ctx.expect_overlap(drip_tray, brew_head, axes="x", min_overlap=0.08)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
