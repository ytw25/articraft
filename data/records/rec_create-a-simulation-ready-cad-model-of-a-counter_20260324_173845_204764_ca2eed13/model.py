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

    body_black = model.material("body_black", rgba=(0.12, 0.12, 0.13, 1.0))
    fascia_black = model.material("fascia_black", rgba=(0.05, 0.05, 0.06, 1.0))
    matte_gray = model.material("matte_gray", rgba=(0.22, 0.23, 0.24, 1.0))
    smoke_glass = model.material("smoke_glass", rgba=(0.28, 0.30, 0.31, 0.55))
    screen_glass = model.material("screen_glass", rgba=(0.10, 0.16, 0.22, 0.90))
    steel = model.material("steel", rgba=(0.69, 0.71, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.48, 0.50, 0.52, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.164, 0.238, 0.168)),
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
        material=body_black,
        name="main_lower",
    )
    body.visual(
        Box((0.154, 0.222, 0.094)),
        origin=Origin(xyz=(0.0, 0.0, 0.215)),
        material=body_black,
        name="main_upper",
    )
    body.visual(
        Box((0.128, 0.010, 0.128)),
        origin=Origin(xyz=(0.0, 0.121, 0.094)),
        material=fascia_black,
        name="door_frame",
    )
    body.visual(
        Box((0.122, 0.028, 0.072)),
        origin=Origin(xyz=(0.0, 0.125, 0.228)),
        material=fascia_black,
        name="control_fascia",
    )
    body.visual(
        Box((0.082, 0.006, 0.028)),
        origin=Origin(xyz=(0.0, 0.141, 0.237)),
        material=screen_glass,
        name="display_screen",
    )
    body.visual(
        Box((0.074, 0.046, 0.040)),
        origin=Origin(xyz=(0.0, 0.102, 0.162)),
        material=matte_gray,
        name="brew_nose",
    )
    body.visual(
        Box((0.052, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.129, 0.143)),
        material=dark_steel,
        name="spout_bridge",
    )
    for x in (-0.015, 0.015):
        body.visual(
            Cylinder(radius=0.005, length=0.012),
            origin=Origin(xyz=(x, 0.143, 0.133), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"spout_{'left' if x < 0 else 'right'}",
        )
    body.visual(
        Box((0.130, 0.094, 0.008)),
        origin=Origin(xyz=(0.0, 0.014, 0.266)),
        material=dark_steel,
        name="cup_warmer",
    )
    body.visual(
        Box((0.112, 0.082, 0.006)),
        origin=Origin(xyz=(0.0, 0.018, 0.313)),
        material=matte_gray,
        name="hopper_seat",
    )
    body.visual(
        Box((0.100, 0.076, 0.052)),
        origin=Origin(xyz=(0.0, 0.018, 0.290)),
        material=smoke_glass,
        name="hopper_bin",
    )
    body.visual(
        Box((0.140, 0.030, 0.022)),
        origin=Origin(xyz=(0.0, 0.087, 0.011)),
        material=body_black,
        name="tray_mount",
    )
    body.visual(
        Box((0.016, 0.026, 0.030)),
        origin=Origin(xyz=(0.082, 0.072, 0.206)),
        material=matte_gray,
        name="steam_mount_block",
    )
    body.visual(
        Box((0.002, 0.016, 0.018)),
        origin=Origin(xyz=(0.091, 0.072, 0.206)),
        material=dark_steel,
        name="steam_socket",
    )
    for x in (-0.056, 0.056):
        for y in (-0.090, 0.090):
            body.visual(
                Cylinder(radius=0.012, length=0.012),
                origin=Origin(xyz=(x, y, 0.006)),
                material=rubber,
                name=f"foot_{'l' if x < 0 else 'r'}_{'rear' if y < 0 else 'front'}",
            )
    body.inertial = Inertial.from_geometry(
        Box((0.170, 0.240, 0.320)),
        mass=9.8,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.146, 0.054, 0.018)),
        origin=Origin(xyz=(0.0, 0.027, 0.009)),
        material=dark_steel,
        name="tray_body",
    )
    drip_tray.visual(
        Box((0.136, 0.048, 0.003)),
        origin=Origin(xyz=(0.0, 0.028, 0.0195)),
        material=steel,
        name="tray_surface",
    )
    drip_tray.visual(
        Box((0.146, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.049, 0.015)),
        material=matte_gray,
        name="tray_front",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.146, 0.054, 0.030)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.027, 0.015)),
    )

    service_door = model.part("service_door")
    service_door.visual(
        Box((0.012, 0.158, 0.144)),
        origin=Origin(xyz=(0.006, 0.079, 0.0)),
        material=body_black,
        name="door_panel",
    )
    service_door.visual(
        Box((0.004, 0.144, 0.128)),
        origin=Origin(xyz=(0.014, 0.079, 0.0)),
        material=fascia_black,
        name="door_trim",
    )
    service_door.visual(
        Box((0.020, 0.016, 0.086)),
        origin=Origin(xyz=(0.024, 0.122, -0.006)),
        material=matte_gray,
        name="door_handle",
    )
    service_door.visual(
        Box((0.004, 0.050, 0.024)),
        origin=Origin(xyz=(0.014, 0.052, 0.038)),
        material=screen_glass,
        name="door_window",
    )
    service_door.inertial = Inertial.from_geometry(
        Box((0.020, 0.160, 0.144)),
        mass=1.15,
        origin=Origin(xyz=(0.010, 0.080, 0.0)),
    )

    hopper_lid = model.part("hopper_lid")
    hopper_lid.visual(
        Box((0.108, 0.082, 0.008)),
        origin=Origin(xyz=(0.0, 0.041, 0.004)),
        material=matte_gray,
        name="lid_panel",
    )
    hopper_lid.visual(
        Box((0.050, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.067, 0.013)),
        material=steel,
        name="lid_handle",
    )
    hopper_lid.inertial = Inertial.from_geometry(
        Box((0.108, 0.082, 0.018)),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.041, 0.009)),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="wand_collar",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="wand_arm",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.110),
        origin=Origin(xyz=(0.030, 0.0, -0.055)),
        material=steel,
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.0035, length=0.012),
        origin=Origin(xyz=(0.034, 0.0, -0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="wand_tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.045, 0.016, 0.126)),
        mass=0.18,
        origin=Origin(xyz=(0.022, 0.0, -0.050)),
    )

    model.articulation(
        "body_to_drip_tray",
        ArticulationType.FIXED,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.0, 0.102, 0.0)),
    )
    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_door,
        origin=Origin(xyz=(0.082, -0.087, 0.110)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "body_to_hopper_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hopper_lid,
        origin=Origin(xyz=(0.0, -0.020, 0.316)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.2,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.097, 0.072, 0.206)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.8,
            lower=0.0,
            upper=1.15,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    drip_tray = object_model.get_part("drip_tray")
    service_door = object_model.get_part("service_door")
    hopper_lid = object_model.get_part("hopper_lid")
    steam_wand = object_model.get_part("steam_wand")

    door_joint = object_model.get_articulation("body_to_service_door")
    lid_joint = object_model.get_articulation("body_to_hopper_lid")
    wand_joint = object_model.get_articulation("body_to_steam_wand")

    main_lower = body.get_visual("main_lower")
    door_panel = service_door.get_visual("door_panel")
    door_handle = service_door.get_visual("door_handle")
    hopper_seat = body.get_visual("hopper_seat")
    lid_panel = hopper_lid.get_visual("lid_panel")
    lid_handle = hopper_lid.get_visual("lid_handle")
    tray_mount = body.get_visual("tray_mount")
    tray_body = drip_tray.get_visual("tray_body")
    tray_surface = drip_tray.get_visual("tray_surface")
    brew_nose = body.get_visual("brew_nose")
    steam_socket = body.get_visual("steam_socket")
    wand_collar = steam_wand.get_visual("wand_collar")
    wand_tube = steam_wand.get_visual("wand_tube")
    main_upper = body.get_visual("main_upper")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_coplanar_surfaces(service_door, body, reason="service door closes flush against the right-side service opening")
    ctx.allow_coplanar_surfaces(hopper_lid, body, reason="hopper lid closes flush on the bean hopper seat")
    ctx.allow_coplanar_surfaces(drip_tray, body, reason="drip tray docks flush against the tray mount")

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        service_door,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=door_panel,
        negative_elem=main_lower,
    )
    ctx.expect_overlap(service_door, body, axes="yz", min_overlap=0.10)

    ctx.expect_gap(
        hopper_lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lid_panel,
        negative_elem=hopper_seat,
    )
    ctx.expect_origin_distance(hopper_lid, body, axes="x", max_dist=0.005)

    ctx.expect_gap(
        drip_tray,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tray_body,
        negative_elem=tray_mount,
    )
    ctx.expect_overlap(drip_tray, body, axes="xy", min_overlap=0.02)
    ctx.expect_origin_distance(drip_tray, body, axes="x", max_dist=0.002)

    ctx.expect_contact(steam_wand, body, elem_a=wand_collar, elem_b=steam_socket)
    ctx.expect_gap(
        steam_wand,
        body,
        axis="x",
        min_gap=0.010,
        positive_elem=wand_tube,
        negative_elem=main_upper,
    )

    ctx.expect_gap(
        body,
        drip_tray,
        axis="z",
        min_gap=0.090,
        positive_elem=brew_nose,
        negative_elem=tray_surface,
    )

    with ctx.pose({door_joint: 1.20}):
        ctx.expect_gap(
            service_door,
            body,
            axis="x",
            min_gap=0.030,
            positive_elem=door_handle,
            negative_elem=main_lower,
        )

    with ctx.pose({lid_joint: 1.10}):
        ctx.expect_gap(
            hopper_lid,
            body,
            axis="z",
            min_gap=0.040,
            positive_elem=lid_handle,
            negative_elem=hopper_seat,
        )

    with ctx.pose({wand_joint: 1.05}):
        ctx.expect_contact(steam_wand, body, elem_a=wand_collar, elem_b=steam_socket)
        ctx.expect_gap(
            steam_wand,
            body,
            axis="x",
            min_gap=0.010,
            positive_elem=wand_tube,
            negative_elem=main_upper,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
