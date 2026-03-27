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
    mesh_from_geometry,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    def rect_loop(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
        hx = width / 2.0
        hy = depth / 2.0
        return [
            (-hx, -hy, z),
            (hx, -hy, z),
            (hx, hy, z),
            (-hx, hy, z),
        ]

    model = ArticulatedObject(name="bean_to_cup_coffee_machine", assets=ASSETS)

    body_silver = model.material("body_silver", rgba=(0.71, 0.72, 0.73, 1.0))
    matte_black = model.material("matte_black", rgba=(0.13, 0.13, 0.14, 1.0))
    display_black = model.material("display_black", rgba=(0.05, 0.06, 0.07, 1.0))
    chrome = model.material("chrome", rgba=(0.84, 0.85, 0.87, 1.0))
    smoke = model.material("smoke", rgba=(0.24, 0.26, 0.28, 0.58))

    body = model.part("main_body")
    body.visual(
        Box((0.24, 0.28, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=body_silver,
        name="main_shell",
    )
    body.visual(
        Box((0.20, 0.22, 0.10)),
        origin=Origin(xyz=(0.0, 0.02, 0.35)),
        material=body_silver,
        name="top_shoulder",
    )
    body.visual(
        Box((0.18, 0.035, 0.10)),
        origin=Origin(xyz=(0.0, -0.1575, 0.31)),
        material=matte_black,
        name="control_panel",
    )
    body.visual(
        Box((0.09, 0.012, 0.04)),
        origin=Origin(xyz=(-0.02, -0.181, 0.327)),
        material=display_black,
        name="display_screen",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.055, -0.183, 0.295), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="control_dial",
    )
    body.visual(
        Box((0.11, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, -0.1675, 0.205)),
        material=matte_black,
        name="brew_head",
    )
    body.visual(
        Box((0.09, 0.015, 0.05)),
        origin=Origin(xyz=(0.0, -0.1325, 0.205)),
        material=matte_black,
        name="spout_support",
    )
    body.visual(
        Box((0.18, 0.02, 0.055)),
        origin=Origin(xyz=(0.0, -0.13, 0.048)),
        material=matte_black,
        name="tray_support",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.014),
        origin=Origin(xyz=(0.127, -0.095, 0.215), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="steam_wand_mount",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.24, 0.28, 0.40)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    hopper_geom = section_loft(
        [
            rect_loop(0.12, 0.13, 0.0),
            rect_loop(0.106, 0.118, 0.055),
            rect_loop(0.095, 0.104, 0.098),
        ]
    )
    hopper_mesh = mesh_from_geometry(hopper_geom, ASSETS.mesh_path("bean_hopper.obj"))

    hopper = model.part("bean_hopper")
    hopper.visual(hopper_mesh, material=smoke, name="bean_hopper_shell")
    hopper.visual(
        Cylinder(radius=0.028, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=matte_black,
        name="grinder_collar",
    )
    hopper.inertial = Inertial.from_geometry(
        Box((0.12, 0.13, 0.10)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    lid = model.part("hopper_lid")
    lid.visual(
        Box((0.11, 0.104, 0.008)),
        origin=Origin(xyz=(0.0, -0.052, 0.004)),
        material=smoke,
        name="lid_panel",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.082),
        origin=Origin(xyz=(0.0, -0.088, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="lid_handle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.11, 0.104, 0.025)),
        mass=0.14,
        origin=Origin(xyz=(0.0, -0.052, 0.0125)),
    )

    dispenser = model.part("dispenser_head")
    dispenser.visual(
        Box((0.11, 0.055, 0.032)),
        origin=Origin(xyz=(0.0, -0.0275, 0.0)),
        material=matte_black,
        name="spout_housing",
    )
    dispenser.visual(
        Box((0.096, 0.008, 0.014)),
        origin=Origin(xyz=(0.0, -0.059, 0.006)),
        material=chrome,
        name="spout_trim",
    )
    dispenser.visual(
        Cylinder(radius=0.007, length=0.03),
        origin=Origin(xyz=(-0.026, -0.046, -0.029), rpy=(0.0, 0.0, 0.0)),
        material=chrome,
        name="left_nozzle",
    )
    dispenser.visual(
        Cylinder(radius=0.007, length=0.03),
        origin=Origin(xyz=(0.026, -0.046, -0.029), rpy=(0.0, 0.0, 0.0)),
        material=chrome,
        name="right_nozzle",
    )
    dispenser.inertial = Inertial.from_geometry(
        Box((0.11, 0.06, 0.06)),
        mass=0.22,
        origin=Origin(xyz=(0.0, -0.03, -0.014)),
    )

    tray = model.part("drip_tray")
    tray.visual(
        Box((0.17, 0.12, 0.018)),
        origin=Origin(xyz=(0.0, -0.075, 0.009)),
        material=chrome,
        name="tray_plate",
    )
    tray.visual(
        Box((0.16, 0.015, 0.05)),
        origin=Origin(xyz=(0.0, -0.0075, 0.025)),
        material=matte_black,
        name="tray_back",
    )
    tray.visual(
        Box((0.17, 0.01, 0.015)),
        origin=Origin(xyz=(0.0, -0.13, 0.0075)),
        material=matte_black,
        name="tray_front_lip",
    )
    for index, y_pos in enumerate((-0.035, -0.052, -0.069, -0.086, -0.103), start=1):
        tray.visual(
            Box((0.145, 0.006, 0.005)),
            origin=Origin(xyz=(0.0, y_pos, 0.0205)),
            material=matte_black,
            name=f"grate_bar_{index}",
        )
    tray.inertial = Inertial.from_geometry(
        Box((0.17, 0.13, 0.06)),
        mass=0.45,
        origin=Origin(xyz=(0.0, -0.075, 0.03)),
    )

    wand = model.part("steam_wand")
    wand.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="wand_collar",
    )
    wand.visual(
        Cylinder(radius=0.006, length=0.14),
        origin=Origin(xyz=(0.014, 0.0, -0.07)),
        material=chrome,
        name="wand_tube",
    )
    wand.visual(
        Cylinder(radius=0.0045, length=0.036),
        origin=Origin(xyz=(0.014, -0.018, -0.132), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="wand_nozzle",
    )
    wand.visual(
        Cylinder(radius=0.006, length=0.01),
        origin=Origin(xyz=(0.014, -0.033, -0.132), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="wand_tip_sleeve",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.03, 0.04, 0.15)),
        mass=0.18,
        origin=Origin(xyz=(0.015, -0.01, -0.065)),
    )

    model.articulation(
        "body_to_hopper",
        ArticulationType.FIXED,
        parent=body,
        child=hopper,
        origin=Origin(xyz=(0.0, 0.055, 0.40)),
    )
    model.articulation(
        "hopper_to_lid",
        ArticulationType.REVOLUTE,
        parent=hopper,
        child=lid,
        origin=Origin(xyz=(0.0, 0.052, 0.098)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-1.25, upper=0.0),
    )
    model.articulation(
        "body_to_dispenser",
        ArticulationType.FIXED,
        parent=body,
        child=dispenser,
        origin=Origin(xyz=(0.0, -0.14, 0.205)),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -0.14, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.15, lower=0.0, upper=0.09),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(xyz=(0.134, -0.095, 0.215)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("main_body")
    hopper = object_model.get_part("bean_hopper")
    lid = object_model.get_part("hopper_lid")
    dispenser = object_model.get_part("dispenser_head")
    tray = object_model.get_part("drip_tray")
    wand = object_model.get_part("steam_wand")

    lid_joint = object_model.get_articulation("hopper_to_lid")
    tray_joint = object_model.get_articulation("body_to_tray")
    wand_joint = object_model.get_articulation("body_to_steam_wand")

    top_shoulder = body.get_visual("top_shoulder")
    spout_support = body.get_visual("spout_support")
    tray_support = body.get_visual("tray_support")
    wand_mount = body.get_visual("steam_wand_mount")

    hopper_shell = hopper.get_visual("bean_hopper_shell")
    lid_panel = lid.get_visual("lid_panel")
    lid_handle = lid.get_visual("lid_handle")
    spout_housing = dispenser.get_visual("spout_housing")
    tray_back = tray.get_visual("tray_back")
    wand_collar = wand.get_visual("wand_collar")
    wand_nozzle = wand.get_visual("wand_nozzle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(body, wand, reason="steam wand pivot collar nests around the side pivot mount")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(hopper, body, axes="xy", min_overlap=0.05)
    ctx.expect_gap(
        hopper,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=hopper_shell,
        negative_elem=top_shoulder,
    )
    ctx.expect_contact(hopper, body, elem_a=hopper_shell, elem_b=top_shoulder)

    ctx.expect_overlap(lid, hopper, axes="xy", min_overlap=0.05)
    ctx.expect_gap(
        lid,
        hopper,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=lid_panel,
        negative_elem=hopper_shell,
    )

    ctx.expect_contact(body, dispenser, elem_a=spout_support, elem_b=spout_housing)
    ctx.expect_gap(
        body,
        dispenser,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=spout_support,
        negative_elem=spout_housing,
    )
    ctx.expect_origin_distance(dispenser, tray, axes="x", max_dist=0.015)
    ctx.expect_gap(dispenser, tray, axis="z", min_gap=0.08)

    ctx.expect_contact(body, tray, elem_a=tray_support, elem_b=tray_back)
    ctx.expect_gap(
        body,
        tray,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tray_support,
        negative_elem=tray_back,
    )
    ctx.expect_overlap(tray, body, axes="xz", min_overlap=0.03)

    ctx.expect_contact(body, wand, elem_a=wand_mount, elem_b=wand_collar)
    ctx.expect_gap(
        wand,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=wand_collar,
        negative_elem=wand_mount,
    )
    ctx.expect_overlap(wand, body, axes="yz", min_overlap=0.02)

    with ctx.pose({lid_joint: -1.05}):
        ctx.expect_gap(
            lid,
            hopper,
            axis="z",
            min_gap=0.05,
            positive_elem=lid_handle,
            negative_elem=hopper_shell,
        )
        ctx.expect_origin_distance(lid, hopper, axes="x", max_dist=0.001)

    with ctx.pose({tray_joint: 0.08}):
        ctx.expect_gap(
            body,
            tray,
            axis="y",
            min_gap=0.07,
            positive_elem=tray_support,
            negative_elem=tray_back,
        )
        ctx.expect_origin_distance(dispenser, tray, axes="x", max_dist=0.015)

    with ctx.pose({wand_joint: 1.0}):
        ctx.expect_gap(wand, body, axis="x", min_gap=0.003, positive_elem=wand_nozzle)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
