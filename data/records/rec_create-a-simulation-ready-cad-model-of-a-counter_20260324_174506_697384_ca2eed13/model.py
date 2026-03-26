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
    model = ArticulatedObject(
        name="countertop_bean_to_cup_coffee_machine",
        assets=ASSETS,
    )

    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.13, 0.14, 1.0))
    silver = model.material("silver", rgba=(0.75, 0.76, 0.78, 1.0))
    stainless = model.material("stainless", rgba=(0.70, 0.72, 0.74, 1.0))
    smoked = model.material("smoked_plastic", rgba=(0.28, 0.31, 0.34, 0.45))
    water_blue = model.material("water_blue", rgba=(0.60, 0.74, 0.86, 0.32))
    display_black = model.material("display_black", rgba=(0.04, 0.05, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.26, 0.38, 0.30)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=graphite,
        name="cabinet",
    )
    body.visual(
        Box((0.22, 0.18, 0.02)),
        origin=Origin(xyz=(0.0, -0.03, 0.31)),
        material=charcoal,
        name="top_deck",
    )
    body.visual(
        Box((0.19, 0.07, 0.09)),
        origin=Origin(
            xyz=(0.0, 0.153, 0.255),
            rpy=(-0.42, 0.0, 0.0),
        ),
        material=silver,
        name="control_fascia",
    )
    body.visual(
        Box((0.09, 0.012, 0.04)),
        origin=Origin(
            xyz=(0.0, 0.171, 0.267),
            rpy=(-0.42, 0.0, 0.0),
        ),
        material=display_black,
        name="display",
    )
    body.visual(
        Box((0.18, 0.012, 0.03)),
        origin=Origin(xyz=(0.0, 0.194, 0.06)),
        material=charcoal,
        name="front_base_trim",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.26, 0.38, 0.32)),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    hopper = model.part("hopper")
    hopper.visual(
        Box((0.16, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=smoked,
        name="hopper_bin",
    )
    hopper.visual(
        Box((0.17, 0.15, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=charcoal,
        name="hopper_rim",
    )
    hopper.inertial = Inertial.from_geometry(
        Box((0.16, 0.14, 0.10)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
    )

    water_tank = model.part("water_tank")
    water_tank.visual(
        Box((0.06, 0.15, 0.24)),
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        material=water_blue,
        name="tank_shell",
    )
    water_tank.visual(
        Cylinder(radius=0.018, length=0.02),
        origin=Origin(xyz=(0.03, 0.0, 0.13)),
        material=charcoal,
        name="tank_cap",
    )
    water_tank.inertial = Inertial.from_geometry(
        Box((0.06, 0.15, 0.24)),
        mass=1.2,
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
    )

    dispenser = model.part("dispenser")
    dispenser.visual(
        Box((0.10, 0.05, 0.07)),
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
        material=charcoal,
        name="dispenser_block",
    )
    dispenser.visual(
        Box((0.09, 0.008, 0.035)),
        origin=Origin(xyz=(0.0, 0.052, 0.002)),
        material=graphite,
        name="dispenser_face",
    )
    dispenser.visual(
        Cylinder(radius=0.005, length=0.05),
        origin=Origin(xyz=(-0.02, 0.025, -0.06)),
        material=stainless,
        name="left_nozzle",
    )
    dispenser.visual(
        Cylinder(radius=0.005, length=0.05),
        origin=Origin(xyz=(0.02, 0.025, -0.06)),
        material=stainless,
        name="right_nozzle",
    )
    dispenser.inertial = Inertial.from_geometry(
        Box((0.10, 0.05, 0.10)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.025, 0.0)),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.16, 0.14, 0.02)),
        origin=Origin(xyz=(0.0, 0.07, 0.0)),
        material=charcoal,
        name="tray_deck",
    )
    drip_tray.visual(
        Box((0.13, 0.11, 0.006)),
        origin=Origin(xyz=(0.0, 0.07, 0.012)),
        material=stainless,
        name="tray_grill",
    )
    drip_tray.visual(
        Box((0.16, 0.02, 0.05)),
        origin=Origin(xyz=(0.0, 0.13, 0.015)),
        material=silver,
        name="tray_front",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.16, 0.14, 0.05)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.07, 0.015)),
    )

    hopper_lid = model.part("hopper_lid")
    hopper_lid.visual(
        Box((0.166, 0.142, 0.008)),
        origin=Origin(xyz=(0.0, 0.071, 0.0)),
        material=charcoal,
        name="lid_panel",
    )
    hopper_lid.visual(
        Box((0.16, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.147, 0.006)),
        material=graphite,
        name="lid_front_lip",
    )
    hopper_lid.inertial = Inertial.from_geometry(
        Box((0.166, 0.142, 0.008)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.071, 0.0)),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.010, length=0.02),
        material=stainless,
        name="wand_joint",
    )
    steam_wand.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(
            xyz=(0.0, 0.016, -0.004),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=stainless,
        name="wand_connector",
    )
    steam_wand.visual(
        Cylinder(radius=0.005, length=0.12),
        origin=Origin(xyz=(0.0, 0.03, -0.06)),
        material=stainless,
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.004, length=0.02),
        origin=Origin(xyz=(0.0, 0.03, -0.128)),
        material=stainless,
        name="wand_tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.005, length=0.12),
        mass=0.2,
        origin=Origin(xyz=(0.0, 0.03, -0.06)),
    )

    model.articulation(
        "body_to_hopper",
        ArticulationType.FIXED,
        parent=body,
        child=hopper,
        origin=Origin(xyz=(0.0, -0.045, 0.32)),
    )
    model.articulation(
        "body_to_water_tank",
        ArticulationType.FIXED,
        parent=body,
        child=water_tank,
        origin=Origin(xyz=(0.13, -0.03, 0.16)),
    )
    model.articulation(
        "body_to_dispenser",
        ArticulationType.FIXED,
        parent=body,
        child=dispenser,
        origin=Origin(xyz=(0.0, 0.19, 0.215)),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.0, 0.19, 0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=0.0,
            upper=0.08,
        ),
    )
    model.articulation(
        "hopper_to_lid",
        ArticulationType.REVOLUTE,
        parent=hopper,
        child=hopper_lid,
        origin=Origin(xyz=(0.0, -0.07, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.116, 0.20, 0.215)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=1.5,
            lower=-0.95,
            upper=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    hopper = object_model.get_part("hopper")
    water_tank = object_model.get_part("water_tank")
    dispenser = object_model.get_part("dispenser")
    drip_tray = object_model.get_part("drip_tray")
    hopper_lid = object_model.get_part("hopper_lid")
    steam_wand = object_model.get_part("steam_wand")

    tray_slide = object_model.get_articulation("body_to_drip_tray")
    lid_hinge = object_model.get_articulation("hopper_to_lid")
    wand_swing = object_model.get_articulation("body_to_steam_wand")

    cabinet = body.get_visual("cabinet")
    top_deck = body.get_visual("top_deck")
    hopper_bin = hopper.get_visual("hopper_bin")
    tank_shell = water_tank.get_visual("tank_shell")
    dispenser_block = dispenser.get_visual("dispenser_block")
    left_nozzle = dispenser.get_visual("left_nozzle")
    tray_deck = drip_tray.get_visual("tray_deck")
    tray_grill = drip_tray.get_visual("tray_grill")
    lid_panel = hopper_lid.get_visual("lid_panel")
    lid_front_lip = hopper_lid.get_visual("lid_front_lip")
    wand_joint = steam_wand.get_visual("wand_joint")
    wand_tube = steam_wand.get_visual("wand_tube")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_contact(hopper, body, elem_a=hopper_bin, elem_b=top_deck)
    ctx.expect_gap(
        hopper,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=hopper_bin,
        negative_elem=top_deck,
    )
    ctx.expect_overlap(hopper, body, axes="xy", min_overlap=0.02)

    ctx.expect_contact(water_tank, body, elem_a=tank_shell, elem_b=cabinet)
    ctx.expect_gap(
        water_tank,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tank_shell,
        negative_elem=cabinet,
    )

    ctx.expect_contact(dispenser, body, elem_a=dispenser_block, elem_b=cabinet)
    ctx.expect_gap(
        dispenser,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=dispenser_block,
        negative_elem=cabinet,
    )
    ctx.expect_gap(
        dispenser,
        drip_tray,
        axis="z",
        min_gap=0.06,
        positive_elem=left_nozzle,
        negative_elem=tray_grill,
    )

    ctx.expect_gap(
        drip_tray,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tray_deck,
        negative_elem=cabinet,
    )
    ctx.expect_overlap(drip_tray, dispenser, axes="x", min_overlap=0.09)

    ctx.expect_contact(hopper_lid, hopper, elem_a=lid_panel, elem_b=hopper_bin)
    ctx.expect_gap(
        hopper_lid,
        hopper,
        axis="z",
        max_gap=0.005,
        max_penetration=0.0,
        positive_elem=lid_panel,
        negative_elem=hopper_bin,
    )

    ctx.expect_contact(steam_wand, body, elem_a=wand_joint, elem_b=cabinet)
    ctx.expect_gap(
        steam_wand,
        body,
        axis="y",
        min_gap=0.02,
        positive_elem=wand_tube,
        negative_elem=cabinet,
    )

    with ctx.pose({lid_hinge: 1.1}):
        ctx.expect_gap(
            hopper_lid,
            hopper,
            axis="z",
            min_gap=0.03,
            positive_elem=lid_front_lip,
            negative_elem=hopper_bin,
        )

    with ctx.pose({tray_slide: 0.08}):
        ctx.expect_gap(
            drip_tray,
            body,
            axis="y",
            min_gap=0.07,
            positive_elem=tray_deck,
            negative_elem=cabinet,
        )
        ctx.expect_overlap(drip_tray, dispenser, axes="x", min_overlap=0.09)

    with ctx.pose({wand_swing: -0.9}):
        ctx.expect_gap(
            steam_wand,
            body,
            axis="y",
            min_gap=0.01,
            positive_elem=wand_tube,
            negative_elem=cabinet,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
