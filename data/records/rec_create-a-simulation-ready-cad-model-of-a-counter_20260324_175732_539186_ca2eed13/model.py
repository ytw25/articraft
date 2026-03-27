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

    body_dark = model.material("body_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    body_black = model.material("body_black", rgba=(0.08, 0.09, 0.10, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    silver = model.material("silver", rgba=(0.73, 0.74, 0.76, 1.0))
    steel = model.material("steel", rgba=(0.61, 0.63, 0.66, 1.0))
    smoke = model.material("smoke", rgba=(0.38, 0.40, 0.43, 0.72))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.22, 0.24, 0.03)),
        origin=Origin(xyz=(-0.02, 0.0, 0.015)),
        material=body_black,
        name="rear_base_strip",
    )
    chassis.visual(
        Box((0.15, 0.22, 0.32)),
        origin=Origin(xyz=(-0.055, 0.0, 0.19)),
        material=body_dark,
        name="rear_body",
    )
    chassis.visual(
        Box((0.12, 0.22, 0.22)),
        origin=Origin(xyz=(0.08, 0.0, 0.29)),
        material=graphite,
        name="upper_front_body",
    )
    chassis.visual(
        Box((0.12, 0.03, 0.12)),
        origin=Origin(xyz=(0.03, 0.095, 0.09)),
        material=body_dark,
        name="left_lower_column",
    )
    chassis.visual(
        Box((0.12, 0.03, 0.12)),
        origin=Origin(xyz=(0.03, -0.095, 0.09)),
        material=body_dark,
        name="right_lower_column",
    )
    chassis.visual(
        Box((0.10, 0.16, 0.004)),
        origin=Origin(xyz=(0.09, 0.0, 0.033)),
        material=body_black,
        name="tray_floor",
    )
    chassis.visual(
        Box((0.02, 0.15, 0.09)),
        origin=Origin(xyz=(0.03, 0.0, 0.11)),
        material=body_black,
        name="cup_recess_back",
    )
    chassis.visual(
        Box((0.025, 0.19, 0.20)),
        origin=Origin(xyz=(0.1275, 0.0, 0.29)),
        material=silver,
        name="front_face",
    )
    chassis.visual(
        Box((0.010, 0.12, 0.055)),
        origin=Origin(xyz=(0.145, 0.0, 0.335)),
        material=body_black,
        name="display_bezel",
    )
    chassis.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.146, 0.0, 0.265), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="control_knob",
    )
    chassis.visual(
        Box((0.050, 0.085, 0.045)),
        origin=Origin(xyz=(0.120, 0.0, 0.210)),
        material=silver,
        name="brew_head",
    )
    chassis.visual(
        Cylinder(radius=0.006, length=0.044),
        origin=Origin(xyz=(0.138, 0.020, 0.167)),
        material=steel,
        name="left_spout",
    )
    chassis.visual(
        Cylinder(radius=0.006, length=0.044),
        origin=Origin(xyz=(0.138, -0.020, 0.167)),
        material=steel,
        name="right_spout",
    )
    chassis.visual(
        Box((0.20, 0.16, 0.015)),
        origin=Origin(xyz=(-0.01, 0.0, 0.4075)),
        material=body_black,
        name="top_tray",
    )
    chassis.visual(
        Box((0.004, 0.024, 0.028)),
        origin=Origin(xyz=(0.141, -0.075, 0.205)),
        material=silver,
        name="wand_mount",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((0.28, 0.24, 0.42)),
        mass=8.2,
        origin=Origin(xyz=(0.0, 0.0, 0.21)),
    )

    bean_hopper = model.part("bean_hopper")
    bean_hopper.visual(
        Box((0.122, 0.12, 0.116)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=smoke,
        name="hopper_body",
    )
    bean_hopper.visual(
        Box((0.116, 0.116, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=graphite,
        name="lid_seat",
    )
    bean_hopper.visual(
        Box((0.010, 0.120, 0.010)),
        origin=Origin(xyz=(-0.066, 0.0, 0.111)),
        material=body_black,
        name="hinge_mount",
    )
    bean_hopper.inertial = Inertial.from_geometry(
        Box((0.122, 0.12, 0.12)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    hopper_lid = model.part("hopper_lid")
    hopper_lid.visual(
        Box((0.13, 0.128, 0.006)),
        origin=Origin(xyz=(0.075, 0.0, 0.003)),
        material=smoke,
        name="lid_panel",
    )
    hopper_lid.visual(
        Cylinder(radius=0.005, length=0.126),
        origin=Origin(xyz=(0.005, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_black,
        name="hinge_barrel",
    )
    hopper_lid.visual(
        Box((0.018, 0.050, 0.008)),
        origin=Origin(xyz=(0.114, 0.0, 0.009)),
        material=silver,
        name="lid_handle",
    )
    hopper_lid.inertial = Inertial.from_geometry(
        Box((0.14, 0.13, 0.016)),
        mass=0.16,
        origin=Origin(xyz=(0.072, 0.0, 0.008)),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.10, 0.15, 0.018)),
        origin=Origin(xyz=(0.05, 0.0, 0.009)),
        material=body_black,
        name="tray_base",
    )
    drip_tray.visual(
        Box((0.085, 0.14, 0.004)),
        origin=Origin(xyz=(0.0525, 0.0, 0.020)),
        material=steel,
        name="tray_grid",
    )
    drip_tray.visual(
        Box((0.002, 0.10, 0.025)),
        origin=Origin(xyz=(0.101, 0.0, 0.0125)),
        material=silver,
        name="tray_face",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.10, 0.15, 0.025)),
        mass=0.38,
        origin=Origin(xyz=(0.05, 0.0, 0.0125)),
    )

    steam_wand = model.part("steam_wand")
    steam_wand.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=silver,
        name="wand_pivot",
    )
    steam_wand.visual(
        Cylinder(radius=0.0042, length=0.022),
        origin=Origin(xyz=(0.011, 0.0, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="wand_elbow",
    )
    steam_wand.visual(
        Cylinder(radius=0.0038, length=0.078),
        origin=Origin(xyz=(0.022, 0.0, -0.055)),
        material=steel,
        name="wand_tube",
    )
    steam_wand.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.022, 0.0, -0.102), rpy=(0.0, 0.35, 0.0)),
        material=steel,
        name="wand_tip",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.04, 0.02, 0.13)),
        mass=0.12,
        origin=Origin(xyz=(0.020, 0.0, -0.052)),
    )

    model.articulation(
        "chassis_to_bean_hopper",
        ArticulationType.FIXED,
        parent=chassis,
        child=bean_hopper,
        origin=Origin(xyz=(-0.025, 0.0, 0.415)),
    )
    model.articulation(
        "bean_hopper_to_lid",
        ArticulationType.REVOLUTE,
        parent=bean_hopper,
        child=hopper_lid,
        origin=Origin(xyz=(-0.061, 0.0, 0.120)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=-1.2,
            upper=0.0,
        ),
    )
    model.articulation(
        "chassis_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=drip_tray,
        origin=Origin(xyz=(0.04, 0.0, 0.035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.25,
            lower=0.0,
            upper=0.10,
        ),
    )
    model.articulation(
        "chassis_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=steam_wand,
        origin=Origin(xyz=(0.147, -0.075, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=1.8,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    bean_hopper = object_model.get_part("bean_hopper")
    hopper_lid = object_model.get_part("hopper_lid")
    drip_tray = object_model.get_part("drip_tray")
    steam_wand = object_model.get_part("steam_wand")

    lid_hinge = object_model.get_articulation("bean_hopper_to_lid")
    tray_slide = object_model.get_articulation("chassis_to_drip_tray")
    wand_swing = object_model.get_articulation("chassis_to_steam_wand")

    top_tray = chassis.get_visual("top_tray")
    front_face = chassis.get_visual("front_face")
    tray_floor = chassis.get_visual("tray_floor")
    wand_mount = chassis.get_visual("wand_mount")

    hopper_body = bean_hopper.get_visual("hopper_body")
    lid_seat = bean_hopper.get_visual("lid_seat")
    hinge_mount = bean_hopper.get_visual("hinge_mount")

    lid_panel = hopper_lid.get_visual("lid_panel")
    hinge_barrel = hopper_lid.get_visual("hinge_barrel")
    lid_handle = hopper_lid.get_visual("lid_handle")

    tray_base = drip_tray.get_visual("tray_base")
    tray_grid = drip_tray.get_visual("tray_grid")
    tray_face = drip_tray.get_visual("tray_face")

    wand_pivot = steam_wand.get_visual("wand_pivot")
    wand_tip = steam_wand.get_visual("wand_tip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(bean_hopper, chassis, axes="xy", min_overlap=0.04)
    ctx.expect_gap(
        bean_hopper,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=hopper_body,
        negative_elem=top_tray,
    )

    ctx.expect_overlap(hopper_lid, bean_hopper, axes="xy", min_overlap=0.08)
    ctx.expect_gap(
        hopper_lid,
        bean_hopper,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=lid_panel,
        negative_elem=lid_seat,
    )
    ctx.expect_gap(
        hopper_lid,
        bean_hopper,
        axis="z",
        min_gap=0.003,
        max_gap=0.008,
        positive_elem=hinge_barrel,
        negative_elem=hinge_mount,
    )

    ctx.expect_within(drip_tray, chassis, axes="y")
    ctx.expect_gap(
        drip_tray,
        chassis,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tray_base,
        negative_elem=tray_floor,
    )
    ctx.expect_gap(
        drip_tray,
        chassis,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=tray_face,
        negative_elem=front_face,
    )

    ctx.expect_gap(
        steam_wand,
        chassis,
        axis="x",
        max_gap=0.005,
        max_penetration=0.0,
        positive_elem=wand_pivot,
        negative_elem=wand_mount,
    )
    ctx.expect_overlap(
        steam_wand,
        chassis,
        axes="yz",
        min_overlap=0.007,
        elem_a=wand_pivot,
        elem_b=wand_mount,
    )

    with ctx.pose({lid_hinge: -1.1}):
        ctx.expect_overlap(hopper_lid, bean_hopper, axes="y", min_overlap=0.10)
        ctx.expect_gap(
            hopper_lid,
            bean_hopper,
            axis="z",
            min_gap=0.05,
            positive_elem=lid_handle,
            negative_elem=lid_seat,
        )
    with ctx.pose({tray_slide: 0.08}):
        ctx.expect_gap(
            drip_tray,
            chassis,
            axis="x",
            min_gap=0.078,
            positive_elem=tray_face,
            negative_elem=front_face,
        )
        ctx.expect_gap(
            drip_tray,
            chassis,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=tray_base,
            negative_elem=tray_floor,
        )

    with ctx.pose({tray_slide: 0.08, wand_swing: 1.0}):
        ctx.expect_within(
            steam_wand,
            drip_tray,
            axes="xy",
            inner_elem=wand_tip,
            outer_elem=tray_grid,
        )
        ctx.expect_gap(
            steam_wand,
            drip_tray,
            axis="z",
            min_gap=0.03,
            positive_elem=wand_tip,
            negative_elem=tray_grid,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
