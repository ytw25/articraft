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
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prosumer_espresso_machine")

    stainless = model.material("stainless", rgba=(0.79, 0.81, 0.82, 1.0))
    brushed_dark = model.material("brushed_dark", rgba=(0.28, 0.30, 0.32, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.11, 0.12, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    steam_tip_metal = model.material("steam_tip_metal", rgba=(0.70, 0.71, 0.73, 1.0))
    tray_steel = model.material("tray_steel", rgba=(0.72, 0.73, 0.75, 1.0))

    body = model.part("machine_body")
    body.visual(
        Box((0.34, 0.43, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=stainless,
        name="main_shell",
    )
    body.visual(
        Box((0.34, 0.16, 0.02)),
        origin=Origin(xyz=(0.0, -0.125, 0.35)),
        material=stainless,
        name="top_panel",
    )
    body.visual(
        Box((0.34, 0.06, 0.02)),
        origin=Origin(xyz=(0.0, 0.165, 0.35)),
        material=stainless,
        name="rear_hinge_strip",
    )
    body.visual(
        Box((0.32, 0.020, 0.21)),
        origin=Origin(xyz=(0.0, -0.205, 0.195)),
        material=brushed_dark,
        name="front_panel",
    )
    body.visual(
        Box((0.03, 0.39, 0.06)),
        origin=Origin(xyz=(-0.155, 0.0, 0.03)),
        material=stainless,
        name="left_base_leg",
    )
    body.visual(
        Box((0.03, 0.39, 0.06)),
        origin=Origin(xyz=(0.155, 0.0, 0.03)),
        material=stainless,
        name="right_base_leg",
    )
    body.visual(
        Box((0.28, 0.12, 0.06)),
        origin=Origin(xyz=(0.0, 0.155, 0.03)),
        material=stainless,
        name="rear_base_bridge",
    )
    body.visual(
        Box((0.14, 0.020, 0.045)),
        origin=Origin(xyz=(0.0, -0.195, 0.135)),
        material=brushed_dark,
        name="group_head_mount",
    )
    body.visual(
        Cylinder(radius=0.033, length=0.045),
        origin=Origin(xyz=(0.0, -0.2375, 0.17), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_dark,
        name="group_head",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.028),
        origin=Origin(xyz=(0.11, -0.220, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_dark,
        name="knob_panel_boss",
    )
    body.visual(
        Box((0.030, 0.020, 0.052)),
        origin=Origin(xyz=(0.148, -0.205, 0.205)),
        material=brushed_dark,
        name="wand_support_block",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.159, -0.205, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_dark,
        name="wand_pivot_boss",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.34, 0.43, 0.37)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
    )

    drip_tray = model.part("drip_tray")
    grate_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.246, 0.198),
            0.004,
            slot_size=(0.030, 0.005),
            pitch=(0.040, 0.016),
            frame=0.010,
            corner_radius=0.006,
            slot_angle_deg=0.0,
            stagger=True,
        ),
        "espresso_tray_grate",
    )
    drip_tray.visual(
        Box((0.252, 0.208, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=matte_black,
        name="tray_body",
    )
    drip_tray.visual(
        Box((0.252, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.099, 0.049)),
        material=brushed_dark,
        name="tray_front_lip",
    )
    drip_tray.visual(
        grate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=tray_steel,
        name="tray_grate",
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.252, 0.208, 0.052)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )
    model.articulation(
        "body_to_drip_tray",
        ArticulationType.FIXED,
        parent=body,
        child=drip_tray,
        origin=Origin(xyz=(0.0, -0.102, 0.004)),
    )

    cup_rail = model.part("cup_warming_rail")
    rail_geom = wire_from_points(
        [
            (-0.125, -0.055, 0.0),
            (-0.125, -0.055, 0.042),
            (-0.125, -0.140, 0.042),
            (0.125, -0.140, 0.042),
            (0.125, -0.055, 0.042),
            (0.125, -0.055, 0.0),
        ],
        radius=0.004,
        radial_segments=14,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.018,
        corner_segments=8,
    )
    cup_rail.visual(
        mesh_from_geometry(rail_geom, "espresso_cup_warming_rail"),
        material=steam_tip_metal,
        name="rail_loop",
    )
    for x_pos in (-0.120, 0.120):
        for y_pos in (-0.140, -0.055):
            cup_rail.visual(
                Cylinder(radius=0.0032, length=0.046),
                origin=Origin(xyz=(x_pos, y_pos, 0.019)),
                material=steam_tip_metal,
                name=f"rail_post_{'l' if x_pos < 0.0 else 'r'}_{'rear' if y_pos < -0.10 else 'front'}",
            )
    cup_rail.inertial = Inertial.from_geometry(
        Box((0.26, 0.16, 0.05)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.04, 0.021)),
    )
    model.articulation(
        "body_to_cup_rail",
        ArticulationType.FIXED,
        parent=body,
        child=cup_rail,
        origin=Origin(xyz=(0.0, 0.0, 0.364)),
    )

    lid = model.part("reservoir_lid")
    lid.visual(
        Box((0.248, 0.160, 0.012)),
        origin=Origin(xyz=(0.0, -0.080, 0.006)),
        material=stainless,
        name="lid_panel",
    )
    lid.visual(
        Box((0.090, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, -0.145, 0.014)),
        material=brushed_dark,
        name="lid_pull",
    )
    lid.visual(
        Box((0.248, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.006, 0.010)),
        material=stainless,
        name="lid_hinge_leaf",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.248, 0.160, 0.022)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.080, 0.011)),
    )
    model.articulation(
        "body_to_reservoir_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.148, 0.36)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=1.28,
        ),
    )

    steam_knob = model.part("steam_knob")
    steam_knob.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=brushed_dark,
        name="knob_shaft",
    )
    steam_knob.visual(
        Cylinder(radius=0.022, length=0.020),
        origin=Origin(xyz=(0.0, -0.020, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    steam_knob.visual(
        Box((0.022, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, -0.0305, 0.016)),
        material=tray_steel,
        name="knob_indicator",
    )
    steam_knob.inertial = Inertial.from_geometry(
        Box((0.050, 0.046, 0.050)),
        mass=0.10,
        origin=Origin(xyz=(0.0, -0.023, 0.0)),
    )
    model.articulation(
        "body_to_steam_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_knob,
        origin=Origin(xyz=(0.11, -0.234, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=6.0,
            lower=0.0,
            upper=2.6,
        ),
    )

    steam_wand = model.part("steam_wand")
    wand_geom = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, -0.004, -0.028),
            (0.0, -0.006, -0.080),
            (0.0, -0.002, -0.136),
            (0.0, 0.020, -0.160),
        ],
        radius=0.0042,
        samples_per_segment=12,
        radial_segments=14,
        cap_ends=True,
    )
    steam_wand.visual(
        Cylinder(radius=0.0075, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_dark,
        name="wand_pivot",
    )
    steam_wand.visual(
        mesh_from_geometry(wand_geom, "espresso_steam_wand"),
        material=steam_tip_metal,
        name="wand_tube",
    )
    steam_wand.inertial = Inertial.from_geometry(
        Box((0.020, 0.035, 0.165)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.010, -0.080)),
    )
    model.articulation(
        "body_to_steam_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=steam_wand,
        origin=Origin(xyz=(0.170, -0.205, 0.205)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=0.0,
            upper=1.18,
        ),
    )

    portafilter = model.part("portafilter")
    portafilter.visual(
        Cylinder(radius=0.031, length=0.022),
        origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_dark,
        name="pf_collar",
    )
    portafilter.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=Origin(xyz=(0.0, 0.016, -0.065)),
        material=tray_steel,
        name="basket",
    )
    portafilter.visual(
        Box((0.040, 0.020, 0.060)),
        origin=Origin(xyz=(0.020, 0.016, -0.030)),
        material=brushed_dark,
        name="neck_block",
    )
    portafilter.visual(
        Box((0.125, 0.020, 0.018)),
        origin=Origin(xyz=(0.090, 0.020, -0.058)),
        material=matte_black,
        name="handle",
    )
    portafilter.inertial = Inertial.from_geometry(
        Box((0.165, 0.050, 0.080)),
        mass=0.7,
        origin=Origin(xyz=(0.070, 0.018, -0.052)),
    )
    model.articulation(
        "body_to_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=portafilter,
        origin=Origin(xyz=(0.0, -0.283, 0.17)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.80,
            upper=0.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("machine_body")
    tray = object_model.get_part("drip_tray")
    lid = object_model.get_part("reservoir_lid")
    portafilter = object_model.get_part("portafilter")
    steam_wand = object_model.get_part("steam_wand")

    lid_joint = object_model.get_articulation("body_to_reservoir_lid")
    portafilter_joint = object_model.get_articulation("body_to_portafilter")
    wand_joint = object_model.get_articulation("body_to_steam_wand")

    ctx.expect_overlap(
        tray,
        body,
        axes="x",
        elem_a="tray_grate",
        elem_b="group_head",
        min_overlap=0.055,
        name="group head stays centered over the drip tray",
    )
    ctx.expect_gap(
        body,
        tray,
        axis="z",
        positive_elem="group_head",
        negative_elem="tray_grate",
        min_gap=0.085,
        max_gap=0.150,
        name="group head sits above the removable drip tray",
    )

    with ctx.pose({portafilter_joint: 0.0}):
        ctx.expect_overlap(
            portafilter,
            body,
            axes="x",
            elem_a="basket",
            elem_b="group_head",
            min_overlap=0.045,
            name="portafilter basket stays under the group head centerline",
        )
        ctx.expect_gap(
            body,
            portafilter,
            axis="z",
            positive_elem="group_head",
            negative_elem="basket",
            min_gap=0.010,
            max_gap=0.055,
            name="portafilter basket hangs below the group head",
        )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="top_panel",
            max_gap=0.002,
            max_penetration=0.0,
            name="reservoir lid rests flush on the top panel when closed",
        )

    with ctx.pose({lid_joint: 1.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_pull",
            negative_elem="top_panel",
            min_gap=0.060,
            name="reservoir lid lifts clearly above the machine when opened",
        )

    rest_wand_aabb = ctx.part_element_world_aabb(steam_wand, elem="wand_tube")
    with ctx.pose({wand_joint: 0.95}):
        open_wand_aabb = ctx.part_element_world_aabb(steam_wand, elem="wand_tube")
    ctx.check(
        "steam wand swings forward from its side pivot",
        rest_wand_aabb is not None
        and open_wand_aabb is not None
        and open_wand_aabb[0][1] < rest_wand_aabb[0][1] - 0.020,
        details=f"rest={rest_wand_aabb}, open={open_wand_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
