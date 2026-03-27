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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mag_drill_tilting_adapter")

    tilt_axis_z = 0.086

    frame_gray = model.material("frame_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    machine_green = model.material("machine_green", rgba=(0.20, 0.42, 0.28, 1.0))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    clamp_black = model.material("clamp_black", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.260, 0.170, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=frame_gray,
        name="base_plate",
    )
    frame.visual(
        Box((0.170, 0.020, 0.028)),
        origin=Origin(xyz=(-0.020, -0.060, 0.036)),
        material=frame_gray,
        name="front_rail",
    )
    frame.visual(
        Box((0.170, 0.020, 0.028)),
        origin=Origin(xyz=(-0.020, 0.060, 0.036)),
        material=frame_gray,
        name="rear_rail",
    )
    frame.visual(
        Box((0.050, 0.120, 0.040)),
        origin=Origin(xyz=(-0.075, 0.000, 0.042)),
        material=frame_gray,
        name="left_spine",
    )
    frame.visual(
        Box((0.028, 0.018, 0.064)),
        origin=Origin(xyz=(0.000, -0.082, 0.054)),
        material=frame_gray,
        name="front_pivot_block",
    )
    frame.visual(
        Box((0.028, 0.018, 0.064)),
        origin=Origin(xyz=(0.000, 0.082, 0.054)),
        material=steel,
        name="rear_pivot_block",
    )
    frame.visual(
        Box((0.055, 0.060, 0.050)),
        origin=Origin(xyz=(0.105, 0.000, 0.047)),
        material=frame_gray,
        name="worm_pedestal",
    )
    frame.visual(
        Box((0.016, 0.028, 0.024)),
        origin=Origin(xyz=(0.085, 0.000, 0.063)),
        material=steel,
        name="left_bearing_block",
    )
    frame.visual(
        Box((0.016, 0.028, 0.024)),
        origin=Origin(xyz=(0.125, 0.000, 0.063)),
        material=steel,
        name="right_bearing_block",
    )

    platform = model.part("tilt_platform")
    platform.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.000, -0.064, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_trunnion",
    )
    platform.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.000, 0.064, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_trunnion",
    )
    platform.visual(
        Box((0.030, 0.110, 0.034)),
        origin=Origin(xyz=(-0.022, 0.000, 0.018)),
        material=frame_gray,
        name="tilt_web",
    )
    platform.visual(
        Box((0.076, 0.110, 0.026)),
        origin=Origin(xyz=(0.016, 0.000, 0.022)),
        material=steel,
        name="center_rib",
    )
    platform.visual(
        Box((0.210, 0.148, 0.012)),
        origin=Origin(xyz=(-0.008, 0.000, 0.034)),
        material=frame_gray,
        name="platform_plate",
    )
    platform.visual(
        Box((0.010, 0.020, 0.088)),
        origin=Origin(xyz=(0.050, 0.000, 0.000)),
        material=frame_gray,
        name="sector_bracket",
    )
    platform.visual(
        Box((0.016, 0.112, 0.032)),
        origin=Origin(xyz=(0.034, 0.000, 0.020)),
        material=steel,
        name="side_cheek",
    )
    tooth_angles = (-0.52, -0.40, -0.28, -0.16, -0.04, 0.08, 0.20, 0.32, 0.44)
    for index, angle in enumerate(tooth_angles):
        tooth_radius = 0.068
        platform.visual(
            Box((0.012, 0.018, 0.012)),
            origin=Origin(
                xyz=(0.060 * math.cos(angle), 0.000, 0.060 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=frame_gray,
            name=f"sector_back_{index}",
        )
        platform.visual(
            Box((0.012, 0.018, 0.010)),
            origin=Origin(
                xyz=(tooth_radius * math.cos(angle), 0.000, tooth_radius * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=steel,
            name=f"sector_tooth_{index}",
        )

    clamp = model.part("drill_clamp")
    clamp.visual(
        Box((0.132, 0.104, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=clamp_black,
        name="clamp_bottom",
    )
    clamp.visual(
        Box((0.010, 0.086, 0.036)),
        origin=Origin(xyz=(-0.056, 0.000, 0.023)),
        material=clamp_black,
        name="left_guide",
    )
    clamp.visual(
        Box((0.010, 0.086, 0.036)),
        origin=Origin(xyz=(0.056, 0.000, 0.023)),
        material=clamp_black,
        name="right_guide",
    )
    clamp.visual(
        Box((0.112, 0.094, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.042)),
        material=clamp_black,
        name="top_strap",
    )
    clamp.visual(
        Box((0.018, 0.112, 0.030)),
        origin=Origin(xyz=(-0.042, 0.000, 0.020)),
        material=clamp_black,
        name="rear_stop",
    )

    drill = model.part("magnetic_drill")
    drill.visual(
        Box((0.102, 0.074, 0.036)),
        origin=Origin(xyz=(0.000, 0.000, 0.018)),
        material=dark_steel,
        name="mag_base",
    )
    drill.visual(
        Cylinder(radius=0.017, length=0.214),
        origin=Origin(xyz=(-0.020, 0.000, 0.143)),
        material=dark_steel,
        name="column",
    )
    drill.visual(
        Box((0.066, 0.058, 0.094)),
        origin=Origin(xyz=(0.006, 0.000, 0.152)),
        material=machine_green,
        name="slide_carriage",
    )
    drill.visual(
        Cylinder(radius=0.031, length=0.108),
        origin=Origin(xyz=(0.026, 0.000, 0.208), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machine_green,
        name="motor_housing",
    )
    drill.visual(
        Box((0.050, 0.050, 0.060)),
        origin=Origin(xyz=(0.024, 0.000, 0.146)),
        material=machine_green,
        name="gearbox",
    )
    drill.visual(
        Cylinder(radius=0.011, length=0.064),
        origin=Origin(xyz=(0.030, 0.000, 0.096)),
        material=steel,
        name="spindle_nose",
    )
    drill.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(-0.035, 0.000, 0.166), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="feed_hub",
    )
    drill.visual(
        Cylinder(radius=0.0045, length=0.054),
        origin=Origin(xyz=(-0.062, 0.000, 0.184), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="feed_handle_top",
    )
    drill.visual(
        Cylinder(radius=0.0045, length=0.054),
        origin=Origin(xyz=(-0.062, 0.016, 0.157), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="feed_handle_front",
    )
    drill.visual(
        Cylinder(radius=0.0045, length=0.054),
        origin=Origin(xyz=(-0.062, -0.016, 0.157), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_black,
        name="feed_handle_rear",
    )

    worm = model.part("worm_drive")
    worm.visual(
        Cylinder(radius=0.005, length=0.082),
        origin=Origin(xyz=(0.030, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="shaft",
    )
    worm.visual(
        Cylinder(radius=0.010, length=0.044),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="worm_body",
    )
    for index, x_pos in enumerate((-0.018, -0.010, -0.002, 0.006, 0.014)):
        worm.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(x_pos, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"worm_thread_{index}",
        )
    worm.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(-0.010, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_bearing_collar",
    )
    worm.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.030, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_bearing_collar",
    )
    worm.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.050, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="handwheel_hub",
    )
    worm.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(0.068, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="handwheel_rim",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        worm.visual(
            Box((0.004, 0.004, 0.050)),
            origin=Origin(
                xyz=(0.068, -0.012 * math.sin(angle), 0.012 * math.cos(angle)),
                rpy=(angle, 0.0, 0.0),
            ),
            material=steel,
            name=f"handwheel_spoke_{index}",
        )

    model.articulation(
        "frame_to_platform",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=platform,
        origin=Origin(xyz=(0.000, 0.000, tilt_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.8,
            lower=-0.45,
            upper=0.30,
        ),
    )
    model.articulation(
        "platform_to_clamp",
        ArticulationType.FIXED,
        parent=platform,
        child=clamp,
        origin=Origin(xyz=(-0.008, 0.000, 0.040)),
    )
    model.articulation(
        "clamp_to_drill",
        ArticulationType.FIXED,
        parent=clamp,
        child=drill,
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
    )
    model.articulation(
        "frame_to_worm",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=worm,
        origin=Origin(xyz=(0.095, 0.000, tilt_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    platform = object_model.get_part("tilt_platform")
    clamp = object_model.get_part("drill_clamp")
    drill = object_model.get_part("magnetic_drill")
    worm = object_model.get_part("worm_drive")
    tilt = object_model.get_articulation("frame_to_platform")

    base_plate = frame.get_visual("base_plate")
    front_pivot_block = frame.get_visual("front_pivot_block")
    rear_pivot_block = frame.get_visual("rear_pivot_block")
    left_bearing_block = frame.get_visual("left_bearing_block")
    right_bearing_block = frame.get_visual("right_bearing_block")

    platform_plate = platform.get_visual("platform_plate")
    front_trunnion = platform.get_visual("front_trunnion")
    rear_trunnion = platform.get_visual("rear_trunnion")
    sector_tooth_rest = platform.get_visual("sector_tooth_4")
    sector_tooth_down = platform.get_visual("sector_tooth_1")
    sector_tooth_up = platform.get_visual("sector_tooth_6")

    clamp_bottom = clamp.get_visual("clamp_bottom")
    left_guide = clamp.get_visual("left_guide")
    right_guide = clamp.get_visual("right_guide")
    drill_base = drill.get_visual("mag_base")
    worm_body = worm.get_visual("worm_body")
    left_bearing_collar = worm.get_visual("left_bearing_collar")
    right_bearing_collar = worm.get_visual("right_bearing_collar")
    handwheel_rim = worm.get_visual("handwheel_rim")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.allow_overlap(
        platform,
        worm,
        reason="worm threads intentionally mesh into the side sector teeth across the tilt range",
    )
    ctx.expect_contact(frame, platform, elem_a=front_pivot_block, elem_b=front_trunnion)
    ctx.expect_contact(frame, platform, elem_a=rear_pivot_block, elem_b=rear_trunnion)
    ctx.expect_gap(
        clamp,
        platform,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=clamp_bottom,
        negative_elem=platform_plate,
    )
    ctx.expect_within(
        clamp,
        platform,
        axes="xy",
        inner_elem=clamp_bottom,
        outer_elem=platform_plate,
    )
    ctx.expect_gap(
        drill,
        clamp,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=drill_base,
        negative_elem=clamp_bottom,
    )
    ctx.expect_overlap(
        drill,
        clamp,
        axes="xy",
        min_overlap=0.060,
        elem_a=drill_base,
        elem_b=clamp_bottom,
    )
    ctx.expect_contact(clamp, drill, elem_a=left_guide, elem_b=drill_base)
    ctx.expect_contact(clamp, drill, elem_a=right_guide, elem_b=drill_base)
    ctx.expect_gap(
        drill,
        frame,
        axis="z",
        min_gap=0.080,
        positive_elem=drill_base,
        negative_elem=base_plate,
    )
    ctx.expect_contact(frame, worm, elem_a=left_bearing_block, elem_b=left_bearing_collar)
    ctx.expect_contact(frame, worm, elem_a=right_bearing_block, elem_b=right_bearing_collar)
    ctx.expect_overlap(
        platform,
        worm,
        axes="yz",
        min_overlap=0.008,
        elem_a=sector_tooth_rest,
        elem_b=worm_body,
    )
    ctx.expect_gap(
        worm,
        frame,
        axis="x",
        min_gap=0.010,
        positive_elem=handwheel_rim,
        negative_elem=right_bearing_block,
    )

    with ctx.pose({tilt: -0.40}):
        ctx.expect_gap(
            drill,
            frame,
            axis="z",
            min_gap=0.040,
            positive_elem=drill_base,
            negative_elem=base_plate,
        )
        ctx.expect_overlap(
            platform,
            worm,
            axes="yz",
            min_overlap=0.004,
            elem_a=sector_tooth_down,
            elem_b=worm_body,
        )
    with ctx.pose({tilt: 0.20}):
        ctx.expect_overlap(
            platform,
            worm,
            axes="yz",
            min_overlap=0.004,
            elem_a=sector_tooth_up,
            elem_b=worm_body,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
