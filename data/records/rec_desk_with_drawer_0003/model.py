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
    model = ArticulatedObject(name="standing_desk_with_hanging_pedestal", assets=ASSETS)

    top_laminate = model.material("top_laminate", rgba=(0.66, 0.52, 0.38, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    glide_nylon = model.material("glide_nylon", rgba=(0.17, 0.17, 0.18, 1.0))
    pedestal_paint = model.material("pedestal_paint", rgba=(0.83, 0.85, 0.87, 1.0))
    drawer_shadow = model.material("drawer_shadow", rgba=(0.14, 0.15, 0.16, 1.0))
    slide_steel = model.material("slide_steel", rgba=(0.66, 0.69, 0.73, 1.0))
    caster_rubber = model.material("caster_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    desktop_width = 1.82
    desktop_depth = 0.86
    desktop_thickness = 0.040
    frame_tube = 0.045
    desktop_center_z = 1.03
    frame_center_z = desktop_center_z - (desktop_thickness / 2.0) - (frame_tube / 2.0)
    leg_x = 0.7975
    leg_y = 0.3625
    leg_bottom_z = 0.028
    leg_top_z = frame_center_z - (frame_tube / 2.0)
    leg_height = leg_top_z - leg_bottom_z
    leg_center_z = leg_bottom_z + (leg_height / 2.0)

    desk_frame = model.part("desk_frame")
    desk_frame.visual(
        Box((desktop_width, desktop_depth, desktop_thickness)),
        origin=Origin(xyz=(0.0, 0.0, desktop_center_z)),
        material=top_laminate,
        name="desktop",
    )
    desk_frame.visual(
        Box((1.64, frame_tube, frame_tube)),
        origin=Origin(xyz=(0.0, -leg_y, frame_center_z)),
        material=frame_steel,
        name="front_apron",
    )
    desk_frame.visual(
        Box((1.64, frame_tube, frame_tube)),
        origin=Origin(xyz=(0.0, leg_y, frame_center_z)),
        material=frame_steel,
        name="rear_apron",
    )
    desk_frame.visual(
        Box((frame_tube, 0.77, frame_tube)),
        origin=Origin(xyz=(-leg_x, 0.0, frame_center_z)),
        material=frame_steel,
        name="left_apron",
    )
    desk_frame.visual(
        Box((frame_tube, 0.77, frame_tube)),
        origin=Origin(xyz=(leg_x, 0.0, frame_center_z)),
        material=frame_steel,
        name="right_apron",
    )
    desk_frame.visual(
        Box((frame_tube, 0.46, frame_tube)),
        origin=Origin(xyz=(0.39, -0.04, frame_center_z)),
        material=frame_steel,
        name="pedestal_support_inner",
    )
    desk_frame.visual(
        Box((frame_tube, 0.46, frame_tube)),
        origin=Origin(xyz=(0.65, -0.04, frame_center_z)),
        material=frame_steel,
        name="pedestal_support_outer",
    )

    leg_positions = {
        "front_left": (-leg_x, -leg_y),
        "front_right": (leg_x, -leg_y),
        "rear_left": (-leg_x, leg_y),
        "rear_right": (leg_x, leg_y),
    }
    for leg_name, (x_pos, y_pos) in leg_positions.items():
        desk_frame.visual(
            Box((frame_tube, frame_tube, leg_height)),
            origin=Origin(xyz=(x_pos, y_pos, leg_center_z)),
            material=frame_steel,
            name=f"leg_{leg_name}",
        )
        desk_frame.visual(
            Cylinder(radius=0.006, length=0.018),
            origin=Origin(xyz=(x_pos, y_pos, 0.019)),
            material=frame_steel,
            name=f"foot_stem_{leg_name}",
        )
        desk_frame.visual(
            Cylinder(radius=0.026, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, 0.005)),
            material=glide_nylon,
            name=f"foot_pad_{leg_name}",
        )
    desk_frame.inertial = Inertial.from_geometry(
        Box((desktop_width, desktop_depth, 1.05)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
    )

    pedestal = model.part("pedestal")
    pedestal_vertical_shift = 0.329
    pedestal.visual(
        Box((0.40, 0.56, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009 - pedestal_vertical_shift)),
        material=pedestal_paint,
        name="shell_top",
    )
    pedestal.visual(
        Box((0.016, 0.56, 0.50)),
        origin=Origin(xyz=(-0.192, 0.0, -0.25 - pedestal_vertical_shift)),
        material=pedestal_paint,
        name="shell_left",
    )
    pedestal.visual(
        Box((0.016, 0.56, 0.50)),
        origin=Origin(xyz=(0.192, 0.0, -0.25 - pedestal_vertical_shift)),
        material=pedestal_paint,
        name="shell_right",
    )
    pedestal.visual(
        Box((0.368, 0.014, 0.50)),
        origin=Origin(xyz=(0.0, 0.273, -0.25 - pedestal_vertical_shift)),
        material=pedestal_paint,
        name="shell_back",
    )
    pedestal.visual(
        Box((0.368, 0.546, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.491 - pedestal_vertical_shift)),
        material=pedestal_paint,
        name="shell_bottom",
    )
    pedestal.visual(
        Box((0.032, 0.42, pedestal_vertical_shift)),
        origin=Origin(xyz=(-0.13, -0.04, -(pedestal_vertical_shift / 2.0))),
        material=frame_steel,
        name="hanger_inner",
    )
    pedestal.visual(
        Box((0.032, 0.42, pedestal_vertical_shift)),
        origin=Origin(xyz=(0.13, -0.04, -(pedestal_vertical_shift / 2.0))),
        material=frame_steel,
        name="hanger_outer",
    )
    pedestal.visual(
        Box((0.368, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.26, -0.028 - pedestal_vertical_shift)),
        material=pedestal_paint,
        name="top_drawer_stop",
    )
    pedestal.visual(
        Box((0.368, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.26, -0.165 - pedestal_vertical_shift)),
        material=pedestal_paint,
        name="file_drawer_stop",
    )

    slide_length = 0.50
    pedestal.visual(
        Box((0.010, slide_length, 0.034)),
        origin=Origin(xyz=(-0.179, 0.0, -0.082 - pedestal_vertical_shift)),
        material=slide_steel,
        name="top_left_outer_slide",
    )
    pedestal.visual(
        Box((0.010, slide_length, 0.034)),
        origin=Origin(xyz=(0.179, 0.0, -0.082 - pedestal_vertical_shift)),
        material=slide_steel,
        name="top_right_outer_slide",
    )
    pedestal.visual(
        Box((0.010, slide_length, 0.046)),
        origin=Origin(xyz=(-0.179, 0.0, -0.301 - pedestal_vertical_shift)),
        material=slide_steel,
        name="file_left_outer_slide",
    )
    pedestal.visual(
        Box((0.010, slide_length, 0.046)),
        origin=Origin(xyz=(0.179, 0.0, -0.301 - pedestal_vertical_shift)),
        material=slide_steel,
        name="file_right_outer_slide",
    )

    pedestal.visual(
        Box((0.36, 0.52, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.511 - pedestal_vertical_shift)),
        material=frame_steel,
        name="caster_base",
    )
    caster_positions = {
        "front_left": (-0.15, -0.21),
        "front_right": (0.15, -0.21),
        "rear_left": (-0.15, 0.21),
        "rear_right": (0.15, 0.21),
    }
    for caster_name, (x_pos, y_pos) in caster_positions.items():
        pedestal.visual(
            Cylinder(radius=0.006, length=0.030),
            origin=Origin(xyz=(x_pos, y_pos, -0.537 - pedestal_vertical_shift)),
            material=slide_steel,
            name=f"caster_stem_{caster_name}",
        )
        pedestal.visual(
            Box((0.022, 0.012, 0.078)),
            origin=Origin(xyz=(x_pos, y_pos, -0.561 - pedestal_vertical_shift)),
            material=frame_steel,
            name=f"caster_fork_{caster_name}",
        )
        pedestal.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(
                xyz=(x_pos, y_pos, -0.618 - pedestal_vertical_shift),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=caster_rubber,
            name=f"caster_wheel_{caster_name}",
        )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.40, 0.56, 0.965)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.4825)),
    )

    top_drawer = model.part("top_drawer")
    top_drawer.visual(
        Box((0.356, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.241, 0.047)),
        material=pedestal_paint,
        name="face_top",
    )
    top_drawer.visual(
        Box((0.356, 0.018, 0.046)),
        origin=Origin(xyz=(0.0, -0.241, -0.032)),
        material=pedestal_paint,
        name="face_bottom",
    )
    top_drawer.visual(
        Box((0.109, 0.018, 0.048)),
        origin=Origin(xyz=(-0.1245, -0.241, 0.015)),
        material=pedestal_paint,
        name="face_left",
    )
    top_drawer.visual(
        Box((0.109, 0.018, 0.048)),
        origin=Origin(xyz=(0.1245, -0.241, 0.015)),
        material=pedestal_paint,
        name="face_right",
    )
    top_drawer.visual(
        Box((0.138, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, -0.220, 0.015)),
        material=drawer_shadow,
        name="pull_back",
    )
    top_drawer.visual(
        Box((0.004, 0.006, 0.032)),
        origin=Origin(xyz=(-0.069, -0.229, 0.015)),
        material=drawer_shadow,
        name="pull_wall_left",
    )
    top_drawer.visual(
        Box((0.004, 0.006, 0.032)),
        origin=Origin(xyz=(0.069, -0.229, 0.015)),
        material=drawer_shadow,
        name="pull_wall_right",
    )
    top_drawer.visual(
        Box((0.320, 0.490, 0.006)),
        origin=Origin(xyz=(0.0, 0.013, -0.036)),
        material=pedestal_paint,
        name="body_bottom",
    )
    top_drawer.visual(
        Box((0.010, 0.490, 0.084)),
        origin=Origin(xyz=(-0.155, 0.013, -0.002)),
        material=pedestal_paint,
        name="body_left",
    )
    top_drawer.visual(
        Box((0.010, 0.490, 0.084)),
        origin=Origin(xyz=(0.155, 0.013, -0.002)),
        material=pedestal_paint,
        name="body_right",
    )
    top_drawer.visual(
        Box((0.300, 0.008, 0.084)),
        origin=Origin(xyz=(0.0, 0.254, -0.002)),
        material=pedestal_paint,
        name="body_back",
    )
    top_drawer.visual(
        Box((0.008, slide_length, 0.034)),
        origin=Origin(xyz=(-0.169, 0.0, -0.005)),
        material=slide_steel,
        name="left_slide",
    )
    top_drawer.visual(
        Box((0.008, slide_length, 0.034)),
        origin=Origin(xyz=(0.169, 0.0, -0.005)),
        material=slide_steel,
        name="right_slide",
    )
    top_drawer.inertial = Inertial.from_geometry(
        Box((0.356, 0.508, 0.110)),
        mass=4.0,
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
    )

    file_drawer = model.part("file_drawer")
    file_drawer.visual(
        Box((0.356, 0.018, 0.056)),
        origin=Origin(xyz=(0.0, -0.241, 0.123)),
        material=pedestal_paint,
        name="face_top",
    )
    file_drawer.visual(
        Box((0.356, 0.018, 0.166)),
        origin=Origin(xyz=(0.0, -0.241, -0.068)),
        material=pedestal_paint,
        name="face_bottom",
    )
    file_drawer.visual(
        Box((0.098, 0.018, 0.034)),
        origin=Origin(xyz=(-0.129, -0.241, 0.066)),
        material=pedestal_paint,
        name="face_left",
    )
    file_drawer.visual(
        Box((0.098, 0.018, 0.034)),
        origin=Origin(xyz=(0.129, -0.241, 0.066)),
        material=pedestal_paint,
        name="face_right",
    )
    file_drawer.visual(
        Box((0.160, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, -0.220, 0.066)),
        material=drawer_shadow,
        name="pull_back",
    )
    file_drawer.visual(
        Box((0.004, 0.006, 0.034)),
        origin=Origin(xyz=(-0.080, -0.229, 0.066)),
        material=drawer_shadow,
        name="pull_wall_left",
    )
    file_drawer.visual(
        Box((0.004, 0.006, 0.034)),
        origin=Origin(xyz=(0.080, -0.229, 0.066)),
        material=drawer_shadow,
        name="pull_wall_right",
    )
    file_drawer.visual(
        Box((0.320, 0.490, 0.006)),
        origin=Origin(xyz=(0.0, 0.013, -0.113)),
        material=pedestal_paint,
        name="body_bottom",
    )
    file_drawer.visual(
        Box((0.010, 0.490, 0.234)),
        origin=Origin(xyz=(-0.155, 0.013, -0.021)),
        material=pedestal_paint,
        name="body_left",
    )
    file_drawer.visual(
        Box((0.010, 0.490, 0.234)),
        origin=Origin(xyz=(0.155, 0.013, -0.021)),
        material=pedestal_paint,
        name="body_right",
    )
    file_drawer.visual(
        Box((0.300, 0.008, 0.234)),
        origin=Origin(xyz=(0.0, 0.254, -0.021)),
        material=pedestal_paint,
        name="body_back",
    )
    file_drawer.visual(
        Box((0.008, slide_length, 0.046)),
        origin=Origin(xyz=(-0.169, 0.0, 0.0)),
        material=slide_steel,
        name="left_slide",
    )
    file_drawer.visual(
        Box((0.008, slide_length, 0.046)),
        origin=Origin(xyz=(0.169, 0.0, 0.0)),
        material=slide_steel,
        name="right_slide",
    )
    file_drawer.visual(
        Box((0.006, 0.440, 0.006)),
        origin=Origin(xyz=(-0.105, -0.012, 0.074)),
        material=slide_steel,
        name="file_rail_left",
    )
    file_drawer.visual(
        Box((0.006, 0.440, 0.006)),
        origin=Origin(xyz=(0.105, -0.012, 0.074)),
        material=slide_steel,
        name="file_rail_right",
    )
    file_drawer.inertial = Inertial.from_geometry(
        Box((0.356, 0.508, 0.302)),
        mass=6.0,
        origin=Origin(xyz=(0.0, -0.005, 0.0)),
    )

    model.articulation(
        "desk_to_pedestal",
        ArticulationType.FIXED,
        parent=desk_frame,
        child=pedestal,
        origin=Origin(xyz=(0.52, -0.05, leg_top_z)),
    )
    model.articulation(
        "pedestal_to_top_drawer",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=top_drawer,
        origin=Origin(xyz=(0.0, -0.03, -0.087 - pedestal_vertical_shift)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.8,
            lower=0.0,
            upper=0.42,
        ),
    )
    model.articulation(
        "pedestal_to_file_drawer",
        ArticulationType.PRISMATIC,
        parent=pedestal,
        child=file_drawer,
        origin=Origin(xyz=(0.0, -0.03, -0.321 - pedestal_vertical_shift)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.8,
            lower=0.0,
            upper=0.42,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk_frame = object_model.get_part("desk_frame")
    pedestal = object_model.get_part("pedestal")
    top_drawer = object_model.get_part("top_drawer")
    file_drawer = object_model.get_part("file_drawer")

    top_slide = object_model.get_articulation("pedestal_to_top_drawer")
    file_slide = object_model.get_articulation("pedestal_to_file_drawer")

    desktop = desk_frame.get_visual("desktop")
    support_inner = desk_frame.get_visual("pedestal_support_inner")
    support_outer = desk_frame.get_visual("pedestal_support_outer")
    leg_front_left = desk_frame.get_visual("leg_front_left")
    leg_front_right = desk_frame.get_visual("leg_front_right")
    leg_rear_left = desk_frame.get_visual("leg_rear_left")
    leg_rear_right = desk_frame.get_visual("leg_rear_right")
    foot_stem_front_left = desk_frame.get_visual("foot_stem_front_left")
    foot_stem_front_right = desk_frame.get_visual("foot_stem_front_right")
    foot_stem_rear_left = desk_frame.get_visual("foot_stem_rear_left")
    foot_stem_rear_right = desk_frame.get_visual("foot_stem_rear_right")
    foot_pad_front_left = desk_frame.get_visual("foot_pad_front_left")
    foot_pad_front_right = desk_frame.get_visual("foot_pad_front_right")
    foot_pad_rear_left = desk_frame.get_visual("foot_pad_rear_left")
    foot_pad_rear_right = desk_frame.get_visual("foot_pad_rear_right")

    shell_top = pedestal.get_visual("shell_top")
    hanger_inner = pedestal.get_visual("hanger_inner")
    hanger_outer = pedestal.get_visual("hanger_outer")
    top_stop = pedestal.get_visual("top_drawer_stop")
    file_stop = pedestal.get_visual("file_drawer_stop")
    top_left_outer_slide = pedestal.get_visual("top_left_outer_slide")
    top_right_outer_slide = pedestal.get_visual("top_right_outer_slide")
    file_left_outer_slide = pedestal.get_visual("file_left_outer_slide")
    file_right_outer_slide = pedestal.get_visual("file_right_outer_slide")
    caster_base = pedestal.get_visual("caster_base")

    top_face_top = top_drawer.get_visual("face_top")
    top_left_slide = top_drawer.get_visual("left_slide")
    top_right_slide = top_drawer.get_visual("right_slide")
    top_pull_back = top_drawer.get_visual("pull_back")

    file_face_top = file_drawer.get_visual("face_top")
    file_left_slide = file_drawer.get_visual("left_slide")
    file_right_slide = file_drawer.get_visual("right_slide")
    file_pull_back = file_drawer.get_visual("pull_back")
    file_rail_left = file_drawer.get_visual("file_rail_left")
    file_rail_right = file_drawer.get_visual("file_rail_right")
    file_body_left = file_drawer.get_visual("body_left")
    file_body_right = file_drawer.get_visual("body_right")
    top_body_bottom = top_drawer.get_visual("body_bottom")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
    ctx.expect_within(pedestal, desk_frame, axes="xy", inner_elem=shell_top, outer_elem=desktop)
    ctx.expect_contact(pedestal, desk_frame, elem_a=hanger_inner, elem_b=support_inner)
    ctx.expect_contact(pedestal, desk_frame, elem_a=hanger_outer, elem_b=support_outer)
    ctx.expect_overlap(pedestal, desk_frame, axes="xy", elem_a=caster_base, elem_b=desktop, min_overlap=0.12)
    ctx.expect_within(top_drawer, top_drawer, axes="xz", inner_elem=top_pull_back)
    ctx.expect_within(file_drawer, file_drawer, axes="xz", inner_elem=file_pull_back)
    ctx.expect_contact(desk_frame, desk_frame, elem_a=leg_front_left, elem_b=foot_stem_front_left)
    ctx.expect_contact(desk_frame, desk_frame, elem_a=foot_stem_front_left, elem_b=foot_pad_front_left)
    ctx.expect_contact(desk_frame, desk_frame, elem_a=leg_front_right, elem_b=foot_stem_front_right)
    ctx.expect_contact(desk_frame, desk_frame, elem_a=foot_stem_front_right, elem_b=foot_pad_front_right)
    ctx.expect_contact(desk_frame, desk_frame, elem_a=leg_rear_left, elem_b=foot_stem_rear_left)
    ctx.expect_contact(desk_frame, desk_frame, elem_a=foot_stem_rear_left, elem_b=foot_pad_rear_left)
    ctx.expect_contact(desk_frame, desk_frame, elem_a=leg_rear_right, elem_b=foot_stem_rear_right)
    ctx.expect_contact(desk_frame, desk_frame, elem_a=foot_stem_rear_right, elem_b=foot_pad_rear_right)
    ctx.expect_gap(
        top_drawer,
        top_drawer,
        axis="y",
        min_gap=0.004,
        max_gap=0.010,
        positive_elem=top_pull_back,
        negative_elem=top_face_top,
    )
    ctx.expect_gap(
        file_drawer,
        file_drawer,
        axis="y",
        min_gap=0.004,
        max_gap=0.010,
        positive_elem=file_pull_back,
        negative_elem=file_face_top,
    )
    ctx.expect_within(file_drawer, file_drawer, axes="xy", inner_elem=file_rail_left)
    ctx.expect_within(file_drawer, file_drawer, axes="xy", inner_elem=file_rail_right)
    ctx.expect_gap(
        top_drawer,
        file_drawer,
        axis="z",
        min_gap=0.03,
        positive_elem=top_body_bottom,
        negative_elem=file_face_top,
    )

    ctx.expect_gap(
        top_drawer,
        pedestal,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=top_left_slide,
        negative_elem=top_left_outer_slide,
    )
    ctx.expect_gap(
        pedestal,
        top_drawer,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=top_right_outer_slide,
        negative_elem=top_right_slide,
    )
    ctx.expect_gap(
        file_drawer,
        pedestal,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=file_left_slide,
        negative_elem=file_left_outer_slide,
    )
    ctx.expect_gap(
        pedestal,
        file_drawer,
        axis="x",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=file_right_outer_slide,
        negative_elem=file_right_slide,
    )

    ctx.expect_gap(
        pedestal,
        top_drawer,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=top_stop,
        negative_elem=top_face_top,
    )
    ctx.expect_gap(
        pedestal,
        file_drawer,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=file_stop,
        negative_elem=file_face_top,
    )

    with ctx.pose({top_slide: 0.42}):
        ctx.expect_gap(
            pedestal,
            top_drawer,
            axis="y",
            min_gap=0.30,
            positive_elem=top_stop,
            negative_elem=top_face_top,
        )
        ctx.expect_overlap(
            top_drawer,
            pedestal,
            axes="yz",
            min_overlap=0.01,
            elem_a=top_left_slide,
            elem_b=top_left_outer_slide,
        )
        ctx.expect_overlap(
            top_drawer,
            pedestal,
            axes="yz",
            min_overlap=0.01,
            elem_a=top_right_slide,
            elem_b=top_right_outer_slide,
        )

    with ctx.pose({file_slide: 0.42}):
        ctx.expect_gap(
            pedestal,
            file_drawer,
            axis="y",
            min_gap=0.30,
            positive_elem=file_stop,
            negative_elem=file_face_top,
        )
        ctx.expect_overlap(
            file_drawer,
            pedestal,
            axes="yz",
            min_overlap=0.015,
            elem_a=file_left_slide,
            elem_b=file_left_outer_slide,
        )
        ctx.expect_overlap(
            file_drawer,
            pedestal,
            axes="yz",
            min_overlap=0.015,
            elem_a=file_right_slide,
            elem_b=file_right_outer_slide,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
