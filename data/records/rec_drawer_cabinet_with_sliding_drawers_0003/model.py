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
    Sphere,
    TestContext,
    TestReport,
)

CABINET_WIDTH = 0.56
CABINET_DEPTH = 0.43
BODY_BOTTOM_Z = 0.13
BODY_HEIGHT = 0.69
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_HEIGHT

DRAWER_FACE_WIDTH = 0.494
DRAWER_BOX_WIDTH = 0.456
DRAWER_BOX_LENGTH = 0.354
DRAWER_FACE_THICKNESS = 0.018


def _add_caster_geometry(part, *, fork_material, wheel_material, steel_material) -> None:
    part.visual(
        Box((0.068, 0.068, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=steel_material,
        name="top_plate",
    )
    part.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=steel_material,
        name="swivel_stem",
    )
    part.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=fork_material,
        name="swivel_head",
    )
    part.visual(
        Box((0.012, 0.048, 0.040)),
        origin=Origin(xyz=(0.0, 0.016, -0.058)),
        material=fork_material,
        name="fork_left",
    )
    part.visual(
        Box((0.012, 0.048, 0.040)),
        origin=Origin(xyz=(0.0, -0.016, -0.058)),
        material=fork_material,
        name="fork_right",
    )
    part.visual(
        Box((0.010, 0.044, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        material=fork_material,
        name="axle_bridge",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.022),
        origin=Origin(
            xyz=(0.0, 0.0, -0.076),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=wheel_material,
        name="wheel",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(
            xyz=(0.0, 0.0, -0.076),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel_material,
        name="axle",
    )


def _add_slide_pack_geometry(part, *, slide_material, bearing_material) -> None:
    outer_length = 0.362
    middle_length = 0.340
    outer_y = 0.253
    middle_y = 0.247
    rail_center_x = 0.016
    slide_levels = {
        "upper": 0.018,
        "lower": -0.018,
    }

    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        y_outer = sign * outer_y
        y_middle = sign * middle_y
        for level_name, z_offset in slide_levels.items():
            level_prefix = "" if level_name == "upper" else f"{level_name}_"
            part.visual(
                Box((outer_length, 0.010, 0.014)),
                origin=Origin(xyz=(rail_center_x, y_outer, z_offset)),
                material=slide_material,
                name=f"{side_name}_{level_prefix}outer_web",
            )
            part.visual(
                Box((outer_length, 0.005, 0.004)),
                origin=Origin(xyz=(rail_center_x, sign * 0.2495, z_offset + 0.006)),
                material=slide_material,
                name=f"{side_name}_{level_prefix}outer_top_flange",
            )
            part.visual(
                Box((outer_length, 0.005, 0.004)),
                origin=Origin(xyz=(rail_center_x, sign * 0.2495, z_offset - 0.006)),
                material=slide_material,
                name=f"{side_name}_{level_prefix}outer_bottom_flange",
            )
            part.visual(
                Box((middle_length, 0.008, 0.010)),
                origin=Origin(xyz=(0.005, y_middle, z_offset)),
                material=slide_material,
                name=f"{side_name}_{level_prefix}middle_slide",
            )
            for index, x_pos in enumerate((-0.120, -0.040, 0.040, 0.120), start=1):
                part.visual(
                    Sphere(radius=0.0018),
                    origin=Origin(xyz=(x_pos, sign * 0.2505, z_offset + 0.0035)),
                    material=bearing_material,
                    name=f"{side_name}_{level_prefix}bearing_top_{index}",
                )
                part.visual(
                    Sphere(radius=0.0018),
                    origin=Origin(xyz=(x_pos, sign * 0.2505, z_offset - 0.0035)),
                    material=bearing_material,
                    name=f"{side_name}_{level_prefix}bearing_bottom_{index}",
                )

    part.visual(
        Box((0.020, 0.508, 0.080)),
        origin=Origin(xyz=(-0.175, 0.0, 0.0)),
        material=slide_material,
        name="rear_tie_bar",
    )


def _add_drawer_geometry(
    part,
    *,
    face_height: float,
    face_material,
    box_material,
    slide_material,
) -> tuple[float, float]:
    box_height = face_height - 0.018
    face_center_x = 0.206
    box_center_x = 0.025
    pocket_width = 0.310
    pocket_height = min(max(face_height * 0.34, 0.028), 0.055)
    side_frame_width = (DRAWER_FACE_WIDTH - pocket_width) * 0.5
    top_frame_height = (face_height - pocket_height) * 0.5

    part.visual(
        Box((DRAWER_BOX_LENGTH, DRAWER_BOX_WIDTH, 0.008)),
        origin=Origin(xyz=(box_center_x, 0.0, -box_height * 0.5 + 0.004)),
        material=box_material,
        name="drawer_bottom",
    )
    part.visual(
        Box((DRAWER_BOX_LENGTH, 0.010, box_height)),
        origin=Origin(xyz=(box_center_x, 0.223, 0.0)),
        material=box_material,
        name="drawer_left_side",
    )
    part.visual(
        Box((DRAWER_BOX_LENGTH, 0.010, box_height)),
        origin=Origin(xyz=(box_center_x, -0.223, 0.0)),
        material=box_material,
        name="drawer_right_side",
    )
    part.visual(
        Box((0.008, DRAWER_BOX_WIDTH, box_height)),
        origin=Origin(xyz=(-0.148, 0.0, 0.0)),
        material=box_material,
        name="drawer_back",
    )

    part.visual(
        Box((DRAWER_FACE_THICKNESS, side_frame_width, face_height)),
        origin=Origin(xyz=(face_center_x, 0.202, 0.0)),
        material=face_material,
        name="face_left_frame",
    )
    part.visual(
        Box((DRAWER_FACE_THICKNESS, side_frame_width, face_height)),
        origin=Origin(xyz=(face_center_x, -0.202, 0.0)),
        material=face_material,
        name="face_right_frame",
    )
    part.visual(
        Box((DRAWER_FACE_THICKNESS, pocket_width, top_frame_height)),
        origin=Origin(xyz=(face_center_x, 0.0, face_height * 0.5 - top_frame_height * 0.5)),
        material=face_material,
        name="face_top_frame",
    )
    part.visual(
        Box((DRAWER_FACE_THICKNESS, pocket_width, top_frame_height)),
        origin=Origin(xyz=(face_center_x, 0.0, -face_height * 0.5 + top_frame_height * 0.5)),
        material=face_material,
        name="face_bottom_frame",
    )

    part.visual(
        Box((0.010, pocket_width, pocket_height)),
        origin=Origin(xyz=(0.165, 0.0, 0.0)),
        material=box_material,
        name="pocket_back",
    )
    part.visual(
        Box((0.027, 0.010, pocket_height)),
        origin=Origin(xyz=(0.1835, pocket_width * 0.5 + 0.005, 0.0)),
        material=box_material,
        name="pocket_left_wall",
    )
    part.visual(
        Box((0.027, 0.010, pocket_height)),
        origin=Origin(xyz=(0.1835, -pocket_width * 0.5 - 0.005, 0.0)),
        material=box_material,
        name="pocket_right_wall",
    )
    part.visual(
        Box((0.027, pocket_width + 0.020, 0.006)),
        origin=Origin(xyz=(0.1835, 0.0, pocket_height * 0.5 + 0.003)),
        material=box_material,
        name="pocket_top_wall",
    )
    part.visual(
        Box((0.027, pocket_width + 0.020, 0.006)),
        origin=Origin(xyz=(0.1835, 0.0, -pocket_height * 0.5 - 0.003)),
        material=box_material,
        name="pocket_bottom_wall",
    )

    part.visual(
        Box((0.308, 0.016, 0.010)),
        origin=Origin(xyz=(0.010, 0.248, 0.018)),
        material=slide_material,
        name="left_inner_rail",
    )
    part.visual(
        Box((0.308, 0.016, 0.010)),
        origin=Origin(xyz=(0.010, -0.248, 0.018)),
        material=slide_material,
        name="right_inner_rail",
    )
    part.visual(
        Box((0.308, 0.016, 0.010)),
        origin=Origin(xyz=(0.010, 0.248, -0.018)),
        material=slide_material,
        name="left_lower_inner_rail",
    )
    part.visual(
        Box((0.308, 0.016, 0.010)),
        origin=Origin(xyz=(0.010, -0.248, -0.018)),
        material=slide_material,
        name="right_lower_inner_rail",
    )
    part.visual(
        Box((0.308, 0.012, 0.010)),
        origin=Origin(xyz=(0.010, 0.234, 0.018)),
        material=slide_material,
        name="left_upper_mount_strip",
    )
    part.visual(
        Box((0.308, 0.012, 0.010)),
        origin=Origin(xyz=(0.010, -0.234, 0.018)),
        material=slide_material,
        name="right_upper_mount_strip",
    )
    part.visual(
        Box((0.308, 0.012, 0.010)),
        origin=Origin(xyz=(0.010, 0.234, -0.018)),
        material=slide_material,
        name="left_lower_mount_strip",
    )
    part.visual(
        Box((0.308, 0.012, 0.010)),
        origin=Origin(xyz=(0.010, -0.234, -0.018)),
        material=slide_material,
        name="right_lower_mount_strip",
    )
    return pocket_width, pocket_height


def _add_handle_geometry(
    part,
    *,
    pocket_width: float,
    handle_material,
    bracket_material,
) -> None:
    bar_length = pocket_width - 0.062
    bar_offset = bar_length * 0.5 - 0.030
    part.visual(
        Cylinder(radius=0.005, length=bar_length),
        origin=Origin(
            xyz=(0.176, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_material,
        name="handle_bar",
    )
    for side_name, sign in (("left", 1.0), ("right", -1.0)):
        part.visual(
            Box((0.011, 0.008, 0.014)),
            origin=Origin(xyz=(0.1755, sign * bar_offset, 0.0)),
            material=bracket_material,
            name=f"{side_name}_handle_post",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tool_pedestal")

    cabinet_red = model.material("cabinet_red", rgba=(0.72, 0.08, 0.10, 1.0))
    drawer_red = model.material("drawer_red", rgba=(0.66, 0.07, 0.09, 1.0))
    inner_charcoal = model.material("inner_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    tray_black = model.material("tray_black", rgba=(0.11, 0.12, 0.13, 1.0))
    rubber_mat = model.material("rubber_mat", rgba=(0.09, 0.10, 0.10, 1.0))
    slide_steel = model.material("slide_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.83, 0.84, 0.86, 1.0))
    frame_black = model.material("frame_black", rgba=(0.13, 0.13, 0.14, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    fork_dark = model.material("fork_dark", rgba=(0.15, 0.16, 0.17, 1.0))
    wheel_dark = model.material("wheel_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("cabinet_body")
    body.visual(
        Box((CABINET_DEPTH, 0.022, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.269, BODY_BOTTOM_Z + BODY_HEIGHT * 0.5)),
        material=cabinet_red,
        name="left_side_panel",
    )
    body.visual(
        Box((CABINET_DEPTH, 0.022, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.269, BODY_BOTTOM_Z + BODY_HEIGHT * 0.5)),
        material=cabinet_red,
        name="right_side_panel",
    )
    body.visual(
        Box((0.018, 0.516, BODY_HEIGHT)),
        origin=Origin(xyz=(-0.206, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT * 0.5)),
        material=cabinet_red,
        name="back_panel",
    )
    body.visual(
        Box((0.404, 0.516, 0.018)),
        origin=Origin(xyz=(0.003, 0.0, 0.139)),
        material=inner_charcoal,
        name="bottom_panel",
    )
    body.visual(
        Box((0.018, 0.516, 0.034)),
        origin=Origin(xyz=(0.206, 0.0, 0.147)),
        material=cabinet_red,
        name="bottom_front_rail",
    )
    body.visual(
        Box((0.018, 0.516, 0.032)),
        origin=Origin(xyz=(0.206, 0.0, 0.781)),
        material=cabinet_red,
        name="top_front_header",
    )
    body.visual(
        Box((0.404, 0.516, 0.012)),
        origin=Origin(xyz=(0.003, 0.0, 0.796)),
        material=inner_charcoal,
        name="top_support_panel",
    )
    body.inertial = Inertial.from_geometry(
        Box((CABINET_DEPTH, CABINET_WIDTH, BODY_HEIGHT)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT * 0.5)),
    )

    top_tray = model.part("top_tray")
    top_tray.visual(
        Box((0.402, 0.502, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=tray_black,
        name="tray_floor",
    )
    top_tray.visual(
        Box((0.014, 0.516, 0.030)),
        origin=Origin(xyz=(0.208, 0.0, 0.020)),
        material=tray_black,
        name="front_lip",
    )
    top_tray.visual(
        Box((0.014, 0.516, 0.030)),
        origin=Origin(xyz=(-0.208, 0.0, 0.020)),
        material=tray_black,
        name="rear_lip",
    )
    top_tray.visual(
        Box((0.416, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, 0.258, 0.020)),
        material=tray_black,
        name="left_lip",
    )
    top_tray.visual(
        Box((0.416, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, -0.258, 0.020)),
        material=tray_black,
        name="right_lip",
    )
    top_tray.inertial = Inertial.from_geometry(
        Box((0.430, 0.530, 0.035)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
    )

    tray_mat_part = model.part("tray_mat")
    tray_mat_part.visual(
        Box((0.360, 0.452, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber_mat,
        name="rubber_mat",
    )
    tray_mat_part.inertial = Inertial.from_geometry(
        Box((0.360, 0.452, 0.004)),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    caster_frame = model.part("caster_frame")
    caster_frame.visual(
        Box((0.370, 0.032, 0.026)),
        origin=Origin(xyz=(0.0, 0.240, 0.013)),
        material=frame_black,
        name="left_frame_rail",
    )
    caster_frame.visual(
        Box((0.370, 0.032, 0.026)),
        origin=Origin(xyz=(0.0, -0.240, 0.013)),
        material=frame_black,
        name="right_frame_rail",
    )
    caster_frame.visual(
        Box((0.026, 0.472, 0.026)),
        origin=Origin(xyz=(0.176, 0.0, 0.013)),
        material=frame_black,
        name="front_frame_rail",
    )
    caster_frame.visual(
        Box((0.026, 0.472, 0.026)),
        origin=Origin(xyz=(-0.176, 0.0, 0.013)),
        material=frame_black,
        name="rear_frame_rail",
    )
    caster_frame.visual(
        Box((0.040, 0.448, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=frame_black,
        name="center_crossmember",
    )
    caster_mounts = {
        "front_left": (0.176, 0.240),
        "front_right": (0.176, -0.240),
        "rear_left": (-0.176, 0.240),
        "rear_right": (-0.176, -0.240),
    }
    for mount_name, (x_pos, y_pos) in caster_mounts.items():
        caster_frame.visual(
            Box((0.072, 0.072, 0.008)),
            origin=Origin(xyz=(x_pos, y_pos, 0.004)),
            material=frame_black,
            name=f"mount_{mount_name}",
        )
    caster_frame.inertial = Inertial.from_geometry(
        Box((0.430, 0.560, 0.026)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    model.articulation(
        "body_to_top_tray",
        ArticulationType.FIXED,
        parent=body,
        child=top_tray,
        origin=Origin(xyz=(0.0, 0.0, 0.797)),
    )
    model.articulation(
        "top_tray_to_mat",
        ArticulationType.FIXED,
        parent=top_tray,
        child=tray_mat_part,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )
    model.articulation(
        "body_to_caster_frame",
        ArticulationType.FIXED,
        parent=body,
        child=caster_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.104)),
    )

    caster_yaws = {
        "front_left": math.radians(28.0),
        "front_right": math.radians(-24.0),
        "rear_left": math.radians(152.0),
        "rear_right": math.radians(-140.0),
    }
    for mount_name, (x_pos, y_pos) in caster_mounts.items():
        caster = model.part(f"caster_{mount_name}")
        _add_caster_geometry(
            caster,
            fork_material=fork_dark,
            wheel_material=wheel_dark,
            steel_material=slide_steel,
        )
        caster.inertial = Inertial.from_geometry(
            Box((0.080, 0.080, 0.115)),
            mass=0.7,
            origin=Origin(xyz=(0.0, 0.0, -0.0575)),
        )
        model.articulation(
            f"caster_frame_to_{mount_name}",
            ArticulationType.FIXED,
            parent=caster_frame,
            child=caster,
            origin=Origin(xyz=(x_pos, y_pos, 0.0), rpy=(0.0, 0.0, caster_yaws[mount_name])),
        )

    drawer_face_heights = [0.074, 0.084, 0.104, 0.128, 0.168]
    drawer_top = 0.758
    drawer_gap = 0.007
    for index, face_height in enumerate(drawer_face_heights, start=1):
        drawer_top -= face_height
        z_center = drawer_top + face_height * 0.5
        drawer_top -= drawer_gap

        slide_pack = model.part(f"drawer_{index}_slides")
        _add_slide_pack_geometry(slide_pack, slide_material=slide_steel, bearing_material=bearing_steel)
        slide_pack.inertial = Inertial.from_geometry(
            Box((0.372, 0.492, 0.040)),
            mass=0.55,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        model.articulation(
            f"body_to_drawer_{index}_slides",
            ArticulationType.FIXED,
            parent=body,
            child=slide_pack,
            origin=Origin(xyz=(0.0, 0.0, z_center)),
        )

        drawer = model.part(f"drawer_{index}")
        pocket_width, pocket_height = _add_drawer_geometry(
            drawer,
            face_height=face_height,
            face_material=drawer_red,
            box_material=inner_charcoal,
            slide_material=slide_steel,
        )
        drawer.inertial = Inertial.from_geometry(
            Box((0.372, 0.494, face_height)),
            mass=1.1 + face_height * 2.0,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        model.articulation(
            f"drawer_{index}_travel",
            ArticulationType.PRISMATIC,
            parent=slide_pack,
            child=drawer,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=90.0,
                velocity=0.45,
                lower=0.0,
                upper=0.285,
            ),
        )

        handle = model.part(f"drawer_{index}_handle")
        _add_handle_geometry(
            handle,
            pocket_width=pocket_width,
            handle_material=handle_dark,
            bracket_material=slide_steel,
        )
        handle.inertial = Inertial.from_geometry(
            Box((0.020, pocket_width, pocket_height)),
            mass=0.08,
            origin=Origin(xyz=(0.176, 0.0, 0.0)),
        )
        model.articulation(
            f"drawer_{index}_to_handle",
            ArticulationType.FIXED,
            parent=drawer,
            child=handle,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root="/")

    body = object_model.get_part("cabinet_body")
    top_tray = object_model.get_part("top_tray")
    tray_mat = object_model.get_part("tray_mat")
    caster_frame = object_model.get_part("caster_frame")
    for index in range(1, 6):
        ctx.allow_overlap(
            object_model.get_part(f"drawer_{index}"),
            object_model.get_part(f"drawer_{index}_slides"),
            reason="telescoping ball-bearing side-mount slide members intentionally nest during travel",
        )

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
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    body_top_support = body.get_visual("top_support_panel")
    tray_floor = top_tray.get_visual("tray_floor")
    mat_surface = tray_mat.get_visual("rubber_mat")
    ctx.expect_gap(
        top_tray,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.006,
        positive_elem=tray_floor,
        negative_elem=body_top_support,
        name="top_tray_seated_on_body",
    )
    ctx.expect_within(
        tray_mat,
        top_tray,
        axes="xy",
        inner_elem=mat_surface,
        outer_elem=tray_floor,
    )
    ctx.expect_gap(
        tray_mat,
        top_tray,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=mat_surface,
        negative_elem=tray_floor,
        name="rubber_mat_seated_in_tray",
    )

    for mount_name in ("front_left", "front_right", "rear_left", "rear_right"):
        caster = object_model.get_part(f"caster_{mount_name}")
        mount_pad = caster_frame.get_visual(f"mount_{mount_name}")
        top_plate = caster.get_visual("top_plate")
        ctx.expect_overlap(
            caster,
            caster_frame,
            axes="xy",
            min_overlap=0.003,
            elem_a=top_plate,
            elem_b=mount_pad,
            name=f"{mount_name}_caster_plate_over_mount",
        )
        ctx.expect_gap(
            caster_frame,
            caster,
            axis="z",
            max_gap=0.001,
            max_penetration=0.001,
            positive_elem=mount_pad,
            negative_elem=top_plate,
            name=f"{mount_name}_caster_seated_to_frame",
        )

    drawer_articulations = {
        1: object_model.get_articulation("drawer_1_travel"),
        2: object_model.get_articulation("drawer_2_travel"),
        3: object_model.get_articulation("drawer_3_travel"),
        4: object_model.get_articulation("drawer_4_travel"),
        5: object_model.get_articulation("drawer_5_travel"),
    }
    for index in range(1, 6):
        slide_pack = object_model.get_part(f"drawer_{index}_slides")
        drawer = object_model.get_part(f"drawer_{index}")
        handle = object_model.get_part(f"drawer_{index}_handle")

        left_outer = slide_pack.get_visual("left_outer_web")
        right_outer = slide_pack.get_visual("right_outer_web")
        left_lower_outer = slide_pack.get_visual("left_lower_outer_web")
        right_lower_outer = slide_pack.get_visual("right_lower_outer_web")
        left_inner = drawer.get_visual("left_inner_rail")
        right_inner = drawer.get_visual("right_inner_rail")
        left_lower_inner = drawer.get_visual("left_lower_inner_rail")
        right_lower_inner = drawer.get_visual("right_lower_inner_rail")
        face_left = drawer.get_visual("face_left_frame")
        handle_bar = handle.get_visual("handle_bar")
        pocket_back = drawer.get_visual("pocket_back")

        ctx.expect_overlap(
            drawer,
            slide_pack,
            axes="yz",
            min_overlap=0.0008,
            elem_a=left_inner,
            elem_b=left_outer,
            name=f"drawer_{index}_left_slide_alignment",
        )
        ctx.expect_overlap(
            drawer,
            slide_pack,
            axes="yz",
            min_overlap=0.0008,
            elem_a=right_inner,
            elem_b=right_outer,
            name=f"drawer_{index}_right_slide_alignment",
        )
        ctx.expect_overlap(
            drawer,
            slide_pack,
            axes="yz",
            min_overlap=0.0008,
            elem_a=left_lower_inner,
            elem_b=left_lower_outer,
            name=f"drawer_{index}_left_lower_slide_alignment",
        )
        ctx.expect_overlap(
            drawer,
            slide_pack,
            axes="yz",
            min_overlap=0.0008,
            elem_a=right_lower_inner,
            elem_b=right_lower_outer,
            name=f"drawer_{index}_right_lower_slide_alignment",
        )
        ctx.expect_gap(
            drawer,
            slide_pack,
            axis="x",
            max_gap=0.001,
            max_penetration=0.001,
            positive_elem=face_left,
            negative_elem=left_outer,
            name=f"drawer_{index}_closed_face_stop",
        )
        ctx.expect_within(
            handle,
            drawer,
            axes="yz",
            inner_elem=handle_bar,
            outer_elem=pocket_back,
            name=f"drawer_{index}_handle_within_recess_band",
        )
        ctx.expect_gap(
            drawer,
            handle,
            axis="x",
            min_gap=0.010,
            positive_elem=face_left,
            negative_elem=handle_bar,
            name=f"drawer_{index}_bar_pull_recessed",
        )

    with ctx.pose({drawer_articulations[1]: 0.240, drawer_articulations[5]: 0.265}):
        for index, min_gap in ((1, 0.18), (5, 0.20)):
            slide_pack = object_model.get_part(f"drawer_{index}_slides")
            drawer = object_model.get_part(f"drawer_{index}")
            left_outer = slide_pack.get_visual("left_outer_web")
            right_outer = slide_pack.get_visual("right_outer_web")
            left_lower_outer = slide_pack.get_visual("left_lower_outer_web")
            right_lower_outer = slide_pack.get_visual("right_lower_outer_web")
            left_inner = drawer.get_visual("left_inner_rail")
            right_inner = drawer.get_visual("right_inner_rail")
            left_lower_inner = drawer.get_visual("left_lower_inner_rail")
            right_lower_inner = drawer.get_visual("right_lower_inner_rail")
            face_left = drawer.get_visual("face_left_frame")
            ctx.expect_gap(
                drawer,
                slide_pack,
                axis="x",
                min_gap=min_gap,
                positive_elem=face_left,
                negative_elem=left_outer,
                name=f"drawer_{index}_full_extension_travel",
            )
            ctx.expect_overlap(
                drawer,
                slide_pack,
                axes="yz",
                min_overlap=0.0008,
                elem_a=left_inner,
                elem_b=left_outer,
                name=f"drawer_{index}_left_slide_stays_nested_open",
            )
            ctx.expect_overlap(
                drawer,
                slide_pack,
                axes="yz",
                min_overlap=0.0008,
                elem_a=right_inner,
                elem_b=right_outer,
                name=f"drawer_{index}_right_slide_stays_nested_open",
            )
            ctx.expect_overlap(
                drawer,
                slide_pack,
                axes="yz",
                min_overlap=0.0008,
                elem_a=left_lower_inner,
                elem_b=left_lower_outer,
                name=f"drawer_{index}_left_lower_slide_stays_nested_open",
            )
            ctx.expect_overlap(
                drawer,
                slide_pack,
                axes="yz",
                min_overlap=0.0008,
                elem_a=right_lower_inner,
                elem_b=right_lower_outer,
                name=f"drawer_{index}_right_lower_slide_stays_nested_open",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
