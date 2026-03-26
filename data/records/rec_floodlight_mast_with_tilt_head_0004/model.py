from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

_ORIGINAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIGINAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return _ORIGINAL_GETCWD()


os.getcwd = _safe_getcwd
os.chdir(_safe_getcwd())

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

BASE_LENGTH = 1.20
BASE_WIDTH = 0.82
FRAME_RAIL = 0.08
FRAME_HEIGHT = 0.05
FRAME_CENTER_Z = 0.085
FRAME_TOP_Z = FRAME_CENTER_Z + (FRAME_HEIGHT * 0.5)
FOOT_SOCKET_HEIGHT = 0.036
FOOT_SOCKET_CENTER_Z = 0.042
FOOT_STEM_LENGTH = 0.044
FOOT_STEM_CENTER_Z = 0.038
FOOT_PAD_LENGTH = 0.016
FOOT_PAD_CENTER_Z = 0.008
FOOT_NUT_LENGTH = 0.020
FOOT_NUT_CENTER_Z = 0.026
FOOT_X = (BASE_LENGTH * 0.5) - (FRAME_RAIL * 0.5)
FOOT_Y = (BASE_WIDTH * 0.5) - (FRAME_RAIL * 0.5)

MAST_SIZE = 0.12
MAST_SECTION_HEIGHT = 0.74
MAST_BASE_FLANGE_HEIGHT = 0.022
MAST_TOP_PLATE_HEIGHT = 0.022
MAST_TOTAL_HEIGHT = (3.0 * MAST_SECTION_HEIGHT) + MAST_BASE_FLANGE_HEIGHT + MAST_TOP_PLATE_HEIGHT

HEAD_CROSSBAR_Z = 0.26
HEAD_YOKE_CENTER_Z = 0.125
HEAD_LAMP_AXIS_Z = 0.105
LAMP_CENTER_Y = 0.21
YOKE_OFFSET_Y = 0.142
YOKE_CHEEK_THICKNESS = 0.012
LAMP_AXLE_LENGTH = 2.0 * (YOKE_OFFSET_Y - (YOKE_CHEEK_THICKNESS * 0.5))
def _add_corner_foot(part, prefix: str, x_pos: float, y_pos: float, metal, rubber) -> None:
    part.visual(
        Box((0.10, 0.10, FOOT_SOCKET_HEIGHT)),
        origin=Origin(xyz=(x_pos, y_pos, FOOT_SOCKET_CENTER_Z)),
        material=metal,
        name=f"{prefix}_socket",
    )
    part.visual(
        Cylinder(radius=0.012, length=FOOT_STEM_LENGTH),
        origin=Origin(xyz=(x_pos, y_pos, FOOT_STEM_CENTER_Z)),
        material=metal,
        name=f"{prefix}_stem",
    )
    part.visual(
        Cylinder(radius=0.028, length=FOOT_NUT_LENGTH),
        origin=Origin(xyz=(x_pos, y_pos, FOOT_NUT_CENTER_Z)),
        material=metal,
        name=f"{prefix}_nut",
    )
    part.visual(
        Cylinder(radius=0.055, length=FOOT_PAD_LENGTH),
        origin=Origin(xyz=(x_pos, y_pos, FOOT_PAD_CENTER_Z)),
        material=rubber,
        name=f"{prefix}_pad",
    )


def _add_collar(part, prefix: str, z_pos: float, collar_metal, bolt_metal) -> None:
    part.visual(
        Box((0.014, 0.148, 0.054)),
        origin=Origin(xyz=(0.067, 0.0, z_pos)),
        material=collar_metal,
        name=f"{prefix}_front_plate",
    )
    part.visual(
        Box((0.014, 0.148, 0.054)),
        origin=Origin(xyz=(-0.067, 0.0, z_pos)),
        material=collar_metal,
        name=f"{prefix}_back_plate",
    )
    part.visual(
        Box((0.148, 0.014, 0.054)),
        origin=Origin(xyz=(0.0, 0.067, z_pos)),
        material=collar_metal,
        name=f"{prefix}_left_plate",
    )
    part.visual(
        Box((0.148, 0.014, 0.054)),
        origin=Origin(xyz=(0.0, -0.067, z_pos)),
        material=collar_metal,
        name=f"{prefix}_right_plate",
    )

    for bolt_index, y_pos in enumerate((-0.052, 0.052), start=1):
        for row_index, z_offset in enumerate((-0.014, 0.014), start=1):
            part.visual(
                Cylinder(radius=0.007, length=0.014),
                origin=Origin(
                    xyz=(0.081, y_pos, z_pos + z_offset),
                    rpy=(0.0, math.pi * 0.5, 0.0),
                ),
                material=bolt_metal,
                name=f"{prefix}_front_bolt_{bolt_index}_{row_index}",
            )
            part.visual(
                Cylinder(radius=0.007, length=0.014),
                origin=Origin(
                    xyz=(-0.081, y_pos, z_pos + z_offset),
                    rpy=(0.0, math.pi * 0.5, 0.0),
                ),
                material=bolt_metal,
                name=f"{prefix}_back_bolt_{bolt_index}_{row_index}",
            )


def _add_yoke(part, prefix: str, center_y: float, metal, bolt_metal) -> None:
    outer_y = center_y + YOKE_OFFSET_Y
    inner_y = center_y - YOKE_OFFSET_Y
    if center_y < 0.0:
        outer_y, inner_y = center_y - YOKE_OFFSET_Y, center_y + YOKE_OFFSET_Y
    pin_head_length = 0.016
    outer_head_sign = 1.0 if outer_y > center_y else -1.0
    inner_head_sign = 1.0 if inner_y > center_y else -1.0

    part.visual(
        Box((0.060, YOKE_CHEEK_THICKNESS, 0.210)),
        origin=Origin(xyz=(0.120, outer_y, HEAD_YOKE_CENTER_Z)),
        material=metal,
        name=f"{prefix}_yoke_outer",
    )
    part.visual(
        Box((0.060, YOKE_CHEEK_THICKNESS, 0.210)),
        origin=Origin(xyz=(0.120, inner_y, HEAD_YOKE_CENTER_Z)),
        material=metal,
        name=f"{prefix}_yoke_inner",
    )
    part.visual(
        Cylinder(radius=0.016, length=pin_head_length),
        origin=Origin(
            xyz=(
                0.120,
                outer_y + (outer_head_sign * ((YOKE_CHEEK_THICKNESS * 0.5) + (pin_head_length * 0.5))),
                HEAD_LAMP_AXIS_Z,
            ),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=bolt_metal,
        name=f"{prefix}_pin_head_outer",
    )
    part.visual(
        Cylinder(radius=0.016, length=pin_head_length),
        origin=Origin(
            xyz=(
                0.120,
                inner_y + (inner_head_sign * ((YOKE_CHEEK_THICKNESS * 0.5) + (pin_head_length * 0.5))),
                HEAD_LAMP_AXIS_Z,
            ),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=bolt_metal,
        name=f"{prefix}_pin_head_inner",
    )


def _add_floodlight_body(part, shell_material, lens_material, emitter_material, rear_material) -> None:
    part.visual(
        Box((0.112, 0.226, 0.170)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=shell_material,
        name="shell",
    )
    part.visual(
        Box((0.048, 0.200, 0.146)),
        origin=Origin(xyz=(-0.052, 0.0, 0.0)),
        material=shell_material,
        name="rear_taper",
    )
    part.visual(
        Box((0.016, 0.250, 0.014)),
        origin=Origin(xyz=(0.063, 0.0, 0.089)),
        material=shell_material,
        name="bezel_top",
    )
    part.visual(
        Box((0.016, 0.250, 0.014)),
        origin=Origin(xyz=(0.063, 0.0, -0.089)),
        material=shell_material,
        name="bezel_bottom",
    )
    part.visual(
        Box((0.016, 0.014, 0.178)),
        origin=Origin(xyz=(0.063, 0.118, 0.0)),
        material=shell_material,
        name="bezel_left",
    )
    part.visual(
        Box((0.016, 0.014, 0.178)),
        origin=Origin(xyz=(0.063, -0.118, 0.0)),
        material=shell_material,
        name="bezel_right",
    )
    part.visual(
        Box((0.004, 0.188, 0.134)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=lens_material,
        name="lens",
    )
    part.visual(
        Box((0.014, 0.160, 0.108)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=emitter_material,
        name="emitter",
    )
    part.visual(
        Box((0.030, 0.154, 0.110)),
        origin=Origin(xyz=(-0.028, 0.0, 0.0)),
        material=rear_material,
        name="driver_box",
    )
    part.visual(
        Cylinder(radius=0.014, length=LAMP_AXLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=rear_material,
        name="trunnion_axle",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="temporary_light_tower", assets=ASSETS)

    galvanized = model.material("galvanized", rgba=(0.69, 0.73, 0.76, 1.0))
    zinc = model.material("zinc", rgba=(0.77, 0.79, 0.81, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.20, 0.21, 0.23, 1.0))
    lamp_black = model.material("lamp_black", rgba=(0.17, 0.18, 0.19, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.84, 0.90, 0.96, 0.58))
    emitter_white = model.material("emitter_white", rgba=(0.95, 0.94, 0.86, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    base = model.part("base_frame")
    base.visual(
        Box((BASE_LENGTH, FRAME_RAIL, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.0, (BASE_WIDTH * 0.5) - (FRAME_RAIL * 0.5), FRAME_CENTER_Z)),
        material=dark_frame,
        name="left_rail",
    )
    base.visual(
        Box((BASE_LENGTH, FRAME_RAIL, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.0, -((BASE_WIDTH * 0.5) - (FRAME_RAIL * 0.5)), FRAME_CENTER_Z)),
        material=dark_frame,
        name="right_rail",
    )
    base.visual(
        Box((FRAME_RAIL, BASE_WIDTH - FRAME_RAIL, FRAME_HEIGHT)),
        origin=Origin(xyz=((BASE_LENGTH * 0.5) - (FRAME_RAIL * 0.5), 0.0, FRAME_CENTER_Z)),
        material=dark_frame,
        name="front_rail",
    )
    base.visual(
        Box((FRAME_RAIL, BASE_WIDTH - FRAME_RAIL, FRAME_HEIGHT)),
        origin=Origin(xyz=(-((BASE_LENGTH * 0.5) - (FRAME_RAIL * 0.5)), 0.0, FRAME_CENTER_Z)),
        material=dark_frame,
        name="rear_rail",
    )
    base.visual(
        Box((BASE_LENGTH - 0.14, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_CENTER_Z)),
        material=dark_frame,
        name="center_tie",
    )
    base.visual(
        Box((0.30, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_TOP_Z + 0.009)),
        material=galvanized,
        name="mast_mount_plate",
    )
    _add_corner_foot(base, "front_left", FOOT_X, FOOT_Y, zinc, rubber)
    _add_corner_foot(base, "front_right", FOOT_X, -FOOT_Y, zinc, rubber)
    _add_corner_foot(base, "rear_left", -FOOT_X, FOOT_Y, zinc, rubber)
    _add_corner_foot(base, "rear_right", -FOOT_X, -FOOT_Y, zinc, rubber)
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.16)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    mast = model.part("mast")
    mast.visual(
        Box((0.190, 0.190, MAST_BASE_FLANGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MAST_BASE_FLANGE_HEIGHT * 0.5)),
        material=galvanized,
        name="base_flange",
    )
    mast.visual(
        Box((MAST_SIZE, MAST_SIZE, MAST_SECTION_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, MAST_BASE_FLANGE_HEIGHT + (MAST_SECTION_HEIGHT * 0.5))),
        material=galvanized,
        name="lower_section",
    )
    mast.visual(
        Box((MAST_SIZE, MAST_SIZE, MAST_SECTION_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                MAST_BASE_FLANGE_HEIGHT + MAST_SECTION_HEIGHT + (MAST_SECTION_HEIGHT * 0.5),
            )
        ),
        material=galvanized,
        name="middle_section",
    )
    mast.visual(
        Box((MAST_SIZE, MAST_SIZE, MAST_SECTION_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                MAST_BASE_FLANGE_HEIGHT + (2.0 * MAST_SECTION_HEIGHT) + (MAST_SECTION_HEIGHT * 0.5),
            )
        ),
        material=galvanized,
        name="upper_section",
    )
    lower_seam_z = MAST_BASE_FLANGE_HEIGHT + MAST_SECTION_HEIGHT
    upper_seam_z = MAST_BASE_FLANGE_HEIGHT + (2.0 * MAST_SECTION_HEIGHT)
    top_mount_z = MAST_BASE_FLANGE_HEIGHT + (3.0 * MAST_SECTION_HEIGHT)
    _add_collar(mast, "lower_collar", lower_seam_z, zinc, dark_frame)
    _add_collar(mast, "upper_collar", upper_seam_z, zinc, dark_frame)
    _add_collar(mast, "top_collar", top_mount_z, zinc, dark_frame)
    mast.visual(
        Box((0.170, 0.170, MAST_TOP_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, top_mount_z + (MAST_TOP_PLATE_HEIGHT * 0.5))),
        material=galvanized,
        name="top_plate",
    )
    mast.inertial = Inertial.from_geometry(
        Box((0.22, 0.22, MAST_TOTAL_HEIGHT)),
        mass=66.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_TOTAL_HEIGHT * 0.5)),
    )

    head = model.part("head_frame")
    head.visual(
        Box((0.10, 0.10, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=galvanized,
        name="spigot",
    )
    head.visual(
        Box((0.080, 0.080, 0.240)),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=galvanized,
        name="upright",
    )
    head.visual(
        Box((0.180, 0.080, 0.060)),
        origin=Origin(xyz=(0.090, 0.0, HEAD_CROSSBAR_Z)),
        material=galvanized,
        name="forward_arm",
    )
    head.visual(
        Box((0.080, 0.840, 0.060)),
        origin=Origin(xyz=(0.120, 0.0, HEAD_CROSSBAR_Z)),
        material=galvanized,
        name="crossbar",
    )
    _add_yoke(head, "left", LAMP_CENTER_Y, galvanized, dark_frame)
    _add_yoke(head, "right", -LAMP_CENTER_Y, galvanized, dark_frame)
    head.inertial = Inertial.from_geometry(
        Box((0.26, 0.90, 0.34)),
        mass=18.0,
        origin=Origin(xyz=(0.08, 0.0, 0.17)),
    )

    left_light = model.part("left_floodlight")
    _add_floodlight_body(left_light, lamp_black, lens_glass, emitter_white, dark_frame)
    left_light.inertial = Inertial.from_geometry(
        Box((0.14, 0.28, 0.20)),
        mass=5.5,
        origin=Origin(),
    )

    right_light = model.part("right_floodlight")
    _add_floodlight_body(right_light, lamp_black, lens_glass, emitter_white, dark_frame)
    right_light.inertial = Inertial.from_geometry(
        Box((0.14, 0.28, 0.20)),
        mass=5.5,
        origin=Origin(),
    )

    model.articulation(
        "base_to_mast",
        ArticulationType.FIXED,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, FRAME_TOP_Z + 0.018)),
    )
    model.articulation(
        "mast_to_head",
        ArticulationType.FIXED,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, top_mount_z + MAST_TOP_PLATE_HEIGHT)),
    )
    model.articulation(
        "head_to_left_floodlight",
        ArticulationType.REVOLUTE,
        parent=head,
        child=left_light,
        origin=Origin(xyz=(0.120, LAMP_CENTER_Y, HEAD_LAMP_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.0, upper=0.18),
    )
    model.articulation(
        "head_to_right_floodlight",
        ArticulationType.REVOLUTE,
        parent=head,
        child=right_light,
        origin=Origin(xyz=(0.120, -LAMP_CENTER_Y, HEAD_LAMP_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-1.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_frame")
    mast = object_model.get_part("mast")
    head = object_model.get_part("head_frame")
    left_light = object_model.get_part("left_floodlight")
    right_light = object_model.get_part("right_floodlight")

    left_tilt = object_model.get_articulation("head_to_left_floodlight")
    right_tilt = object_model.get_articulation("head_to_right_floodlight")

    mast_mount_plate = base.get_visual("mast_mount_plate")
    front_rail = base.get_visual("front_rail")
    front_left_pad = base.get_visual("front_left_pad")
    rear_left_pad = base.get_visual("rear_left_pad")
    front_right_pad = base.get_visual("front_right_pad")

    mast_base_flange = mast.get_visual("base_flange")
    mast_top_plate = mast.get_visual("top_plate")
    lower_section = mast.get_visual("lower_section")
    lower_collar_front = mast.get_visual("lower_collar_front_plate")
    lower_collar_back = mast.get_visual("lower_collar_back_plate")
    upper_section = mast.get_visual("upper_section")
    upper_collar_front = mast.get_visual("upper_collar_front_plate")
    upper_collar_back = mast.get_visual("upper_collar_back_plate")

    head_spigot = head.get_visual("spigot")
    crossbar = head.get_visual("crossbar")
    left_yoke_outer = head.get_visual("left_yoke_outer")
    left_yoke_inner = head.get_visual("left_yoke_inner")
    right_yoke_outer = head.get_visual("right_yoke_outer")
    right_yoke_inner = head.get_visual("right_yoke_inner")

    left_shell = left_light.get_visual("shell")
    left_lens = left_light.get_visual("lens")
    left_bezel_top = left_light.get_visual("bezel_top")
    left_axle = left_light.get_visual("trunnion_axle")

    right_shell = right_light.get_visual("shell")
    right_axle = right_light.get_visual("trunnion_axle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # The lamp tilt axes sit centered between wide yoke cheeks, so a tighter default
    # consumer-scale tolerance would report the intentional open clevis spacing.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.13)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(mast, base, axes="xy", elem_a=mast_base_flange, elem_b=mast_mount_plate, min_overlap=0.03)
    ctx.expect_gap(
        mast,
        base,
        axis="z",
        positive_elem=mast_base_flange,
        negative_elem=mast_mount_plate,
        max_gap=0.001,
        max_penetration=0.002,
    )
    ctx.expect_overlap(head, mast, axes="xy", elem_a=head_spigot, elem_b=mast_top_plate, min_overlap=0.01)
    ctx.expect_gap(
        head,
        mast,
        axis="z",
        positive_elem=head_spigot,
        negative_elem=mast_top_plate,
        max_gap=0.001,
        max_penetration=0.002,
    )
    ctx.expect_within(head, mast, axes="xy", inner_elem=head_spigot, outer_elem=mast_top_plate)
    ctx.expect_gap(base, base, axis="z", positive_elem=front_rail, negative_elem=front_left_pad, min_gap=0.035)
    ctx.expect_gap(
        base,
        base,
        axis="x",
        positive_elem=front_left_pad,
        negative_elem=rear_left_pad,
        min_gap=1.00,
    )
    ctx.expect_gap(
        base,
        base,
        axis="y",
        positive_elem=front_left_pad,
        negative_elem=front_right_pad,
        min_gap=0.60,
    )
    ctx.expect_gap(
        mast,
        mast,
        axis="x",
        positive_elem=lower_collar_front,
        negative_elem=lower_section,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        mast,
        mast,
        axis="x",
        positive_elem=upper_section,
        negative_elem=upper_collar_back,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        mast,
        mast,
        axis="x",
        positive_elem=upper_collar_front,
        negative_elem=upper_section,
        max_gap=0.001,
        max_penetration=0.0,
    )
    ctx.expect_gap(left_light, left_light, axis="x", positive_elem=left_bezel_top, negative_elem=left_lens, min_gap=0.005)
    ctx.expect_gap(
        head,
        left_light,
        axis="y",
        positive_elem=left_yoke_outer,
        negative_elem=left_shell,
        min_gap=0.020,
    )
    ctx.expect_gap(
        left_light,
        head,
        axis="y",
        positive_elem=left_shell,
        negative_elem=left_yoke_inner,
        min_gap=0.020,
    )
    ctx.expect_gap(
        right_light,
        head,
        axis="y",
        positive_elem=right_shell,
        negative_elem=right_yoke_outer,
        min_gap=0.020,
    )
    ctx.expect_gap(
        head,
        right_light,
        axis="y",
        positive_elem=right_yoke_inner,
        negative_elem=right_shell,
        min_gap=0.020,
    )
    ctx.expect_contact(left_light, head, elem_a=left_axle, elem_b=left_yoke_outer)
    ctx.expect_contact(left_light, head, elem_a=left_axle, elem_b=left_yoke_inner)
    ctx.expect_contact(right_light, head, elem_a=right_axle, elem_b=right_yoke_outer)
    ctx.expect_contact(right_light, head, elem_a=right_axle, elem_b=right_yoke_inner)
    ctx.expect_gap(
        left_light,
        right_light,
        axis="y",
        positive_elem=left_shell,
        negative_elem=right_shell,
        min_gap=0.10,
    )
    with ctx.pose({left_tilt: -0.85, right_tilt: 0.12}):
        ctx.expect_contact(left_light, head, elem_a=left_axle, elem_b=left_yoke_outer)
        ctx.expect_contact(left_light, head, elem_a=left_axle, elem_b=left_yoke_inner)
        ctx.expect_contact(right_light, head, elem_a=right_axle, elem_b=right_yoke_outer)
        ctx.expect_contact(right_light, head, elem_a=right_axle, elem_b=right_yoke_inner)
        ctx.expect_gap(
            head,
            left_light,
            axis="y",
            positive_elem=left_yoke_outer,
            negative_elem=left_shell,
            min_gap=0.010,
        )
        ctx.expect_gap(
            left_light,
            head,
            axis="y",
            positive_elem=left_shell,
            negative_elem=left_yoke_inner,
            min_gap=0.010,
        )
        ctx.expect_gap(
            head,
            left_light,
            axis="z",
            positive_elem=crossbar,
            negative_elem=left_shell,
            min_gap=0.015,
        )
        ctx.expect_gap(
            head,
            right_light,
            axis="z",
            positive_elem=crossbar,
            negative_elem=right_shell,
            min_gap=0.015,
        )
        ctx.expect_gap(
            left_light,
            right_light,
            axis="y",
            positive_elem=left_shell,
            negative_elem=right_shell,
            min_gap=0.10,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
