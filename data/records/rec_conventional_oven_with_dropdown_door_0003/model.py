from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return _REAL_GETCWD()


os.getcwd = _safe_getcwd

from math import pi

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
    model = ArticulatedObject(name="built_in_oven", assets=ASSETS)

    cabinetry = model.material("cabinetry", rgba=(0.84, 0.82, 0.78, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.77, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.10, 0.11, 0.13, 0.65))
    cavity_dark = model.material("cavity_dark", rgba=(0.14, 0.14, 0.15, 1.0))
    gasket = model.material("gasket", rgba=(0.18, 0.19, 0.20, 1.0))

    shell_width = 0.60
    shell_height = 0.60
    shell_depth = 0.52
    shell_front_setback = 0.04
    shell_wall = 0.03

    cabinet_width = 0.84
    cabinet_height = 0.82
    cabinet_depth = shell_front_setback

    channel_outer = 0.538
    channel_strip = 0.018
    channel_center_z = 0.30

    door_width = 0.596
    door_height = 0.596
    door_thickness = shell_front_setback
    door_front_thickness = 0.022
    door_liner_depth = door_thickness - door_front_thickness

    housing = model.part("housing")
    housing.visual(
        Box((0.12, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(-0.36, -0.02, 0.29)),
        material=cabinetry,
        name="cabinet_left",
    )
    housing.visual(
        Box((0.12, cabinet_depth, cabinet_height)),
        origin=Origin(xyz=(0.36, -0.02, 0.29)),
        material=cabinetry,
        name="cabinet_right",
    )
    housing.visual(
        Box((cabinet_width, cabinet_depth, 0.12)),
        origin=Origin(xyz=(0.0, -0.02, -0.06)),
        material=cabinetry,
        name="cabinet_bottom",
    )
    housing.visual(
        Box((cabinet_width, cabinet_depth, 0.10)),
        origin=Origin(xyz=(0.0, -0.02, 0.65)),
        material=cabinetry,
        name="cabinet_top",
    )

    housing.visual(
        Box((shell_wall, shell_depth, shell_height)),
        origin=Origin(xyz=(-0.285, -0.30, 0.30)),
        material=stainless,
        name="left_side_wall",
    )
    housing.visual(
        Box((shell_wall, shell_depth, shell_height)),
        origin=Origin(xyz=(0.285, -0.30, 0.30)),
        material=stainless,
        name="right_side_wall",
    )
    housing.visual(
        Box((shell_width - 2.0 * shell_wall, shell_depth, shell_wall)),
        origin=Origin(xyz=(0.0, -0.30, 0.015)),
        material=stainless,
        name="lower_shell_rail",
    )
    housing.visual(
        Box((shell_width - 2.0 * shell_wall, shell_depth, shell_wall)),
        origin=Origin(xyz=(0.0, -0.30, 0.585)),
        material=stainless,
        name="upper_shell_rail",
    )
    housing.visual(
        Box((0.50, 0.03, 0.50)),
        origin=Origin(xyz=(0.0, -0.545, 0.30)),
        material=cavity_dark,
        name="cavity_back",
    )
    housing.visual(
        Box((0.02, shell_depth, 0.50)),
        origin=Origin(xyz=(-0.26, -0.30, 0.30)),
        material=cavity_dark,
        name="liner_left",
    )
    housing.visual(
        Box((0.02, shell_depth, 0.50)),
        origin=Origin(xyz=(0.26, -0.30, 0.30)),
        material=cavity_dark,
        name="liner_right",
    )
    housing.visual(
        Box((0.50, shell_depth, 0.02)),
        origin=Origin(xyz=(0.0, -0.30, 0.04)),
        material=cavity_dark,
        name="liner_bottom",
    )
    housing.visual(
        Box((0.50, shell_depth, 0.02)),
        origin=Origin(xyz=(0.0, -0.30, 0.56)),
        material=cavity_dark,
        name="liner_top",
    )

    housing.visual(
        Box((channel_strip, 0.010, channel_outer)),
        origin=Origin(xyz=(-0.26, -0.045, channel_center_z)),
        material=gasket,
        name="seal_channel_left",
    )
    housing.visual(
        Box((channel_strip, 0.010, channel_outer)),
        origin=Origin(xyz=(0.26, -0.045, channel_center_z)),
        material=gasket,
        name="seal_channel_right",
    )
    housing.visual(
        Box((channel_outer - 2.0 * channel_strip, 0.010, channel_strip)),
        origin=Origin(xyz=(0.0, -0.045, 0.56)),
        material=gasket,
        name="seal_channel_top",
    )
    housing.visual(
        Box((channel_outer - 2.0 * channel_strip, 0.010, channel_strip)),
        origin=Origin(xyz=(0.0, -0.045, 0.04)),
        material=gasket,
        name="seal_channel_bottom",
    )

    housing.visual(
        Box((0.014, 0.14, 0.08)),
        origin=Origin(xyz=(-0.283, -0.11, 0.085)),
        material=cavity_dark,
        name="left_spring_hinge",
    )
    housing.visual(
        Box((0.014, 0.14, 0.08)),
        origin=Origin(xyz=(0.283, -0.11, 0.085)),
        material=cavity_dark,
        name="right_spring_hinge",
    )
    housing.inertial = Inertial.from_geometry(
        Box((cabinet_width, shell_depth + cabinet_depth, cabinet_height)),
        mass=42.0,
        origin=Origin(xyz=(0.0, -0.28, 0.29)),
    )

    door = model.part("door")
    door.visual(
        Box((door_width, door_front_thickness, door_height)),
        origin=Origin(xyz=(0.0, -door_front_thickness / 2.0, door_height / 2.0)),
        material=stainless,
        name="door_body",
    )
    door.visual(
        Box((0.53, door_liner_depth, 0.522)),
        origin=Origin(xyz=(0.0, -(door_front_thickness + door_liner_depth / 2.0), 0.29)),
        material=cavity_dark,
        name="door_inner_liner",
    )
    door.visual(
        Box((0.52, 0.006, 0.45)),
        origin=Origin(xyz=(0.0, -0.003, 0.32)),
        material=dark_glass,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.36),
        origin=Origin(xyz=(0.0, 0.042, 0.505), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="handle_bar",
    )
    door.visual(
        Box((0.02, 0.032, 0.024)),
        origin=Origin(xyz=(-0.13, 0.016, 0.505)),
        material=stainless,
        name="handle_left_bracket",
    )
    door.visual(
        Box((0.02, 0.032, 0.024)),
        origin=Origin(xyz=(0.13, 0.016, 0.505)),
        material=stainless,
        name="handle_right_bracket",
    )
    door.visual(
        Box((channel_strip, 0.008, channel_outer)),
        origin=Origin(xyz=(-0.26, -0.036, channel_center_z)),
        material=gasket,
        name="seal_land_left",
    )
    door.visual(
        Box((channel_strip, 0.008, channel_outer)),
        origin=Origin(xyz=(0.26, -0.036, channel_center_z)),
        material=gasket,
        name="seal_land_right",
    )
    door.visual(
        Box((channel_outer - 2.0 * channel_strip, 0.008, channel_strip)),
        origin=Origin(xyz=(0.0, -0.036, 0.56)),
        material=gasket,
        name="seal_land_top",
    )
    door.visual(
        Box((channel_outer - 2.0 * channel_strip, 0.008, channel_strip)),
        origin=Origin(xyz=(0.0, -0.036, 0.04)),
        material=gasket,
        name="seal_land_bottom",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, door_thickness, door_height)),
        mass=12.0,
        origin=Origin(xyz=(0.0, -door_thickness / 2.0, door_height / 2.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.8,
            lower=-1.35,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")

    left_side_wall = housing.get_visual("left_side_wall")
    right_side_wall = housing.get_visual("right_side_wall")
    left_spring_hinge = housing.get_visual("left_spring_hinge")
    right_spring_hinge = housing.get_visual("right_spring_hinge")
    seal_left = housing.get_visual("seal_channel_left")
    seal_right = housing.get_visual("seal_channel_right")
    seal_top = housing.get_visual("seal_channel_top")
    seal_bottom = housing.get_visual("seal_channel_bottom")

    door_body = door.get_visual("door_body")
    door_inner_liner = door.get_visual("door_inner_liner")
    door_glass = door.get_visual("door_glass")
    handle_bar = door.get_visual("handle_bar")
    handle_left_bracket = door.get_visual("handle_left_bracket")
    handle_right_bracket = door.get_visual("handle_right_bracket")
    seal_land_left = door.get_visual("seal_land_left")
    seal_land_right = door.get_visual("seal_land_right")
    seal_land_top = door.get_visual("seal_land_top")
    seal_land_bottom = door.get_visual("seal_land_bottom")

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

    ctx.expect_origin_distance(door, housing, axes="x", max_dist=0.001)
    ctx.expect_overlap(door, housing, axes="xz", min_overlap=0.34)
    ctx.expect_within(door, door, axes="xz", inner_elem=door_glass, outer_elem=door_body)
    ctx.expect_within(door, door, axes="xz", inner_elem=door_inner_liner, outer_elem=door_body)

    ctx.expect_contact(door, door, elem_a=handle_bar, elem_b=handle_left_bracket)
    ctx.expect_contact(door, door, elem_a=handle_bar, elem_b=handle_right_bracket)
    ctx.expect_contact(door, door, elem_a=handle_left_bracket, elem_b=door_body)
    ctx.expect_contact(door, door, elem_a=handle_right_bracket, elem_b=door_body)
    ctx.expect_contact(door, door, elem_a=seal_land_left, elem_b=door_inner_liner)
    ctx.expect_contact(door, door, elem_a=seal_land_right, elem_b=door_inner_liner)
    ctx.expect_contact(door, door, elem_a=seal_land_top, elem_b=door_inner_liner)
    ctx.expect_contact(door, door, elem_a=seal_land_bottom, elem_b=door_inner_liner)

    ctx.expect_contact(door, housing, elem_a=seal_land_left, elem_b=seal_left)
    ctx.expect_contact(door, housing, elem_a=seal_land_right, elem_b=seal_right)
    ctx.expect_contact(door, housing, elem_a=seal_land_top, elem_b=seal_top)
    ctx.expect_contact(door, housing, elem_a=seal_land_bottom, elem_b=seal_bottom)
    ctx.expect_within(door, door, axes="xz", inner_elem=seal_land_left, outer_elem=door_body)
    ctx.expect_within(door, door, axes="xz", inner_elem=seal_land_right, outer_elem=door_body)

    ctx.expect_within(housing, housing, axes="yz", inner_elem=left_spring_hinge, outer_elem=left_side_wall)
    ctx.expect_within(housing, housing, axes="yz", inner_elem=right_spring_hinge, outer_elem=right_side_wall)

    with ctx.pose({door_hinge: -1.25}):
        ctx.expect_overlap(door, housing, axes="x", min_overlap=0.55)
        ctx.expect_gap(
            door,
            housing,
            axis="y",
            min_gap=0.30,
            positive_elem=seal_land_top,
            negative_elem=seal_top,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
