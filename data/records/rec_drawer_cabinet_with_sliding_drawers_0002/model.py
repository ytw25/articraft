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
    Sphere,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

CABINET_WIDTH = 1.22
CABINET_DEPTH = 0.50
BODY_HEIGHT = 0.99
FEET_HEIGHT = 0.028
WALL_THICKNESS = 0.018
BACK_THICKNESS = 0.012
DIVIDER_THICKNESS = 0.014
FACE_REVEAL = 0.004
DRAWER_TRAVEL = 0.28

INTERIOR_HEIGHT = BODY_HEIGHT - 2.0 * WALL_THICKNESS
OPENING_HEIGHT = (INTERIOR_HEIGHT - 2.0 * DIVIDER_THICKNESS) / 3.0
DRAWER_FACE_WIDTH = CABINET_WIDTH - 0.012
DRAWER_FACE_HEIGHT = (BODY_HEIGHT - 4.0 * FACE_REVEAL) / 3.0
DRAWER_FACE_THICKNESS = 0.018

DRAWER_BOX_WIDTH = 1.132
DRAWER_BOX_DEPTH = 0.412
DRAWER_BOX_HEIGHT = 0.270
DRAWER_BOX_OFFSET_Z = 0.020
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.010
DRAWER_BACK_THICKNESS = 0.012

FIXED_SLIDE_THICKNESS = 0.012
FIXED_SLIDE_HEIGHT = 0.046
FIXED_SLIDE_LENGTH = 0.360
MIDDLE_SLIDE_THICKNESS = 0.010
MIDDLE_SLIDE_HEIGHT = 0.034
MIDDLE_SLIDE_LENGTH = 0.360
INNER_SLIDE_THICKNESS = 0.008
INNER_SLIDE_HEIGHT = 0.026
INNER_SLIDE_LENGTH = 0.250

FILE_RAIL_THICKNESS = 0.012
FILE_RAIL_HEIGHT = 0.012
FILE_RAIL_LENGTH = 0.355

HANDLE_STANDOFF_RADIUS = 0.0055
HANDLE_STANDOFF_LENGTH = 0.022
HANDLE_BAR_RADIUS = 0.007
HANDLE_BAR_LENGTH = 0.900
HANDLE_STANDOFF_X = 0.380


def _drawer_face_bottoms() -> dict[str, float]:
    bottom = FEET_HEIGHT + FACE_REVEAL
    middle = bottom + DRAWER_FACE_HEIGHT + FACE_REVEAL
    top = middle + DRAWER_FACE_HEIGHT + FACE_REVEAL
    return {"bottom": bottom, "middle": middle, "top": top}


def _opening_bottoms() -> dict[str, float]:
    bottom = FEET_HEIGHT + WALL_THICKNESS
    middle = bottom + OPENING_HEIGHT + DIVIDER_THICKNESS
    top = middle + OPENING_HEIGHT + DIVIDER_THICKNESS
    return {"bottom": bottom, "middle": middle, "top": top}


def _drawer_slide_z(level: str) -> float:
    return _drawer_face_bottoms()[level] + DRAWER_BOX_OFFSET_Z + DRAWER_BOX_HEIGHT * 0.50 - 0.004


def _add_carcass_slide_set(part, level: str, rail_material, ball_material) -> None:
    rail_z = _drawer_slide_z(level)
    rail_center_y = 0.060
    rail_x = CABINET_WIDTH * 0.5 - WALL_THICKNESS - FIXED_SLIDE_THICKNESS * 0.5 + 0.001

    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((FIXED_SLIDE_THICKNESS, FIXED_SLIDE_LENGTH, FIXED_SLIDE_HEIGHT)),
            origin=Origin(xyz=(sign * rail_x, rail_center_y, rail_z)),
            material=rail_material,
            name=f"{level}_{side_name}_slide_fixed",
        )
        for index, bearing_y in enumerate((0.170, 0.085, 0.000, -0.085)):
            part.visual(
                Sphere(radius=0.004),
                origin=Origin(xyz=(sign * rail_x, bearing_y, rail_z)),
                material=ball_material,
                name=f"{level}_{side_name}_bearing_{index}",
            )


def _add_drawer(
    part,
    *,
    level: str,
    drawer_material,
    rail_material,
    pull_material,
    with_lock: bool,
) -> None:
    half_face_width = DRAWER_FACE_WIDTH * 0.5
    half_box_width = DRAWER_BOX_WIDTH * 0.5
    face_center_z = DRAWER_FACE_HEIGHT * 0.5
    box_center_y = -DRAWER_BOX_DEPTH * 0.5
    box_center_z = DRAWER_BOX_OFFSET_Z + DRAWER_BOX_HEIGHT * 0.5
    side_center_x = half_box_width - DRAWER_SIDE_THICKNESS * 0.5
    back_center_y = -DRAWER_BOX_DEPTH + DRAWER_BACK_THICKNESS * 0.5

    part.visual(
        Box((DRAWER_FACE_WIDTH, DRAWER_FACE_THICKNESS, DRAWER_FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, DRAWER_FACE_THICKNESS * 0.5, face_center_z)),
        material=drawer_material,
        name="front_panel",
    )
    part.visual(
        Box((DRAWER_BOX_WIDTH, DRAWER_BOX_DEPTH, DRAWER_BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(0.0, box_center_y, DRAWER_BOX_OFFSET_Z + DRAWER_BOTTOM_THICKNESS * 0.5)
        ),
        material=drawer_material,
        name="drawer_bottom_pan",
    )
    part.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
        origin=Origin(xyz=(-side_center_x, box_center_y, box_center_z)),
        material=drawer_material,
        name="left_wall",
    )
    part.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
        origin=Origin(xyz=(side_center_x, box_center_y, box_center_z)),
        material=drawer_material,
        name="right_wall",
    )
    part.visual(
        Box((DRAWER_BOX_WIDTH, DRAWER_BACK_THICKNESS, DRAWER_BOX_HEIGHT)),
        origin=Origin(xyz=(0.0, back_center_y, box_center_z)),
        material=drawer_material,
        name="back_wall",
    )

    file_rail_x = half_box_width - DRAWER_SIDE_THICKNESS - FILE_RAIL_THICKNESS * 0.5 + 0.001
    file_rail_y = -0.185
    file_rail_z = DRAWER_BOX_OFFSET_Z + DRAWER_BOX_HEIGHT - 0.022
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((FILE_RAIL_THICKNESS, FILE_RAIL_LENGTH, FILE_RAIL_HEIGHT)),
            origin=Origin(xyz=(sign * file_rail_x, file_rail_y, file_rail_z)),
            material=rail_material,
            name=f"{side_name}_file_rail",
        )

    slide_z = DRAWER_BOX_OFFSET_Z + DRAWER_BOX_HEIGHT * 0.50 - 0.004
    middle_slide_x = half_box_width + MIDDLE_SLIDE_THICKNESS * 0.5 - 0.001
    inner_slide_x = half_box_width + MIDDLE_SLIDE_THICKNESS + INNER_SLIDE_THICKNESS * 0.5 - 0.004
    for side_name, sign in (("left", -1.0), ("right", 1.0)):
        part.visual(
            Box((MIDDLE_SLIDE_THICKNESS, MIDDLE_SLIDE_LENGTH, MIDDLE_SLIDE_HEIGHT)),
            origin=Origin(xyz=(sign * middle_slide_x, -0.230, slide_z)),
            material=rail_material,
            name=f"{side_name}_middle_slide",
        )
        part.visual(
            Box((INNER_SLIDE_THICKNESS, INNER_SLIDE_LENGTH, INNER_SLIDE_HEIGHT)),
            origin=Origin(xyz=(sign * inner_slide_x, -0.125, slide_z)),
            material=rail_material,
            name=f"{side_name}_inner_slide",
        )

    handle_z = DRAWER_FACE_HEIGHT * 0.53
    standoff_y = DRAWER_FACE_THICKNESS + HANDLE_STANDOFF_LENGTH * 0.5 - 0.001
    part.visual(
        Cylinder(radius=HANDLE_STANDOFF_RADIUS, length=HANDLE_STANDOFF_LENGTH),
        origin=Origin(
            xyz=(-HANDLE_STANDOFF_X, standoff_y, handle_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=pull_material,
        name="left_pull_standoff",
    )
    part.visual(
        Cylinder(radius=HANDLE_STANDOFF_RADIUS, length=HANDLE_STANDOFF_LENGTH),
        origin=Origin(
            xyz=(HANDLE_STANDOFF_X, standoff_y, handle_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=pull_material,
        name="right_pull_standoff",
    )
    part.visual(
        Cylinder(radius=HANDLE_BAR_RADIUS, length=HANDLE_BAR_LENGTH),
        origin=Origin(
            xyz=(0.0, DRAWER_FACE_THICKNESS + HANDLE_STANDOFF_LENGTH - 0.003, handle_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=pull_material,
        name="pull_bar",
    )

    if with_lock:
        lock_x = half_face_width - 0.050
        lock_z = DRAWER_FACE_HEIGHT - 0.052
        part.visual(
            Box((0.038, 0.002, 0.034)),
            origin=Origin(xyz=(lock_x, DRAWER_FACE_THICKNESS + 0.001, lock_z)),
            material=pull_material,
            name="lock_escutcheon",
        )
        part.visual(
            Cylinder(radius=0.009, length=0.010),
            origin=Origin(
                xyz=(lock_x, DRAWER_FACE_THICKNESS + 0.005, lock_z),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=rail_material,
            name="lock_cylinder",
        )
        part.visual(
            Box((0.012, 0.0015, 0.0025)),
            origin=Origin(xyz=(lock_x, DRAWER_FACE_THICKNESS + 0.009, lock_z - 0.006)),
            material=rail_material,
            name="key_slot",
        )

    part.inertial = Inertial.from_geometry(
        Box((DRAWER_FACE_WIDTH, DRAWER_FACE_THICKNESS + DRAWER_BOX_DEPTH, DRAWER_FACE_HEIGHT)),
        mass=12.0 if level == "top" else 11.0,
        origin=Origin(
            xyz=(
                0.0,
                (DRAWER_FACE_THICKNESS - DRAWER_BOX_DEPTH) * 0.5,
                DRAWER_FACE_HEIGHT * 0.5,
            )
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lateral_file_cabinet", assets=ASSETS)

    body_material = model.material("body_paint", rgba=(0.55, 0.58, 0.61, 1.0))
    drawer_material = model.material("drawer_paint", rgba=(0.64, 0.67, 0.70, 1.0))
    rail_material = model.material("slide_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    pull_material = model.material("pull_aluminum", rgba=(0.86, 0.87, 0.88, 1.0))
    foot_material = model.material("foot_black", rgba=(0.11, 0.11, 0.12, 1.0))
    bearing_material = model.material("bearing_steel", rgba=(0.69, 0.71, 0.73, 1.0))

    carcass = model.part("carcass")
    carcass.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FEET_HEIGHT + WALL_THICKNESS * 0.5)),
        material=body_material,
        name="bottom_panel",
    )
    carcass.visual(
        Box((CABINET_WIDTH, CABINET_DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FEET_HEIGHT + BODY_HEIGHT - WALL_THICKNESS * 0.5)),
        material=body_material,
        name="top_panel",
    )
    carcass.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(-CABINET_WIDTH * 0.5 + WALL_THICKNESS * 0.5, 0.0, FEET_HEIGHT + BODY_HEIGHT * 0.5)
        ),
        material=body_material,
        name="left_wall",
    )
    carcass.visual(
        Box((WALL_THICKNESS, CABINET_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(CABINET_WIDTH * 0.5 - WALL_THICKNESS * 0.5, 0.0, FEET_HEIGHT + BODY_HEIGHT * 0.5)
        ),
        material=body_material,
        name="right_wall",
    )
    carcass.visual(
        Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS, BACK_THICKNESS, BODY_HEIGHT - 2.0 * WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH * 0.5 + BACK_THICKNESS * 0.5,
                FEET_HEIGHT + WALL_THICKNESS + (BODY_HEIGHT - 2.0 * WALL_THICKNESS) * 0.5,
            )
        ),
        material=body_material,
        name="back_panel",
    )

    divider_zs = (
        _opening_bottoms()["bottom"] + OPENING_HEIGHT + DIVIDER_THICKNESS * 0.5,
        _opening_bottoms()["middle"] + OPENING_HEIGHT + DIVIDER_THICKNESS * 0.5,
    )
    for index, divider_z in enumerate(divider_zs):
        carcass.visual(
            Box((CABINET_WIDTH - 2.0 * WALL_THICKNESS, 0.018, DIVIDER_THICKNESS)),
            origin=Origin(xyz=(0.0, CABINET_DEPTH * 0.5 - 0.009, divider_z)),
            material=body_material,
            name=f"divider_lip_{index}",
        )

    foot_x = CABINET_WIDTH * 0.5 - 0.130
    foot_y = CABINET_DEPTH * 0.5 - 0.055
    for foot_name, x_sign, y_sign in (
        ("front_left", -1.0, 1.0),
        ("front_right", 1.0, 1.0),
        ("rear_left", -1.0, -1.0),
        ("rear_right", 1.0, -1.0),
    ):
        carcass.visual(
            Cylinder(radius=0.022, length=0.010),
            origin=Origin(xyz=(x_sign * foot_x, y_sign * foot_y, 0.005)),
            material=foot_material,
            name=f"{foot_name}_foot_pad",
        )
        carcass.visual(
            Cylinder(radius=0.006, length=FEET_HEIGHT - 0.010),
            origin=Origin(
                xyz=(x_sign * foot_x, y_sign * foot_y, 0.010 + (FEET_HEIGHT - 0.010) * 0.5)
            ),
            material=pull_material,
            name=f"{foot_name}_foot_stem",
        )

    for level in ("bottom", "middle", "top"):
        _add_carcass_slide_set(carcass, level, rail_material, bearing_material)

    carcass.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, BODY_HEIGHT + FEET_HEIGHT)),
        mass=62.0,
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT + FEET_HEIGHT) * 0.5)),
    )

    face_bottoms = _drawer_face_bottoms()
    for level in ("bottom", "middle", "top"):
        drawer = model.part(f"{level}_drawer")
        _add_drawer(
            drawer,
            level=level,
            drawer_material=drawer_material,
            rail_material=rail_material,
            pull_material=pull_material,
            with_lock=level == "top",
        )
        model.articulation(
            f"{level}_drawer_slide",
            ArticulationType.PRISMATIC,
            parent=carcass,
            child=drawer,
            origin=Origin(xyz=(0.0, CABINET_DEPTH * 0.5, face_bottoms[level])),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=100.0,
                velocity=0.35,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    top_drawer = object_model.get_part("top_drawer")
    middle_drawer = object_model.get_part("middle_drawer")
    bottom_drawer = object_model.get_part("bottom_drawer")
    top_slide = object_model.get_articulation("top_drawer_slide")
    middle_slide = object_model.get_articulation("middle_drawer_slide")
    bottom_slide = object_model.get_articulation("bottom_drawer_slide")

    left_wall = carcass.get_visual("left_wall")
    top_panel = carcass.get_visual("top_panel")
    bottom_panel = carcass.get_visual("bottom_panel")
    front_left_foot_pad = carcass.get_visual("front_left_foot_pad")
    front_left_foot_stem = carcass.get_visual("front_left_foot_stem")
    top_fixed_left_slide = carcass.get_visual("top_left_slide_fixed")
    middle_fixed_left_slide = carcass.get_visual("middle_left_slide_fixed")
    bottom_fixed_left_slide = carcass.get_visual("bottom_left_slide_fixed")
    top_left_bearing = carcass.get_visual("top_left_bearing_1")
    middle_left_bearing = carcass.get_visual("middle_left_bearing_1")
    bottom_left_bearing = carcass.get_visual("bottom_left_bearing_1")

    top_face = top_drawer.get_visual("front_panel")
    middle_face = middle_drawer.get_visual("front_panel")
    bottom_face = bottom_drawer.get_visual("front_panel")
    top_pull = top_drawer.get_visual("pull_bar")
    middle_pull = middle_drawer.get_visual("pull_bar")
    bottom_pull = bottom_drawer.get_visual("pull_bar")
    top_right_standoff = top_drawer.get_visual("right_pull_standoff")
    top_lock = top_drawer.get_visual("lock_cylinder")
    top_lock_plate = top_drawer.get_visual("lock_escutcheon")
    top_file_rail = top_drawer.get_visual("left_file_rail")
    middle_file_rail = middle_drawer.get_visual("left_file_rail")
    bottom_file_rail = bottom_drawer.get_visual("left_file_rail")
    top_bottom_pan = top_drawer.get_visual("drawer_bottom_pan")
    middle_bottom_pan = middle_drawer.get_visual("drawer_bottom_pan")
    bottom_bottom_pan = bottom_drawer.get_visual("drawer_bottom_pan")
    top_left_wall = top_drawer.get_visual("left_wall")
    middle_left_wall = middle_drawer.get_visual("left_wall")
    bottom_left_wall = bottom_drawer.get_visual("left_wall")
    top_middle_slide = top_drawer.get_visual("left_middle_slide")
    middle_middle_slide = middle_drawer.get_visual("left_middle_slide")
    bottom_middle_slide = bottom_drawer.get_visual("left_middle_slide")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    for drawer, face, level in (
        (top_drawer, top_face, "top"),
        (middle_drawer, middle_face, "middle"),
        (bottom_drawer, bottom_face, "bottom"),
    ):
        ctx.expect_overlap(
            drawer,
            carcass,
            elem_a=face,
            elem_b=top_panel,
            axes="x",
            min_overlap=1.19,
            name=f"{level}_drawer_is_full_width",
        )
        ctx.expect_contact(
            drawer,
            carcass,
            elem_a=face,
            elem_b=left_wall,
            name=f"{level}_drawer_face_seats_against_cabinet_front",
        )

    ctx.expect_gap(
        middle_drawer,
        bottom_drawer,
        axis="z",
        positive_elem=middle_face,
        negative_elem=bottom_face,
        min_gap=0.003,
        max_gap=0.005,
        name="bottom_and_middle_faces_have_thin_reveal",
    )
    ctx.expect_gap(
        top_drawer,
        middle_drawer,
        axis="z",
        positive_elem=top_face,
        negative_elem=middle_face,
        min_gap=0.003,
        max_gap=0.005,
        name="middle_and_top_faces_have_thin_reveal",
    )

    for drawer, pull, level in (
        (top_drawer, top_pull, "top"),
        (middle_drawer, middle_pull, "middle"),
        (bottom_drawer, bottom_pull, "bottom"),
    ):
        ctx.expect_overlap(
            drawer,
            drawer,
            elem_a=pull,
            elem_b=drawer.get_visual("front_panel"),
            axes="x",
            min_overlap=0.88,
            name=f"{level}_bar_pull_spans_most_of_drawer",
        )
        ctx.expect_gap(
            drawer,
            drawer,
            axis="y",
            positive_elem=pull,
            negative_elem=drawer.get_visual("front_panel"),
            min_gap=0.010,
            max_gap=0.014,
            name=f"{level}_bar_pull_stands_proud",
        )

    for drawer, rail, left_drawer_wall, bottom_pan, level in (
        (top_drawer, top_file_rail, top_left_wall, top_bottom_pan, "top"),
        (middle_drawer, middle_file_rail, middle_left_wall, middle_bottom_pan, "middle"),
        (bottom_drawer, bottom_file_rail, bottom_left_wall, bottom_bottom_pan, "bottom"),
    ):
        ctx.expect_contact(
            drawer,
            drawer,
            elem_a=rail,
            elem_b=left_drawer_wall,
            name=f"{level}_file_rail_mounts_to_drawer_side",
        )
        ctx.expect_gap(
            drawer,
            drawer,
            axis="z",
            positive_elem=rail,
            negative_elem=bottom_pan,
            min_gap=0.22,
            max_gap=0.24,
            name=f"{level}_file_rail_sits_high_inside_drawer",
        )
        ctx.expect_gap(
            drawer,
            drawer,
            axis="y",
            positive_elem=drawer.get_visual("front_panel"),
            negative_elem=rail,
            min_gap=0.006,
            max_gap=0.010,
            name=f"{level}_file_rail_starts_just_inside_drawer_front",
        )

    for bearing, fixed_slide, level in (
        (top_left_bearing, top_fixed_left_slide, "top"),
        (middle_left_bearing, middle_fixed_left_slide, "middle"),
        (bottom_left_bearing, bottom_fixed_left_slide, "bottom"),
    ):
        ctx.expect_within(
            carcass,
            carcass,
            inner_elem=bearing,
            outer_elem=fixed_slide,
            axes="yz",
            name=f"{level}_ball_bearing_sits_in_fixed_slide_channel",
        )

    for drawer, moving_slide, bearing, level in (
        (top_drawer, top_middle_slide, top_left_bearing, "top"),
        (middle_drawer, middle_middle_slide, middle_left_bearing, "middle"),
        (bottom_drawer, bottom_middle_slide, bottom_left_bearing, "bottom"),
    ):
        ctx.expect_overlap(
            drawer,
            carcass,
            elem_a=moving_slide,
            elem_b=bearing,
            axes="yz",
            min_overlap=0.007,
            name=f"{level}_ball_bearing_track_aligns_with_moving_slide_when_closed",
        )

    ctx.expect_within(
        top_drawer,
        top_drawer,
        inner_elem=top_lock,
        outer_elem=top_lock_plate,
        axes="xz",
    )
    ctx.expect_gap(
        top_drawer,
        top_drawer,
        axis="x",
        positive_elem=top_lock_plate,
        negative_elem=top_right_standoff,
        min_gap=0.14,
        max_gap=0.17,
        name="top_lock_sits_at_upper_right_of_top_drawer",
    )
    ctx.expect_gap(
        top_drawer,
        top_drawer,
        axis="z",
        positive_elem=top_lock_plate,
        negative_elem=top_pull,
        min_gap=0.07,
        max_gap=0.09,
        name="top_lock_is_above_top_pull_bar",
    )

    ctx.expect_gap(
        carcass,
        carcass,
        axis="z",
        positive_elem=bottom_panel,
        negative_elem=front_left_foot_pad,
        min_gap=0.017,
        max_gap=0.019,
        name="cabinet_body_sits_on_leveling_feet",
    )
    ctx.expect_contact(
        carcass,
        carcass,
        elem_a=front_left_foot_stem,
        elem_b=bottom_panel,
        name="leveling_foot_stem_reaches_cabinet_body",
    )

    with ctx.pose({top_slide: DRAWER_TRAVEL}):
        ctx.expect_gap(
            top_drawer,
            carcass,
            axis="y",
            positive_elem=top_face,
            negative_elem=left_wall,
            min_gap=0.27,
            max_gap=0.29,
            name="top_drawer_opens_forward",
        )
        ctx.expect_overlap(
            top_drawer,
            carcass,
            elem_a=top_middle_slide,
            elem_b=top_fixed_left_slide,
            axes="y",
            min_overlap=0.10,
            name="top_slide_stays_engaged_at_full_extension",
        )

    with ctx.pose({middle_slide: DRAWER_TRAVEL}):
        ctx.expect_gap(
            middle_drawer,
            carcass,
            axis="y",
            positive_elem=middle_face,
            negative_elem=left_wall,
            min_gap=0.27,
            max_gap=0.29,
            name="middle_drawer_opens_forward",
        )
        ctx.expect_overlap(
            middle_drawer,
            carcass,
            elem_a=middle_middle_slide,
            elem_b=middle_fixed_left_slide,
            axes="y",
            min_overlap=0.10,
            name="middle_slide_stays_engaged_at_full_extension",
        )

    with ctx.pose({bottom_slide: DRAWER_TRAVEL}):
        ctx.expect_gap(
            bottom_drawer,
            carcass,
            axis="y",
            positive_elem=bottom_face,
            negative_elem=left_wall,
            min_gap=0.27,
            max_gap=0.29,
            name="bottom_drawer_opens_forward",
        )
        ctx.expect_overlap(
            bottom_drawer,
            carcass,
            elem_a=bottom_middle_slide,
            elem_b=bottom_fixed_left_slide,
            axes="y",
            min_overlap=0.10,
            name="bottom_slide_stays_engaged_at_full_extension",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
