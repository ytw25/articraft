from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")
        return "/"


os.getcwd = _safe_getcwd
if _REAL_GETCWD is not _safe_getcwd:
    try:
        _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/")

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

SCRIPT_DIR = Path(__file__).parent if "__file__" in globals() else Path("/")
os.chdir(str(SCRIPT_DIR) if SCRIPT_DIR.exists() else "/")
HERE = SCRIPT_DIR


def _add_cage_nut_strip(
    chassis,
    *,
    side: str,
    x: float,
    rail_y: float,
    strip_material,
    void_material,
) -> None:
    chassis.visual(
        Box((0.010, 0.004, 0.162)),
        origin=Origin(xyz=(x, rail_y - 0.0005, 0.089)),
        material=strip_material,
        name=f"{side}_cage_strip",
    )

    hole_zs = [0.018 + i * 0.013 for i in range(12)]
    for index, z in enumerate(hole_zs, start=1):
        chassis.visual(
            Box((0.0075, 0.0018, 0.0075)),
            origin=Origin(xyz=(x, rail_y - 0.0021, z)),
            material=void_material,
            name=f"{side}_cage_hole_{index:02d}",
        )
        chassis.visual(
            Cylinder(radius=0.0019, length=0.0026),
            origin=Origin(
                xyz=(x, rail_y - 0.0023, z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=void_material,
            name=f"{side}_thread_opening_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rackmount_chassis")

    chassis_gray = model.material("chassis_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.44, 0.46, 0.49, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.12, 0.12, 0.13, 1.0))
    bezel_inset = model.material("bezel_inset", rgba=(0.07, 0.08, 0.09, 1.0))
    void_black = model.material("void_black", rgba=(0.03, 0.03, 0.03, 1.0))
    zinc = model.material("zinc", rgba=(0.70, 0.72, 0.75, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.438, 0.420, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=chassis_gray,
        name="floor_pan",
    )
    chassis.visual(
        Box((0.003, 0.420, 0.172)),
        origin=Origin(xyz=(-0.2175, 0.0, 0.086)),
        material=chassis_gray,
        name="left_wall",
    )
    chassis.visual(
        Box((0.003, 0.420, 0.172)),
        origin=Origin(xyz=(0.2175, 0.0, 0.086)),
        material=chassis_gray,
        name="right_wall",
    )
    chassis.visual(
        Box((0.432, 0.003, 0.172)),
        origin=Origin(xyz=(0.0, 0.2085, 0.086)),
        material=chassis_gray,
        name="rear_wall",
    )
    chassis.visual(
        Box((0.416, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.205, 0.008)),
        material=chassis_gray,
        name="front_bottom_crossbar",
    )
    chassis.visual(
        Box((0.416, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.205, 0.164)),
        material=chassis_gray,
        name="front_top_crossbar",
    )
    chassis.visual(
        Box((0.004, 0.022, 0.1778)),
        origin=Origin(xyz=(-0.220, -0.221, 0.0889)),
        material=chassis_gray,
        name="left_rail_web",
    )
    chassis.visual(
        Box((0.020, 0.022, 0.1778)),
        origin=Origin(xyz=(-0.231, -0.221, 0.0889)),
        material=chassis_gray,
        name="left_rail",
    )
    chassis.visual(
        Box((0.004, 0.022, 0.1778)),
        origin=Origin(xyz=(0.220, -0.221, 0.0889)),
        material=chassis_gray,
        name="right_rail_web",
    )
    chassis.visual(
        Box((0.020, 0.022, 0.1778)),
        origin=Origin(xyz=(0.231, -0.221, 0.0889)),
        material=chassis_gray,
        name="right_rail",
    )
    _add_cage_nut_strip(
        chassis,
        side="left",
        x=-0.231,
        rail_y=-0.221,
        strip_material=bezel_black,
        void_material=void_black,
    )
    _add_cage_nut_strip(
        chassis,
        side="right",
        x=0.231,
        rail_y=-0.221,
        strip_material=bezel_black,
        void_material=void_black,
    )
    chassis.visual(
        Box((0.012, 0.012, 0.046)),
        origin=Origin(xyz=(-0.202, -0.208, 0.086)),
        material=chassis_gray,
        name="left_latch_catch",
    )
    chassis.visual(
        Box((0.012, 0.012, 0.156)),
        origin=Origin(xyz=(-0.202, -0.205, 0.086)),
        material=chassis_gray,
        name="left_front_post",
    )
    chassis.visual(
        Box((0.012, 0.012, 0.046)),
        origin=Origin(xyz=(0.202, -0.208, 0.086)),
        material=chassis_gray,
        name="right_latch_catch",
    )
    chassis.visual(
        Box((0.012, 0.012, 0.156)),
        origin=Origin(xyz=(0.202, -0.205, 0.086)),
        material=chassis_gray,
        name="right_front_post",
    )
    for side, x in (("left", -0.168), ("right", 0.168)):
        chassis.visual(
            Box((0.018, 0.0016, 0.016)),
            origin=Origin(xyz=(x, 0.2097, 0.160)),
            material=zinc,
            name=f"{side}_thumbscrew_plate",
        )
        chassis.visual(
            Cylinder(radius=0.0042, length=0.0036),
            origin=Origin(
                xyz=(x, 0.2082, 0.160),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=void_black,
            name=f"{side}_threaded_insert",
        )
    chassis.visual(
        Box((0.044, 0.0016, 0.118)),
        origin=Origin(xyz=(0.168, 0.2093, 0.090)),
        material=void_black,
        name="expansion_slot_stack",
    )
    chassis.visual(
        Box((0.014, 0.002, 0.110)),
        origin=Origin(xyz=(0.1595, 0.2105, 0.090)),
        material=zinc,
        name="blanking_panel_left",
    )
    chassis.visual(
        Box((0.014, 0.002, 0.110)),
        origin=Origin(xyz=(0.1765, 0.2105, 0.090)),
        material=zinc,
        name="blanking_panel_right",
    )

    bezel = model.part("bezel")
    bezel.visual(
        Box((0.482, 0.012, 0.176)),
        origin=Origin(xyz=(0.202, -0.030, 0.002)),
        material=bezel_black,
        name="bezel_panel",
    )
    bezel.visual(
        Box((0.392, 0.0025, 0.112)),
        origin=Origin(xyz=(0.202, -0.0325, 0.008)),
        material=bezel_inset,
        name="bezel_inset",
    )
    bezel.visual(
        Box((0.012, 0.018, 0.046)),
        origin=Origin(xyz=(0.0, -0.015, 0.0)),
        material=bezel_black,
        name="left_latch_tab",
    )
    bezel.visual(
        Box((0.012, 0.018, 0.046)),
        origin=Origin(xyz=(0.404, -0.015, 0.0)),
        material=bezel_black,
        name="right_latch_tab",
    )

    cover = model.part("cover")
    cover.visual(
        Box((0.446, 0.424, 0.003)),
        origin=Origin(xyz=(0.0, -0.205, 0.0135)),
        material=cover_gray,
        name="cover_top",
    )
    cover.visual(
        Box((0.003, 0.424, 0.020)),
        origin=Origin(xyz=(-0.2245, -0.205, 0.002)),
        material=cover_gray,
        name="left_cover_flange",
    )
    cover.visual(
        Box((0.003, 0.424, 0.020)),
        origin=Origin(xyz=(0.2245, -0.205, 0.002)),
        material=cover_gray,
        name="right_cover_flange",
    )
    cover.visual(
        Box((0.446, 0.003, 0.022)),
        origin=Origin(xyz=(0.0, 0.006, 0.001)),
        material=cover_gray,
        name="cover_rear_flange",
    )
    for side, x in (("left", -0.168), ("right", 0.168)):
        cover.visual(
            Cylinder(radius=0.003, length=0.009),
            origin=Origin(
                xyz=(x, 0.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc,
            name=f"{side}_thumbscrew_shaft",
        )
        cover.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(
                xyz=(x, 0.0075, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc,
            name=f"{side}_thumbscrew_head",
        )
        cover.visual(
            Box((0.016, 0.002, 0.004)),
            origin=Origin(xyz=(x, 0.0075, 0.0)),
            material=zinc,
            name=f"{side}_thumbscrew_bar",
        )

    model.articulation(
        "bezel_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=bezel,
        origin=Origin(xyz=(-0.202, -0.208, 0.086)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.15,
            lower=0.0,
            upper=0.04,
        ),
    )
    model.articulation(
        "cover_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=cover,
        origin=Origin(xyz=(0.0, 0.2145, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    chassis = object_model.get_part("chassis")
    bezel = object_model.get_part("bezel")
    cover = object_model.get_part("cover")
    bezel_slide = object_model.get_articulation("bezel_slide")
    cover_slide = object_model.get_articulation("cover_slide")

    left_rail = chassis.get_visual("left_rail")
    right_rail = chassis.get_visual("right_rail")
    left_strip = chassis.get_visual("left_cage_strip")
    right_strip = chassis.get_visual("right_cage_strip")
    left_bottom_hole = chassis.get_visual("left_cage_hole_01")
    left_top_hole = chassis.get_visual("left_cage_hole_12")
    left_threaded_insert = chassis.get_visual("left_threaded_insert")
    right_threaded_insert = chassis.get_visual("right_threaded_insert")
    expansion_slot_stack = chassis.get_visual("expansion_slot_stack")
    blanking_panel_left = chassis.get_visual("blanking_panel_left")
    blanking_panel_right = chassis.get_visual("blanking_panel_right")
    left_latch_catch = chassis.get_visual("left_latch_catch")
    right_latch_catch = chassis.get_visual("right_latch_catch")

    left_latch_tab = bezel.get_visual("left_latch_tab")
    right_latch_tab = bezel.get_visual("right_latch_tab")

    left_thumbscrew_shaft = cover.get_visual("left_thumbscrew_shaft")
    right_thumbscrew_shaft = cover.get_visual("right_thumbscrew_shaft")

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

    ctx.expect_gap(
        chassis,
        chassis,
        axis="x",
        min_gap=0.44,
        positive_elem=right_rail,
        negative_elem=left_rail,
        name="side rails span full rack width",
    )
    ctx.expect_within(
        chassis,
        chassis,
        axes="xz",
        inner_elem=left_strip,
        outer_elem=left_rail,
    )
    ctx.expect_within(
        chassis,
        chassis,
        axes="xz",
        inner_elem=right_strip,
        outer_elem=right_rail,
    )
    ctx.expect_gap(
        chassis,
        chassis,
        axis="z",
        min_gap=0.13,
        positive_elem=left_top_hole,
        negative_elem=left_bottom_hole,
        name="left cage strip shows a full 4U hole run",
    )
    ctx.expect_within(
        chassis,
        chassis,
        axes="xz",
        inner_elem=blanking_panel_left,
        outer_elem=expansion_slot_stack,
    )
    ctx.expect_within(
        chassis,
        chassis,
        axes="xz",
        inner_elem=blanking_panel_right,
        outer_elem=expansion_slot_stack,
    )
    ctx.expect_gap(
        chassis,
        chassis,
        axis="x",
        min_gap=0.002,
        positive_elem=blanking_panel_right,
        negative_elem=blanking_panel_left,
        name="rear blanking panels are distinct slot covers",
    )
    ctx.expect_overlap(bezel, chassis, axes="xz", min_overlap=0.07)
    ctx.expect_gap(
        chassis,
        bezel,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem=left_latch_catch,
        negative_elem=left_latch_tab,
    )
    ctx.expect_gap(
        chassis,
        bezel,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem=right_latch_catch,
        negative_elem=right_latch_tab,
    )
    with ctx.pose({bezel_slide: 0.03}):
        ctx.expect_gap(
            chassis,
            bezel,
            axis="y",
            min_gap=0.02,
            positive_elem=left_latch_catch,
            negative_elem=left_latch_tab,
        )
        ctx.expect_gap(
            chassis,
            bezel,
            axis="y",
            min_gap=0.02,
            positive_elem=right_latch_catch,
            negative_elem=right_latch_tab,
        )

    ctx.expect_overlap(cover, chassis, axes="xy", min_overlap=0.15)
    ctx.expect_gap(
        cover,
        chassis,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_thumbscrew_shaft,
        negative_elem=left_threaded_insert,
    )
    ctx.expect_gap(
        cover,
        chassis,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_thumbscrew_shaft,
        negative_elem=right_threaded_insert,
    )
    with ctx.pose({cover_slide: 0.16}):
        ctx.expect_gap(
            cover,
            chassis,
            axis="y",
            min_gap=0.12,
            positive_elem=left_thumbscrew_shaft,
            negative_elem=left_threaded_insert,
        )
        ctx.expect_gap(
            cover,
            chassis,
            axis="y",
            min_gap=0.12,
            positive_elem=right_thumbscrew_shaft,
            negative_elem=right_threaded_insert,
        )
        ctx.expect_overlap(cover, chassis, axes="xy", min_overlap=0.08)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
