from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
from pathlib import Path

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/tmp")
        return "/tmp"


os.getcwd = _safe_getcwd


def _ensure_runtime_cwd() -> None:
    _safe_getcwd()


try:
    os.getcwd()
except FileNotFoundError:
    _ensure_runtime_cwd()

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

CASE_W = 0.28
CASE_D = 0.56
CASE_H = 0.62
TOP_PANEL_W = 0.22
TOP_PANEL_D = 0.40
TOP_PANEL_T = 0.004
TOP_PANEL_LOCAL_OFFSET = (0.095, -0.175, 0.007)
ASSET_ROOT = Path("/tmp")
ASSETS = AssetContext(ASSET_ROOT)


def build_object_model() -> ArticulatedObject:
    _ensure_runtime_cwd()
    model = ArticulatedObject(name="gaming_tower_case", assets=ASSETS)

    body_dark = model.material("body_dark", rgba=(0.10, 0.11, 0.12, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    steel = model.material("steel", rgba=(0.45, 0.47, 0.50, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    glass = model.material("tempered_glass", rgba=(0.52, 0.62, 0.68, 0.25))
    mesh_dark = model.material("mesh_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    latch_dark = model.material("latch_dark", rgba=(0.20, 0.20, 0.22, 1.0))
    drive_dark = model.material("drive_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    tray_dark = model.material("tray_dark", rgba=(0.30, 0.31, 0.33, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((CASE_W, CASE_D, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=body_dark,
        name="floor",
    )
    chassis.visual(
        Box((CASE_W, 0.03, CASE_H)),
        origin=Origin(xyz=(0.0, 0.265, CASE_H * 0.5)),
        material=body_dark,
        name="rear_wall",
    )
    chassis.visual(
        Box((0.03, 0.03, CASE_H)),
        origin=Origin(xyz=(-0.125, -0.265, CASE_H * 0.5)),
        material=body_dark,
        name="front_left_pillar",
    )
    chassis.visual(
        Box((0.03, 0.03, CASE_H)),
        origin=Origin(xyz=(0.125, -0.265, CASE_H * 0.5)),
        material=body_dark,
        name="front_right_pillar",
    )
    chassis.visual(
        Box((0.03, 0.50, 0.05)),
        origin=Origin(xyz=(-0.125, 0.0, 0.055)),
        material=panel_dark,
        name="left_bottom_rail",
    )
    chassis.visual(
        Box((0.03, 0.50, 0.05)),
        origin=Origin(xyz=(-0.125, 0.0, 0.565)),
        material=panel_dark,
        name="left_top_rail",
    )
    chassis.visual(
        Box((0.03, 0.06, 0.52)),
        origin=Origin(xyz=(-0.125, 0.25, 0.31)),
        material=panel_dark,
        name="left_rear_post",
    )
    chassis.visual(
        Box((0.03, CASE_D, 0.03)),
        origin=Origin(xyz=(-0.125, 0.0, 0.605)),
        material=body_dark,
        name="top_left_rail",
    )
    chassis.visual(
        Box((0.03, CASE_D, 0.03)),
        origin=Origin(xyz=(0.125, 0.0, 0.605)),
        material=body_dark,
        name="top_right_rail",
    )
    chassis.visual(
        Box((CASE_W - 0.06, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, -0.265, 0.605)),
        material=body_dark,
        name="top_front_rail",
    )
    chassis.visual(
        Box((CASE_W - 0.06, 0.03, 0.03)),
        origin=Origin(xyz=(0.0, 0.265, 0.605)),
        material=body_dark,
        name="top_rear_rail",
    )
    chassis.visual(
        Box((0.006, 0.50, 0.54)),
        origin=Origin(xyz=(0.137, 0.0, 0.31)),
        material=panel_dark,
        name="right_side_panel",
    )
    chassis.visual(
        Box((0.01, 0.48, 0.46)),
        origin=Origin(xyz=(0.060, 0.0, 0.32)),
        material=steel,
        name="motherboard_tray",
    )
    chassis.visual(
        Box((0.22, 0.38, 0.11)),
        origin=Origin(xyz=(0.0, 0.070, 0.085)),
        material=panel_dark,
        name="psu_shroud",
    )
    chassis.visual(
        Box((0.22, 0.03, 0.04)),
        origin=Origin(xyz=(0.0, -0.265, 0.58)),
        material=panel_dark,
        name="front_header",
    )
    chassis.visual(
        Box((0.008, 0.18, 0.36)),
        origin=Origin(xyz=(-0.060, -0.140, 0.21)),
        material=drive_dark,
        name="drive_bay_left_wall",
    )
    chassis.visual(
        Box((0.008, 0.18, 0.36)),
        origin=Origin(xyz=(0.060, -0.140, 0.21)),
        material=drive_dark,
        name="drive_bay_right_wall",
    )
    chassis.visual(
        Box((0.12, 0.008, 0.36)),
        origin=Origin(xyz=(0.0, -0.054, 0.21)),
        material=drive_dark,
        name="drive_bay_back",
    )
    for index, z in enumerate((0.10, 0.18, 0.26, 0.34)):
        chassis.visual(
            Box((0.12, 0.16, 0.01)),
            origin=Origin(xyz=(0.0, -0.140, z)),
            material=drive_dark,
            name=f"drive_shelf_{index}",
        )
    for index, z in enumerate((0.11, 0.19, 0.27, 0.35)):
        chassis.visual(
            Box((0.106, 0.012, 0.05)),
            origin=Origin(xyz=(0.0, -0.226, z)),
            material=tray_dark,
            name=f"tray_front_{index}",
        )
    chassis.visual(
        Box((0.12, 0.012, 0.34)),
        origin=Origin(xyz=(0.0, -0.226, 0.24)),
        material=tray_dark,
        name="drive_bay_face",
    )
    chassis.visual(
        Box((0.014, 0.008, 0.05)),
        origin=Origin(xyz=(0.122, -0.276, 0.32)),
        material=hinge_steel,
        name="door_strike",
    )
    chassis.visual(
        Box((0.010, 0.030, 0.06)),
        origin=Origin(xyz=(-0.138, -0.245, 0.32)),
        material=hinge_steel,
        name="side_cam_strike",
    )
    chassis.visual(
        Box((0.004, 0.014, 0.52)),
        origin=Origin(xyz=(-0.141, 0.243, 0.31)),
        material=hinge_steel,
        name="side_hinge_leaf",
    )
    chassis.visual(
        Cylinder(radius=0.005, length=0.54),
        origin=Origin(xyz=(-0.136, 0.256, 0.31)),
        material=hinge_steel,
        name="side_hinge_barrel",
    )
    chassis.visual(
        Box((0.012, 0.010, 0.52)),
        origin=Origin(xyz=(-0.130, -0.275, 0.31)),
        material=hinge_steel,
        name="front_hinge_leaf",
    )
    for name, z in (("front_hinge_top", 0.49), ("front_hinge_mid", 0.31), ("front_hinge_bottom", 0.13)):
        chassis.visual(
            Cylinder(radius=0.0035, length=0.05),
            origin=Origin(xyz=(-0.136, -0.276, z)),
            material=hinge_steel,
            name=name,
        )
    for suffix, x, y in (
        ("nw", -0.095, 0.175),
        ("ne", 0.095, 0.175),
        ("sw", -0.095, -0.175),
        ("se", 0.095, -0.175),
    ):
        chassis.visual(
            Box((0.024, 0.016, 0.014)),
            origin=Origin(xyz=(-0.104 if x < 0.0 else 0.104, y, 0.607)),
            material=panel_dark,
            name=f"socket_pad_{suffix}",
        )
        chassis.visual(
            Cylinder(radius=0.004, length=0.010),
            origin=Origin(xyz=(x, y, 0.615)),
            material=hinge_steel,
            name=f"socket_{suffix}",
        )
    chassis.inertial = Inertial.from_geometry(
        Box((CASE_W, CASE_D, CASE_H)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, CASE_H * 0.5)),
    )

    side_panel = model.part("side_panel")
    side_panel.visual(
        Box((0.004, 0.50, 0.52)),
        origin=Origin(xyz=(-0.004, -0.25, 0.0)),
        material=glass,
        name="glass",
    )
    side_panel.visual(
        Box((0.008, 0.50, 0.018)),
        origin=Origin(xyz=(-0.004, -0.25, 0.251)),
        material=panel_dark,
        name="frame_top",
    )
    side_panel.visual(
        Box((0.008, 0.50, 0.018)),
        origin=Origin(xyz=(-0.004, -0.25, -0.251)),
        material=panel_dark,
        name="frame_bottom",
    )
    side_panel.visual(
        Box((0.008, 0.018, 0.46)),
        origin=Origin(xyz=(-0.004, -0.491, 0.0)),
        material=panel_dark,
        name="frame_front",
    )
    side_panel.visual(
        Box((0.003, 0.014, 0.52)),
        origin=Origin(xyz=(-0.0015, -0.007, 0.0)),
        material=hinge_steel,
        name="hinge_strip",
    )
    side_panel.visual(
        Box((0.010, 0.022, 0.052)),
        origin=Origin(xyz=(-0.005, -0.489, 0.02)),
        material=latch_dark,
        name="cam_latch",
    )
    side_panel.visual(
        Box((0.008, 0.016, 0.078)),
        origin=Origin(xyz=(-0.012, -0.489, 0.02)),
        material=hinge_steel,
        name="cam_handle",
    )
    side_panel.inertial = Inertial.from_geometry(
        Box((0.02, 0.50, 0.52)),
        mass=4.2,
        origin=Origin(xyz=(-0.010, -0.25, 0.0)),
    )

    front_door = model.part("front_door")
    front_door.visual(
        Box((0.246, 0.018, 0.54)),
        origin=Origin(xyz=(0.133, -0.009, 0.0)),
        material=panel_dark,
        name="door_outer",
    )
    front_door.visual(
        Box((0.206, 0.010, 0.48)),
        origin=Origin(xyz=(0.133, -0.005, 0.0)),
        material=body_dark,
        name="door_inner_recess",
    )
    front_door.visual(
        Box((0.018, 0.008, 0.05)),
        origin=Origin(xyz=(0.246, -0.004, 0.02)),
        material=hinge_steel,
        name="magnet_latch",
    )
    front_door.inertial = Inertial.from_geometry(
        Box((0.258, 0.02, 0.54)),
        mass=3.2,
        origin=Origin(xyz=(0.129, -0.010, 0.0)),
    )

    top_panel = model.part("top_panel")
    top_panel.visual(
        Box((TOP_PANEL_W, 0.014, TOP_PANEL_T)),
        origin=Origin(
            xyz=(
                TOP_PANEL_LOCAL_OFFSET[0],
                TOP_PANEL_LOCAL_OFFSET[1] + TOP_PANEL_D * 0.5 - 0.007,
                TOP_PANEL_LOCAL_OFFSET[2],
            )
        ),
        material=panel_dark,
        name="frame_rear",
    )
    top_panel.visual(
        Box((TOP_PANEL_W, 0.014, TOP_PANEL_T)),
        origin=Origin(
            xyz=(
                TOP_PANEL_LOCAL_OFFSET[0],
                TOP_PANEL_LOCAL_OFFSET[1] - TOP_PANEL_D * 0.5 + 0.007,
                TOP_PANEL_LOCAL_OFFSET[2],
            )
        ),
        material=panel_dark,
        name="frame_front",
    )
    top_panel.visual(
        Box((0.014, TOP_PANEL_D, TOP_PANEL_T)),
        origin=Origin(
            xyz=(
                TOP_PANEL_LOCAL_OFFSET[0] - TOP_PANEL_W * 0.5 + 0.007,
                TOP_PANEL_LOCAL_OFFSET[1],
                TOP_PANEL_LOCAL_OFFSET[2],
            )
        ),
        material=panel_dark,
        name="frame_left",
    )
    top_panel.visual(
        Box((0.014, TOP_PANEL_D, TOP_PANEL_T)),
        origin=Origin(
            xyz=(
                TOP_PANEL_LOCAL_OFFSET[0] + TOP_PANEL_W * 0.5 - 0.007,
                TOP_PANEL_LOCAL_OFFSET[1],
                TOP_PANEL_LOCAL_OFFSET[2],
            )
        ),
        material=panel_dark,
        name="frame_right",
    )
    for index, x in enumerate((-0.078, -0.052, -0.026, 0.0, 0.026, 0.052, 0.078)):
        top_panel.visual(
            Box((0.008, 0.372, TOP_PANEL_T)),
            origin=Origin(
                xyz=(
                    TOP_PANEL_LOCAL_OFFSET[0] + x,
                    TOP_PANEL_LOCAL_OFFSET[1],
                    TOP_PANEL_LOCAL_OFFSET[2],
                )
            ),
            material=mesh_dark,
            name=f"mesh_bar_{index}",
        )
    for index, y in enumerate((-0.120, -0.060, 0.0, 0.060, 0.120)):
        top_panel.visual(
            Box((0.194, 0.004, TOP_PANEL_T)),
            origin=Origin(
                xyz=(
                    TOP_PANEL_LOCAL_OFFSET[0],
                    TOP_PANEL_LOCAL_OFFSET[1] + y,
                    TOP_PANEL_LOCAL_OFFSET[2] + 0.0005,
                )
            ),
            material=mesh_dark,
            name=f"mesh_cross_{index}",
        )
    for suffix, x, y in (
        ("nw", -0.095, 0.175),
        ("ne", 0.095, 0.175),
        ("sw", -0.095, -0.175),
        ("se", 0.095, -0.175),
    ):
        top_panel.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(
                xyz=(
                    TOP_PANEL_LOCAL_OFFSET[0] + x,
                    TOP_PANEL_LOCAL_OFFSET[1] + y,
                    TOP_PANEL_LOCAL_OFFSET[2] + 0.004,
                )
            ),
            material=hinge_steel,
            name=f"pin_{suffix}",
        )
        top_panel.visual(
            Cylinder(radius=0.003, length=0.010),
            origin=Origin(
                xyz=(
                    TOP_PANEL_LOCAL_OFFSET[0] + x,
                    TOP_PANEL_LOCAL_OFFSET[1] + y,
                    TOP_PANEL_LOCAL_OFFSET[2] - 0.007,
                )
            ),
            material=hinge_steel,
            name=f"pin_{suffix}_shaft",
        )
    top_panel.inertial = Inertial.from_geometry(
        Box((TOP_PANEL_W, TOP_PANEL_D, 0.02)),
        mass=0.9,
        origin=Origin(xyz=TOP_PANEL_LOCAL_OFFSET),
    )

    model.articulation(
        "side_glass_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=side_panel,
        origin=Origin(xyz=(-0.143, 0.250, 0.31)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "front_door_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=front_door,
        origin=Origin(xyz=(-0.129, -0.280, 0.31)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "top_panel_mount",
        ArticulationType.FIXED,
        parent=chassis,
        child=top_panel,
        origin=Origin(xyz=(-0.095, 0.175, 0.615)),
    )

    return model


def run_tests() -> TestReport:
    _ensure_runtime_cwd()
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    side_panel = object_model.get_part("side_panel")
    front_door = object_model.get_part("front_door")
    top_panel = object_model.get_part("top_panel")
    side_glass_hinge = object_model.get_articulation("side_glass_hinge")
    front_door_hinge = object_model.get_articulation("front_door_hinge")

    side_hinge_strip = side_panel.get_visual("hinge_strip")
    cam_latch = side_panel.get_visual("cam_latch")
    side_glass = side_panel.get_visual("glass")
    door_outer = front_door.get_visual("door_outer")
    door_magnet = front_door.get_visual("magnet_latch")
    top_mesh = top_panel.get_visual("mesh_bar_3")

    side_hinge_leaf = chassis.get_visual("side_hinge_leaf")
    side_cam_strike = chassis.get_visual("side_cam_strike")
    front_left_pillar = chassis.get_visual("front_left_pillar")
    front_right_pillar = chassis.get_visual("front_right_pillar")
    door_strike = chassis.get_visual("door_strike")
    drive_bay_face = chassis.get_visual("drive_bay_face")
    motherboard_tray = chassis.get_visual("motherboard_tray")
    left_bottom_rail = chassis.get_visual("left_bottom_rail")
    top_left_rail = chassis.get_visual("top_left_rail")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(side_panel, chassis, axes="yz", min_overlap=0.20)
    ctx.expect_gap(
        chassis,
        side_panel,
        axis="x",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=left_bottom_rail,
        negative_elem=side_glass,
    )
    ctx.expect_overlap(side_panel, chassis, axes="yz", min_overlap=0.010, elem_a=side_hinge_strip, elem_b=side_hinge_leaf)
    ctx.expect_gap(
        chassis,
        side_panel,
        axis="x",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem=side_hinge_leaf,
        negative_elem=side_hinge_strip,
    )
    ctx.expect_overlap(side_panel, chassis, axes="yz", min_overlap=0.015, elem_a=cam_latch, elem_b=side_cam_strike)
    ctx.expect_gap(
        chassis,
        side_panel,
        axis="x",
        max_gap=0.003,
        max_penetration=0.001,
        positive_elem=side_cam_strike,
        negative_elem=cam_latch,
    )

    ctx.expect_overlap(front_door, chassis, axes="xz", min_overlap=0.20)
    ctx.expect_gap(
        chassis,
        front_door,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=front_left_pillar,
        negative_elem=door_outer,
    )
    ctx.expect_gap(
        chassis,
        front_door,
        axis="y",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=front_right_pillar,
        negative_elem=door_outer,
    )
    ctx.expect_overlap(front_door, chassis, axes="xz", min_overlap=0.008, elem_a=door_outer, elem_b=front_left_pillar)
    ctx.expect_overlap(front_door, chassis, axes="xz", min_overlap=0.010, elem_a=door_magnet, elem_b=door_strike)
    ctx.expect_gap(
        chassis,
        front_door,
        axis="y",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem=door_strike,
        negative_elem=door_magnet,
    )
    ctx.expect_overlap(chassis, front_door, axes="xz", min_overlap=0.10, elem_a=drive_bay_face, elem_b=door_outer)
    ctx.expect_gap(
        chassis,
        front_door,
        axis="y",
        min_gap=0.03,
        max_gap=0.08,
        positive_elem=drive_bay_face,
        negative_elem=door_outer,
    )

    ctx.expect_overlap(top_panel, chassis, axes="xy", min_overlap=0.15)
    ctx.expect_gap(
        top_panel,
        chassis,
        axis="z",
        max_gap=0.003,
        max_penetration=0.0,
        positive_elem=top_mesh,
        negative_elem=top_left_rail,
    )
    for suffix in ("nw", "ne", "sw", "se"):
        ctx.expect_overlap(
            top_panel,
            chassis,
            axes="xy",
            min_overlap=0.004,
            elem_a=top_panel.get_visual(f"pin_{suffix}_shaft"),
            elem_b=chassis.get_visual(f"socket_{suffix}"),
        )

    with ctx.pose({side_glass_hinge: 1.20}):
        ctx.expect_gap(
            chassis,
            side_panel,
            axis="x",
            min_gap=0.15,
            positive_elem=motherboard_tray,
            negative_elem=side_glass,
        )
        ctx.expect_gap(
            chassis,
            side_panel,
            axis="x",
            min_gap=0.10,
            positive_elem=side_cam_strike,
            negative_elem=cam_latch,
        )

    with ctx.pose({front_door_hinge: 1.35}):
        ctx.expect_gap(
            chassis,
            front_door,
            axis="y",
            min_gap=0.12,
            positive_elem=door_strike,
            negative_elem=door_magnet,
        )
        ctx.expect_gap(
            chassis,
            front_door,
            axis="y",
            min_gap=0.05,
            positive_elem=drive_bay_face,
            negative_elem=door_outer,
        )

    return ctx.report()


# >>> USER_CODE_END

_ensure_runtime_cwd()
object_model = build_object_model()
