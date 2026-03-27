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
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _canopy_loop(
    rear_width: float,
    shoulder_width: float,
    front_width: float,
    rear_y: float,
    shoulder_y: float,
    front_y: float,
    nose_y: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [
        (-0.50 * rear_width, rear_y, z),
        (0.50 * rear_width, rear_y, z),
        (0.50 * shoulder_width, shoulder_y, z),
        (0.50 * front_width, front_y, z),
        (0.22 * front_width, nose_y, z),
        (0.0, nose_y + 0.003, z),
        (-0.22 * front_width, nose_y, z),
        (-0.50 * front_width, front_y, z),
        (-0.50 * shoulder_width, shoulder_y, z),
    ]


def _build_canopy_mesh():
    return repair_loft(
        section_loft(
            [
                _canopy_loop(0.036, 0.033, 0.027, -0.018, -0.005, 0.009, 0.015, 0.000),
                _canopy_loop(0.032, 0.029, 0.022, -0.015, -0.003, 0.008, 0.014, 0.004),
                _canopy_loop(0.024, 0.021, 0.016, -0.010, -0.001, 0.006, 0.011, 0.008),
                _canopy_loop(0.015, 0.013, 0.009, -0.004, 0.001, 0.003, 0.006, 0.011),
            ]
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="micro_racing_drone", assets=ASSETS)

    carbon = model.material("carbon_black", rgba=(0.10, 0.11, 0.12, 1.0))
    board_green = model.material("board_green", rgba=(0.14, 0.38, 0.20, 1.0))
    electronics_black = model.material("electronics_black", rgba=(0.07, 0.08, 0.09, 1.0))
    canopy_smoke = model.material("canopy_smoke", rgba=(0.35, 0.28, 0.20, 0.78))
    battery_grey = model.material("battery_grey", rgba=(0.33, 0.35, 0.38, 1.0))
    cell_blue = model.material("cell_blue", rgba=(0.18, 0.29, 0.62, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.026, 0.026, 0.0024)),
        origin=Origin(xyz=(0.0, 0.0, 0.0012), rpy=(0.0, 0.0, math.pi / 4.0)),
        material=carbon,
        name="center_plate",
    )
    frame.visual(
        Box((0.028, 0.028, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0044)),
        material=board_green,
        name="electronics_board",
    )
    frame.visual(
        Box((0.012, 0.010, 0.0032)),
        origin=Origin(xyz=(0.0, 0.002, 0.0068)),
        material=electronics_black,
        name="processor_stack",
    )
    frame.visual(
        Box((0.010, 0.006, 0.0020)),
        origin=Origin(xyz=(0.0, -0.008, 0.0060)),
        material=electronics_black,
        name="receiver_block",
    )

    for post_name, post_xyz in (
        ("front_left_standoff", (-0.009, 0.009, 0.0030)),
        ("front_right_standoff", (0.009, 0.009, 0.0030)),
        ("rear_left_standoff", (-0.009, -0.009, 0.0030)),
        ("rear_right_standoff", (0.009, -0.009, 0.0030)),
    ):
        frame.visual(
            Cylinder(radius=0.0018, length=0.0040),
            origin=Origin(xyz=post_xyz),
            material=electronics_black,
            name=post_name,
        )

    arm_specs = [
        ("front_right_arm", "front_right_boss", (0.0152, 0.0152, 0.0012), (0.0265, 0.0265, 0.0040), math.pi / 4.0),
        ("front_left_arm", "front_left_boss", (-0.0152, 0.0152, 0.0012), (-0.0265, 0.0265, 0.0040), -math.pi / 4.0),
        ("rear_left_arm", "rear_left_boss", (-0.0152, -0.0152, 0.0012), (-0.0265, -0.0265, 0.0040), math.pi / 4.0),
        ("rear_right_arm", "rear_right_boss", (0.0152, -0.0152, 0.0012), (0.0265, -0.0265, 0.0040), -math.pi / 4.0),
    ]
    for arm_name, boss_name, arm_xyz, boss_xyz, yaw in arm_specs:
        frame.visual(
            Box((0.023, 0.0046, 0.0024)),
            origin=Origin(xyz=arm_xyz, rpy=(0.0, 0.0, yaw)),
            material=carbon,
            name=arm_name,
        )
        frame.visual(
            Cylinder(radius=0.0056, length=0.0032),
            origin=Origin(xyz=boss_xyz),
            material=carbon,
            name=boss_name,
        )
        frame.visual(
            Cylinder(radius=0.0038, length=0.0012),
            origin=Origin(xyz=(boss_xyz[0], boss_xyz[1], boss_xyz[2] + 0.0022)),
            material=electronics_black,
            name=f"{boss_name}_motor_pad",
        )

    frame.visual(
        Box((0.0022, 0.026, 0.0022)),
        origin=Origin(xyz=(-0.0065, -0.0120, 0.0015)),
        material=carbon,
        name="left_battery_rail",
    )
    frame.visual(
        Box((0.0022, 0.026, 0.0022)),
        origin=Origin(xyz=(0.0065, -0.0120, 0.0015)),
        material=carbon,
        name="right_battery_rail",
    )
    frame.visual(
        Box((0.014, 0.0020, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0010, -0.0004)),
        material=electronics_black,
        name="battery_stop",
    )

    tab_specs = [
        ("front_left_tab", (-0.0135, 0.0100, 0.0078)),
        ("front_right_tab", (0.0135, 0.0100, 0.0078)),
        ("rear_left_tab", (-0.0135, -0.0080, 0.0078)),
        ("rear_right_tab", (0.0135, -0.0080, 0.0078)),
    ]
    for tab_name, tab_xyz in tab_specs:
        frame.visual(
            Box((0.0024, 0.0030, 0.0052)),
            origin=Origin(xyz=tab_xyz),
            material=electronics_black,
            name=tab_name,
        )

    frame.inertial = Inertial.from_geometry(
        Box((0.076, 0.076, 0.014)),
        mass=0.048,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
    )

    canopy = model.part("canopy")
    canopy.visual(
        _save_mesh(_build_canopy_mesh(), "micro_drone_canopy.obj"),
        origin=Origin(xyz=(0.0, 0.002, 0.0095)),
        material=canopy_smoke,
        name="canopy_shell",
    )
    canopy.visual(
        Box((0.011, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, 0.013, 0.0145), rpy=(0.30, 0.0, 0.0)),
        material=canopy_smoke,
        name="camera_nose",
    )
    for clip_name, clip_xyz in (
        ("front_left_clip", (-0.0135, 0.0100, 0.0122)),
        ("front_right_clip", (0.0135, 0.0100, 0.0122)),
        ("rear_left_clip", (-0.0135, -0.0080, 0.0122)),
        ("rear_right_clip", (0.0135, -0.0080, 0.0122)),
    ):
        canopy.visual(
            Box((0.0036, 0.0042, 0.0036)),
            origin=Origin(xyz=clip_xyz),
            material=canopy_smoke,
            name=clip_name,
        )
    canopy.inertial = Inertial.from_geometry(
        Box((0.038, 0.032, 0.022)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.002, 0.015)),
    )

    battery_sled = model.part("battery_sled")
    battery_sled.visual(
        Box((0.0155, 0.028, 0.0016)),
        origin=Origin(xyz=(0.0, -0.0150, -0.0010)),
        material=battery_grey,
        name="tray_floor",
    )
    battery_sled.visual(
        Box((0.0020, 0.022, 0.0028)),
        origin=Origin(xyz=(-0.0065, -0.0115, -0.0010)),
        material=battery_grey,
        name="left_runner",
    )
    battery_sled.visual(
        Box((0.0020, 0.022, 0.0028)),
        origin=Origin(xyz=(0.0065, -0.0115, -0.0010)),
        material=battery_grey,
        name="right_runner",
    )
    battery_sled.visual(
        Box((0.0130, 0.0185, 0.0062)),
        origin=Origin(xyz=(0.0, -0.0140, -0.0049)),
        material=cell_blue,
        name="battery_cell",
    )
    battery_sled.visual(
        Box((0.011, 0.0020, 0.0016)),
        origin=Origin(xyz=(0.0, -0.0010, -0.0016)),
        material=battery_grey,
        name="front_latch",
    )
    battery_sled.visual(
        Box((0.012, 0.0035, 0.0045)),
        origin=Origin(xyz=(0.0, -0.0305, -0.0003)),
        material=electronics_black,
        name="rear_pull",
    )
    battery_sled.inertial = Inertial.from_geometry(
        Box((0.018, 0.035, 0.010)),
        mass=0.026,
        origin=Origin(xyz=(0.0, -0.016, -0.0035)),
    )

    model.articulation(
        "frame_to_canopy",
        ArticulationType.FIXED,
        parent=frame,
        child=canopy,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_battery_sled",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=battery_sled,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.12,
            lower=0.0,
            upper=0.016,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    canopy = object_model.get_part("canopy")
    battery_sled = object_model.get_part("battery_sled")
    battery_slide = object_model.get_articulation("frame_to_battery_sled")

    center_plate = frame.get_visual("center_plate")
    board = frame.get_visual("electronics_board")
    canopy_shell = canopy.get_visual("canopy_shell")
    battery_stop = frame.get_visual("battery_stop")
    left_rail = frame.get_visual("left_battery_rail")
    right_rail = frame.get_visual("right_battery_rail")
    left_runner = battery_sled.get_visual("left_runner")
    right_runner = battery_sled.get_visual("right_runner")
    tray_floor = battery_sled.get_visual("tray_floor")
    front_latch = battery_sled.get_visual("front_latch")
    rear_pull = battery_sled.get_visual("rear_pull")

    front_left_tab = frame.get_visual("front_left_tab")
    front_right_tab = frame.get_visual("front_right_tab")
    rear_left_tab = frame.get_visual("rear_left_tab")
    rear_right_tab = frame.get_visual("rear_right_tab")
    front_left_clip = canopy.get_visual("front_left_clip")
    front_right_clip = canopy.get_visual("front_right_clip")
    rear_left_clip = canopy.get_visual("rear_left_clip")
    rear_right_clip = canopy.get_visual("rear_right_clip")

    front_left_arm = frame.get_visual("front_left_arm")
    front_right_arm = frame.get_visual("front_right_arm")
    rear_left_arm = frame.get_visual("rear_left_arm")
    rear_right_arm = frame.get_visual("rear_right_arm")
    front_left_boss = frame.get_visual("front_left_boss")
    front_right_boss = frame.get_visual("front_right_boss")
    rear_left_boss = frame.get_visual("rear_left_boss")
    rear_right_boss = frame.get_visual("rear_right_boss")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.0005,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.0005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(frame, frame, axes="xy", min_overlap=0.00030, elem_a=board, elem_b=center_plate)
    ctx.expect_within(frame, canopy, axes="xy", inner_elem=board, outer_elem=canopy_shell)
    ctx.expect_gap(
        canopy,
        frame,
        axis="z",
        min_gap=0.0030,
        max_gap=0.0065,
        positive_elem=canopy_shell,
        negative_elem=board,
    )

    for clip, tab in (
        (front_left_clip, front_left_tab),
        (front_right_clip, front_right_tab),
        (rear_left_clip, rear_left_tab),
        (rear_right_clip, rear_right_tab),
    ):
        ctx.expect_overlap(canopy, frame, axes="xy", min_overlap=0.000006, elem_a=clip, elem_b=tab)
        ctx.expect_gap(
            canopy,
            frame,
            axis="z",
            max_gap=0.0002,
            max_penetration=0.0002,
            positive_elem=clip,
            negative_elem=tab,
        )

    for arm, boss in (
        (front_left_arm, front_left_boss),
        (front_right_arm, front_right_boss),
        (rear_left_arm, rear_left_boss),
        (rear_right_arm, rear_right_boss),
    ):
        ctx.expect_overlap(frame, frame, axes="xy", min_overlap=0.000015, elem_a=arm, elem_b=boss)
        ctx.expect_gap(
            frame,
            frame,
            axis="z",
            max_gap=0.0010,
            max_penetration=0.0,
            positive_elem=boss,
            negative_elem=arm,
        )

    ctx.expect_contact(battery_sled, frame, elem_a=front_latch, elem_b=battery_stop)
    ctx.expect_overlap(
        battery_sled,
        frame,
        axes="xy",
        min_overlap=0.000020,
        elem_a=left_runner,
        elem_b=left_rail,
    )
    ctx.expect_overlap(
        battery_sled,
        frame,
        axes="xy",
        min_overlap=0.000020,
        elem_a=right_runner,
        elem_b=right_rail,
    )
    ctx.expect_gap(
        frame,
        battery_sled,
        axis="z",
        max_gap=0.0004,
        max_penetration=0.0002,
        positive_elem=left_rail,
        negative_elem=left_runner,
    )
    ctx.expect_gap(
        frame,
        battery_sled,
        axis="z",
        max_gap=0.0004,
        max_penetration=0.0002,
        positive_elem=right_rail,
        negative_elem=right_runner,
    )
    ctx.expect_gap(
        frame,
        battery_sled,
        axis="y",
        min_gap=0.010,
        positive_elem=board,
        negative_elem=rear_pull,
    )
    ctx.expect_overlap(frame, battery_sled, axes="xy", min_overlap=0.00018, elem_a=board, elem_b=tray_floor)

    with ctx.pose({battery_slide: 0.012}):
        ctx.expect_overlap(
            battery_sled,
            frame,
            axes="xy",
            min_overlap=0.000008,
            elem_a=left_runner,
            elem_b=left_rail,
        )
        ctx.expect_overlap(
            battery_sled,
            frame,
            axes="xy",
            min_overlap=0.000008,
            elem_a=right_runner,
            elem_b=right_rail,
        )
        ctx.expect_gap(
            frame,
            battery_sled,
            axis="z",
            max_gap=0.0004,
            max_penetration=0.0002,
            positive_elem=left_rail,
            negative_elem=left_runner,
        )
        ctx.expect_gap(
            frame,
            battery_sled,
            axis="z",
            max_gap=0.0004,
            max_penetration=0.0002,
            positive_elem=right_rail,
            negative_elem=right_runner,
        )
        ctx.expect_gap(
            frame,
            battery_sled,
            axis="y",
            min_gap=0.020,
            positive_elem=board,
            negative_elem=rear_pull,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
