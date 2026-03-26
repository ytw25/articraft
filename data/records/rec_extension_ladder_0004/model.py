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
        return "/tmp"


os.getcwd = _safe_getcwd
try:
    os.chdir("/tmp")
except OSError:
    pass

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

ASSET_ROOT = "/tmp"

TOP_Z = 1.18
PIN_Y = 0.04
BASE_SPREAD_ANGLE = math.radians(28.0)
LADDER_WIDTH = 0.46
LADDER_HEIGHT = 1.16
RAIL_THICK = 0.032
RAIL_DEPTH = 0.024
TOP_HEAD_DEPTH = 0.050
TOP_HEAD_HEIGHT = 0.070
RUNG_LENGTH = LADDER_WIDTH - 2 * RAIL_THICK + 0.012
RUNG_DEPTH = 0.034
RUNG_HEIGHT = 0.026
RUNG_ZS = (-0.22, -0.44, -0.66, -0.88)
FOOT_WIDTH = 0.070
FOOT_DEPTH = 0.040
FOOT_HEIGHT = 0.024
FOOT_Z = -LADDER_HEIGHT + FOOT_HEIGHT / 2.0
HOOK_SLOT_DEPTH = 0.052
HOOK_LEG_THICK = 0.012
HOOK_LEG_DROP = 0.095
HOOK_BRIDGE_X = 0.270
HOOK_BRIDGE_THICK = 0.012
HOOK_BRIDGE_CLEARANCE = 0.0
PLANK_LENGTH = 0.58
PLANK_WIDTH = 0.22
PLANK_THICK = 0.036
PLANK_STANDOFF = 0.010


def _box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material=None,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _cylinder_x(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material=None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _build_ladder_section(part, metal, tread, rubber) -> None:
    rail_x = LADDER_WIDTH / 2.0 - RAIL_THICK / 2.0

    _box(
        part,
        "left_rail",
        (RAIL_THICK, RAIL_DEPTH, LADDER_HEIGHT),
        (-rail_x, 0.0, -LADDER_HEIGHT / 2.0),
        material=metal,
    )
    _box(
        part,
        "right_rail",
        (RAIL_THICK, RAIL_DEPTH, LADDER_HEIGHT),
        (rail_x, 0.0, -LADDER_HEIGHT / 2.0),
        material=metal,
    )
    _box(
        part,
        "top_head",
        (LADDER_WIDTH, TOP_HEAD_DEPTH, TOP_HEAD_HEIGHT),
        (0.0, 0.0, -TOP_HEAD_HEIGHT / 2.0),
        material=metal,
    )

    for index, rung_z in enumerate(RUNG_ZS, start=1):
        _box(
            part,
            f"rung_{index}",
            (RUNG_LENGTH, RUNG_DEPTH, RUNG_HEIGHT),
            (0.0, 0.0, rung_z),
            material=tread,
        )

    _box(
        part,
        "left_foot",
        (FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT),
        (-rail_x, 0.0, FOOT_Z),
        material=rubber,
    )
    _box(
        part,
        "right_foot",
        (FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT),
        (rail_x, 0.0, FOOT_Z),
        material=rubber,
    )


def _build_plank(part, board, bracket) -> None:
    rung_center_y = PIN_Y + abs(RUNG_ZS[0]) * math.sin(BASE_SPREAD_ANGLE)
    bridge_span_y = HOOK_SLOT_DEPTH + 2.0 * HOOK_LEG_THICK
    plank_center_z = HOOK_BRIDGE_THICK / 2.0 + PLANK_STANDOFF + PLANK_THICK / 2.0
    leg_center_z = -HOOK_BRIDGE_THICK / 2.0 - HOOK_LEG_DROP / 2.0
    leg_offset = HOOK_SLOT_DEPTH / 2.0 + HOOK_LEG_THICK / 2.0
    mount_center_z = HOOK_BRIDGE_THICK / 2.0 + PLANK_STANDOFF / 2.0

    _box(
        part,
        "deck",
        (PLANK_WIDTH, PLANK_LENGTH, PLANK_THICK),
        (0.0, -rung_center_y, plank_center_z),
        material=board,
    )

    for prefix, hook_y in (("front", 0.0), ("rear", -2.0 * rung_center_y)):
        _box(
            part,
            f"{prefix}_hook_bridge",
            (HOOK_BRIDGE_X, bridge_span_y, HOOK_BRIDGE_THICK),
            (0.0, hook_y, 0.0),
            material=bracket,
        )
        _box(
            part,
            f"{prefix}_mount_pad",
            (HOOK_BRIDGE_X, 0.040, PLANK_STANDOFF),
            (0.0, hook_y, mount_center_z),
            material=bracket,
        )
        _box(
            part,
            f"{prefix}_hook_outer_leg",
            (HOOK_BRIDGE_X, HOOK_LEG_THICK, HOOK_LEG_DROP),
            (0.0, hook_y + leg_offset, leg_center_z),
            material=bracket,
        )
        _box(
            part,
            f"{prefix}_hook_inner_leg",
            (HOOK_BRIDGE_X, HOOK_LEG_THICK, HOOK_LEG_DROP),
            (0.0, hook_y - leg_offset, leg_center_z),
            material=bracket,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platform_stepladder")

    aluminum = model.material("aluminum", rgba=(0.79, 0.82, 0.85, 1.0))
    tread = model.material("tread_gray", rgba=(0.60, 0.62, 0.66, 1.0))
    steel = model.material("steel", rgba=(0.31, 0.33, 0.36, 1.0))
    wood = model.material("wood", rgba=(0.73, 0.57, 0.39, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    bracket = model.part("spreader_bracket")
    _box(
        bracket,
        "top_saddle",
        (0.30, 0.16, 0.050),
        (0.0, 0.0, TOP_Z + 0.025),
        material=steel,
    )
    _box(
        bracket,
        "front_hinge_block",
        (0.28, 0.028, 0.090),
        (0.0, PIN_Y, TOP_Z - 0.005),
        material=steel,
    )
    _box(
        bracket,
        "rear_hinge_block",
        (0.28, 0.028, 0.090),
        (0.0, -PIN_Y, TOP_Z - 0.005),
        material=steel,
    )
    _box(
        bracket,
        "center_bracket_fin",
        (0.13, 0.040, 0.110),
        (0.0, 0.0, TOP_Z + 0.080),
        material=steel,
    )
    pin_x = LADDER_WIDTH / 2.0 - RAIL_THICK / 2.0
    cheek_x = (pin_x + 0.14) / 2.0
    cheek_width = pin_x - 0.14 + 0.032
    for block_y, prefix in ((PIN_Y, "front"), (-PIN_Y, "rear")):
        for side_x, side_name in ((-cheek_x, "left"), (cheek_x, "right")):
            _box(
                bracket,
                f"{prefix}_{side_name}_cheek",
                (cheek_width, 0.032, 0.090),
                (side_x, block_y, TOP_Z - 0.005),
                material=steel,
            )
    for side_x, side_name in ((-pin_x, "left"), (pin_x, "right")):
        _cylinder_x(
            bracket,
            f"front_{side_name}_pin",
            radius=0.011,
            length=0.032,
            xyz=(side_x, PIN_Y, TOP_Z),
            material=tread,
        )
        _cylinder_x(
            bracket,
            f"rear_{side_name}_pin",
            radius=0.011,
            length=0.032,
            xyz=(side_x, -PIN_Y, TOP_Z),
            material=tread,
        )

    front_section = model.part("front_section")
    _build_ladder_section(front_section, aluminum, tread, rubber)

    rear_section = model.part("rear_section")
    _build_ladder_section(rear_section, aluminum, tread, rubber)

    walk_board = model.part("walk_board")
    _build_plank(walk_board, wood, steel)

    model.articulation(
        "bracket_to_front_section",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=front_section,
        origin=Origin(xyz=(0.0, PIN_Y, TOP_Z), rpy=(BASE_SPREAD_ANGLE, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=-0.06,
            upper=0.0,
        ),
    )
    model.articulation(
        "bracket_to_rear_section",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=rear_section,
        origin=Origin(xyz=(0.0, -PIN_Y, TOP_Z), rpy=(-BASE_SPREAD_ANGLE, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=0.0,
            upper=0.06,
        ),
    )
    rung_center_y = PIN_Y + abs(RUNG_ZS[0]) * math.sin(BASE_SPREAD_ANGLE)
    rung_center_z = TOP_Z + RUNG_ZS[0] * math.cos(BASE_SPREAD_ANGLE)
    rung_top_z = (
        rung_center_z
        + (RUNG_DEPTH / 2.0) * math.sin(BASE_SPREAD_ANGLE)
        + (RUNG_HEIGHT / 2.0) * math.cos(BASE_SPREAD_ANGLE)
    )
    front_bridge_world_z = (
        rung_top_z + HOOK_BRIDGE_THICK / 2.0 + HOOK_BRIDGE_CLEARANCE
    )
    rel_y = rung_center_y - PIN_Y
    rel_z = front_bridge_world_z - TOP_Z
    cos_a = math.cos(BASE_SPREAD_ANGLE)
    sin_a = math.sin(BASE_SPREAD_ANGLE)
    board_mount_y = rel_y * cos_a + rel_z * sin_a
    board_mount_z = -rel_y * sin_a + rel_z * cos_a
    model.articulation(
        "front_section_to_walk_board",
        ArticulationType.FIXED,
        parent=front_section,
        child=walk_board,
        origin=Origin(
            xyz=(0.0, board_mount_y, board_mount_z),
            rpy=(-BASE_SPREAD_ANGLE, 0.0, 0.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSET_ROOT)
    bracket = object_model.get_part("spreader_bracket")
    front_section = object_model.get_part("front_section")
    rear_section = object_model.get_part("rear_section")
    walk_board = object_model.get_part("walk_board")
    front_hinge = object_model.get_articulation("bracket_to_front_section")
    rear_hinge = object_model.get_articulation("bracket_to_rear_section")

    front_top_head = front_section.get_visual("top_head")
    rear_top_head = rear_section.get_visual("top_head")
    front_upper_rung = front_section.get_visual("rung_1")
    rear_upper_rung = rear_section.get_visual("rung_1")
    front_foot = front_section.get_visual("left_foot")
    rear_foot = rear_section.get_visual("left_foot")
    front_block = bracket.get_visual("front_hinge_block")
    rear_block = bracket.get_visual("rear_hinge_block")
    front_hook_bridge = walk_board.get_visual("front_hook_bridge")
    rear_hook_bridge = walk_board.get_visual("rear_hook_bridge")
    front_hook_outer_leg = walk_board.get_visual("front_hook_outer_leg")
    front_hook_inner_leg = walk_board.get_visual("front_hook_inner_leg")
    rear_hook_outer_leg = walk_board.get_visual("rear_hook_outer_leg")
    rear_hook_inner_leg = walk_board.get_visual("rear_hook_inner_leg")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        front_section,
        bracket,
        reason="front ladder head nests into the front hinge block around the pivot pin",
    )
    ctx.allow_overlap(
        rear_section,
        bracket,
        reason="rear ladder head nests into the rear hinge block around the pivot pin",
    )
    ctx.allow_overlap(
        walk_board,
        front_section,
        reason="the front hook bracket intentionally clasps the outer rung with seated bridge contact",
    )
    ctx.allow_overlap(
        walk_board,
        rear_section,
        reason="the rear hook bracket intentionally clasps the outer rung in the supported platform stance",
    )

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        front_section,
        bracket,
        axes="xz",
        elem_a=front_top_head,
        elem_b=front_block,
        min_overlap=0.012,
    )
    ctx.expect_overlap(
        rear_section,
        bracket,
        axes="xz",
        elem_a=rear_top_head,
        elem_b=rear_block,
        min_overlap=0.012,
    )
    ctx.expect_overlap(
        walk_board,
        front_section,
        axes="xy",
        elem_a=front_hook_bridge,
        elem_b=front_upper_rung,
        min_overlap=0.007,
    )
    ctx.expect_overlap(
        walk_board,
        rear_section,
        axes="xy",
        elem_a=rear_hook_bridge,
        elem_b=rear_upper_rung,
        min_overlap=0.007,
    )
    ctx.expect_gap(
        walk_board,
        front_section,
        axis="z",
        positive_elem=front_hook_bridge,
        negative_elem=front_upper_rung,
        max_gap=0.003,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        walk_board,
        rear_section,
        axis="z",
        positive_elem=rear_hook_bridge,
        negative_elem=rear_upper_rung,
        max_gap=0.007,
        max_penetration=0.0,
    )
    ctx.expect_contact(
        walk_board,
        front_section,
        elem_a=front_hook_bridge,
        elem_b=front_upper_rung,
    )
    ctx.expect_contact(
        walk_board,
        rear_section,
        elem_a=rear_hook_bridge,
        elem_b=rear_upper_rung,
    )
    ctx.expect_gap(
        walk_board,
        front_section,
        axis="y",
        positive_elem=front_hook_outer_leg,
        negative_elem=front_upper_rung,
        max_gap=0.008,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        front_section,
        walk_board,
        axis="y",
        positive_elem=front_upper_rung,
        negative_elem=front_hook_inner_leg,
        max_gap=0.008,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        walk_board,
        rear_section,
        axis="y",
        positive_elem=rear_hook_outer_leg,
        negative_elem=rear_upper_rung,
        max_gap=0.008,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        rear_section,
        walk_board,
        axis="y",
        positive_elem=rear_upper_rung,
        negative_elem=rear_hook_inner_leg,
        max_gap=0.008,
        max_penetration=0.0,
    )
    ctx.expect_gap(
        front_section,
        rear_section,
        axis="y",
        positive_elem=front_foot,
        negative_elem=rear_foot,
        min_gap=1.00,
    )

    with ctx.pose({front_hinge: -0.06, rear_hinge: 0.06}):
        ctx.expect_gap(
            front_section,
            rear_section,
            axis="y",
            positive_elem=front_foot,
            negative_elem=rear_foot,
            min_gap=0.95,
        )
        ctx.expect_overlap(
            front_section,
            bracket,
            axes="xz",
            elem_a=front_top_head,
            elem_b=front_block,
            min_overlap=0.010,
        )
        ctx.expect_overlap(
            rear_section,
            bracket,
            axes="xz",
            elem_a=rear_top_head,
            elem_b=rear_block,
            min_overlap=0.010,
        )
        ctx.expect_gap(
            walk_board,
            rear_section,
            axis="z",
            positive_elem=rear_hook_bridge,
            negative_elem=rear_upper_rung,
            min_gap=0.012,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
