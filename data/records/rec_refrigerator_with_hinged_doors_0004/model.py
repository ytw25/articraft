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
    BoxGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.92
BODY_DEPTH = 0.72
BODY_HEIGHT = 1.78

SIDE_WALL = 0.03
TOP_WALL = 0.03
BOTTOM_WALL = 0.05
BACK_WALL = 0.02
CENTER_DIVIDER = 0.03

SIDE_REVEAL = 0.004
CENTER_SEAM = 0.004
DOOR_THICKNESS = 0.06
DOOR_BOTTOM = 0.02
DOOR_HEIGHT = BODY_HEIGHT - 0.04
DOOR_WIDTH = (BODY_WIDTH - (2.0 * SIDE_REVEAL) - CENTER_SEAM) / 2.0
DOOR_CENTER_Y = DOOR_THICKNESS / 4.0
HINGE_FORWARD_OFFSET = DOOR_THICKNESS / 4.0

HANDLE_HEIGHT = 0.80
HANDLE_CHANNEL_DEPTH = 0.046
HANDLE_CHANNEL_WIDTH = 0.028

OPEN_ANGLE_RAD = math.radians(100.0)


def _cabinet_shell():
    outer = BoxGeometry((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT))
    outer.translate(0.0, 0.0, BODY_HEIGHT / 2.0)

    inner_width = BODY_WIDTH - (2.0 * SIDE_WALL)
    inner_depth = BODY_DEPTH - BACK_WALL + 0.01
    inner_height = BODY_HEIGHT - TOP_WALL - BOTTOM_WALL

    inner = BoxGeometry((inner_width, inner_depth, inner_height))
    inner.translate(
        0.0,
        (BACK_WALL + 0.01) / 2.0,
        BOTTOM_WALL + (inner_height / 2.0),
    )

    shell = boolean_difference(outer, inner)
    return mesh_from_geometry(shell, ASSETS.mesh_path("refrigerator_cabinet_shell.obj"))


def _door_panel(side: str):
    x_sign = 1.0 if side == "left" else -1.0

    outer = BoxGeometry((DOOR_WIDTH, DOOR_THICKNESS, DOOR_HEIGHT))
    outer.translate(
        x_sign * (DOOR_WIDTH / 2.0),
        DOOR_CENTER_Y,
        DOOR_HEIGHT / 2.0,
    )

    handle_channel = BoxGeometry((HANDLE_CHANNEL_WIDTH, HANDLE_CHANNEL_DEPTH, HANDLE_HEIGHT))
    handle_channel.translate(
        x_sign * (HANDLE_CHANNEL_WIDTH / 2.0 - 0.002),
        DOOR_CENTER_Y + 0.008,
        DOOR_HEIGHT * 0.55,
    )

    panel = boolean_difference(outer, handle_channel)
    return mesh_from_geometry(panel, ASSETS.mesh_path(f"{side}_door_panel.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_by_side_refrigerator", assets=ASSETS)

    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    shell_gray = model.material("shell_gray", rgba=(0.62, 0.64, 0.67, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(_cabinet_shell(), material=shell_gray, name="cabinet_shell")
    cabinet.visual(
        Box((CENTER_DIVIDER, BODY_DEPTH - BACK_WALL, BODY_HEIGHT - TOP_WALL - BOTTOM_WALL)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_WALL / 2.0,
                BOTTOM_WALL + ((BODY_HEIGHT - TOP_WALL - BOTTOM_WALL) / 2.0),
            )
        ),
        material=shell_gray,
        name="center_divider",
    )

    left_door = model.part("left_door")
    left_door.visual(_door_panel("left"), material=steel, name="door_panel")

    right_door = model.part("right_door")
    right_door.visual(_door_panel("right"), material=steel, name="door_panel")

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_door,
        origin=Origin(
            xyz=(
                -(BODY_WIDTH / 2.0) + SIDE_REVEAL,
                (BODY_DEPTH / 2.0) + HINGE_FORWARD_OFFSET,
                DOOR_BOTTOM,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=OPEN_ANGLE_RAD,
        ),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_door,
        origin=Origin(
            xyz=(
                (BODY_WIDTH / 2.0) - SIDE_REVEAL,
                (BODY_DEPTH / 2.0) + HINGE_FORWARD_OFFSET,
                DOOR_BOTTOM,
            )
        ),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=OPEN_ANGLE_RAD,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    cabinet = object_model.get_part("cabinet")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")

    cabinet_shell = cabinet.get_visual("cabinet_shell")
    left_panel = left_door.get_visual("door_panel")
    right_panel = right_door.get_visual("door_panel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "left hinge is vertical",
        tuple(left_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"left_hinge.axis={left_hinge.axis}",
    )
    ctx.check(
        "right hinge is vertical",
        tuple(right_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"right_hinge.axis={right_hinge.axis}",
    )
    ctx.check(
        "left hinge opens about one hundred degrees",
        (
            abs(left_hinge.motion_limits.lower - 0.0) < 1e-9
            and abs(left_hinge.motion_limits.upper - OPEN_ANGLE_RAD) < 1e-9
        ),
        details=f"limits=({left_hinge.motion_limits.lower}, {left_hinge.motion_limits.upper})",
    )
    ctx.check(
        "right hinge opens about one hundred degrees",
        (
            abs(right_hinge.motion_limits.lower - 0.0) < 1e-9
            and abs(right_hinge.motion_limits.upper - OPEN_ANGLE_RAD) < 1e-9
        ),
        details=f"limits=({right_hinge.motion_limits.lower}, {right_hinge.motion_limits.upper})",
    )

    ctx.expect_gap(
        left_door,
        cabinet,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=left_panel,
        negative_elem=cabinet_shell,
        name="left door seats against cabinet front",
    )
    ctx.expect_gap(
        right_door,
        cabinet,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=right_panel,
        negative_elem=cabinet_shell,
        name="right door seats against cabinet front",
    )
    ctx.expect_overlap(
        left_door,
        cabinet,
        axes="xz",
        min_overlap=0.40,
        elem_a=left_panel,
        elem_b=cabinet_shell,
        name="left door covers the left cabinet opening",
    )
    ctx.expect_overlap(
        right_door,
        cabinet,
        axes="xz",
        min_overlap=0.40,
        elem_a=right_panel,
        elem_b=cabinet_shell,
        name="right door covers the right cabinet opening",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.003,
        max_gap=0.006,
        positive_elem=right_panel,
        negative_elem=left_panel,
        name="doors meet at a narrow center seam",
    )

    with ctx.pose({left_hinge: math.radians(90.0)}):
        ctx.expect_gap(
            left_door,
            cabinet,
            axis="y",
            min_gap=0.014,
            positive_elem=left_panel,
            negative_elem=cabinet_shell,
            name="left door swings forward when opened",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.18,
            positive_elem=right_panel,
            negative_elem=left_panel,
            name="left door clears the center seam when open",
        )

    with ctx.pose({right_hinge: math.radians(90.0)}):
        ctx.expect_gap(
            right_door,
            cabinet,
            axis="y",
            min_gap=0.014,
            positive_elem=right_panel,
            negative_elem=cabinet_shell,
            name="right door swings forward when opened",
        )
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.18,
            positive_elem=right_panel,
            negative_elem=left_panel,
            name="right door clears the center seam when open",
        )

    with ctx.pose({left_hinge: math.radians(90.0), right_hinge: math.radians(90.0)}):
        ctx.expect_gap(
            right_door,
            left_door,
            axis="x",
            min_gap=0.70,
            positive_elem=right_panel,
            negative_elem=left_panel,
            name="both doors can open together without crossing",
        )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=20,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
