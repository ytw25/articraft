from __future__ import annotations

from pathlib import Path

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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

ROWS = 6
COLS = 2

CABINET_W = 0.60
CABINET_D = 0.30
CABINET_H = 0.70
WALL_T = 0.012
BACK_T = 0.006
TOP_T = 0.012
SHELF_T = 0.008
DIVIDER_T = 0.010
TOE_H = 0.075
TOE_SETBACK = 0.045

INTERIOR_D = CABINET_D - BACK_T
STACK_BOTTOM = TOE_H + SHELF_T
STACK_TOP = CABINET_H - TOP_T
OPENING_H = (STACK_TOP - STACK_BOTTOM - (ROWS - 1) * SHELF_T) / ROWS
OPENING_W = (CABINET_W - 2 * WALL_T - DIVIDER_T) / COLS

FRONT_REVEAL_X = 0.004
FRONT_REVEAL_Z = 0.003
DRAWER_FRONT_W = OPENING_W - 2 * FRONT_REVEAL_X
DRAWER_FRONT_H = OPENING_H - 2 * FRONT_REVEAL_Z
DRAWER_FRONT_T = 0.010

DRAWER_TRAY_W = DRAWER_FRONT_W - 0.004
DRAWER_TRAY_H = DRAWER_FRONT_H - 0.014
DRAWER_TRAY_D = 0.255
DRAWER_SIDE_T = 0.005
DRAWER_BACK_T = 0.005
DRAWER_BOTTOM_T = 0.004

RUNNER_T = 0.005
RUNNER_H = 0.011
RUNNER_LEN = 0.220
RUNNER_OFFSET_X = DRAWER_TRAY_W / 2.0 + RUNNER_T / 2.0 + 0.001
RUNNER_WEB_OVERLAP = 0.0003
RUNNER_CLEARANCE_X = RUNNER_OFFSET_X - RUNNER_T / 2.0 - DRAWER_TRAY_W / 2.0
RUNNER_WEB_T = RUNNER_CLEARANCE_X + 2.0 * RUNNER_WEB_OVERLAP

GUIDE_DEPTH = 0.0045
GUIDE_LEN = 0.252
GUIDE_H = 0.004
GUIDE_SLOT_H = 0.011

OPEN_DISTANCE = 0.125


def _row_center(row: int) -> float:
    return STACK_BOTTOM + row * (OPENING_H + SHELF_T) + OPENING_H / 2.0


def _col_center(col: int) -> float:
    left_inner = -CABINET_W / 2.0 + WALL_T
    if col == 0:
        x_min = left_inner
        x_max = -DIVIDER_T / 2.0
    else:
        x_min = DIVIDER_T / 2.0
        x_max = CABINET_W / 2.0 - WALL_T
    return (x_min + x_max) / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hardware_cabinet", assets=ASSETS)

    cabinet_blue = model.material("cabinet_blue", rgba=(0.41, 0.53, 0.68, 1.0))
    cabinet_shadow = model.material("cabinet_shadow", rgba=(0.29, 0.35, 0.43, 1.0))
    drawer_gray = model.material("drawer_gray", rgba=(0.84, 0.85, 0.86, 1.0))
    drawer_side = model.material("drawer_side", rgba=(0.76, 0.77, 0.79, 1.0))
    pull_metal = model.material("pull_metal", rgba=(0.70, 0.71, 0.72, 1.0))

    body = model.part("cabinet")
    body.visual(
        Box((WALL_T, CABINET_D, CABINET_H)),
        origin=Origin(xyz=(-CABINET_W / 2.0 + WALL_T / 2.0, 0.0, CABINET_H / 2.0)),
        material=cabinet_blue,
        name="left_wall",
    )
    body.visual(
        Box((WALL_T, CABINET_D, CABINET_H)),
        origin=Origin(xyz=(CABINET_W / 2.0 - WALL_T / 2.0, 0.0, CABINET_H / 2.0)),
        material=cabinet_blue,
        name="right_wall",
    )
    body.visual(
        Box((CABINET_W - 2 * WALL_T, BACK_T, CABINET_H)),
        origin=Origin(xyz=(0.0, -CABINET_D / 2.0 + BACK_T / 2.0, CABINET_H / 2.0)),
        material=cabinet_shadow,
        name="back_panel",
    )
    body.visual(
        Box((CABINET_W - 2 * WALL_T, INTERIOR_D, TOP_T)),
        origin=Origin(xyz=(0.0, BACK_T / 2.0, CABINET_H - TOP_T / 2.0)),
        material=cabinet_blue,
        name="top_panel",
    )
    body.visual(
        Box((CABINET_W - 2 * WALL_T, INTERIOR_D, SHELF_T)),
        origin=Origin(xyz=(0.0, BACK_T / 2.0, TOE_H + SHELF_T / 2.0)),
        material=cabinet_blue,
        name="deck_panel",
    )
    body.visual(
        Box((CABINET_W - 2 * WALL_T, TOE_SETBACK, SHELF_T)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_D / 2.0 - TOE_SETBACK / 2.0,
                SHELF_T / 2.0,
            )
        ),
        material=cabinet_shadow,
        name="toe_floor",
    )
    body.visual(
        Box((CABINET_W - 2 * WALL_T, WALL_T, TOE_H)),
        origin=Origin(
            xyz=(
                0.0,
                CABINET_D / 2.0 - TOE_SETBACK - WALL_T / 2.0,
                TOE_H / 2.0,
            )
        ),
        material=cabinet_shadow,
        name="toe_panel",
    )
    body.visual(
        Box((DIVIDER_T, INTERIOR_D, STACK_TOP - STACK_BOTTOM)),
        origin=Origin(
            xyz=(
                0.0,
                BACK_T / 2.0,
                (STACK_BOTTOM + STACK_TOP) / 2.0,
            )
        ),
        material=cabinet_blue,
        name="center_divider",
    )

    for row in range(ROWS - 1):
        shelf_z = STACK_BOTTOM + OPENING_H + row * (OPENING_H + SHELF_T) + SHELF_T / 2.0
        body.visual(
            Box((CABINET_W - 2 * WALL_T, INTERIOR_D, SHELF_T)),
            origin=Origin(xyz=(0.0, BACK_T / 2.0, shelf_z)),
            material=cabinet_blue,
            name=f"shelf_{row + 1}",
        )

    guide_y = (CABINET_D / 2.0 - 0.020 + (-CABINET_D / 2.0 + BACK_T + 0.022)) / 2.0
    left_wall_face = -CABINET_W / 2.0 + WALL_T
    right_wall_face = CABINET_W / 2.0 - WALL_T
    divider_left_face = -DIVIDER_T / 2.0
    divider_right_face = DIVIDER_T / 2.0

    for row in range(ROWS):
        row_z = _row_center(row)
        lower_guide_z = row_z - GUIDE_SLOT_H / 2.0 - GUIDE_H / 2.0
        upper_guide_z = row_z + GUIDE_SLOT_H / 2.0 + GUIDE_H / 2.0

        guide_faces = {
            "c0_outer": left_wall_face,
            "c0_inner": divider_left_face,
            "c1_inner": divider_right_face,
            "c1_outer": right_wall_face,
        }
        for face_name, face_x in guide_faces.items():
            face_sign = 1.0 if face_x < 0.0 else -1.0
            guide_x = face_x + face_sign * GUIDE_DEPTH / 2.0
            for level_name, guide_z in (
                ("lower", lower_guide_z),
                ("upper", upper_guide_z),
            ):
                body.visual(
                    Box((GUIDE_DEPTH, GUIDE_LEN, GUIDE_H)),
                    origin=Origin(xyz=(guide_x, guide_y, guide_z)),
                    material=cabinet_shadow,
                    name=f"guide_r{row}_f{face_name}_{level_name}",
                )

    body.inertial = Inertial.from_geometry(
        Box((CABINET_W, CABINET_D, CABINET_H)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_H / 2.0)),
    )

    pull_plate_w = 0.050
    pull_plate_h = 0.018
    pull_plate_t = 0.003
    pull_lip_r = 0.006
    pull_lip_len = 0.034
    knob_r = 0.004
    knob_len = 0.008
    pull_z = -DRAWER_FRONT_H * 0.18
    tray_front_y = DRAWER_TRAY_D / 2.0
    front_center_y = tray_front_y + DRAWER_FRONT_T / 2.0

    for row in range(ROWS):
        for col in range(COLS):
            drawer = model.part(f"drawer_r{row}_c{col}")
            drawer.visual(
                Box((DRAWER_FRONT_W, DRAWER_FRONT_T, DRAWER_FRONT_H)),
                origin=Origin(xyz=(0.0, front_center_y, 0.0)),
                material=drawer_gray,
                name="front",
            )
            drawer.visual(
                Box((DRAWER_TRAY_W, DRAWER_TRAY_D, DRAWER_BOTTOM_T)),
                origin=Origin(
                    xyz=(
                        0.0,
                        0.0,
                        -DRAWER_TRAY_H / 2.0 + DRAWER_BOTTOM_T / 2.0,
                    )
                ),
                material=drawer_side,
                name="tray_bottom",
            )
            drawer.visual(
                Box((DRAWER_SIDE_T, DRAWER_TRAY_D, DRAWER_TRAY_H)),
                origin=Origin(xyz=(-DRAWER_TRAY_W / 2.0 + DRAWER_SIDE_T / 2.0, 0.0, 0.0)),
                material=drawer_side,
                name="tray_left",
            )
            drawer.visual(
                Box((DRAWER_SIDE_T, DRAWER_TRAY_D, DRAWER_TRAY_H)),
                origin=Origin(xyz=(DRAWER_TRAY_W / 2.0 - DRAWER_SIDE_T / 2.0, 0.0, 0.0)),
                material=drawer_side,
                name="tray_right",
            )
            drawer.visual(
                Box((DRAWER_TRAY_W, DRAWER_BACK_T, DRAWER_TRAY_H)),
                origin=Origin(xyz=(0.0, -DRAWER_TRAY_D / 2.0 + DRAWER_BACK_T / 2.0, 0.0)),
                material=drawer_side,
                name="tray_back",
            )
            drawer.visual(
                Box((RUNNER_WEB_T, RUNNER_LEN, RUNNER_H)),
                origin=Origin(
                    xyz=(
                        -(DRAWER_TRAY_W / 2.0 + RUNNER_CLEARANCE_X / 2.0),
                        -0.010,
                        0.0,
                    )
                ),
                material=drawer_side,
                name="runner_left_web",
            )
            drawer.visual(
                Box((RUNNER_WEB_T, RUNNER_LEN, RUNNER_H)),
                origin=Origin(
                    xyz=(
                        DRAWER_TRAY_W / 2.0 + RUNNER_CLEARANCE_X / 2.0,
                        -0.010,
                        0.0,
                    )
                ),
                material=drawer_side,
                name="runner_right_web",
            )
            drawer.visual(
                Box((RUNNER_T, RUNNER_LEN, RUNNER_H)),
                origin=Origin(xyz=(-RUNNER_OFFSET_X, -0.010, 0.0)),
                material=drawer_side,
                name="runner_left",
            )
            drawer.visual(
                Box((RUNNER_T, RUNNER_LEN, RUNNER_H)),
                origin=Origin(xyz=(RUNNER_OFFSET_X, -0.010, 0.0)),
                material=drawer_side,
                name="runner_right",
            )
            drawer.visual(
                Box((pull_plate_w, pull_plate_t, pull_plate_h)),
                origin=Origin(
                    xyz=(
                        0.0,
                        front_center_y + DRAWER_FRONT_T / 2.0 + pull_plate_t / 2.0,
                        pull_z,
                    )
                ),
                material=pull_metal,
                name="pull_plate",
            )
            drawer.visual(
                Cylinder(radius=pull_lip_r, length=pull_lip_len),
                origin=Origin(
                    xyz=(0.0, front_center_y + DRAWER_FRONT_T / 2.0 + 0.0035, pull_z),
                    rpy=(0.0, 1.57079632679, 0.0),
                ),
                material=pull_metal,
                name="cup_pull",
            )
            drawer.visual(
                Cylinder(radius=knob_r, length=knob_len),
                origin=Origin(
                    xyz=(0.0, front_center_y + DRAWER_FRONT_T / 2.0 + knob_len / 2.0, pull_z),
                    rpy=(1.57079632679, 0.0, 0.0),
                ),
                material=pull_metal,
                name="knob",
            )
            drawer.inertial = Inertial.from_geometry(
                Box((DRAWER_FRONT_W, DRAWER_TRAY_D + DRAWER_FRONT_T, DRAWER_FRONT_H)),
                mass=0.65,
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
            )

            model.articulation(
                f"slide_r{row}_c{col}",
                ArticulationType.PRISMATIC,
                parent=body,
                child=drawer,
                origin=Origin(xyz=(_col_center(col), 0.0125, _row_center(row))),
                axis=(0.0, 1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=25.0,
                    velocity=0.25,
                    lower=0.0,
                    upper=OPEN_DISTANCE,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.warn_if_articulation_origin_near_geometry(tol=0.05)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    back_panel = cabinet.get_visual("back_panel")
    toe_panel = cabinet.get_visual("toe_panel")
    toe_floor = cabinet.get_visual("toe_floor")
    center_divider = cabinet.get_visual("center_divider")
    left_wall = cabinet.get_visual("left_wall")
    right_wall = cabinet.get_visual("right_wall")

    for row in range(ROWS):
        left_drawer = object_model.get_part(f"drawer_r{row}_c0")
        right_drawer = object_model.get_part(f"drawer_r{row}_c1")

        left_front = left_drawer.get_visual("front")
        right_front = right_drawer.get_visual("front")
        left_back = left_drawer.get_visual("tray_back")
        right_back = right_drawer.get_visual("tray_back")
        left_runner_outer = left_drawer.get_visual("runner_left")
        left_runner_inner = left_drawer.get_visual("runner_right")
        right_runner_inner = right_drawer.get_visual("runner_left")
        right_runner_outer = right_drawer.get_visual("runner_right")
        left_pull = left_drawer.get_visual("cup_pull")
        right_pull = right_drawer.get_visual("cup_pull")
        left_knob = left_drawer.get_visual("knob")
        right_knob = right_drawer.get_visual("knob")

        left_outer_lower = cabinet.get_visual(f"guide_r{row}_fc0_outer_lower")
        left_outer_upper = cabinet.get_visual(f"guide_r{row}_fc0_outer_upper")
        left_inner_lower = cabinet.get_visual(f"guide_r{row}_fc0_inner_lower")
        left_inner_upper = cabinet.get_visual(f"guide_r{row}_fc0_inner_upper")
        right_inner_lower = cabinet.get_visual(f"guide_r{row}_fc1_inner_lower")
        right_inner_upper = cabinet.get_visual(f"guide_r{row}_fc1_inner_upper")
        right_outer_lower = cabinet.get_visual(f"guide_r{row}_fc1_outer_lower")
        right_outer_upper = cabinet.get_visual(f"guide_r{row}_fc1_outer_upper")

        ctx.expect_overlap(
            left_drawer,
            cabinet,
            axes="xz",
            min_overlap=0.015,
            elem_a=left_front,
            name=f"row_{row}_left_front_reads_inside_cabinet_face",
        )
        ctx.expect_overlap(
            right_drawer,
            cabinet,
            axes="xz",
            min_overlap=0.015,
            elem_a=right_front,
            name=f"row_{row}_right_front_reads_inside_cabinet_face",
        )
        ctx.expect_gap(
            right_drawer,
            left_drawer,
            axis="x",
            min_gap=0.008,
            max_gap=0.020,
            positive_elem=right_front,
            negative_elem=left_front,
            name=f"row_{row}_two_column_gap",
        )
        for drawer, drawer_back, pull, knob, front, prefix in (
            (left_drawer, left_back, left_pull, left_knob, left_front, f"row_{row}_left"),
            (right_drawer, right_back, right_pull, right_knob, right_front, f"row_{row}_right"),
        ):
            ctx.expect_gap(
                drawer,
                cabinet,
                axis="y",
                min_gap=0.028,
                positive_elem=drawer_back,
                negative_elem=back_panel,
                name=f"{prefix}_back_panel_clearance",
            )
            ctx.expect_gap(
                drawer,
                cabinet,
                axis="y",
                min_gap=0.029,
                positive_elem=pull,
                negative_elem=toe_panel,
                name=f"{prefix}_cup_pull_projects_from_face",
            )
            ctx.expect_gap(
                drawer,
                cabinet,
                axis="y",
                min_gap=0.031,
                positive_elem=knob,
                negative_elem=toe_panel,
                name=f"{prefix}_knob_projects_from_face",
            )
            ctx.expect_overlap(
                drawer,
                drawer,
                axes="xz",
                min_overlap=0.0004,
                elem_a=pull,
                elem_b=front,
                name=f"{prefix}_cup_pull_centered_on_front",
            )
            ctx.expect_overlap(
                drawer,
                drawer,
                axes="xz",
                min_overlap=0.00005,
                elem_a=knob,
                elem_b=pull,
                name=f"{prefix}_knob_nested_in_cup_pull",
            )

        for drawer, runner, lower, upper, label in (
            (left_drawer, left_runner_outer, left_outer_lower, left_outer_upper, "left_outer"),
            (left_drawer, left_runner_inner, left_inner_lower, left_inner_upper, "left_inner"),
            (right_drawer, right_runner_inner, right_inner_lower, right_inner_upper, "right_inner"),
            (right_drawer, right_runner_outer, right_outer_lower, right_outer_upper, "right_outer"),
        ):
            ctx.expect_contact(
                drawer,
                cabinet,
                elem_a=runner,
                elem_b=lower,
                name=f"row_{row}_{label}_runner_touches_lower_channel",
            )
            ctx.expect_contact(
                drawer,
                cabinet,
                elem_a=runner,
                elem_b=upper,
                name=f"row_{row}_{label}_runner_touches_upper_channel",
            )

    for row in range(1, ROWS):
        for col in range(COLS):
            upper = object_model.get_part(f"drawer_r{row}_c{col}")
            lower = object_model.get_part(f"drawer_r{row - 1}_c{col}")
            upper_front = upper.get_visual("front")
            lower_front = lower.get_visual("front")
            ctx.expect_gap(
                upper,
                lower,
                axis="z",
                min_gap=0.008,
                max_gap=0.020,
                positive_elem=upper_front,
                negative_elem=lower_front,
                name=f"column_{col}_row_{row}_stacked_drawer_gap",
            )

    for col in range(COLS):
        bottom = object_model.get_part(f"drawer_r0_c{col}")
        bottom_front = bottom.get_visual("front")
        ctx.expect_gap(
            bottom,
            cabinet,
            axis="y",
            min_gap=0.030,
            max_gap=0.040,
            positive_elem=bottom_front,
            negative_elem=toe_panel,
            name=f"column_{col}_toe_kick_setback_visible",
        )
        ctx.expect_gap(
            bottom,
            cabinet,
            axis="z",
            min_gap=0.070,
            positive_elem=bottom_front,
            negative_elem=toe_floor,
            name=f"column_{col}_bottom_drawer_above_toe_recess_floor",
        )

    bottom_left = object_model.get_part("drawer_r0_c0")
    top_right = object_model.get_part("drawer_r5_c1")
    bottom_left_front = bottom_left.get_visual("front")
    top_right_front = top_right.get_visual("front")
    bottom_left_outer_runner = bottom_left.get_visual("runner_left")
    top_right_outer_runner = top_right.get_visual("runner_right")
    bottom_left_outer_lower = cabinet.get_visual("guide_r0_fc0_outer_lower")
    top_right_outer_lower = cabinet.get_visual("guide_r5_fc1_outer_lower")
    bottom_left_slide = object_model.get_articulation("slide_r0_c0")
    top_right_slide = object_model.get_articulation("slide_r5_c1")

    with ctx.pose({bottom_left_slide: OPEN_DISTANCE, top_right_slide: OPEN_DISTANCE * 0.8}):
        ctx.expect_contact(
            bottom_left,
            cabinet,
            elem_a=bottom_left_outer_runner,
            elem_b=bottom_left_outer_lower,
            name="opened_bottom_left_drawer_stays_in_its_channel",
        )
        ctx.expect_contact(
            top_right,
            cabinet,
            elem_a=top_right_outer_runner,
            elem_b=top_right_outer_lower,
            name="opened_top_right_drawer_stays_in_its_channel",
        )
        ctx.expect_gap(
            bottom_left,
            cabinet,
            axis="y",
            min_gap=0.150,
            positive_elem=bottom_left_front,
            negative_elem=toe_panel,
            name="bottom_left_drawer_has_visible_travel",
        )
        ctx.expect_gap(
            top_right,
            cabinet,
            axis="y",
            min_gap=0.125,
            positive_elem=top_right_front,
            negative_elem=toe_panel,
            name="top_right_drawer_has_visible_travel",
        )
        ctx.expect_overlap(
            top_right,
            cabinet,
            axes="xz",
            min_overlap=0.015,
            elem_a=top_right_front,
            name="opened_top_right_drawer_stays_registered_to_opening",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
