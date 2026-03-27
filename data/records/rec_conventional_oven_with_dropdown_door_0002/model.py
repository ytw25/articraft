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
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

WIDTH = 0.91
DEPTH = 0.70
HEIGHT = 0.93
SIDE_T = 0.03
TOP_T = 0.045
BACK_T = 0.025
FRONT_T = 0.03
FRAME_W = WIDTH - 2.0 * SIDE_T
FRONT_Y = DEPTH * 0.5 - FRONT_T * 0.5

DOOR_OPEN_W = 0.78
DOOR_OPEN_H = 0.50
DOOR_BOTTOM_Z = 0.20
DOOR_W = 0.82
DOOR_H = 0.53
DOOR_T = 0.032
DOOR_AXIS_Y = DEPTH * 0.5

DRAWER_OPEN_W = 0.80
DRAWER_OPEN_H = 0.128
DRAWER_BOTTOM_Z = 0.03
DRAWER_FRONT_W = 0.84
DRAWER_H = 0.14
DRAWER_FRONT_H = 0.125
DRAWER_FRONT_T = 0.03
DRAWER_DEPTH = 0.50
DRAWER_BACK_Y = 0.365 - (DRAWER_DEPTH - DRAWER_FRONT_T * 0.5)

WINDOW_W = 0.62
WINDOW_H = 0.285
WINDOW_BOTTOM_Z = 0.135

GRATE_W = 0.22
GRATE_D = 0.20
GRATE_BAR = 0.015
GRATE_H = 0.018

HINGE_ARM_X = DOOR_W * 0.5 + 0.011
HINGE_BRACKET_X = HINGE_ARM_X + 0.020
DRAWER_RUNNER_X = DRAWER_OPEN_W * 0.5 - 0.016
GUIDE_RAIL_X = WIDTH * 0.5 - SIDE_T - 0.013


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_range", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.14, 0.14, 0.15, 1.0))
    black_enamel = model.material("black_enamel", rgba=(0.08, 0.08, 0.09, 1.0))
    oven_glass = model.material("oven_glass", rgba=(0.58, 0.70, 0.78, 0.30))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.80, 0.81, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        Box((SIDE_T, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-(WIDTH * 0.5 - SIDE_T * 0.5), 0.0, HEIGHT * 0.5)),
        material=stainless,
        name="left_side_panel",
    )
    body.visual(
        Box((SIDE_T, DEPTH, HEIGHT)),
        origin=Origin(xyz=(WIDTH * 0.5 - SIDE_T * 0.5, 0.0, HEIGHT * 0.5)),
        material=stainless,
        name="right_side_panel",
    )
    body.visual(
        Box((FRAME_W, BACK_T, HEIGHT - TOP_T)),
        origin=Origin(xyz=(0.0, -(DEPTH * 0.5 - BACK_T * 0.5), (HEIGHT - TOP_T) * 0.5)),
        material=stainless,
        name="rear_panel",
    )
    body.visual(
        Box((WIDTH, DEPTH, TOP_T)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT - TOP_T * 0.5)),
        material=stainless,
        name="top_shell",
    )
    body.visual(
        Box((FRAME_W, FRONT_T, HEIGHT - (DOOR_BOTTOM_Z + DOOR_OPEN_H))),
        origin=Origin(xyz=(0.0, FRONT_Y, DOOR_BOTTOM_Z + DOOR_OPEN_H + (HEIGHT - (DOOR_BOTTOM_Z + DOOR_OPEN_H)) * 0.5)),
        material=stainless,
        name="front_frame",
    )
    body.visual(
        Box((FRAME_W, FRONT_T, DOOR_BOTTOM_Z - (DRAWER_BOTTOM_Z + DRAWER_OPEN_H))),
        origin=Origin(xyz=(0.0, FRONT_Y, DRAWER_BOTTOM_Z + DRAWER_OPEN_H + (DOOR_BOTTOM_Z - (DRAWER_BOTTOM_Z + DRAWER_OPEN_H)) * 0.5)),
        material=stainless,
        name="door_drawer_rail",
    )
    body.visual(
        Box((FRAME_W, FRONT_T, DRAWER_BOTTOM_Z)),
        origin=Origin(xyz=(0.0, FRONT_Y, DRAWER_BOTTOM_Z * 0.5)),
        material=stainless,
        name="toe_kick_rail",
    )
    body.visual(
        Box(((FRAME_W - DOOR_OPEN_W) * 0.5, FRONT_T, DOOR_OPEN_H)),
        origin=Origin(xyz=(-(DOOR_OPEN_W * 0.5 + (FRAME_W - DOOR_OPEN_W) * 0.25), FRONT_Y, DOOR_BOTTOM_Z + DOOR_OPEN_H * 0.5)),
        material=stainless,
        name="left_door_jamb",
    )
    body.visual(
        Box(((FRAME_W - DOOR_OPEN_W) * 0.5, FRONT_T, DOOR_OPEN_H)),
        origin=Origin(xyz=(DOOR_OPEN_W * 0.5 + (FRAME_W - DOOR_OPEN_W) * 0.25, FRONT_Y, DOOR_BOTTOM_Z + DOOR_OPEN_H * 0.5)),
        material=stainless,
        name="right_door_jamb",
    )
    body.visual(
        Box(((FRAME_W - DRAWER_OPEN_W) * 0.5, FRONT_T, DRAWER_OPEN_H)),
        origin=Origin(xyz=(-(DRAWER_OPEN_W * 0.5 + (FRAME_W - DRAWER_OPEN_W) * 0.25), FRONT_Y, DRAWER_BOTTOM_Z + DRAWER_OPEN_H * 0.5)),
        material=stainless,
        name="left_drawer_jamb",
    )
    body.visual(
        Box(((FRAME_W - DRAWER_OPEN_W) * 0.5, FRONT_T, DRAWER_OPEN_H)),
        origin=Origin(xyz=(DRAWER_OPEN_W * 0.5 + (FRAME_W - DRAWER_OPEN_W) * 0.25, FRONT_Y, DRAWER_BOTTOM_Z + DRAWER_OPEN_H * 0.5)),
        material=stainless,
        name="right_drawer_jamb",
    )
    body.visual(
        Box((FRAME_W - 0.02, DEPTH - 0.08, 0.008)),
        origin=Origin(xyz=(0.0, -0.01, HEIGHT - 0.004)),
        material=black_enamel,
        name="cooktop_surface",
    )
    body.visual(
        Box((DOOR_OPEN_W, 0.645, 0.018)),
        origin=Origin(xyz=(0.0, -0.0025, DOOR_BOTTOM_Z + 0.009)),
        material=dark_steel,
        name="oven_floor",
    )
    body.visual(
        Box((DOOR_OPEN_W, 0.645, 0.018)),
        origin=Origin(xyz=(0.0, -0.0025, DOOR_BOTTOM_Z + DOOR_OPEN_H - 0.009)),
        material=dark_steel,
        name="oven_ceiling",
    )
    body.visual(
        Box((0.025, 0.645, DOOR_OPEN_H)),
        origin=Origin(xyz=(-(DOOR_OPEN_W * 0.5 - 0.0125), -0.0025, DOOR_BOTTOM_Z + DOOR_OPEN_H * 0.5)),
        material=dark_steel,
        name="oven_left_liner",
    )
    body.visual(
        Box((0.025, 0.645, DOOR_OPEN_H)),
        origin=Origin(xyz=(DOOR_OPEN_W * 0.5 - 0.0125, -0.0025, DOOR_BOTTOM_Z + DOOR_OPEN_H * 0.5)),
        material=dark_steel,
        name="oven_right_liner",
    )
    body.visual(
        Box((DOOR_OPEN_W, 0.015, DOOR_OPEN_H)),
        origin=Origin(xyz=(0.0, -0.3175, DOOR_BOTTOM_Z + DOOR_OPEN_H * 0.5)),
        material=dark_steel,
        name="oven_back_liner",
    )
    body.visual(
        Box((0.026, 0.34, 0.016)),
        origin=Origin(xyz=(-GUIDE_RAIL_X, -0.005, DRAWER_BOTTOM_Z + 0.055)),
        material=dark_steel,
        name="left_guide_rail",
    )
    body.visual(
        Box((0.026, 0.34, 0.016)),
        origin=Origin(xyz=(GUIDE_RAIL_X, -0.005, DRAWER_BOTTOM_Z + 0.055)),
        material=dark_steel,
        name="right_guide_rail",
    )
    body.visual(
        Box((0.022, 0.050, 0.080)),
        origin=Origin(xyz=(-HINGE_BRACKET_X, 0.336, DOOR_BOTTOM_Z + 0.050)),
        material=dark_steel,
        name="left_hinge_bracket",
    )
    body.visual(
        Box((0.022, 0.050, 0.080)),
        origin=Origin(xyz=(HINGE_BRACKET_X, 0.336, DOOR_BOTTOM_Z + 0.050)),
        material=dark_steel,
        name="right_hinge_bracket",
    )
    for index, x_pos in enumerate((-0.30, -0.18, -0.06, 0.06, 0.18, 0.30), start=1):
        body.visual(
            Cylinder(radius=0.018, length=0.032),
            origin=Origin(xyz=(x_pos, 0.366, 0.790), rpy=(math.pi * 0.5, 0.0, 0.0)),
            material=dark_steel,
            name=f"control_knob_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((WIDTH, DEPTH, HEIGHT)),
        mass=110.0,
        origin=Origin(xyz=(0.0, 0.0, HEIGHT * 0.5)),
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        Box((0.82, 0.54, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=black_enamel,
        name="grate_support_tray",
    )
    grate_positions = [
        (-0.27, 0.12),
        (0.0, 0.12),
        (0.27, 0.12),
        (-0.27, -0.10),
        (0.0, -0.10),
        (0.27, -0.10),
    ]
    for index, (x_pos, y_pos) in enumerate(grate_positions, start=1):
        cooktop.visual(
            Box((GRATE_W, GRATE_BAR, GRATE_H)),
            origin=Origin(xyz=(x_pos, y_pos - (GRATE_D * 0.5 - GRATE_BAR * 0.5), GRATE_H * 0.5)),
            material=cast_iron,
            name=f"grate_{index}",
        )
        cooktop.visual(
            Box((GRATE_W, GRATE_BAR, GRATE_H)),
            origin=Origin(xyz=(x_pos, y_pos + (GRATE_D * 0.5 - GRATE_BAR * 0.5), GRATE_H * 0.5)),
            material=cast_iron,
            name=f"grate_{index}_rear_bar",
        )
        cooktop.visual(
            Box((GRATE_BAR, GRATE_D - 2.0 * GRATE_BAR, GRATE_H)),
            origin=Origin(xyz=(x_pos - (GRATE_W * 0.5 - GRATE_BAR * 0.5), y_pos, GRATE_H * 0.5)),
            material=cast_iron,
            name=f"grate_{index}_left_bar",
        )
        cooktop.visual(
            Box((GRATE_BAR, GRATE_D - 2.0 * GRATE_BAR, GRATE_H)),
            origin=Origin(xyz=(x_pos + (GRATE_W * 0.5 - GRATE_BAR * 0.5), y_pos, GRATE_H * 0.5)),
            material=cast_iron,
            name=f"grate_{index}_right_bar",
        )
        cooktop.visual(
            Box((GRATE_W * 0.72, GRATE_BAR, GRATE_H)),
            origin=Origin(xyz=(x_pos, y_pos, GRATE_H * 0.5)),
            material=cast_iron,
            name=f"grate_{index}_cross_x",
        )
        cooktop.visual(
            Box((GRATE_BAR, GRATE_D * 0.72, GRATE_H)),
            origin=Origin(xyz=(x_pos, y_pos, GRATE_H * 0.5)),
            material=cast_iron,
            name=f"grate_{index}_cross_y",
        )
        cooktop.visual(
            Cylinder(radius=0.036, length=0.012),
            origin=Origin(xyz=(x_pos, y_pos, 0.006)),
            material=black_enamel,
            name=f"burner_cap_{index}",
        )
    cooktop.inertial = Inertial.from_geometry(
        Box((0.82, 0.54, 0.03)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H - (WINDOW_BOTTOM_Z + WINDOW_H))),
        origin=Origin(xyz=(0.0, DOOR_T * 0.5, WINDOW_BOTTOM_Z + WINDOW_H + (DOOR_H - (WINDOW_BOTTOM_Z + WINDOW_H)) * 0.5)),
        material=stainless,
        name="door_frame",
    )
    door.visual(
        Box((DOOR_W, DOOR_T, WINDOW_BOTTOM_Z)),
        origin=Origin(xyz=(0.0, DOOR_T * 0.5, WINDOW_BOTTOM_Z * 0.5)),
        material=stainless,
        name="door_bottom_rail",
    )
    door.visual(
        Box(((DOOR_W - WINDOW_W) * 0.5, DOOR_T, WINDOW_H)),
        origin=Origin(xyz=(-(WINDOW_W * 0.5 + (DOOR_W - WINDOW_W) * 0.25), DOOR_T * 0.5, WINDOW_BOTTOM_Z + WINDOW_H * 0.5)),
        material=stainless,
        name="door_left_stile",
    )
    door.visual(
        Box(((DOOR_W - WINDOW_W) * 0.5, DOOR_T, WINDOW_H)),
        origin=Origin(xyz=(WINDOW_W * 0.5 + (DOOR_W - WINDOW_W) * 0.25, DOOR_T * 0.5, WINDOW_BOTTOM_Z + WINDOW_H * 0.5)),
        material=stainless,
        name="door_right_stile",
    )
    door.visual(
        Box((DOOR_W - 0.08, 0.018, DOOR_H - 0.08)),
        origin=Origin(xyz=(0.0, 0.010, DOOR_H * 0.5)),
        material=dark_steel,
        name="door_inner_panel",
    )
    door.visual(
        Box((WINDOW_W - 0.025, 0.004, WINDOW_H - 0.020)),
        origin=Origin(xyz=(0.0, 0.010, WINDOW_BOTTOM_Z + WINDOW_H * 0.5)),
        material=oven_glass,
        name="inner_glass",
    )
    door.visual(
        Box((WINDOW_W - 0.010, 0.004, WINDOW_H - 0.010)),
        origin=Origin(xyz=(0.0, 0.024, WINDOW_BOTTOM_Z + WINDOW_H * 0.5)),
        material=oven_glass,
        name="outer_glass",
    )
    door.visual(
        Box((WINDOW_W - 0.024, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.017, WINDOW_BOTTOM_Z + 0.006)),
        material=dark_steel,
        name="window_spacer_bottom",
    )
    door.visual(
        Box((WINDOW_W - 0.024, 0.010, 0.012)),
        origin=Origin(xyz=(0.0, 0.017, WINDOW_BOTTOM_Z + WINDOW_H - 0.006)),
        material=dark_steel,
        name="window_spacer_top",
    )
    door.visual(
        Box((0.012, 0.010, WINDOW_H - 0.020)),
        origin=Origin(xyz=(-(WINDOW_W * 0.5 - 0.006), 0.017, WINDOW_BOTTOM_Z + WINDOW_H * 0.5)),
        material=dark_steel,
        name="window_spacer_left",
    )
    door.visual(
        Box((0.012, 0.010, WINDOW_H - 0.020)),
        origin=Origin(xyz=(WINDOW_W * 0.5 - 0.006, 0.017, WINDOW_BOTTOM_Z + WINDOW_H * 0.5)),
        material=dark_steel,
        name="window_spacer_right",
    )
    door.visual(
        Box((0.048, 0.012, 0.072)),
        origin=Origin(xyz=(-(DOOR_W * 0.5 - 0.075), DOOR_T + 0.006, 0.410)),
        material=brushed_aluminum,
        name="left_handle_bracket_base",
    )
    door.visual(
        Box((0.048, 0.012, 0.072)),
        origin=Origin(xyz=(DOOR_W * 0.5 - 0.075, DOOR_T + 0.006, 0.410)),
        material=brushed_aluminum,
        name="right_handle_bracket_base",
    )
    door.visual(
        Box((0.024, 0.034, 0.028)),
        origin=Origin(xyz=(-(DOOR_W * 0.5 - 0.075), DOOR_T + 0.029, 0.410)),
        material=brushed_aluminum,
        name="left_handle_bracket_arm",
    )
    door.visual(
        Box((0.024, 0.034, 0.028)),
        origin=Origin(xyz=(DOOR_W * 0.5 - 0.075, DOOR_T + 0.029, 0.410)),
        material=brushed_aluminum,
        name="right_handle_bracket_arm",
    )
    door.visual(
        Cylinder(radius=0.014, length=DOOR_W - 0.174),
        origin=Origin(xyz=(0.0, DOOR_T + 0.030, 0.410), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=brushed_aluminum,
        name="handle_bar",
    )
    door.visual(
        Box((0.018, 0.044, 0.018)),
        origin=Origin(xyz=(-HINGE_ARM_X, 0.033, 0.050)),
        material=dark_steel,
        name="left_hinge_arm",
    )
    door.visual(
        Box((0.018, 0.044, 0.018)),
        origin=Origin(xyz=(HINGE_ARM_X, 0.033, 0.050)),
        material=dark_steel,
        name="right_hinge_arm",
    )
    door.visual(
        Box((0.028, 0.032, 0.028)),
        origin=Origin(xyz=(-(DOOR_W * 0.5 - 0.001), 0.024, 0.050)),
        material=dark_steel,
        name="left_hinge_mount",
    )
    door.visual(
        Box((0.028, 0.032, 0.028)),
        origin=Origin(xyz=(DOOR_W * 0.5 - 0.001, 0.024, 0.050)),
        material=dark_steel,
        name="right_hinge_mount",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, 0.14, DOOR_H)),
        mass=21.0,
        origin=Origin(xyz=(0.0, 0.070, DOOR_H * 0.5)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_FRONT_W, DRAWER_FRONT_T, DRAWER_FRONT_H)),
        origin=Origin(xyz=(0.0, DRAWER_DEPTH - DRAWER_FRONT_T * 0.5, DRAWER_FRONT_H * 0.5)),
        material=stainless,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.74, DRAWER_DEPTH - 0.10, 0.012)),
        origin=Origin(xyz=(0.0, 0.21, 0.006)),
        material=dark_steel,
        name="drawer_floor",
    )
    drawer.visual(
        Box((0.012, DRAWER_DEPTH - 0.03, 0.096)),
        origin=Origin(xyz=(-0.366, 0.235, 0.055)),
        material=dark_steel,
        name="drawer_left_side",
    )
    drawer.visual(
        Box((0.012, DRAWER_DEPTH - 0.03, 0.096)),
        origin=Origin(xyz=(0.366, 0.235, 0.055)),
        material=dark_steel,
        name="drawer_right_side",
    )
    drawer.visual(
        Box((0.74, 0.012, 0.096)),
        origin=Origin(xyz=(0.0, 0.006, 0.055)),
        material=dark_steel,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.016, 0.32, 0.016)),
        origin=Origin(xyz=(-DRAWER_RUNNER_X, 0.17, 0.055)),
        material=dark_steel,
        name="left_runner",
    )
    drawer.visual(
        Box((0.016, 0.32, 0.016)),
        origin=Origin(xyz=(DRAWER_RUNNER_X, 0.17, 0.055)),
        material=dark_steel,
        name="right_runner",
    )
    drawer.visual(
        Box((0.016, 0.050, 0.020)),
        origin=Origin(xyz=(-0.376, 0.160, 0.055)),
        material=dark_steel,
        name="left_runner_mount",
    )
    drawer.visual(
        Box((0.016, 0.050, 0.020)),
        origin=Origin(xyz=(0.376, 0.160, 0.055)),
        material=dark_steel,
        name="right_runner_mount",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FRONT_W, DRAWER_DEPTH, DRAWER_H)),
        mass=12.0,
        origin=Origin(xyz=(0.0, DRAWER_DEPTH * 0.5, DRAWER_H * 0.5)),
    )

    model.articulation(
        "body_to_cooktop",
        ArticulationType.FIXED,
        parent=body,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, HEIGHT)),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, DOOR_AXIS_Y, DOOR_BOTTOM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.6, lower=0.0, upper=1.45),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, DRAWER_BACK_Y, DRAWER_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.24),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    cooktop = object_model.get_part("cooktop")
    door = object_model.get_part("door")
    drawer = object_model.get_part("drawer")
    door_hinge = object_model.get_articulation("body_to_door")
    drawer_slide = object_model.get_articulation("body_to_drawer")

    front_frame = body.get_visual("front_frame")
    cooktop_surface = body.get_visual("cooktop_surface")
    left_guide = body.get_visual("left_guide_rail")
    right_guide = body.get_visual("right_guide_rail")
    left_hinge_bracket = body.get_visual("left_hinge_bracket")
    right_hinge_bracket = body.get_visual("right_hinge_bracket")

    door_frame = door.get_visual("door_frame")
    door_bottom_rail = door.get_visual("door_bottom_rail")
    door_left_stile = door.get_visual("door_left_stile")
    door_right_stile = door.get_visual("door_right_stile")
    inner_glass = door.get_visual("inner_glass")
    outer_glass = door.get_visual("outer_glass")
    handle_bar = door.get_visual("handle_bar")
    left_handle_bracket_arm = door.get_visual("left_handle_bracket_arm")
    right_handle_bracket_arm = door.get_visual("right_handle_bracket_arm")
    left_hinge_arm = door.get_visual("left_hinge_arm")
    right_hinge_arm = door.get_visual("right_hinge_arm")

    drawer_front = drawer.get_visual("drawer_front")
    left_runner = drawer.get_visual("left_runner")
    right_runner = drawer.get_visual("right_runner")

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

    for index in range(1, 7):
        grate_front = cooktop.get_visual(f"grate_{index}")
        grate_side = cooktop.get_visual(f"grate_{index}_left_bar")
        ctx.expect_contact(cooktop, body, elem_a=grate_front, elem_b=cooktop_surface, name=f"grate_{index}_seats_on_cooktop")
        ctx.expect_overlap(
            cooktop,
            body,
            axes="x",
            elem_a=grate_front,
            elem_b=cooktop_surface,
            min_overlap=0.18,
            name=f"grate_{index}_spans_burner_width",
        )
        ctx.expect_overlap(
            cooktop,
            body,
            axes="y",
            elem_a=grate_side,
            elem_b=cooktop_surface,
            min_overlap=0.15,
            name=f"grate_{index}_spans_burner_depth",
        )

    ctx.expect_overlap(door, body, axes="x", elem_a=door_frame, elem_b=front_frame, min_overlap=0.76)
    ctx.expect_gap(door, drawer, axis="z", positive_elem=door_bottom_rail, negative_elem=drawer_front, min_gap=0.03)

    ctx.expect_overlap(door, door, axes="x", elem_a=outer_glass, elem_b=door_frame, min_overlap=0.60)
    ctx.expect_overlap(door, door, axes="x", elem_a=outer_glass, elem_b=door_bottom_rail, min_overlap=0.60)
    ctx.expect_overlap(door, door, axes="z", elem_a=outer_glass, elem_b=door_left_stile, min_overlap=0.24)
    ctx.expect_overlap(door, door, axes="z", elem_a=outer_glass, elem_b=door_right_stile, min_overlap=0.24)
    ctx.expect_gap(
        door,
        door,
        axis="y",
        positive_elem=outer_glass,
        negative_elem=inner_glass,
        min_gap=0.010,
        max_gap=0.020,
    )
    ctx.expect_contact(door, door, elem_a=handle_bar, elem_b=left_handle_bracket_arm)
    ctx.expect_contact(door, door, elem_a=handle_bar, elem_b=right_handle_bracket_arm)
    ctx.expect_overlap(door, door, axes="x", elem_a=handle_bar, elem_b=door_frame, min_overlap=0.60)
    ctx.expect_gap(door, door, axis="y", positive_elem=handle_bar, negative_elem=outer_glass, min_gap=0.018)

    ctx.expect_overlap(drawer, body, axes="yz", elem_a=left_runner, elem_b=left_guide, min_overlap=0.012)
    ctx.expect_overlap(drawer, body, axes="yz", elem_a=right_runner, elem_b=right_guide, min_overlap=0.012)
    ctx.expect_overlap(drawer, body, axes="x", elem_a=drawer_front, elem_b=front_frame, min_overlap=0.78)
    ctx.expect_contact(door, body, elem_a=left_hinge_arm, elem_b=left_hinge_bracket)
    ctx.expect_contact(door, body, elem_a=right_hinge_arm, elem_b=right_hinge_bracket)

    with ctx.pose({door_hinge: 1.35}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem=left_hinge_arm,
            negative_elem=left_hinge_bracket,
            max_gap=0.035,
            max_penetration=0.0,
            name="left_hinge_arm_tracks_wall_bracket_when_open",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem=right_hinge_arm,
            negative_elem=right_hinge_bracket,
            max_gap=0.035,
            max_penetration=0.0,
            name="right_hinge_arm_tracks_wall_bracket_when_open",
        )
        ctx.expect_gap(door, drawer, axis="z", positive_elem=door_bottom_rail, negative_elem=drawer_front, min_gap=0.012)

    with ctx.pose({drawer_slide: 0.22}):
        ctx.expect_gap(drawer, body, axis="y", positive_elem=drawer_front, negative_elem=front_frame, min_gap=0.18)
        ctx.expect_overlap(drawer, body, axes="yz", elem_a=left_runner, elem_b=left_guide, min_overlap=0.010)
        ctx.expect_overlap(drawer, body, axes="yz", elem_a=right_runner, elem_b=right_guide, min_overlap=0.010)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
