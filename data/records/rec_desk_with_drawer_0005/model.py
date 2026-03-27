from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

SAFE_ASSET_ROOT = "/tmp"


CASE_WIDTH = 0.94
CASE_DEPTH = 0.42
FOOT_HEIGHT = 0.11
CASE_HEIGHT = 1.46
SIDE_THICKNESS = 0.022
BACK_THICKNESS = 0.016
TOP_THICKNESS = 0.024
FRAME_DEPTH = 0.020
FRONT_STILE_WIDTH = 0.075

LID_WIDTH = 0.782
LID_HEIGHT = 0.552
LID_THICKNESS = 0.024
LID_BOTTOM_Z = 0.79

DRAWER_FRONT_WIDTH = 0.782
DRAWER_FRONT_HEIGHT = 0.234
DRAWER_FRONT_THICKNESS = 0.022
DRAWER_BOX_WIDTH = 0.76
DRAWER_BOX_DEPTH = 0.325
DRAWER_BOX_HEIGHT = 0.205
DRAWER_SIDE_THICKNESS = 0.012
DRAWER_BOTTOM_THICKNESS = 0.010
LOWER_DRAWER_Z = 0.172
UPPER_DRAWER_Z = 0.462

GUIDE_RAIL_THICKNESS = 0.014
RUNNER_THICKNESS = 0.010
GUIDE_RAIL_HEIGHT = 0.018
GUIDE_RAIL_LENGTH = 0.285
GUIDE_RAIL_START_Y = 0.055
OPEN_LID_ANGLE = 1.53
STAY_X = 0.365
STAY_PAD_Y = LID_THICKNESS - 0.007
STAY_PAD_Z = 0.210
STAY_ANCHOR_Y = 0.048 - 0.018 / 2.0
STAY_ANCHOR_Z = LID_BOTTOM_Z + 0.258


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="secretary_desk")

    walnut = model.material("walnut", rgba=(0.36, 0.22, 0.13, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.24, 0.14, 0.08, 1.0))
    maple = model.material("maple", rgba=(0.71, 0.58, 0.42, 1.0))
    leather = model.material("leather_green", rgba=(0.18, 0.29, 0.17, 1.0))
    brass = model.material("aged_brass", rgba=(0.72, 0.61, 0.31, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((CASE_WIDTH - 0.24, 0.028, 0.048)),
        origin=Origin(xyz=(0.0, 0.014, 0.086)),
        material=walnut_dark,
        name="front_bracket_apron",
    )
    cabinet.visual(
        Box((0.028, CASE_DEPTH - 0.14, 0.050)),
        origin=Origin(
            xyz=(
                -(CASE_WIDTH / 2.0) + 0.014,
                0.11 + (CASE_DEPTH - 0.14) / 2.0,
                0.085,
            )
        ),
        material=walnut_dark,
        name="left_side_skirt",
    )
    cabinet.visual(
        Box((0.028, CASE_DEPTH - 0.14, 0.050)),
        origin=Origin(
            xyz=(
                (CASE_WIDTH / 2.0) - 0.014,
                0.11 + (CASE_DEPTH - 0.14) / 2.0,
                0.085,
            )
        ),
        material=walnut_dark,
        name="right_side_skirt",
    )
    cabinet.visual(
        Box((CASE_WIDTH - 0.06, 0.028, 0.058)),
        origin=Origin(xyz=(0.0, CASE_DEPTH - 0.014, 0.081)),
        material=walnut_dark,
        name="rear_stretcher",
    )
    for side, sign in ((("front_left_foot"), -1.0), (("front_right_foot"), 1.0)):
        cabinet.visual(
            Box((0.100, 0.102, FOOT_HEIGHT)),
            origin=Origin(
                xyz=(
                    sign * ((CASE_WIDTH / 2.0) - 0.050),
                    0.051,
                    FOOT_HEIGHT / 2.0,
                )
            ),
            material=walnut_dark,
            name=side,
        )
    for side, sign in ((("rear_left_foot"), -1.0), (("rear_right_foot"), 1.0)):
        cabinet.visual(
            Box((0.075, 0.082, FOOT_HEIGHT * 0.92)),
            origin=Origin(
                xyz=(
                    sign * ((CASE_WIDTH / 2.0) - 0.0375),
                    CASE_DEPTH - 0.041,
                    FOOT_HEIGHT * 0.46,
                )
            ),
            material=walnut_dark,
            name=side,
        )
    cabinet.visual(
        Box((CASE_WIDTH - 2.0 * SIDE_THICKNESS, CASE_DEPTH - BACK_THICKNESS, 0.022)),
        origin=Origin(
            xyz=(
                0.0,
                (CASE_DEPTH - BACK_THICKNESS) / 2.0,
                FOOT_HEIGHT + 0.011,
            )
        ),
        material=walnut_dark,
        name="case_floor",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, CASE_DEPTH, CASE_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CASE_WIDTH / 2.0) + (SIDE_THICKNESS / 2.0),
                CASE_DEPTH / 2.0,
                FOOT_HEIGHT + CASE_HEIGHT / 2.0,
            )
        ),
        material=walnut,
        name="left_side",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, CASE_DEPTH, CASE_HEIGHT)),
        origin=Origin(
            xyz=(
                (CASE_WIDTH / 2.0) - (SIDE_THICKNESS / 2.0),
                CASE_DEPTH / 2.0,
                FOOT_HEIGHT + CASE_HEIGHT / 2.0,
            )
        ),
        material=walnut,
        name="right_side",
    )
    cabinet.visual(
        Box((CASE_WIDTH - 2.0 * SIDE_THICKNESS, BACK_THICKNESS, CASE_HEIGHT - TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                CASE_DEPTH - BACK_THICKNESS / 2.0,
                FOOT_HEIGHT + (CASE_HEIGHT - TOP_THICKNESS) / 2.0,
            )
        ),
        material=walnut,
        name="back_panel",
    )
    cabinet.visual(
        Box((CASE_WIDTH - 2.0 * SIDE_THICKNESS, CASE_DEPTH, TOP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                CASE_DEPTH / 2.0,
                FOOT_HEIGHT + CASE_HEIGHT - TOP_THICKNESS / 2.0,
            )
        ),
        material=walnut_dark,
        name="case_top",
    )
    cabinet.visual(
        Box((CASE_WIDTH + 0.04, CASE_DEPTH + 0.03, 0.03)),
        origin=Origin(
            xyz=(
                0.0,
                (CASE_DEPTH + 0.03) / 2.0 - 0.01,
                FOOT_HEIGHT + CASE_HEIGHT + 0.015,
            )
        ),
        material=walnut_dark,
        name="top_cornice",
    )
    cabinet.visual(
        Box((FRONT_STILE_WIDTH, FRAME_DEPTH, 1.32)),
        origin=Origin(
            xyz=(
                -(CASE_WIDTH / 2.0) + FRONT_STILE_WIDTH / 2.0,
                FRAME_DEPTH / 2.0,
                0.28 + 1.32 / 2.0,
            )
        ),
        material=walnut_dark,
        name="front_stile_left",
    )
    cabinet.visual(
        Box((FRONT_STILE_WIDTH, FRAME_DEPTH, 1.32)),
        origin=Origin(
            xyz=(
                (CASE_WIDTH / 2.0) - FRONT_STILE_WIDTH / 2.0,
                FRAME_DEPTH / 2.0,
                0.28 + 1.32 / 2.0,
            )
        ),
        material=walnut_dark,
        name="front_stile_right",
    )
    cabinet.visual(
        Box((CASE_WIDTH - 2.0 * FRONT_STILE_WIDTH, FRAME_DEPTH, 0.058)),
        origin=Origin(xyz=(0.0, FRAME_DEPTH / 2.0, FOOT_HEIGHT + 0.029)),
        material=walnut_dark,
        name="drawer_bottom_rail",
    )
    cabinet.visual(
        Box((CASE_WIDTH - 2.0 * FRONT_STILE_WIDTH, FRAME_DEPTH, 0.048)),
        origin=Origin(xyz=(0.0, FRAME_DEPTH / 2.0, 0.434)),
        material=walnut_dark,
        name="drawer_mid_rail",
    )
    cabinet.visual(
        Box((CASE_WIDTH - 2.0 * FRONT_STILE_WIDTH, FRAME_DEPTH, 0.094)),
        origin=Origin(xyz=(0.0, FRAME_DEPTH / 2.0, 0.743)),
        material=walnut_dark,
        name="hinge_rail",
    )
    cabinet.visual(
        Box((CASE_WIDTH - 2.0 * FRONT_STILE_WIDTH, FRAME_DEPTH, 0.080)),
        origin=Origin(xyz=(0.0, FRAME_DEPTH / 2.0, 1.39)),
        material=walnut_dark,
        name="top_rail",
    )
    cabinet.visual(
        Box((0.62, CASE_DEPTH - BACK_THICKNESS - 0.05, 0.020)),
        origin=Origin(
            xyz=(
                0.0,
                0.05 + (CASE_DEPTH - BACK_THICKNESS - 0.05) / 2.0,
                LID_BOTTOM_Z + 0.012,
            )
        ),
        material=maple,
        name="desk_floor",
    )
    cabinet.visual(
        Box((0.60, 0.19, 0.014)),
        origin=Origin(xyz=(0.0, CASE_DEPTH - BACK_THICKNESS - 0.19 / 2.0, 1.11)),
        material=maple,
        name="organizer_shelf",
    )
    cabinet.visual(
        Box((0.012, 0.19, 0.19)),
        origin=Origin(xyz=(-0.16, CASE_DEPTH - BACK_THICKNESS - 0.19 / 2.0, 1.015)),
        material=maple,
        name="organizer_divider_left",
    )
    cabinet.visual(
        Box((0.012, 0.19, 0.19)),
        origin=Origin(xyz=(0.16, CASE_DEPTH - BACK_THICKNESS - 0.19 / 2.0, 1.015)),
        material=maple,
        name="organizer_divider_right",
    )
    rail_x = DRAWER_BOX_WIDTH / 2.0 + RUNNER_THICKNESS + GUIDE_RAIL_THICKNESS / 2.0
    rail_support_width = (
        ((CASE_WIDTH / 2.0) - SIDE_THICKNESS) - (rail_x + GUIDE_RAIL_THICKNESS / 2.0) + 0.004
    )
    rail_support_center_x = rail_x + GUIDE_RAIL_THICKNESS / 2.0 + rail_support_width / 2.0 - 0.002
    rail_y = GUIDE_RAIL_START_Y + GUIDE_RAIL_LENGTH / 2.0
    for prefix, z0 in (("lower", LOWER_DRAWER_Z), ("upper", UPPER_DRAWER_Z)):
        cabinet.visual(
            Box((GUIDE_RAIL_THICKNESS, GUIDE_RAIL_LENGTH, GUIDE_RAIL_HEIGHT)),
            origin=Origin(xyz=(-rail_x, rail_y, z0 + 0.12)),
            material=walnut_dark,
            name=f"{prefix}_left_rail",
        )
        cabinet.visual(
            Box((GUIDE_RAIL_THICKNESS, GUIDE_RAIL_LENGTH, GUIDE_RAIL_HEIGHT)),
            origin=Origin(xyz=(rail_x, rail_y, z0 + 0.12)),
            material=walnut_dark,
            name=f"{prefix}_right_rail",
        )
        cabinet.visual(
            Box((rail_support_width, GUIDE_RAIL_LENGTH, GUIDE_RAIL_HEIGHT)),
            origin=Origin(xyz=(-rail_support_center_x, rail_y, z0 + 0.12)),
            material=walnut_dark,
            name=f"{prefix}_left_rail_support",
        )
        cabinet.visual(
            Box((rail_support_width, GUIDE_RAIL_LENGTH, GUIDE_RAIL_HEIGHT)),
            origin=Origin(xyz=(rail_support_center_x, rail_y, z0 + 0.12)),
            material=walnut_dark,
            name=f"{prefix}_right_rail_support",
        )

    hinge_bracket_x = LID_WIDTH / 2.0 + 0.010
    cabinet.visual(
        Box((0.022, 0.024, 0.040)),
        origin=Origin(xyz=(-hinge_bracket_x, 0.012, LID_BOTTOM_Z + 0.020)),
        material=brass,
        name="left_hinge_bracket",
    )
    cabinet.visual(
        Box((0.022, 0.024, 0.040)),
        origin=Origin(xyz=(hinge_bracket_x, 0.012, LID_BOTTOM_Z + 0.020)),
        material=brass,
        name="right_hinge_bracket",
    )
    cabinet.visual(
        Box((0.022, 0.018, 0.028)),
        origin=Origin(xyz=(-0.365, 0.048, LID_BOTTOM_Z + 0.258)),
        material=brass,
        name="left_stay_anchor",
    )
    cabinet.visual(
        Box((0.022, 0.018, 0.028)),
        origin=Origin(xyz=(0.365, 0.048, LID_BOTTOM_Z + 0.258)),
        material=brass,
        name="right_stay_anchor",
    )
    anchor_bridge_width = ((CASE_WIDTH / 2.0) - SIDE_THICKNESS) - (0.365 + 0.022 / 2.0) + 0.004
    anchor_bridge_center_x = 0.365 + 0.022 / 2.0 + anchor_bridge_width / 2.0 - 0.002
    cabinet.visual(
        Box((anchor_bridge_width, 0.018, 0.028)),
        origin=Origin(xyz=(-anchor_bridge_center_x, 0.048, LID_BOTTOM_Z + 0.258)),
        material=brass,
        name="left_stay_anchor_bridge",
    )
    cabinet.visual(
        Box((anchor_bridge_width, 0.018, 0.028)),
        origin=Origin(xyz=(anchor_bridge_center_x, 0.048, LID_BOTTOM_Z + 0.258)),
        material=brass,
        name="right_stay_anchor_bridge",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((CASE_WIDTH, CASE_DEPTH, FOOT_HEIGHT + CASE_HEIGHT + 0.03)),
        mass=48.0,
        origin=Origin(
            xyz=(0.0, CASE_DEPTH / 2.0, (FOOT_HEIGHT + CASE_HEIGHT + 0.03) / 2.0)
        ),
    )

    lid = model.part("lid_panel")
    lid.visual(
        Box((LID_WIDTH, LID_THICKNESS, LID_HEIGHT)),
        origin=Origin(xyz=(0.0, LID_THICKNESS / 2.0, LID_HEIGHT / 2.0)),
        material=walnut,
        name="lid_face",
    )
    lid.visual(
        Box((LID_WIDTH - 0.12, 0.004, LID_HEIGHT - 0.11)),
        origin=Origin(
            xyz=(0.0, LID_THICKNESS - 0.002, 0.055 + (LID_HEIGHT - 0.11) / 2.0)
        ),
        material=leather,
        name="writing_surface",
    )
    lid.visual(
        Box((LID_WIDTH - 0.06, 0.008, LID_HEIGHT - 0.05)),
        origin=Origin(xyz=(0.0, 0.004, LID_HEIGHT / 2.0)),
        material=walnut_dark,
        name="outer_raised_panel",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(
            xyz=(-(LID_WIDTH / 2.0) + 0.006, LID_THICKNESS / 2.0, 0.014),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="left_pin",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(
            xyz=((LID_WIDTH / 2.0) - 0.006, LID_THICKNESS / 2.0, 0.014),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=brass,
        name="right_pin",
    )
    lid.visual(
        Box((0.022, 0.014, 0.020)),
        origin=Origin(xyz=(-STAY_X, STAY_PAD_Y, STAY_PAD_Z)),
        material=brass,
        name="left_stay_pad",
    )
    lid.visual(
        Box((0.022, 0.014, 0.020)),
        origin=Origin(xyz=(STAY_X, STAY_PAD_Y, STAY_PAD_Z)),
        material=brass,
        name="right_stay_pad",
    )
    anchor_local_y = math.cos(OPEN_LID_ANGLE) * STAY_ANCHOR_Y + math.sin(OPEN_LID_ANGLE) * (
        STAY_ANCHOR_Z - LID_BOTTOM_Z
    )
    anchor_local_z = -math.sin(OPEN_LID_ANGLE) * STAY_ANCHOR_Y + math.cos(OPEN_LID_ANGLE) * (
        STAY_ANCHOR_Z - LID_BOTTOM_Z
    )
    stay_dy = anchor_local_y - STAY_PAD_Y
    stay_dz = anchor_local_z - STAY_PAD_Z
    stay_length = math.hypot(stay_dy, stay_dz)
    stay_roll = -math.atan2(stay_dy, stay_dz)
    for side_name, sign in ((("left_drop_stay"), -1.0), (("right_drop_stay"), 1.0)):
        lid.visual(
            Cylinder(radius=0.0032, length=stay_length),
            origin=Origin(
                xyz=(sign * STAY_X, STAY_PAD_Y + stay_dy / 2.0, STAY_PAD_Z + stay_dz / 2.0),
                rpy=(stay_roll, 0.0, 0.0),
            ),
            material=brass,
            name=side_name,
        )
    lid.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(
            xyz=(0.0, -0.002, LID_HEIGHT * 0.53),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass,
        name="lid_escutcheon",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_WIDTH, LID_THICKNESS, LID_HEIGHT)),
        mass=7.0,
        origin=Origin(xyz=(0.0, LID_THICKNESS / 2.0, LID_HEIGHT / 2.0)),
    )

    def _build_drawer(name: str) -> tuple[object, float]:
        drawer = model.part(name)
        drawer.visual(
            Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FRONT_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    DRAWER_FRONT_THICKNESS / 2.0,
                    DRAWER_FRONT_HEIGHT / 2.0,
                )
            ),
            material=walnut,
            name="drawer_front",
        )
        drawer.visual(
            Box((DRAWER_BOX_WIDTH, DRAWER_BOX_DEPTH, DRAWER_BOTTOM_THICKNESS)),
            origin=Origin(
                xyz=(
                    0.0,
                    DRAWER_FRONT_THICKNESS + DRAWER_BOX_DEPTH / 2.0,
                    DRAWER_BOTTOM_THICKNESS / 2.0,
                )
            ),
            material=maple,
            name="drawer_bottom",
        )
        drawer.visual(
            Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
            origin=Origin(
                xyz=(
                    -(DRAWER_BOX_WIDTH / 2.0) + DRAWER_SIDE_THICKNESS / 2.0,
                    DRAWER_FRONT_THICKNESS + DRAWER_BOX_DEPTH / 2.0,
                    DRAWER_BOX_HEIGHT / 2.0,
                )
            ),
            material=maple,
            name="left_side",
        )
        drawer.visual(
            Box((DRAWER_SIDE_THICKNESS, DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
            origin=Origin(
                xyz=(
                    (DRAWER_BOX_WIDTH / 2.0) - DRAWER_SIDE_THICKNESS / 2.0,
                    DRAWER_FRONT_THICKNESS + DRAWER_BOX_DEPTH / 2.0,
                    DRAWER_BOX_HEIGHT / 2.0,
                )
            ),
            material=maple,
            name="right_side",
        )
        drawer.visual(
            Box((DRAWER_BOX_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS, DRAWER_SIDE_THICKNESS, DRAWER_BOX_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    DRAWER_FRONT_THICKNESS + DRAWER_BOX_DEPTH - DRAWER_SIDE_THICKNESS / 2.0,
                    DRAWER_BOX_HEIGHT / 2.0,
                )
            ),
            material=maple,
            name="back",
        )
        runner_y = GUIDE_RAIL_START_Y + (GUIDE_RAIL_LENGTH - 0.002) / 2.0
        for side, sign in (("left", -1.0), ("right", 1.0)):
            drawer.visual(
                Box((RUNNER_THICKNESS, GUIDE_RAIL_LENGTH - 0.002, GUIDE_RAIL_HEIGHT - 0.002)),
                origin=Origin(
                    xyz=(
                        sign * (DRAWER_BOX_WIDTH / 2.0 + RUNNER_THICKNESS / 2.0),
                        runner_y,
                        0.12,
                    )
                ),
                material=walnut_dark,
                name=f"{side}_runner",
            )
        for x_pos in (-0.18, 0.18):
            drawer.visual(
                Cylinder(radius=0.005, length=0.014),
                origin=Origin(
                    xyz=(x_pos, -0.007, DRAWER_FRONT_HEIGHT / 2.0),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=brass,
                name=f"pull_post_{'left' if x_pos < 0 else 'right'}",
            )
            drawer.visual(
                Sphere(radius=0.011),
                origin=Origin(xyz=(x_pos, -0.017, DRAWER_FRONT_HEIGHT / 2.0)),
                material=brass,
                name=f"pull_knob_{'left' if x_pos < 0 else 'right'}",
            )
        drawer.inertial = Inertial.from_geometry(
            Box((DRAWER_BOX_WIDTH, DRAWER_FRONT_THICKNESS + DRAWER_BOX_DEPTH, DRAWER_BOX_HEIGHT)),
            mass=4.8,
            origin=Origin(
                xyz=(
                    0.0,
                    (DRAWER_FRONT_THICKNESS + DRAWER_BOX_DEPTH) / 2.0,
                    DRAWER_BOX_HEIGHT / 2.0,
                )
            ),
        )
        return drawer

    upper_drawer = _build_drawer("upper_drawer")
    lower_drawer = _build_drawer("lower_drawer")

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, LID_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=OPEN_LID_ANGLE,
        ),
    )
    model.articulation(
        "cabinet_to_upper_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_drawer,
        origin=Origin(xyz=(0.0, 0.0, UPPER_DRAWER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.5,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "cabinet_to_lower_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_drawer,
        origin=Origin(xyz=(0.0, 0.0, LOWER_DRAWER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.5,
            lower=0.0,
            upper=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SAFE_ASSET_ROOT)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid_panel")
    upper_drawer = object_model.get_part("upper_drawer")
    lower_drawer = object_model.get_part("lower_drawer")

    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    upper_slide = object_model.get_articulation("cabinet_to_upper_drawer")
    lower_slide = object_model.get_articulation("cabinet_to_lower_drawer")

    lid_face = lid.get_visual("lid_face")
    writing_surface = lid.get_visual("writing_surface")
    left_pin = lid.get_visual("left_pin")
    right_pin = lid.get_visual("right_pin")
    left_stay = lid.get_visual("left_drop_stay")
    right_stay = lid.get_visual("right_drop_stay")
    left_bracket = cabinet.get_visual("left_hinge_bracket")
    right_bracket = cabinet.get_visual("right_hinge_bracket")
    left_anchor = cabinet.get_visual("left_stay_anchor")
    right_anchor = cabinet.get_visual("right_stay_anchor")
    desk_floor = cabinet.get_visual("desk_floor")
    hinge_rail = cabinet.get_visual("hinge_rail")
    top_rail = cabinet.get_visual("top_rail")
    frame_left = cabinet.get_visual("front_stile_left")
    frame_ref = cabinet.get_visual("drawer_mid_rail")
    upper_front = upper_drawer.get_visual("drawer_front")
    lower_front = lower_drawer.get_visual("drawer_front")
    upper_left_runner = upper_drawer.get_visual("left_runner")
    upper_right_runner = upper_drawer.get_visual("right_runner")
    lower_left_runner = lower_drawer.get_visual("left_runner")
    lower_right_runner = lower_drawer.get_visual("right_runner")
    upper_left_rail = cabinet.get_visual("upper_left_rail")
    upper_right_rail = cabinet.get_visual("upper_right_rail")
    lower_left_rail = cabinet.get_visual("lower_left_rail")
    lower_right_rail = cabinet.get_visual("lower_right_rail")

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

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.allow_overlap(
        lid,
        cabinet,
        elem_a=left_pin,
        elem_b=left_bracket,
        reason="left hinge pin intentionally nests into the brass side bracket",
    )
    ctx.allow_overlap(
        lid,
        cabinet,
        elem_a=right_pin,
        elem_b=right_bracket,
        reason="right hinge pin intentionally nests into the brass side bracket",
    )
    ctx.expect_contact(lid, cabinet, elem_a=left_pin, elem_b=left_bracket)
    ctx.expect_contact(lid, cabinet, elem_a=right_pin, elem_b=right_bracket)
    ctx.expect_overlap(lid, cabinet, axes="x", min_overlap=0.74, elem_a=lid_face, elem_b=top_rail)
    ctx.expect_overlap(lid, cabinet, axes="z", min_overlap=0.45, elem_a=lid_face, elem_b=frame_left)
    ctx.expect_gap(
        lid,
        upper_drawer,
        axis="z",
        min_gap=0.09,
        positive_elem=lid_face,
        negative_elem=upper_front,
    )
    ctx.expect_gap(
        upper_drawer,
        lower_drawer,
        axis="z",
        min_gap=0.05,
        positive_elem=upper_front,
        negative_elem=lower_front,
    )

    ctx.expect_contact(upper_drawer, cabinet, elem_a=upper_left_runner, elem_b=upper_left_rail)
    ctx.expect_contact(upper_drawer, cabinet, elem_a=upper_right_runner, elem_b=upper_right_rail)
    ctx.expect_contact(lower_drawer, cabinet, elem_a=lower_left_runner, elem_b=lower_left_rail)
    ctx.expect_contact(lower_drawer, cabinet, elem_a=lower_right_runner, elem_b=lower_right_rail)

    with ctx.pose({lid_hinge: OPEN_LID_ANGLE}):
        ctx.expect_contact(lid, cabinet, elem_a=left_stay, elem_b=left_anchor)
        ctx.expect_contact(lid, cabinet, elem_a=right_stay, elem_b=right_anchor)
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem=writing_surface,
            negative_elem=desk_floor,
        )
        ctx.expect_overlap(lid, cabinet, axes="x", min_overlap=0.60, elem_a=writing_surface, elem_b=desk_floor)
        ctx.expect_gap(
            cabinet,
            lid,
            axis="y",
            min_gap=0.05,
            positive_elem=hinge_rail,
            negative_elem=writing_surface,
        )
        ctx.expect_gap(
            lid,
            upper_drawer,
            axis="z",
            min_gap=0.09,
            positive_elem=writing_surface,
            negative_elem=upper_front,
        )

    with ctx.pose({upper_slide: 0.18}):
        ctx.expect_gap(
            cabinet,
            upper_drawer,
            axis="y",
            min_gap=0.14,
            positive_elem=frame_ref,
            negative_elem=upper_front,
        )
        ctx.expect_contact(upper_drawer, cabinet, elem_a=upper_left_runner, elem_b=upper_left_rail)
        ctx.expect_contact(upper_drawer, cabinet, elem_a=upper_right_runner, elem_b=upper_right_rail)

    with ctx.pose({lower_slide: 0.20}):
        ctx.expect_gap(
            cabinet,
            lower_drawer,
            axis="y",
            min_gap=0.16,
            positive_elem=frame_ref,
            negative_elem=lower_front,
        )
        ctx.expect_contact(lower_drawer, cabinet, elem_a=lower_left_runner, elem_b=lower_left_rail)
        ctx.expect_contact(lower_drawer, cabinet, elem_a=lower_right_runner, elem_b=lower_right_rail)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
