from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_DEPTH = 0.48
BODY_WIDTH = 0.43
BODY_WALL = 0.025
BODY_HEIGHT = 0.80
TOP_DECK_THICKNESS = 0.025
TOP_HEIGHT = BODY_HEIGHT + TOP_DECK_THICKNESS

CONSOLE_DEPTH = 0.09
CONSOLE_HEIGHT = 0.105
CONSOLE_CAP_THICKNESS = 0.015

OPENING_BACK_X = -0.13
OPENING_FRONT_X = 0.21
OPENING_DEPTH = OPENING_FRONT_X - OPENING_BACK_X
OPENING_HALF_WIDTH = 0.16

HINGE_AXIS_X = OPENING_BACK_X
HINGE_RADIUS = 0.008
LID_PANEL_DROP = 0.018
HINGE_Z = TOP_HEIGHT + HINGE_RADIUS + LID_PANEL_DROP
HINGE_SUPPORT_Y = 0.148
HINGE_PIN_LENGTH = 0.03

BASKET_CENTER_X = 0.04
BASKET_JOINT_Z = 0.35
BASKET_RADIUS = 0.145
BASKET_HEIGHT = 0.36

LID_DEPTH = 0.345
LID_OUTER_WIDTH = 0.348
LID_GLAZING_WIDTH = 0.318
LID_GLAZING_DEPTH = 0.327
LID_THICKNESS = 0.016

DIAL_Z = BODY_HEIGHT + 0.092
DIAL_Y_OFFSET = 0.082
DIAL_RADIUS = 0.031
DIAL_LENGTH = 0.022
DIAL_CAP_LENGTH = 0.008


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="apartment_top_load_washer")

    enamel = model.material("enamel_white", rgba=(0.93, 0.94, 0.95, 1.0))
    shadow = model.material("charcoal_trim", rgba=(0.18, 0.19, 0.21, 1.0))
    smoked = model.material("smoked_glass", rgba=(0.20, 0.24, 0.28, 0.45))
    steel = model.material("basket_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    silver = model.material("dial_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    agitator_gray = model.material("agitator_gray", rgba=(0.82, 0.84, 0.87, 1.0))

    cabinet = model.part("cabinet")

    wall_height = BODY_HEIGHT - 0.03
    wall_z = 0.03 + wall_height / 2.0
    half_depth = BODY_DEPTH / 2.0
    half_width = BODY_WIDTH / 2.0

    cabinet.visual(
        Box((BODY_DEPTH, BODY_WALL, wall_height)),
        origin=Origin(xyz=(0.0, -(half_width - BODY_WALL / 2.0), wall_z)),
        material=enamel,
        name="left_wall",
    )
    cabinet.visual(
        Box((BODY_DEPTH, BODY_WALL, wall_height)),
        origin=Origin(xyz=(0.0, half_width - BODY_WALL / 2.0, wall_z)),
        material=enamel,
        name="right_wall",
    )
    cabinet.visual(
        Box((BODY_WALL, BODY_WIDTH - 2.0 * BODY_WALL, wall_height)),
        origin=Origin(xyz=(half_depth - BODY_WALL / 2.0, 0.0, wall_z)),
        material=enamel,
        name="front_wall",
    )
    cabinet.visual(
        Box((BODY_WALL, BODY_WIDTH - 2.0 * BODY_WALL, wall_height)),
        origin=Origin(xyz=(-(half_depth - BODY_WALL / 2.0), 0.0, wall_z)),
        material=enamel,
        name="back_wall",
    )
    cabinet.visual(
        Box((BODY_DEPTH, BODY_WIDTH, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=shadow,
        name="base_pan",
    )

    cabinet.visual(
        Box((half_depth + OPENING_BACK_X, BODY_WIDTH, TOP_DECK_THICKNESS)),
        origin=Origin(
            xyz=(
                (OPENING_BACK_X - half_depth) / 2.0,
                0.0,
                BODY_HEIGHT + TOP_DECK_THICKNESS / 2.0,
            )
        ),
        material=enamel,
        name="rear_top_deck",
    )
    cabinet.visual(
        Box((half_depth - OPENING_FRONT_X, BODY_WIDTH, TOP_DECK_THICKNESS)),
        origin=Origin(
            xyz=(
                (OPENING_FRONT_X + half_depth) / 2.0,
                0.0,
                BODY_HEIGHT + TOP_DECK_THICKNESS / 2.0,
            )
        ),
        material=enamel,
        name="front_rim",
    )
    cabinet.visual(
        Box((OPENING_DEPTH, half_width - OPENING_HALF_WIDTH, TOP_DECK_THICKNESS)),
        origin=Origin(
            xyz=(
                (OPENING_FRONT_X + OPENING_BACK_X) / 2.0,
                -(OPENING_HALF_WIDTH + half_width) / 2.0,
                BODY_HEIGHT + TOP_DECK_THICKNESS / 2.0,
            )
        ),
        material=enamel,
        name="left_rim",
    )
    cabinet.visual(
        Box((OPENING_DEPTH, half_width - OPENING_HALF_WIDTH, TOP_DECK_THICKNESS)),
        origin=Origin(
            xyz=(
                (OPENING_FRONT_X + OPENING_BACK_X) / 2.0,
                (OPENING_HALF_WIDTH + half_width) / 2.0,
                BODY_HEIGHT + TOP_DECK_THICKNESS / 2.0,
            )
        ),
        material=enamel,
        name="right_rim",
    )
    cabinet.visual(
        Box((0.02, BODY_WIDTH, TOP_DECK_THICKNESS)),
        origin=Origin(
            xyz=(OPENING_BACK_X + 0.01, 0.0, BODY_HEIGHT + TOP_DECK_THICKNESS / 2.0)
        ),
        material=enamel,
        name="rear_rim",
    )

    cabinet.visual(
        Box((CONSOLE_DEPTH, BODY_WIDTH, CONSOLE_HEIGHT)),
        origin=Origin(
            xyz=(
                -half_depth + CONSOLE_DEPTH / 2.0,
                0.0,
                BODY_HEIGHT + CONSOLE_HEIGHT / 2.0,
            )
        ),
        material=enamel,
        name="console_body",
    )
    cabinet.visual(
        Box((CONSOLE_DEPTH, BODY_WIDTH, CONSOLE_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                -half_depth + CONSOLE_DEPTH / 2.0,
                0.0,
                BODY_HEIGHT + CONSOLE_HEIGHT + CONSOLE_CAP_THICKNESS / 2.0,
            )
        ),
        material=shadow,
        name="console_cap",
    )

    cabinet.visual(
        Cylinder(radius=0.05, length=0.07),
        origin=Origin(xyz=(BASKET_CENTER_X, 0.0, BASKET_JOINT_Z - 0.035)),
        material=shadow,
        name="drive_pedestal",
    )
    cabinet.visual(
        Cylinder(radius=0.035, length=0.25),
        origin=Origin(xyz=(BASKET_CENTER_X, 0.0, 0.155)),
        material=shadow,
        name="pedestal_column",
    )
    basket = model.part("basket")
    basket_shell = mesh_from_geometry(
        CylinderGeometry(BASKET_RADIUS, BASKET_HEIGHT, radial_segments=56, closed=False),
        "basket_shell",
    )
    basket.visual(
        basket_shell,
        origin=Origin(xyz=(0.0, 0.0, BASKET_HEIGHT / 2.0)),
        material=steel,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=BASKET_RADIUS, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="basket_floor",
    )
    basket.visual(
        Cylinder(radius=0.036, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=shadow,
        name="spindle_base",
    )
    basket.visual(
        Cylinder(radius=0.055, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0425)),
        material=agitator_gray,
        name="agitator_skirt",
    )
    basket.visual(
        Cylinder(radius=0.038, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=agitator_gray,
        name="agitator_column",
    )
    basket.visual(
        Sphere(radius=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=agitator_gray,
        name="agitator_cap",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_GLAZING_DEPTH, LID_GLAZING_WIDTH, LID_THICKNESS)),
        origin=Origin(xyz=(0.018 + LID_GLAZING_DEPTH / 2.0, 0.0, -LID_PANEL_DROP)),
        material=smoked,
        name="glass_panel",
    )
    lid.visual(
        Box((LID_GLAZING_DEPTH, 0.015, LID_THICKNESS)),
        origin=Origin(
            xyz=(
                0.018 + LID_GLAZING_DEPTH / 2.0,
                -(LID_OUTER_WIDTH - 0.015) / 2.0,
                -LID_PANEL_DROP,
            )
        ),
        material=shadow,
        name="left_frame",
    )
    lid.visual(
        Box((LID_GLAZING_DEPTH, 0.015, LID_THICKNESS)),
        origin=Origin(
            xyz=(
                0.018 + LID_GLAZING_DEPTH / 2.0,
                (LID_OUTER_WIDTH - 0.015) / 2.0,
                -LID_PANEL_DROP,
            )
        ),
        material=shadow,
        name="right_frame",
    )
    lid.visual(
        Box((0.014, LID_OUTER_WIDTH, LID_THICKNESS)),
        origin=Origin(xyz=(LID_DEPTH - 0.007, 0.0, -LID_PANEL_DROP)),
        material=shadow,
        name="front_frame",
    )
    lid.visual(
        Box((0.03, LID_OUTER_WIDTH, 0.028)),
        origin=Origin(xyz=(0.015, 0.0, -0.012)),
        material=shadow,
        name="rear_rail",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_PIN_LENGTH),
        origin=Origin(xyz=(0.0, -HINGE_SUPPORT_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shadow,
        name="left_hinge_pin",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=HINGE_PIN_LENGTH),
        origin=Origin(xyz=(0.0, HINGE_SUPPORT_Y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=shadow,
        name="right_hinge_pin",
    )

    left_dial = model.part("left_dial")
    left_dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_LENGTH),
        origin=Origin(xyz=(DIAL_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=silver,
        name="dial_body",
    )
    left_dial.visual(
        Cylinder(radius=0.031, length=DIAL_CAP_LENGTH),
        origin=Origin(
            xyz=(DIAL_LENGTH + DIAL_CAP_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=shadow,
        name="dial_cap",
    )

    right_dial = model.part("right_dial")
    right_dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_LENGTH),
        origin=Origin(xyz=(DIAL_LENGTH / 2.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=silver,
        name="dial_body",
    )
    right_dial.visual(
        Cylinder(radius=0.031, length=DIAL_CAP_LENGTH),
        origin=Origin(
            xyz=(DIAL_LENGTH + DIAL_CAP_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=shadow,
        name="dial_cap",
    )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "cabinet_to_basket",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(BASKET_CENTER_X, 0.0, BASKET_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=8.0),
    )
    model.articulation(
        "cabinet_to_left_dial",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=left_dial,
        origin=Origin(xyz=(-half_depth + CONSOLE_DEPTH, -DIAL_Y_OFFSET, DIAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "cabinet_to_right_dial",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=right_dial,
        origin=Origin(xyz=(-half_depth + CONSOLE_DEPTH, DIAL_Y_OFFSET, DIAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=-2.4,
            upper=2.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    left_dial = object_model.get_part("left_dial")
    right_dial = object_model.get_part("right_dial")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    basket_spin = object_model.get_articulation("cabinet_to_basket")
    left_dial_joint = object_model.get_articulation("cabinet_to_left_dial")
    right_dial_joint = object_model.get_articulation("cabinet_to_right_dial")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
        "washer part tree present",
        all(part is not None for part in (cabinet, lid, basket, left_dial, right_dial)),
        details="Expected cabinet, lid, basket, left_dial, and right_dial parts.",
    )
    ctx.check(
        "lid hinge is horizontal rear revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(lid_hinge.axis[0]) < 1e-9
        and abs(lid_hinge.axis[1] + 1.0) < 1e-9
        and abs(lid_hinge.axis[2]) < 1e-9,
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}",
    )
    ctx.check(
        "basket spin is vertical continuous",
        basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and abs(basket_spin.axis[0]) < 1e-9
        and abs(basket_spin.axis[1]) < 1e-9
        and abs(basket_spin.axis[2] - 1.0) < 1e-9,
        details=f"type={basket_spin.articulation_type}, axis={basket_spin.axis}",
    )
    ctx.check(
        "control dials spin on console-normal axes",
        left_dial_joint.articulation_type == ArticulationType.REVOLUTE
        and right_dial_joint.articulation_type == ArticulationType.REVOLUTE
        and left_dial_joint.axis == (1.0, 0.0, 0.0)
        and right_dial_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"left axis={left_dial_joint.axis}, right axis={right_dial_joint.axis}, "
            f"left type={left_dial_joint.articulation_type}, right type={right_dial_joint.articulation_type}"
        ),
    )

    ctx.expect_contact(
        basket,
        cabinet,
        elem_a="spindle_base",
        elem_b="drive_pedestal",
        name="basket spindle seats on drive pedestal",
    )
    ctx.expect_contact(
        lid,
        cabinet,
        elem_a="rear_rail",
        elem_b="rear_rim",
        name="lid rear rail seats on the rear rim",
    )
    ctx.expect_contact(
        left_dial,
        cabinet,
        elem_a="dial_body",
        elem_b="console_body",
        name="left dial mounts to console face",
    )
    ctx.expect_contact(
        right_dial,
        cabinet,
        elem_a="dial_body",
        elem_b="console_body",
        name="right dial mounts to console face",
    )
    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="glass_panel",
        negative_elem="front_rim",
        min_gap=0.0,
        max_gap=0.005,
        name="closed lid sits just above the top deck",
    )
    ctx.expect_origin_distance(
        right_dial,
        left_dial,
        axes="y",
        min_dist=0.16,
        max_dist=0.22,
        name="dials are separated across the console",
    )

    closed_front_frame_aabb = ctx.part_element_world_aabb(lid, elem="front_frame")
    with ctx.pose({lid_hinge: 1.1}):
        open_front_frame_aabb = ctx.part_element_world_aabb(lid, elem="front_frame")
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="front_frame",
            negative_elem="console_cap",
            min_gap=0.01,
            name="opened lid clears the rear console",
        )
    ctx.check(
        "lid swings upward when opened",
        closed_front_frame_aabb is not None
        and open_front_frame_aabb is not None
        and open_front_frame_aabb[0][2] > closed_front_frame_aabb[1][2] + 0.20
        and open_front_frame_aabb[1][0] < closed_front_frame_aabb[1][0] - 0.15,
        details=f"closed={closed_front_frame_aabb}, open={open_front_frame_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
