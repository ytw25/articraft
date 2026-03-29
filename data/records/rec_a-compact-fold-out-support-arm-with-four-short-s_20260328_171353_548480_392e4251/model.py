from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_PLATE_THICKNESS = 0.006
BASE_PLATE_WIDTH = 0.068
BASE_PLATE_HEIGHT = 0.112
BASE_PIVOT_X = 0.016
BASE_PIVOT_Z = 0.024

LINK_DEPTH_X = 0.012
LINK_THICKNESS_Y = 0.004
LINK_HEAD_LENGTH = 0.014
LINK_LENGTHS = (0.078, 0.072, 0.066, 0.060)

JOINT_Y_LEVELS = (0.006, 0.002, -0.002, -0.006)

BRACKET_TAB_DEPTH = 0.018
BRACKET_HEIGHT = 0.026
BRACKET_SHELF_LENGTH = 0.046
BRACKET_SHELF_THICKNESS = 0.006
BRACKET_LIP_HEIGHT = 0.010

JOINT_1_PACKED_RPY_Y = 0.26
JOINT_2_PACKED_RPY_Y = math.pi - 0.66
JOINT_3_PACKED_RPY_Y = math.pi - 0.58
JOINT_4_PACKED_RPY_Y = math.pi - 0.50

DEPLOYED_POSE = {
    "base_to_link_1": math.pi * 0.5 - JOINT_1_PACKED_RPY_Y,
    "link_1_to_link_2": -JOINT_2_PACKED_RPY_Y,
    "link_2_to_link_3": -JOINT_3_PACKED_RPY_Y,
    "link_3_to_link_4": -JOINT_4_PACKED_RPY_Y,
}


def make_base_plate() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        BASE_PLATE_THICKNESS,
        BASE_PLATE_WIDTH,
        BASE_PLATE_HEIGHT,
        centered=(False, True, False),
    )
    plate = plate.translate((-BASE_PLATE_THICKNESS, 0.0, 0.0))

    hinge_pedestal = cq.Workplane("XY").box(
        BASE_PIVOT_X,
        0.014,
        0.030,
        centered=(False, True, False),
    )
    hinge_pedestal = hinge_pedestal.translate(
        (0.0, JOINT_Y_LEVELS[0], BASE_PIVOT_Z - 0.015)
    )

    upper_rib = cq.Workplane("XY").box(
        BASE_PLATE_THICKNESS,
        0.028,
        0.024,
        centered=(False, True, False),
    )
    upper_rib = upper_rib.translate((-BASE_PLATE_THICKNESS, 0.0, 0.070))

    mounting_holes = (
        cq.Workplane(
            "YZ",
            origin=(-BASE_PLATE_THICKNESS * 0.5, 0.0, BASE_PLATE_HEIGHT * 0.5),
        )
        .pushPoints(
            [
                (-0.022, -0.034),
                (0.022, -0.034),
                (-0.022, 0.034),
                (0.022, 0.034),
            ]
        )
        .circle(0.0032)
        .extrude(BASE_PLATE_THICKNESS * 2.2, both=True)
    )

    service_relief = cq.Workplane("XY").box(
        BASE_PLATE_THICKNESS * 1.6,
        0.018,
        0.018,
        centered=(False, True, False),
    )
    service_relief = service_relief.translate(
        (-BASE_PLATE_THICKNESS * 0.8, 0.0, 0.050)
    )

    return plate.union(hinge_pedestal).union(upper_rib).cut(mounting_holes).cut(service_relief)


def make_link(length: float, *, distal_y: float) -> cq.Workplane:
    mid_y = distal_y * 0.5

    proximal_head = cq.Workplane("XY").box(
        LINK_DEPTH_X,
        LINK_THICKNESS_Y,
        LINK_HEAD_LENGTH,
        centered=(False, True, False),
    )
    proximal_head = proximal_head.translate((0.0, 0.0, 0.0))

    distal_head = cq.Workplane("XY").box(
        LINK_DEPTH_X,
        LINK_THICKNESS_Y,
        LINK_HEAD_LENGTH,
        centered=(False, True, False),
    )
    distal_head = distal_head.translate((0.0, distal_y, length - LINK_HEAD_LENGTH))

    beam = cq.Workplane("XY").box(
        LINK_DEPTH_X,
        LINK_THICKNESS_Y,
        length - LINK_HEAD_LENGTH,
        centered=(False, True, False),
    )
    beam = beam.translate((0.0, mid_y, LINK_HEAD_LENGTH * 0.5))

    link = proximal_head.union(beam).union(distal_head)

    window = cq.Workplane("XY").box(
        LINK_DEPTH_X * 0.48,
        LINK_THICKNESS_Y * 1.8,
        max(length - 0.034, 0.018),
        centered=(False, True, False),
    )
    window = window.translate((LINK_DEPTH_X * 0.26, mid_y, 0.017))

    pivot_holes = (
        cq.Workplane("XZ", origin=(LINK_DEPTH_X * 0.5, 0.0, 0.0))
        .pushPoints([(0.0, 0.0065), (distal_y, length - 0.0065)])
        .circle(0.0022)
        .extrude(LINK_THICKNESS_Y * 1.6, both=True)
    )

    nose_relief = cq.Workplane("XY").box(
        LINK_DEPTH_X * 0.30,
        LINK_THICKNESS_Y * 1.6,
        0.008,
        centered=(False, True, False),
    )
    nose_relief = nose_relief.translate((LINK_DEPTH_X * 0.60, mid_y, length * 0.48))

    return link.cut(window).cut(pivot_holes).cut(nose_relief)


def make_platform_bracket() -> cq.Workplane:
    tab = cq.Workplane("XY").box(
        BRACKET_TAB_DEPTH,
        LINK_THICKNESS_Y,
        BRACKET_HEIGHT,
        centered=(False, True, False),
    )
    tab = tab.translate((0.0, 0.0, -0.006))

    shelf = cq.Workplane("XY").box(
        BRACKET_SHELF_LENGTH,
        LINK_THICKNESS_Y,
        BRACKET_SHELF_THICKNESS,
        centered=(False, True, False),
    )
    shelf = shelf.translate((0.0, 0.0, 0.010))

    lip = cq.Workplane("XY").box(
        0.006,
        LINK_THICKNESS_Y,
        BRACKET_LIP_HEIGHT,
        centered=(False, True, False),
    )
    lip = lip.translate((BRACKET_SHELF_LENGTH - 0.006, 0.0, 0.010))

    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.0, -0.004),
                (0.0, 0.010),
                (BRACKET_SHELF_LENGTH * 0.42, -0.004),
            ]
        )
        .close()
        .extrude(LINK_THICKNESS_Y)
        .translate((0.006, -LINK_THICKNESS_Y * 0.5, 0.0))
    )

    shelf_slot = cq.Workplane("XY").box(
        BRACKET_SHELF_LENGTH * 0.32,
        LINK_THICKNESS_Y * 1.8,
        BRACKET_SHELF_THICKNESS * 0.5,
        centered=(False, True, False),
    )
    shelf_slot = shelf_slot.translate((0.010, 0.0, 0.010))

    return tab.union(shelf).union(lip).union(gusset).cut(shelf_slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_support_arm")

    model.material("base_plate_finish", color=(0.20, 0.21, 0.23, 1.0))
    model.material("link_finish", color=(0.67, 0.69, 0.72, 1.0))
    model.material("bracket_finish", color=(0.31, 0.34, 0.37, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(make_base_plate(), "base_plate"),
        material="base_plate_finish",
        name="base_plate_shell",
    )

    link_1 = model.part("link_1")
    link_1.visual(
        mesh_from_cadquery(
            make_link(
                LINK_LENGTHS[0],
                distal_y=JOINT_Y_LEVELS[1] - JOINT_Y_LEVELS[0],
            ),
            "link_1",
        ),
        material="link_finish",
        name="link_1_shell",
    )

    link_2 = model.part("link_2")
    link_2.visual(
        mesh_from_cadquery(
            make_link(
                LINK_LENGTHS[1],
                distal_y=JOINT_Y_LEVELS[2] - JOINT_Y_LEVELS[1],
            ),
            "link_2",
        ),
        material="link_finish",
        name="link_2_shell",
    )

    link_3 = model.part("link_3")
    link_3.visual(
        mesh_from_cadquery(
            make_link(
                LINK_LENGTHS[2],
                distal_y=JOINT_Y_LEVELS[3] - JOINT_Y_LEVELS[2],
            ),
            "link_3",
        ),
        material="link_finish",
        name="link_3_shell",
    )

    link_4 = model.part("link_4")
    link_4.visual(
        mesh_from_cadquery(make_link(LINK_LENGTHS[3], distal_y=0.0), "link_4"),
        material="link_finish",
        name="link_4_shell",
    )

    platform_bracket = model.part("platform_bracket")
    platform_bracket.visual(
        mesh_from_cadquery(make_platform_bracket(), "platform_bracket"),
        material="bracket_finish",
        name="platform_bracket_shell",
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base_plate,
        child=link_1,
        origin=Origin(
            xyz=(BASE_PIVOT_X, JOINT_Y_LEVELS[0], BASE_PIVOT_Z),
            rpy=(0.0, JOINT_1_PACKED_RPY_Y, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.8,
            lower=-0.35,
            upper=1.20,
        ),
    )

    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(
            xyz=(0.0, JOINT_Y_LEVELS[1] - JOINT_Y_LEVELS[0], LINK_LENGTHS[0]),
            rpy=(0.0, JOINT_2_PACKED_RPY_Y, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=-2.55,
            upper=0.35,
        ),
    )

    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(
            xyz=(0.0, JOINT_Y_LEVELS[2] - JOINT_Y_LEVELS[1], LINK_LENGTHS[1]),
            rpy=(0.0, JOINT_3_PACKED_RPY_Y, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.2,
            lower=-2.70,
            upper=0.35,
        ),
    )

    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=link_4,
        origin=Origin(
            xyz=(0.0, JOINT_Y_LEVELS[3] - JOINT_Y_LEVELS[2], LINK_LENGTHS[2]),
            rpy=(0.0, JOINT_4_PACKED_RPY_Y, 0.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.4,
            lower=-2.80,
            upper=0.35,
        ),
    )

    model.articulation(
        "link_4_to_platform_bracket",
        ArticulationType.FIXED,
        parent=link_4,
        child=platform_bracket,
        origin=Origin(xyz=(0.0, 0.0, LINK_LENGTHS[3])),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_plate = object_model.get_part("base_plate")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")
    platform_bracket = object_model.get_part("platform_bracket")

    base_to_link_1 = object_model.get_articulation("base_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")
    link_3_to_link_4 = object_model.get_articulation("link_3_to_link_4")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    for part in (
        base_plate,
        link_1,
        link_2,
        link_3,
        link_4,
        platform_bracket,
    ):
        ctx.check(
            f"{part.name}_present",
            len(part.visuals) > 0,
            f"{part.name} is missing visual geometry",
        )

    for joint in (
        base_to_link_1,
        link_1_to_link_2,
        link_2_to_link_3,
        link_3_to_link_4,
    ):
        ctx.check(
            f"{joint.name}_uses_parallel_y_axis",
            tuple(joint.axis) == (0.0, 1.0, 0.0),
            f"{joint.name} axis was {joint.axis}",
        )

    ctx.expect_contact(base_plate, link_1, name="base_plate_mounts_first_link")
    ctx.expect_contact(link_1, link_2, name="first_joint_surfaces_touch")
    ctx.expect_contact(link_2, link_3, name="second_joint_surfaces_touch")
    ctx.expect_contact(link_3, link_4, name="third_joint_surfaces_touch")
    ctx.expect_contact(link_4, platform_bracket, name="bracket_attaches_to_fourth_link")

    ctx.expect_gap(
        platform_bracket,
        base_plate,
        axis="x",
        max_gap=0.090,
        name="packed_pose_keeps_platform_close_to_base",
    )

    with ctx.pose(DEPLOYED_POSE):
        ctx.expect_gap(
            platform_bracket,
            base_plate,
            axis="x",
            min_gap=0.170,
            name="deployed_pose_extends_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
