from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.72
BODY_WIDTH = 0.44
BODY_HEIGHT = 0.32
WOOD_THICKNESS = 0.02
LID_ARCH_HEIGHT = 0.17
LID_SKIRT_HEIGHT = 0.05
HINGE_CLEARANCE = 0.0


def _body_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(
        BODY_LENGTH,
        BODY_WIDTH,
        BODY_HEIGHT,
        centered=(True, True, False),
    )
    inner = cq.Workplane("XY").box(
        BODY_LENGTH - 2.0 * WOOD_THICKNESS,
        BODY_WIDTH - 2.0 * WOOD_THICKNESS,
        BODY_HEIGHT - WOOD_THICKNESS,
        centered=(True, True, False),
    ).translate((0.0, 0.0, WOOD_THICKNESS))
    return outer.cut(inner)


def _lid_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0, LID_SKIRT_HEIGHT)
        .threePointArc((BODY_LENGTH * 0.5, LID_ARCH_HEIGHT), (BODY_LENGTH, LID_SKIRT_HEIGHT))
        .lineTo(BODY_LENGTH, 0.0)
        .close()
        .extrude(BODY_WIDTH * 0.5, both=True)
    )
    return outer.faces("<Z").shell(-WOOD_THICKNESS)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wooden_treasure_chest")

    oak = model.material("oak", rgba=(0.56, 0.37, 0.19, 1.0))
    brass = model.material("brass", rgba=(0.76, 0.64, 0.26, 1.0))

    chest = model.part("chest")
    chest.visual(
        mesh_from_cadquery(_body_shell(), "chest_body_shell"),
        material=oak,
        name="body_shell",
    )

    strap_thickness = 0.006
    strap_width = 0.05
    base_strap_height = 0.16
    plate_embed = 0.001
    front_face_x = BODY_LENGTH * 0.5 + strap_thickness * 0.5 - plate_embed
    for index, y in enumerate((-0.13, 0.13)):
        chest.visual(
            Box((strap_thickness, strap_width, base_strap_height)),
            origin=Origin(xyz=(front_face_x, y, BODY_HEIGHT - base_strap_height * 0.5)),
            material=brass,
            name=f"front_strap_{index}",
        )

    chest.visual(
        Box((strap_thickness, 0.07, 0.06)),
        origin=Origin(xyz=(front_face_x, 0.0, BODY_HEIGHT - 0.03)),
        material=brass,
        name="lock_plate",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "chest_lid_shell"),
        material=oak,
        name="lid_shell",
    )

    lid_front_x = BODY_LENGTH + strap_thickness * 0.5 - plate_embed
    lid_strap_height = 0.10
    for index, y in enumerate((-0.13, 0.13)):
        lid.visual(
            Box((strap_thickness, strap_width, lid_strap_height)),
            origin=Origin(xyz=(lid_front_x, y, lid_strap_height * 0.5)),
            material=brass,
            name=f"lid_strap_{index}",
        )

    lid.visual(
        Box((strap_thickness, 0.05, 0.06)),
        origin=Origin(xyz=(lid_front_x, 0.0, 0.03)),
        material=brass,
        name="lid_plate",
    )

    model.articulation(
        "chest_to_lid",
        ArticulationType.REVOLUTE,
        parent=chest,
        child=lid,
        origin=Origin(xyz=(-BODY_LENGTH * 0.5, 0.0, BODY_HEIGHT + HINGE_CLEARANCE)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chest = object_model.get_part("chest")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("chest_to_lid")

    ctx.expect_gap(
        lid,
        chest,
        axis="z",
        max_gap=0.01,
        max_penetration=1e-5,
        name="closed lid rests on the chest rim",
    )
    ctx.expect_overlap(
        lid,
        chest,
        axes="xy",
        min_overlap=0.20,
        name="closed lid covers the chest opening",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    hinge_limits = hinge.motion_limits
    upper_open = 1.0
    if hinge_limits is not None and hinge_limits.upper is not None:
        upper_open = hinge_limits.upper

    with ctx.pose({hinge: upper_open}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")

    opened_upward = (
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.14
    )
    ctx.check(
        "lid swings upward when opened",
        opened_upward,
        details=f"closed_aabb={closed_lid_aabb}, open_aabb={open_lid_aabb}",
    )

    front_edge_retracts = (
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][0] < closed_lid_aabb[1][0] - 0.18
    )
    ctx.check(
        "lid front edge rotates back over the hinge",
        front_edge_retracts,
        details=f"closed_aabb={closed_lid_aabb}, open_aabb={open_lid_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
