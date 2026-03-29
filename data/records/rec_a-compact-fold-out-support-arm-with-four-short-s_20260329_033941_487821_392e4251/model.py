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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


LINK_PITCH = 0.078
LINK_BARREL_RADIUS = 0.007
LINK_BARREL_LENGTH = 0.018
LINK_ROOT_LENGTH = 0.016
LINK_ROOT_WIDTH = 0.016
LINK_ROOT_HEIGHT = 0.016
LINK_BEAM_LENGTH = 0.044
LINK_BEAM_WIDTH = 0.018
LINK_BEAM_HEIGHT = 0.018
FORK_LENGTH = 0.020
FORK_CHEEK_THICKNESS = 0.006
FORK_CHEEK_CENTER_Y = 0.012
FORK_CHEEK_HEIGHT = 0.026
TIP_BLOCK_LENGTH = 0.010
TIP_BLOCK_WIDTH = 0.024
TIP_BLOCK_HEIGHT = 0.016
LINK_BEAM_OFFSET = 0.008

BASE_PLATE_THICKNESS = 0.008
BASE_PLATE_WIDTH = 0.090
BASE_PLATE_HEIGHT = 0.130
BASE_PLATE_CENTER_X = -0.020
BASE_SPACER_LENGTH = 0.010
BASE_SPACER_WIDTH = 0.028
BASE_SPACER_HEIGHT = 0.032
BASE_CHEEK_LENGTH = 0.020
BASE_CHEEK_HEIGHT = 0.030

BRACKET_TONGUE_LENGTH = 0.012
BRACKET_TONGUE_WIDTH = 0.012
BRACKET_TONGUE_HEIGHT = 0.010
BRACKET_RISER_LENGTH = 0.010
BRACKET_RISER_WIDTH = 0.012
BRACKET_RISER_HEIGHT = 0.028
BRACKET_CROSSHEAD_LENGTH = 0.012
BRACKET_CROSSHEAD_WIDTH = 0.030
BRACKET_CROSSHEAD_THICKNESS = 0.008
BRACKET_SHELF_LENGTH = 0.022
BRACKET_SHELF_WIDTH = 0.036
BRACKET_SHELF_THICKNESS = 0.006
BRACKET_LIP_THICKNESS = 0.004
BRACKET_LIP_HEIGHT = 0.014

FOLDED_POSE = {
    "base_to_link_1": 0.50,
    "link_1_to_link_2": -2.20,
    "link_2_to_link_3": 0.40,
    "link_3_to_link_4": -2.00,
}


def _add_base_plate(base, *, base_metal, hardware_metal) -> None:
    base.visual(
        Box((BASE_PLATE_THICKNESS, BASE_PLATE_WIDTH, BASE_PLATE_HEIGHT)),
        origin=Origin(xyz=(BASE_PLATE_CENTER_X, 0.0, 0.0)),
        material=base_metal,
        name="mount_plate",
    )
    base.visual(
        Box((BASE_SPACER_LENGTH, BASE_SPACER_WIDTH, BASE_SPACER_HEIGHT)),
        origin=Origin(xyz=(-0.015, 0.0, 0.0)),
        material=base_metal,
        name="hinge_spacer",
    )
    for suffix, y_center in (("upper", FORK_CHEEK_CENTER_Y), ("lower", -FORK_CHEEK_CENTER_Y)):
        base.visual(
            Box((BASE_CHEEK_LENGTH, FORK_CHEEK_THICKNESS, BASE_CHEEK_HEIGHT)),
            origin=Origin(xyz=(0.0, y_center, 0.0)),
            material=base_metal,
            name=f"base_cheek_{suffix}",
        )
    for index, z_center in enumerate((-0.032, 0.032), start=1):
        base.visual(
            Cylinder(radius=0.0045, length=0.003),
            origin=Origin(
                xyz=(BASE_PLATE_CENTER_X - (BASE_PLATE_THICKNESS * 0.30), 0.0, z_center),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hardware_metal,
            name=f"fastener_head_{index}",
        )


def _add_link_geometry(link, *, link_metal, pin_metal, terminal: bool, beam_offset_y: float) -> None:
    link.visual(
        Cylinder(radius=LINK_BARREL_RADIUS, length=LINK_BARREL_LENGTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=pin_metal,
        name="root_barrel",
    )
    link.visual(
        Box((LINK_ROOT_LENGTH, LINK_ROOT_WIDTH, LINK_ROOT_HEIGHT)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=link_metal,
        name="root_block",
    )
    link.visual(
        Box((LINK_BEAM_LENGTH, LINK_BEAM_WIDTH, LINK_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.040, beam_offset_y, 0.0)),
        material=link_metal,
        name="beam",
    )
    link.visual(
        Box((LINK_BEAM_LENGTH * 0.64, LINK_BEAM_WIDTH * 0.70, LINK_BEAM_HEIGHT * 0.45)),
        origin=Origin(xyz=(0.041, beam_offset_y, 0.0)),
        material=pin_metal,
        name="beam_cap",
    )

    if terminal:
        link.visual(
            Box((FORK_LENGTH, LINK_BEAM_WIDTH, LINK_BEAM_HEIGHT)),
            origin=Origin(xyz=(LINK_PITCH - (FORK_LENGTH * 0.5), beam_offset_y, 0.0)),
            material=link_metal,
            name="nose_block",
        )
    else:
        link.visual(
            Box((TIP_BLOCK_LENGTH, TIP_BLOCK_WIDTH, TIP_BLOCK_HEIGHT)),
            origin=Origin(xyz=(LINK_PITCH - FORK_LENGTH - (TIP_BLOCK_LENGTH * 0.5), 0.0, 0.0)),
            material=link_metal,
            name="tip_block",
        )
        for suffix, y_center in (("upper", FORK_CHEEK_CENTER_Y), ("lower", -FORK_CHEEK_CENTER_Y)):
            link.visual(
                Box((FORK_LENGTH, FORK_CHEEK_THICKNESS, FORK_CHEEK_HEIGHT)),
                origin=Origin(xyz=(LINK_PITCH - (FORK_LENGTH * 0.5), y_center, 0.0)),
                material=link_metal,
                name=f"fork_cheek_{suffix}",
            )


def _add_platform_bracket(bracket, *, bracket_metal, pin_metal) -> None:
    bracket.visual(
        Box((BRACKET_TONGUE_LENGTH, BRACKET_TONGUE_WIDTH, BRACKET_TONGUE_HEIGHT)),
        origin=Origin(xyz=(BRACKET_TONGUE_LENGTH * 0.5, -LINK_BEAM_OFFSET, 0.0)),
        material=bracket_metal,
        name="attachment_tongue",
    )
    bracket.visual(
        Box((BRACKET_RISER_LENGTH, BRACKET_RISER_WIDTH, BRACKET_RISER_HEIGHT)),
        origin=Origin(
            xyz=(0.011, -LINK_BEAM_OFFSET, BRACKET_RISER_HEIGHT * 0.5),
        ),
        material=bracket_metal,
        name="riser",
    )
    bracket.visual(
        Box((BRACKET_CROSSHEAD_LENGTH, BRACKET_CROSSHEAD_WIDTH, BRACKET_CROSSHEAD_THICKNESS)),
        origin=Origin(
            xyz=(
                0.018,
                0.0,
                BRACKET_RISER_HEIGHT,
            ),
        ),
        material=bracket_metal,
        name="crosshead",
    )
    bracket.visual(
        Box((BRACKET_SHELF_LENGTH, BRACKET_SHELF_WIDTH, BRACKET_SHELF_THICKNESS)),
        origin=Origin(
            xyz=(0.029, 0.0, BRACKET_RISER_HEIGHT + 0.003),
        ),
        material=bracket_metal,
        name="shelf",
    )
    bracket.visual(
        Box((BRACKET_LIP_THICKNESS, BRACKET_SHELF_WIDTH * 0.84, BRACKET_LIP_HEIGHT)),
        origin=Origin(
            xyz=(0.042, 0.0, BRACKET_RISER_HEIGHT - 0.001),
        ),
        material=pin_metal,
        name="front_lip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_fold_out_support_arm")

    base_metal = model.material("base_metal", rgba=(0.18, 0.20, 0.22, 1.0))
    link_metal = model.material("link_metal", rgba=(0.64, 0.67, 0.71, 1.0))
    pin_metal = model.material("pin_metal", rgba=(0.43, 0.46, 0.50, 1.0))
    bracket_metal = model.material("bracket_metal", rgba=(0.26, 0.28, 0.30, 1.0))

    base_plate = model.part("base_plate")
    _add_base_plate(base_plate, base_metal=base_metal, hardware_metal=pin_metal)
    base_plate.inertial = Inertial.from_geometry(
        Box((0.050, BASE_PLATE_WIDTH, BASE_PLATE_HEIGHT)),
        mass=0.85,
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
    )

    for link_name, terminal, beam_offset_y in (
        ("link_1", False, LINK_BEAM_OFFSET),
        ("link_2", False, -LINK_BEAM_OFFSET),
        ("link_3", False, LINK_BEAM_OFFSET),
        ("link_4", True, -LINK_BEAM_OFFSET),
    ):
        link = model.part(link_name)
        _add_link_geometry(
            link,
            link_metal=link_metal,
            pin_metal=pin_metal,
            terminal=terminal,
            beam_offset_y=beam_offset_y,
        )
        link.inertial = Inertial.from_geometry(
            Box((LINK_PITCH, 0.030, 0.028)),
            mass=0.32 if not terminal else 0.29,
            origin=Origin(xyz=(LINK_PITCH * 0.5, 0.0, 0.0)),
        )

    platform_bracket = model.part("platform_bracket")
    _add_platform_bracket(platform_bracket, bracket_metal=bracket_metal, pin_metal=pin_metal)
    platform_bracket.inertial = Inertial.from_geometry(
        Box((0.050, 0.036, 0.040)),
        mass=0.16,
        origin=Origin(xyz=(0.025, 0.0, 0.020)),
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base_plate,
        child="link_1",
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.30, upper=1.90),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent="link_1",
        child="link_2",
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.4, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent="link_2",
        child="link_3",
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.4, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent="link_3",
        child="link_4",
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-2.05, upper=2.05),
    )
    model.articulation(
        "link_4_to_platform_bracket",
        ArticulationType.FIXED,
        parent="link_4",
        child=platform_bracket,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
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

    ctx.expect_contact(base_plate, link_1, name="base_contacts_link_1")
    ctx.expect_contact(link_1, link_2, name="link_1_contacts_link_2")
    ctx.expect_contact(link_2, link_3, name="link_2_contacts_link_3")
    ctx.expect_contact(link_3, link_4, name="link_3_contacts_link_4")
    ctx.expect_contact(link_4, platform_bracket, name="link_4_contacts_platform_bracket")

    ctx.expect_origin_gap(
        platform_bracket,
        base_plate,
        axis="x",
        min_gap=0.30,
        name="open_pose_reach",
    )
    ctx.expect_origin_distance(
        platform_bracket,
        base_plate,
        axes="y",
        max_dist=1e-6,
        name="arm_stays_in_single_plane_open",
    )

    joints_parallel = all(
        joint.axis == (0.0, 1.0, 0.0)
        for joint in (base_to_link_1, link_1_to_link_2, link_2_to_link_3, link_3_to_link_4)
    )
    ctx.check(
        "joint_axes_parallel",
        joints_parallel,
        "All four joints should be revolute around the same local Y axis.",
    )

    with ctx.pose(FOLDED_POSE):
        ctx.fail_if_isolated_parts(name="folded_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="folded_pose_no_overlap")
        ctx.expect_origin_distance(
            platform_bracket,
            base_plate,
            axes="y",
            max_dist=1e-6,
            name="arm_stays_in_single_plane_folded",
        )
        folded_platform_position = ctx.part_world_position(platform_bracket)
        base_position = ctx.part_world_position(base_plate)
        assert folded_platform_position is not None
        assert base_position is not None
        compact_x = abs(folded_platform_position[0] - base_position[0]) <= 0.06
        compact_z = abs(folded_platform_position[2] - base_position[2]) <= 0.13
        ctx.check(
            "folded_pose_packs_near_base",
            compact_x and compact_z,
            (
                "Folded bracket should remain close to the base plate; "
                f"dx={abs(folded_platform_position[0] - base_position[0]):.4f}, "
                f"dz={abs(folded_platform_position[2] - base_position[2]):.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
