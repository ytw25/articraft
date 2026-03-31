from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_PLATE_SIZE = (0.008, 0.065, 0.130)
BASE_MOUNT_SIZE = (0.022, 0.028, 0.022)
BASE_JOINT_Z = 0.065

LINK_LENGTH = 0.085
LINK_PAD_LENGTH = 0.014
LINK_PAD_WIDTH = 0.028
LINK_BAR_WIDTH = 0.014
LINK_DEPTH = 0.018
LINK_SIDE_OFFSET = 0.015
LINK_BRIDGE_LENGTH = LINK_LENGTH - 2.0 * LINK_PAD_LENGTH + 0.002

PLATFORM_MOUNT_SIZE = (0.014, 0.028, 0.018)
PLATFORM_SHELF_SIZE = (0.045, 0.052, 0.006)
PLATFORM_LIP_SIZE = (0.006, 0.052, 0.022)
PLATFORM_RIB_SIZE = (0.024, 0.012, 0.018)


def _add_link_geometry(part, *, side_offset: float, body_material: str) -> None:
    part.visual(
        Box((LINK_PAD_LENGTH, LINK_PAD_WIDTH, LINK_DEPTH)),
        origin=Origin(xyz=(LINK_PAD_LENGTH * 0.5, 0.0, 0.0)),
        material=body_material,
        name="root_pad",
    )
    part.visual(
        Box((LINK_BRIDGE_LENGTH, LINK_BAR_WIDTH, LINK_DEPTH)),
        origin=Origin(xyz=(LINK_LENGTH * 0.5, side_offset, 0.0)),
        material=body_material,
        name="main_bar",
    )
    part.visual(
        Box((LINK_PAD_LENGTH, LINK_PAD_WIDTH, LINK_DEPTH)),
        origin=Origin(xyz=(LINK_LENGTH - LINK_PAD_LENGTH * 0.5, 0.0, 0.0)),
        material=body_material,
        name="end_pad",
    )


def _link_limits() -> MotionLimits:
    return MotionLimits(
        effort=20.0,
        velocity=2.5,
        lower=-2.95,
        upper=2.95,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_support_arm")

    base_plate_material = model.material("base_plate_black", rgba=(0.15, 0.16, 0.17, 1.0))
    link_material = model.material("link_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    bracket_material = model.material("bracket_graphite", rgba=(0.36, 0.38, 0.41, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box(BASE_PLATE_SIZE),
        origin=Origin(
            xyz=(BASE_PLATE_SIZE[0] * 0.5, 0.0, BASE_PLATE_SIZE[2] * 0.5),
        ),
        material=base_plate_material,
        name="plate",
    )
    base_plate.visual(
        Box(BASE_MOUNT_SIZE),
        origin=Origin(xyz=(0.019, 0.0, BASE_JOINT_Z)),
        material=base_plate_material,
        name="hinge_mount",
    )

    link_1 = model.part("link_1")
    _add_link_geometry(link_1, side_offset=LINK_SIDE_OFFSET, body_material=link_material.name)

    link_2 = model.part("link_2")
    _add_link_geometry(link_2, side_offset=-LINK_SIDE_OFFSET, body_material=link_material.name)

    link_3 = model.part("link_3")
    _add_link_geometry(link_3, side_offset=LINK_SIDE_OFFSET, body_material=link_material.name)

    link_4 = model.part("link_4")
    _add_link_geometry(link_4, side_offset=-LINK_SIDE_OFFSET, body_material=link_material.name)

    platform_bracket = model.part("platform_bracket")
    platform_bracket.visual(
        Box(PLATFORM_MOUNT_SIZE),
        origin=Origin(xyz=(PLATFORM_MOUNT_SIZE[0] * 0.5, 0.0, 0.0)),
        material=bracket_material,
        name="mount_block",
    )
    platform_bracket.visual(
        Box(PLATFORM_SHELF_SIZE),
        origin=Origin(
            xyz=(
                PLATFORM_MOUNT_SIZE[0] + PLATFORM_SHELF_SIZE[0] * 0.5,
                0.0,
                LINK_DEPTH * 0.35,
            )
        ),
        material=bracket_material,
        name="shelf",
    )
    platform_bracket.visual(
        Box(PLATFORM_LIP_SIZE),
        origin=Origin(
            xyz=(
                PLATFORM_MOUNT_SIZE[0] + PLATFORM_SHELF_SIZE[0] - PLATFORM_LIP_SIZE[0] * 0.5,
                0.0,
                LINK_DEPTH * 0.35 + PLATFORM_SHELF_SIZE[2] * 0.5 + PLATFORM_LIP_SIZE[2] * 0.5,
            )
        ),
        material=bracket_material,
        name="retaining_lip",
    )
    platform_bracket.visual(
        Box(PLATFORM_RIB_SIZE),
        origin=Origin(
            xyz=(
                PLATFORM_MOUNT_SIZE[0] + PLATFORM_RIB_SIZE[0] * 0.5,
                0.0,
                -PLATFORM_RIB_SIZE[2] * 0.5,
            )
        ),
        material=bracket_material,
        name="underside_rib",
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base_plate,
        child=link_1,
        origin=Origin(xyz=(0.030, 0.0, BASE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=_link_limits(),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=_link_limits(),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=_link_limits(),
    )
    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=link_4,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=_link_limits(),
    )
    model.articulation(
        "link_4_to_platform",
        ArticulationType.FIXED,
        parent=link_4,
        child=platform_bracket,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
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
    link_4_to_platform = object_model.get_articulation("link_4_to_platform")

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
        "support_arm_parts_present",
        all(
            part.name
            for part in (base_plate, link_1, link_2, link_3, link_4, platform_bracket)
        ),
        "One or more support-arm parts could not be resolved.",
    )
    ctx.check(
        "four_parallel_revolute_joints",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE and tuple(joint.axis) == (0.0, 1.0, 0.0)
            for joint in (base_to_link_1, link_1_to_link_2, link_2_to_link_3, link_3_to_link_4)
        ),
        "The primary folding joints must all revolve about the shared Y axis.",
    )
    ctx.check(
        "platform_mount_is_fixed",
        link_4_to_platform.articulation_type == ArticulationType.FIXED,
        "The platform bracket should be rigidly mounted to the last link.",
    )

    ctx.expect_contact(base_plate, link_1, name="base_contacts_link_1")
    ctx.expect_contact(link_1, link_2, name="link_1_contacts_link_2")
    ctx.expect_contact(link_2, link_3, name="link_2_contacts_link_3")
    ctx.expect_contact(link_3, link_4, name="link_3_contacts_link_4")
    ctx.expect_contact(link_4, platform_bracket, name="link_4_contacts_platform")

    ctx.expect_gap(
        platform_bracket,
        base_plate,
        axis="x",
        min_gap=0.34,
        name="default_pose_has_long_forward_reach",
    )

    packed_pose = {
        base_to_link_1: 1.80,
        link_1_to_link_2: -2.60,
        link_2_to_link_3: 2.60,
        link_3_to_link_4: -1.80,
    }
    with ctx.pose(packed_pose):
        ctx.expect_gap(
            platform_bracket,
            base_plate,
            axis="x",
            min_gap=0.045,
            max_gap=0.150,
            name="packed_pose_stays_close_to_base",
        )
        ctx.expect_gap(
            base_plate,
            platform_bracket,
            axis="z",
            min_gap=0.0,
            max_gap=0.020,
            name="packed_pose_tucks_just_below_base",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
