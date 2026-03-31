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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_support_arm")

    base_color = model.material("base_powder_coat", rgba=(0.18, 0.19, 0.21, 1.0))
    link_color = model.material("link_anodized", rgba=(0.55, 0.58, 0.62, 1.0))
    hardware_color = model.material("hardware_dark", rgba=(0.32, 0.34, 0.36, 1.0))
    bracket_color = model.material("bracket_finish", rgba=(0.72, 0.74, 0.77, 1.0))

    link_pitch = 0.070
    link_height = 0.012
    hinge_radius = 0.0065
    half_barrel_length = 0.010
    hinge_roll = math.pi / 2.0

    def add_link_geometry(part, *, include_distal_hinge: bool, include_end_mount: bool = False) -> None:
        part.visual(
            Cylinder(radius=hinge_radius, length=half_barrel_length),
            origin=Origin(xyz=(0.0, half_barrel_length / 2.0, 0.0), rpy=(hinge_roll, 0.0, 0.0)),
            material=hardware_color,
            name="proximal_hinge",
        )
        part.visual(
            Box((0.018, 0.010, link_height)),
            origin=Origin(xyz=(0.014, 0.006, 0.0)),
            material=link_color,
            name="proximal_cheek",
        )
        part.visual(
            Box((0.024, 0.008, link_height)),
            origin=Origin(xyz=(0.034, 0.0, 0.0)),
            material=link_color,
            name="link_body",
        )
        if include_distal_hinge:
            part.visual(
                Box((0.020, 0.010, link_height)),
                origin=Origin(xyz=(0.054, -0.006, 0.0)),
                material=link_color,
                name="distal_cheek",
            )
        if include_distal_hinge:
            part.visual(
                Cylinder(radius=hinge_radius, length=half_barrel_length),
                origin=Origin(xyz=(link_pitch, -half_barrel_length / 2.0, 0.0), rpy=(hinge_roll, 0.0, 0.0)),
                material=hardware_color,
                name="distal_hinge",
            )
        if include_end_mount:
            part.visual(
                Box((0.022, 0.010, link_height)),
                origin=Origin(xyz=(0.057, -0.006, 0.0)),
                material=link_color,
                name="end_mount_block",
            )

    base = model.part("base_plate")
    base.visual(
        Box((0.012, 0.080, 0.120)),
        origin=Origin(xyz=(0.006, 0.0, 0.060)),
        material=base_color,
        name="mount_plate",
    )
    base.visual(
        Box((0.008, 0.034, 0.042)),
        origin=Origin(xyz=(0.012, 0.0, 0.091)),
        material=base_color,
        name="hinge_block",
    )
    base.visual(
        Cylinder(radius=hinge_radius, length=half_barrel_length),
        origin=Origin(xyz=(0.0225, -half_barrel_length / 2.0, 0.090), rpy=(hinge_roll, 0.0, 0.0)),
        material=hardware_color,
        name="base_hinge",
    )
    for idx, z_pos in enumerate((0.031, 0.089), start=1):
        base.visual(
            Cylinder(radius=0.006, length=0.003),
            origin=Origin(xyz=(0.0135, 0.0, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware_color,
            name=f"mount_screw_{idx}",
        )

    link1 = model.part("link1")
    add_link_geometry(link1, include_distal_hinge=True)

    link2 = model.part("link2")
    add_link_geometry(link2, include_distal_hinge=True)

    link3 = model.part("link3")
    add_link_geometry(link3, include_distal_hinge=True)

    link4 = model.part("link4")
    add_link_geometry(link4, include_distal_hinge=False, include_end_mount=True)

    bracket = model.part("platform_bracket")
    bracket.visual(
        Box((0.012, 0.022, 0.012)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=bracket_color,
        name="bracket_mount",
    )
    bracket.visual(
        Box((0.008, 0.022, 0.030)),
        origin=Origin(xyz=(0.014, 0.0, 0.015)),
        material=bracket_color,
        name="bracket_riser",
    )
    bracket.visual(
        Box((0.028, 0.030, 0.004)),
        origin=Origin(xyz=(0.028, 0.0, 0.028)),
        material=bracket_color,
        name="platform_pad",
    )
    bracket.visual(
        Box((0.004, 0.032, 0.010)),
        origin=Origin(xyz=(0.040, 0.0, 0.023)),
        material=bracket_color,
        name="platform_lip",
    )

    revolute_limits = MotionLimits(
        effort=12.0,
        velocity=2.5,
        lower=-2.65,
        upper=2.65,
    )
    model.articulation(
        "base_to_link1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link1,
        origin=Origin(xyz=(0.0225, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link1_to_link2",
        ArticulationType.REVOLUTE,
        parent=link1,
        child=link2,
        origin=Origin(xyz=(link_pitch, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link2_to_link3",
        ArticulationType.REVOLUTE,
        parent=link2,
        child=link3,
        origin=Origin(xyz=(link_pitch, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link3_to_link4",
        ArticulationType.REVOLUTE,
        parent=link3,
        child=link4,
        origin=Origin(xyz=(link_pitch, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link4_to_bracket",
        ArticulationType.FIXED,
        parent=link4,
        child=bracket,
        origin=Origin(xyz=(0.068, 0.0, 0.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    link1 = object_model.get_part("link1")
    link2 = object_model.get_part("link2")
    link3 = object_model.get_part("link3")
    link4 = object_model.get_part("link4")
    bracket = object_model.get_part("platform_bracket")

    joints = [
        object_model.get_articulation("base_to_link1"),
        object_model.get_articulation("link1_to_link2"),
        object_model.get_articulation("link2_to_link3"),
        object_model.get_articulation("link3_to_link4"),
    ]

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

    # Keep pose-specific checks lean.
    # Do not add blanket lower/upper pose sweeps or
    # `fail_if_parts_overlap_in_sampled_poses(...)` by default.
    # Add `ctx.warn_if_articulation_overlaps(...)` only when joint clearance is
    # genuinely uncertain or mechanically important.

    ctx.expect_contact(base, link1, name="base_contacts_link1")
    ctx.expect_contact(link1, link2, name="link1_contacts_link2")
    ctx.expect_contact(link2, link3, name="link2_contacts_link3")
    ctx.expect_contact(link3, link4, name="link3_contacts_link4")
    ctx.expect_contact(link4, bracket, name="link4_contacts_bracket")

    axes_parallel = all(tuple(joint.axis) == (0.0, 1.0, 0.0) for joint in joints)
    ctx.check(
        "all_revolute_axes_parallel",
        axes_parallel,
        details="Expected all four articulated joints to rotate about the world/local +Y axis.",
    )

    limits_present = all(
        joint.motion_limits is not None
        and joint.motion_limits.lower is not None
        and joint.motion_limits.upper is not None
        and joint.motion_limits.lower <= -1.5
        and joint.motion_limits.upper >= 1.5
        for joint in joints
    )
    ctx.check(
        "fold_range_is_useful",
        limits_present,
        details="Each revolute joint should have enough angular range to fold and extend the arm.",
    )

    base_front_x = ctx.part_world_aabb(base)[1][0]
    open_bracket_pos = ctx.part_world_position(bracket)
    ctx.check(
        "extended_reach_is_longer_than_base_depth",
        open_bracket_pos is not None and open_bracket_pos[0] - base_front_x >= 0.27,
        details="The end bracket should project well beyond the base plate in the extended pose.",
    )

    compact_pose = {
        joints[0]: 1.10,
        joints[1]: 1.10,
        joints[2]: 1.10,
        joints[3]: 1.10,
    }
    with ctx.pose(compact_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="compact_pose_no_overlaps")
        compact_bracket_pos = ctx.part_world_position(bracket)
        ctx.check(
            "folded_pose_reduces_reach",
            open_bracket_pos is not None
            and compact_bracket_pos is not None
            and compact_bracket_pos[0] <= open_bracket_pos[0] - 0.12,
            details="A representative folded pose should tuck the end bracket substantially closer to the base.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
