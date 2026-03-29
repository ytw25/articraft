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


PLATE_THICKNESS = 0.006
PLATE_WIDTH = 0.050
PLATE_HEIGHT = 0.120
BASE_JOINT_X = PLATE_THICKNESS + 0.006
BASE_JOINT_Z = 0.094

LINK_PITCH = 0.074
LINK_WIDTH = 0.018
LINK_THICKNESS = 0.012
LINK_BODY_LENGTH = LINK_PITCH - 0.018
HINGE_RADIUS = 0.006
CENTER_BARREL_LENGTH = 0.010
FORK_BARREL_LENGTH = 0.005
FORK_BARREL_OFFSET_Y = 0.0075
CHEEK_LENGTH = 0.016
CHEEK_WIDTH = 0.0045
DISTAL_TANG_LENGTH = 0.016
DISTAL_TANG_WIDTH = 0.008
END_MOUNT_LENGTH = 0.022
END_MOUNT_WIDTH = 0.016

BRACKET_UPRIGHT_THICKNESS = 0.010
BRACKET_UPRIGHT_WIDTH = 0.026
BRACKET_UPRIGHT_HEIGHT = 0.032
BRACKET_PAD_LENGTH = 0.024
BRACKET_PAD_WIDTH = 0.024
BRACKET_PAD_THICKNESS = 0.004
BRACKET_SHELF_LENGTH = 0.040
BRACKET_SHELF_WIDTH = 0.028
BRACKET_SHELF_THICKNESS = 0.005
BRACKET_LIP_THICKNESS = 0.005
BRACKET_LIP_HEIGHT = 0.018


def _y_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def _x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0))


def _add_y_barrel(part, *, name: str, x: float, y: float, z: float, radius: float, length: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_y_axis_origin((x, y, z)),
        material=material,
        name=name,
    )


def _build_link(model: ArticulatedObject, name: str, *, material, hardware_material, terminal: bool = False):
    link = model.part(name)

    link.visual(
        Box((LINK_BODY_LENGTH, LINK_WIDTH, LINK_THICKNESS)),
        origin=Origin(xyz=(LINK_PITCH / 2.0, 0.0, 0.0)),
        material=material,
        name="main_beam",
    )
    link.visual(
        Box((CHEEK_LENGTH, CHEEK_WIDTH, LINK_THICKNESS + 0.002)),
        origin=Origin(xyz=(CHEEK_LENGTH / 2.0, -FORK_BARREL_OFFSET_Y, 0.0)),
        material=material,
        name="prox_cheek_left",
    )
    link.visual(
        Box((CHEEK_LENGTH, CHEEK_WIDTH, LINK_THICKNESS + 0.002)),
        origin=Origin(xyz=(CHEEK_LENGTH / 2.0, FORK_BARREL_OFFSET_Y, 0.0)),
        material=material,
        name="prox_cheek_right",
    )

    _add_y_barrel(
        link,
        name="prox_fork_left",
        x=0.0,
        y=-FORK_BARREL_OFFSET_Y,
        z=0.0,
        radius=HINGE_RADIUS,
        length=FORK_BARREL_LENGTH,
        material=hardware_material,
    )
    _add_y_barrel(
        link,
        name="prox_fork_right",
        x=0.0,
        y=FORK_BARREL_OFFSET_Y,
        z=0.0,
        radius=HINGE_RADIUS,
        length=FORK_BARREL_LENGTH,
        material=hardware_material,
    )

    if terminal:
        link.visual(
            Box((END_MOUNT_LENGTH, END_MOUNT_WIDTH, LINK_THICKNESS + 0.002)),
            origin=Origin(xyz=(LINK_PITCH - END_MOUNT_LENGTH / 2.0, 0.0, 0.0)),
            material=material,
            name="end_mount",
        )
    else:
        link.visual(
            Box((DISTAL_TANG_LENGTH, DISTAL_TANG_WIDTH, LINK_THICKNESS + 0.002)),
            origin=Origin(xyz=(LINK_PITCH - DISTAL_TANG_LENGTH / 2.0, 0.0, 0.0)),
            material=material,
            name="distal_tang",
        )
        _add_y_barrel(
            link,
            name="distal_barrel",
            x=LINK_PITCH,
            y=0.0,
            z=0.0,
            radius=HINGE_RADIUS,
            length=CENTER_BARREL_LENGTH,
            material=hardware_material,
        )

    link.inertial = Inertial.from_geometry(
        Box((LINK_PITCH, 0.024, 0.016)),
        mass=0.18 if not terminal else 0.20,
        origin=Origin(xyz=(LINK_PITCH / 2.0, 0.0, 0.0)),
    )
    return link


def _build_platform_bracket(model: ArticulatedObject, *, material, accent_material):
    bracket = model.part("platform_bracket")

    bracket.visual(
        Box((BRACKET_PAD_LENGTH, BRACKET_PAD_WIDTH, BRACKET_PAD_THICKNESS)),
        origin=Origin(
            xyz=(
                BRACKET_PAD_LENGTH / 2.0,
                0.0,
                BRACKET_PAD_THICKNESS / 2.0,
            )
        ),
        material=material,
        name="mount_pad",
    )
    bracket.visual(
        Box((BRACKET_UPRIGHT_THICKNESS, BRACKET_UPRIGHT_WIDTH, BRACKET_UPRIGHT_HEIGHT)),
        origin=Origin(
            xyz=(
                BRACKET_UPRIGHT_THICKNESS / 2.0,
                0.0,
                BRACKET_PAD_THICKNESS + BRACKET_UPRIGHT_HEIGHT / 2.0,
            )
        ),
        material=material,
        name="upright",
    )
    bracket.visual(
        Box((BRACKET_SHELF_LENGTH, BRACKET_SHELF_WIDTH, BRACKET_SHELF_THICKNESS)),
        origin=Origin(
            xyz=(
                BRACKET_UPRIGHT_THICKNESS + BRACKET_SHELF_LENGTH / 2.0,
                0.0,
                BRACKET_PAD_THICKNESS + BRACKET_UPRIGHT_HEIGHT - BRACKET_SHELF_THICKNESS / 2.0,
            )
        ),
        material=accent_material,
        name="shelf",
    )
    bracket.visual(
        Box((BRACKET_LIP_THICKNESS, BRACKET_SHELF_WIDTH, BRACKET_LIP_HEIGHT)),
        origin=Origin(
            xyz=(
                BRACKET_UPRIGHT_THICKNESS + BRACKET_SHELF_LENGTH + BRACKET_LIP_THICKNESS / 2.0,
                0.0,
                BRACKET_PAD_THICKNESS
                + BRACKET_UPRIGHT_HEIGHT
                + BRACKET_LIP_HEIGHT / 2.0
                - BRACKET_SHELF_THICKNESS,
            )
        ),
        material=material,
        name="front_lip",
    )
    bracket.visual(
        Box((0.020, 0.010, 0.018)),
        origin=Origin(xyz=(0.018, 0.0, BRACKET_PAD_THICKNESS + 0.009)),
        material=material,
        name="gusset",
    )

    bracket.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.035)),
        mass=0.14,
        origin=Origin(xyz=(0.030, 0.0, 0.010)),
    )
    return bracket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_support_arm")

    base_material = model.material("base_plate_finish", rgba=(0.21, 0.23, 0.25, 1.0))
    link_material = model.material("link_finish", rgba=(0.34, 0.35, 0.38, 1.0))
    hardware_material = model.material("hinge_hardware", rgba=(0.62, 0.64, 0.67, 1.0))
    bracket_material = model.material("bracket_finish", rgba=(0.74, 0.76, 0.79, 1.0))
    shelf_material = model.material("shelf_pad", rgba=(0.56, 0.58, 0.61, 1.0))

    base = model.part("base_plate")
    base.visual(
        Box((PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)),
        origin=Origin(xyz=(PLATE_THICKNESS / 2.0, 0.0, PLATE_HEIGHT / 2.0)),
        material=base_material,
        name="plate",
    )
    base.visual(
        Box((BASE_JOINT_X, 0.008, 0.024)),
        origin=Origin(xyz=(BASE_JOINT_X / 2.0, 0.0, BASE_JOINT_Z)),
        material=base_material,
        name="hinge_lug",
    )
    _add_y_barrel(
        base,
        name="hinge_barrel",
        x=BASE_JOINT_X,
        y=0.0,
        z=BASE_JOINT_Z,
        radius=HINGE_RADIUS,
        length=CENTER_BARREL_LENGTH,
        material=hardware_material,
    )
    base.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=_x_axis_origin((PLATE_THICKNESS + 0.002, 0.0, 0.032)),
        material=hardware_material,
        name="lower_fastener_head",
    )
    base.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=_x_axis_origin((PLATE_THICKNESS + 0.002, 0.0, 0.080)),
        material=hardware_material,
        name="upper_fastener_head",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.020, PLATE_WIDTH, PLATE_HEIGHT)),
        mass=0.45,
        origin=Origin(xyz=(0.010, 0.0, PLATE_HEIGHT / 2.0)),
    )

    link_1 = _build_link(model, "link_1", material=link_material, hardware_material=hardware_material)
    link_2 = _build_link(model, "link_2", material=link_material, hardware_material=hardware_material)
    link_3 = _build_link(model, "link_3", material=link_material, hardware_material=hardware_material)
    link_4 = _build_link(
        model,
        "link_4",
        material=link_material,
        hardware_material=hardware_material,
        terminal=True,
    )
    bracket = _build_platform_bracket(model, material=bracket_material, accent_material=shelf_material)

    revolute_limits = MotionLimits(effort=20.0, velocity=2.5, lower=-2.85, upper=2.85)

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_1,
        origin=Origin(xyz=(BASE_JOINT_X, 0.0, BASE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=link_4,
        origin=Origin(xyz=(LINK_PITCH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link_4_to_platform_bracket",
        ArticulationType.FIXED,
        parent=link_4,
        child=bracket,
        origin=Origin(xyz=(LINK_PITCH, 0.0, (LINK_THICKNESS + 0.002) / 2.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")
    bracket = object_model.get_part("platform_bracket")

    base_to_link_1 = object_model.get_articulation("base_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")
    link_3_to_link_4 = object_model.get_articulation("link_3_to_link_4")
    link_4_to_platform_bracket = object_model.get_articulation("link_4_to_platform_bracket")

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

    axes_ok = all(
        tuple(round(component, 6) for component in articulation.axis) == (0.0, 1.0, 0.0)
        for articulation in (base_to_link_1, link_1_to_link_2, link_2_to_link_3, link_3_to_link_4)
    )
    revolute_ok = all(
        articulation.articulation_type == ArticulationType.REVOLUTE
        for articulation in (base_to_link_1, link_1_to_link_2, link_2_to_link_3, link_3_to_link_4)
    )
    ctx.check(
        "four_parallel_revolute_joints",
        axes_ok and revolute_ok and link_4_to_platform_bracket.articulation_type == ArticulationType.FIXED,
        details="Expected four serial revolute joints about +Y plus a fixed end bracket.",
    )

    ctx.expect_contact(base, link_1, name="base_hinge_contacts_link_1")
    ctx.expect_contact(link_1, link_2, name="link_1_contacts_link_2")
    ctx.expect_contact(link_2, link_3, name="link_2_contacts_link_3")
    ctx.expect_contact(link_3, link_4, name="link_3_contacts_link_4")
    ctx.expect_contact(link_4, bracket, name="link_4_contacts_platform_bracket")
    ctx.expect_gap(bracket, base, axis="x", min_gap=0.280, name="unfolded_arm_reaches_forward")

    packed_pose = {
        base_to_link_1: 0.80,
        link_1_to_link_2: -1.80,
        link_2_to_link_3: 1.20,
        link_3_to_link_4: -1.80,
    }
    with ctx.pose(packed_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="packed_pose_no_overlaps")
        ctx.expect_gap(
            bracket,
            base,
            axis="x",
            min_gap=0.060,
            max_gap=0.150,
            name="packed_pose_bracket_stays_close_to_base",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
