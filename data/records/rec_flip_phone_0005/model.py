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
    TestContext,
    TestReport,
)

PHONE_WIDTH = 0.074
HALF_DEPTH = 0.080
HALF_THICKNESS = 0.0064
CORNER_RADIUS = 0.0095
HINGE_BACKSET = 0.0062
SEAM_GAP = 0.0006
DISPLAY_WIDTH = 0.0695
DISPLAY_DEPTH = 0.0770
DISPLAY_THICKNESS = 0.0009
HINGE_AXIS_Y = 0.0
HINGE_AXIS_Z = HALF_THICKNESS + SEAM_GAP
LOWER_HINGE_RADIUS = 0.0038
UPPER_HINGE_RADIUS = 0.0033
HINGE_SEGMENT_LENGTH = 0.014
HINGE_LEAF_LENGTH = HINGE_BACKSET + 0.0010
HINGE_LEAF_THICKNESS = 0.0018


def _add_rounded_slab(
    part,
    *,
    width: float,
    depth: float,
    thickness: float,
    radius: float,
    origin_xyz: tuple[float, float, float],
    material,
    name_prefix: str,
) -> None:
    ox, oy, oz = origin_xyz
    part.visual(
        Box((width - 2.0 * radius, depth, thickness)),
        origin=Origin(xyz=(ox, oy, oz + thickness * 0.5)),
        material=material,
        name=f"{name_prefix}_core_x",
    )
    part.visual(
        Box((width, depth - 2.0 * radius, thickness)),
        origin=Origin(xyz=(ox, oy, oz + thickness * 0.5)),
        material=material,
        name=f"{name_prefix}_core_y",
    )
    for ix, x_sign in enumerate((-1.0, 1.0)):
        for iy, y_sign in enumerate((-1.0, 1.0)):
            part.visual(
                Cylinder(radius=radius, length=thickness),
                origin=Origin(
                    xyz=(
                        ox + x_sign * (width * 0.5 - radius),
                        oy + y_sign * (depth * 0.5 - radius),
                        oz + thickness * 0.5,
                    )
                ),
                material=material,
                name=f"{name_prefix}_corner_{ix}_{iy}",
            )


def _add_hinge_modules(
    part,
    *,
    barrel_x_positions: tuple[float, ...],
    barrel_radius: float,
    barrel_center_z: float,
    leaf_center_z: float,
    shell_material,
    hinge_material,
    name_prefix: str,
) -> None:
    for index, center_x in enumerate(barrel_x_positions):
        part.visual(
            Cylinder(radius=barrel_radius, length=HINGE_SEGMENT_LENGTH),
            origin=Origin(
                xyz=(center_x, HINGE_AXIS_Y, barrel_center_z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=hinge_material,
            name=f"{name_prefix}_hinge_barrel_{index}",
        )
        part.visual(
            Box((HINGE_SEGMENT_LENGTH * 0.92, HINGE_LEAF_LENGTH, HINGE_LEAF_THICKNESS)),
            origin=Origin(
                xyz=(center_x, HINGE_LEAF_LENGTH * 0.5, leaf_center_z),
            ),
            material=shell_material,
            name=f"{name_prefix}_hinge_leaf_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_flip_phone")

    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.20, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.33, 0.34, 0.37, 1.0))
    cover_glass = model.material("cover_glass", rgba=(0.05, 0.06, 0.07, 0.97))
    bump_graphite = model.material("bump_graphite", rgba=(0.14, 0.15, 0.17, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.11, 0.13, 0.16, 0.94))

    lower_half = model.part("lower_half")
    _add_rounded_slab(
        lower_half,
        width=PHONE_WIDTH,
        depth=HALF_DEPTH,
        thickness=HALF_THICKNESS,
        radius=CORNER_RADIUS,
        origin_xyz=(0.0, HINGE_BACKSET + HALF_DEPTH * 0.5, 0.0),
        material=graphite,
        name_prefix="lower_shell",
    )
    _add_hinge_modules(
        lower_half,
        barrel_x_positions=(-0.026, 0.0, 0.026),
        barrel_radius=LOWER_HINGE_RADIUS,
        barrel_center_z=HINGE_AXIS_Z,
        leaf_center_z=HALF_THICKNESS - HINGE_LEAF_THICKNESS * 0.5,
        shell_material=graphite,
        hinge_material=hinge_metal,
        name_prefix="lower",
    )
    lower_half.inertial = Inertial.from_geometry(
        Box((PHONE_WIDTH, HALF_DEPTH + HINGE_BACKSET, HALF_THICKNESS)),
        mass=0.095,
        origin=Origin(
            xyz=(
                0.0,
                (HALF_DEPTH + HINGE_BACKSET) * 0.5,
                HALF_THICKNESS * 0.5,
            )
        ),
    )

    upper_half = model.part("upper_half")
    _add_rounded_slab(
        upper_half,
        width=PHONE_WIDTH,
        depth=HALF_DEPTH,
        thickness=HALF_THICKNESS,
        radius=CORNER_RADIUS,
        origin_xyz=(0.0, HINGE_BACKSET + HALF_DEPTH * 0.5, 0.0),
        material=graphite,
        name_prefix="upper_shell",
    )
    _add_hinge_modules(
        upper_half,
        barrel_x_positions=(-0.013, 0.013),
        barrel_radius=UPPER_HINGE_RADIUS,
        barrel_center_z=0.0,
        leaf_center_z=HINGE_LEAF_THICKNESS * 0.5,
        shell_material=graphite,
        hinge_material=hinge_metal,
        name_prefix="upper",
    )
    upper_half.inertial = Inertial.from_geometry(
        Box((PHONE_WIDTH, HALF_DEPTH + HINGE_BACKSET, HALF_THICKNESS)),
        mass=0.105,
        origin=Origin(
            xyz=(
                0.0,
                (HALF_DEPTH + HINGE_BACKSET) * 0.5,
                HALF_THICKNESS * 0.5,
            )
        ),
    )

    exterior_display = model.part("exterior_display")
    exterior_display.visual(
        Box((DISPLAY_WIDTH, DISPLAY_DEPTH, DISPLAY_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                HINGE_BACKSET + HALF_DEPTH * 0.5,
                DISPLAY_THICKNESS * 0.5,
            ),
        ),
        material=cover_glass,
        name="display_panel",
    )
    exterior_display.inertial = Inertial.from_geometry(
        Box((DISPLAY_WIDTH, DISPLAY_DEPTH, DISPLAY_THICKNESS)),
        mass=0.016,
        origin=Origin(
            xyz=(
                0.0,
                HINGE_BACKSET + HALF_DEPTH * 0.5,
                DISPLAY_THICKNESS * 0.5,
            ),
        ),
    )

    camera_module = model.part("camera_module")
    _add_rounded_slab(
        camera_module,
        width=0.028,
        depth=0.036,
        thickness=0.0031,
        radius=0.0048,
        origin_xyz=(-0.0185, HINGE_BACKSET + 0.020, 0.0),
        material=bump_graphite,
        name_prefix="camera_bump",
    )
    for name, x_pos, y_pos, radius, height in (
        ("main_lens_upper", -0.0245, HINGE_BACKSET + 0.0135, 0.0042, 0.0018),
        ("main_lens_lower", -0.0245, HINGE_BACKSET + 0.0280, 0.0042, 0.0018),
        ("aux_lens", -0.0118, HINGE_BACKSET + 0.0205, 0.0034, 0.0016),
    ):
        camera_module.visual(
            Cylinder(radius=radius, length=height),
            origin=Origin(
                xyz=(x_pos, y_pos, 0.0031 + height * 0.5),
            ),
            material=lens_glass,
            name=name,
        )
    camera_module.visual(
        Cylinder(radius=0.0017, length=0.0012),
        origin=Origin(
            xyz=(-0.0090, HINGE_BACKSET + 0.0325, 0.0031 + 0.0006),
        ),
        material=cover_glass,
        name="flash_window",
    )
    camera_module.inertial = Inertial.from_geometry(
        Box((0.028, 0.036, 0.0050)),
        mass=0.014,
        origin=Origin(xyz=(-0.0185, HINGE_BACKSET + 0.020, 0.0025)),
    )

    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_half,
        child=upper_half,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=6.0,
            lower=0.0,
            upper=3.08,
        ),
    )
    model.articulation(
        "upper_to_display",
        ArticulationType.FIXED,
        parent=upper_half,
        child=exterior_display,
        origin=Origin(xyz=(0.0, 0.0, HALF_THICKNESS)),
    )
    model.articulation(
        "upper_to_camera",
        ArticulationType.FIXED,
        parent=upper_half,
        child=camera_module,
        origin=Origin(xyz=(0.0, 0.0, HALF_THICKNESS + DISPLAY_THICKNESS)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_half = object_model.get_part("lower_half")
    upper_half = object_model.get_part("upper_half")
    exterior_display = object_model.get_part("exterior_display")
    camera_module = object_model.get_part("camera_module")
    fold_hinge = object_model.get_articulation("fold_hinge")

    lower_shell = lower_half.get_visual("lower_shell_core_x")
    upper_shell = upper_half.get_visual("upper_shell_core_x")
    display_panel = exterior_display.get_visual("display_panel")
    camera_bump = camera_module.get_visual("camera_bump_core_x")
    main_lens_upper = camera_module.get_visual("main_lens_upper")
    upper_hinge_barrel_left = upper_half.get_visual("upper_hinge_barrel_0")
    upper_hinge_barrel_right = upper_half.get_visual("upper_hinge_barrel_1")
    lower_hinge_barrel_left = lower_half.get_visual("lower_hinge_barrel_0")
    lower_hinge_barrel_center = lower_half.get_visual("lower_hinge_barrel_1")
    lower_hinge_barrel_right = lower_half.get_visual("lower_hinge_barrel_2")

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

    ctx.expect_overlap(
        upper_half,
        lower_half,
        axes="xy",
        min_overlap=0.05,
        elem_a=upper_shell,
        elem_b=lower_shell,
    )
    ctx.expect_gap(
        upper_half,
        lower_half,
        axis="z",
        min_gap=0.0003,
        max_gap=0.0012,
        positive_elem=upper_shell,
        negative_elem=lower_shell,
    )
    ctx.expect_within(
        exterior_display,
        upper_half,
        axes="xy",
        inner_elem=display_panel,
    )
    ctx.expect_overlap(
        exterior_display,
        upper_half,
        axes="xy",
        min_overlap=0.069,
        elem_a=display_panel,
    )
    ctx.expect_gap(
        exterior_display,
        upper_half,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=display_panel,
        negative_elem=upper_shell,
    )
    ctx.expect_within(
        camera_module,
        exterior_display,
        axes="xy",
        inner_elem=camera_bump,
        outer_elem=display_panel,
    )
    ctx.expect_contact(
        camera_module,
        exterior_display,
        elem_a=camera_bump,
        elem_b=display_panel,
    )
    ctx.expect_gap(
        camera_module,
        upper_half,
        axis="z",
        min_gap=0.0035,
        positive_elem=main_lens_upper,
        negative_elem=upper_shell,
    )
    ctx.expect_gap(
        camera_module,
        camera_module,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=main_lens_upper,
        negative_elem=camera_bump,
    )
    ctx.expect_overlap(
        upper_half,
        lower_half,
        axes="yz",
        min_overlap=0.006,
        elem_a=upper_hinge_barrel_left,
        elem_b=lower_hinge_barrel_center,
    )
    ctx.expect_overlap(
        upper_half,
        lower_half,
        axes="yz",
        min_overlap=0.006,
        elem_a=upper_hinge_barrel_left,
        elem_b=lower_hinge_barrel_left,
    )
    ctx.expect_overlap(
        upper_half,
        lower_half,
        axes="yz",
        min_overlap=0.006,
        elem_a=upper_hinge_barrel_right,
        elem_b=lower_hinge_barrel_right,
    )
    with ctx.pose({fold_hinge: 1.7}):
        ctx.expect_overlap(
            upper_half,
            lower_half,
            axes="yz",
            min_overlap=0.006,
            elem_a=upper_hinge_barrel_left,
            elem_b=lower_hinge_barrel_center,
        )
        ctx.expect_overlap(
            upper_half,
            lower_half,
            axes="yz",
            min_overlap=0.006,
            elem_a=upper_hinge_barrel_right,
            elem_b=lower_hinge_barrel_right,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
