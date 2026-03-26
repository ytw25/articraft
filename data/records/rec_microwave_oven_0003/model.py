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
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

BODY_W = 0.54
BODY_D = 0.42
BODY_H = 0.32
FOOT_H = 0.010
BODY_CZ = FOOT_H + (BODY_H * 0.5)
WALL_T = 0.018

OPEN_W = 0.35
OPEN_H = 0.22
OPEN_CX = -0.075
OPEN_CZ = BODY_CZ
OPEN_LEFT = OPEN_CX - (OPEN_W * 0.5)

DOOR_W = 0.356
DOOR_H = 0.232
DOOR_T = 0.025
HINGE_AXIS_X = OPEN_LEFT - 0.010
HINGE_AXIS_Y = (BODY_D * 0.5) + 0.0015
HINGE_OFFSET_X = 0.010
HINGE_BARREL_R = 0.004
HINGE_BARREL_L = 0.048


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _span(aabb, axis: int) -> float:
    return float(aabb[1][axis] - aabb[0][axis])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_microwave", assets=ASSETS)

    enamel = model.material("enamel", rgba=(0.90, 0.91, 0.93, 1.0))
    cavity_enamel = model.material("cavity_enamel", rgba=(0.82, 0.84, 0.86, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    black_trim = model.material("black_trim", rgba=(0.08, 0.09, 0.10, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.30, 0.35, 0.38, 0.42))
    display_glass = model.material("display_glass", rgba=(0.32, 0.45, 0.50, 0.35))
    foot_rubber = model.material("foot_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    housing = model.part("housing")
    housing.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, BODY_CZ)),
    )

    outer_shell = BoxGeometry((BODY_W, BODY_D, BODY_H)).translate(0.0, 0.0, BODY_CZ)
    cavity_cut = BoxGeometry((OPEN_W, BODY_D - WALL_T + 0.006, OPEN_H)).translate(
        OPEN_CX,
        (WALL_T + 0.006) * 0.5,
        OPEN_CZ,
    )
    housing.visual(
        _save_mesh("microwave_housing_shell.obj", boolean_difference(outer_shell, cavity_cut)),
        material=enamel,
        name="housing_shell",
    )
    housing.visual(
        Box((0.118, 0.006, 0.248)),
        origin=Origin(xyz=(0.196, (BODY_D * 0.5) + 0.003, BODY_CZ)),
        material=panel_dark,
        name="control_bezel",
    )
    housing.visual(
        Box((0.085, 0.0035, 0.045)),
        origin=Origin(xyz=(0.196, (BODY_D * 0.5) + 0.00775, BODY_CZ + 0.066)),
        material=display_glass,
        name="display_window",
    )
    housing.visual(
        Box((0.090, 0.0025, 0.132)),
        origin=Origin(xyz=(0.196, (BODY_D * 0.5) + 0.00725, BODY_CZ - 0.010)),
        material=black_trim,
        name="keypad_panel",
    )
    for index, z_pos in enumerate((BODY_CZ + 0.018, BODY_CZ - 0.010, BODY_CZ - 0.038, BODY_CZ - 0.066)):
        housing.visual(
            Box((0.070, 0.003, 0.016)),
            origin=Origin(xyz=(0.196, (BODY_D * 0.5) + 0.0075, z_pos)),
            material=enamel,
            name=f"button_strip_{index}",
        )
    housing.visual(
        Cylinder(radius=0.019, length=0.008),
        origin=Origin(
            xyz=(0.196, (BODY_D * 0.5) + 0.010, BODY_CZ - 0.102),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=black_trim,
        name="timer_knob",
    )
    housing.visual(
        Box((0.006, 0.018, 0.050)),
        origin=Origin(xyz=(HINGE_AXIS_X - 0.007, HINGE_AXIS_Y - 0.006, OPEN_CZ + 0.078)),
        material=panel_dark,
        name="hinge_mount_upper",
    )
    housing.visual(
        Box((0.006, 0.018, 0.050)),
        origin=Origin(xyz=(HINGE_AXIS_X - 0.007, HINGE_AXIS_Y - 0.006, OPEN_CZ - 0.078)),
        material=panel_dark,
        name="hinge_mount_lower",
    )
    housing.visual(
        Cylinder(radius=0.110, length=0.004),
        origin=Origin(xyz=(OPEN_CX, -0.030, OPEN_CZ - (OPEN_H * 0.5) + 0.002)),
        material=smoked_glass,
        name="turntable",
    )
    for x_pos in (-0.195, 0.195):
        for y_pos in (-0.135, 0.135):
            housing.visual(
                Box((0.040, 0.035, FOOT_H)),
                origin=Origin(xyz=(x_pos, y_pos, FOOT_H * 0.5)),
                material=foot_rubber,
            )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        mass=2.0,
        origin=Origin(xyz=(HINGE_OFFSET_X + (DOOR_W * 0.5), DOOR_T * 0.5, 0.0)),
    )

    door_frame_mesh = ExtrudeWithHolesGeometry(
        rounded_rect_profile(DOOR_W, DOOR_H, radius=0.010, corner_segments=6),
        [rounded_rect_profile(0.240, 0.150, radius=0.006, corner_segments=6)],
        height=DOOR_T,
        center=True,
    ).rotate_x(math.pi * 0.5)
    door.visual(
        _save_mesh("microwave_door_frame.obj", door_frame_mesh),
        origin=Origin(xyz=(HINGE_OFFSET_X + (DOOR_W * 0.5), DOOR_T * 0.5, 0.0)),
        material=black_trim,
        name="door_frame",
    )
    door.visual(
        Box((0.252, 0.006, 0.162)),
        origin=Origin(xyz=(HINGE_OFFSET_X + (DOOR_W * 0.5), 0.011, 0.0)),
        material=smoked_glass,
        name="window_glass",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.160),
        origin=Origin(
            xyz=(HINGE_OFFSET_X + DOOR_W - 0.030, 0.036, 0.0),
        ),
        material=panel_dark,
        name="handle_bar",
    )
    door.visual(
        Box((0.012, 0.012, 0.030)),
        origin=Origin(xyz=(HINGE_OFFSET_X + DOOR_W - 0.030, 0.030, 0.052)),
        material=panel_dark,
        name="handle_post_upper",
    )
    door.visual(
        Box((0.012, 0.012, 0.030)),
        origin=Origin(xyz=(HINGE_OFFSET_X + DOOR_W - 0.030, 0.030, -0.052)),
        material=panel_dark,
        name="handle_post_lower",
    )
    door.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_L),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=panel_dark,
        name="hinge_barrel_upper",
    )
    door.visual(
        Cylinder(radius=HINGE_BARREL_R, length=HINGE_BARREL_L),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=panel_dark,
        name="hinge_barrel_lower",
    )
    door.visual(
        Box((0.012, 0.008, 0.034)),
        origin=Origin(xyz=(0.010, 0.004, 0.078)),
        material=panel_dark,
        name="hinge_leaf_upper",
    )
    door.visual(
        Box((0.012, 0.008, 0.034)),
        origin=Origin(xyz=(0.010, 0.004, -0.078)),
        material=panel_dark,
        name="hinge_leaf_lower",
    )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, OPEN_CZ)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(100.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("housing_to_door")
    housing_shell = housing.get_visual("housing_shell")
    control_bezel = housing.get_visual("control_bezel")
    hinge_mount_upper = housing.get_visual("hinge_mount_upper")
    hinge_mount_lower = housing.get_visual("hinge_mount_lower")
    door_frame = door.get_visual("door_frame")
    window_glass = door.get_visual("window_glass")
    handle_bar = door.get_visual("handle_bar")
    hinge_barrel_upper = door.get_visual("hinge_barrel_upper")
    hinge_barrel_lower = door.get_visual("hinge_barrel_lower")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=16)

    ctx.expect_gap(
        door,
        housing,
        axis="y",
        min_gap=0.001,
        max_gap=0.003,
        positive_elem=door_frame,
        negative_elem=housing_shell,
        name="door_sits_just_proud_of_front_shell",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="z",
        min_overlap=0.20,
        elem_a=door_frame,
        elem_b=housing_shell,
        name="door_spans_cavity_height",
    )
    ctx.expect_contact(
        door,
        housing,
        contact_tol=0.0015,
        elem_a=hinge_barrel_upper,
        elem_b=hinge_mount_upper,
        name="upper_hinge_barrel_meets_mount",
    )
    ctx.expect_contact(
        door,
        housing,
        contact_tol=0.0015,
        elem_a=hinge_barrel_lower,
        elem_b=hinge_mount_lower,
        name="lower_hinge_barrel_meets_mount",
    )
    ctx.expect_within(
        door,
        housing,
        axes="z",
        margin=0.06,
        inner_elem=window_glass,
        outer_elem=housing_shell,
        name="window_stays_within_front_aperture_band",
    )

    with ctx.pose({door_hinge: math.radians(100.0)}):
        ctx.expect_gap(
            door,
            housing,
            axis="y",
            min_gap=0.005,
            positive_elem=door_frame,
            negative_elem=housing_shell,
            name="open_door_stays_forward_of_body",
        )

    housing_aabb = ctx.part_world_aabb(housing)
    door_aabb = ctx.part_element_world_aabb(door, elem=door_frame)
    control_aabb = ctx.part_element_world_aabb(housing, elem=control_bezel)
    handle_aabb = ctx.part_element_world_aabb(door, elem=handle_bar)
    ctx.check("housing_exists", housing_aabb is not None, "housing AABB missing")
    ctx.check("door_exists", door_aabb is not None, "door frame AABB missing")
    ctx.check("control_panel_exists", control_aabb is not None, "control bezel AABB missing")
    ctx.check("handle_exists", handle_aabb is not None, "door handle AABB missing")
    if housing_aabb is not None:
        ctx.check(
            "housing_proportions",
            0.50 <= _span(housing_aabb, 0) <= 0.57
            and 0.40 <= _span(housing_aabb, 1) <= 0.44
            and 0.32 <= _span(housing_aabb, 2) <= 0.34,
            f"housing spans were {_span(housing_aabb, 0):.3f}, {_span(housing_aabb, 1):.3f}, {_span(housing_aabb, 2):.3f}",
        )
    if door_aabb is not None:
        ctx.check(
            "door_panel_proportions",
            0.34 <= _span(door_aabb, 0) <= 0.37 and 0.22 <= _span(door_aabb, 2) <= 0.24,
            f"door spans were {_span(door_aabb, 0):.3f}, {_span(door_aabb, 2):.3f}",
        )
    ctx.check(
        "door_hinge_axis_is_vertical",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        f"door hinge axis was {door_hinge.axis}",
    )
    ctx.check(
        "door_hinge_opens_about_100_deg",
        abs(door_hinge.motion_limits.lower - 0.0) < 1e-9
        and abs(door_hinge.motion_limits.upper - math.radians(100.0)) < 1e-6,
        f"door hinge limits were {door_hinge.motion_limits}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
