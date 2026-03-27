from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.79, 0.80, 0.82, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.21, 0.22, 0.24, 1.0))
    control_black = model.material("control_black", rgba=(0.11, 0.11, 0.12, 1.0))
    knob_marker = model.material("knob_marker", rgba=(0.82, 0.22, 0.16, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.50
    canopy_height = 0.14
    shell_t = 0.012

    chimney_width = 0.32
    chimney_depth = 0.26
    chimney_height = 0.75
    chimney_y = 0.05
    chimney_panel_t = 0.010

    front_panel_t = 0.004
    front_panel_height = 0.132

    button_size = 0.026
    button_depth = 0.012
    button_travel = 0.004
    button_front_proud = 0.003
    button_y = -canopy_depth / 2 + button_depth / 2 - button_front_proud

    knob_radius = 0.018
    knob_length = 0.018
    knob_y = -0.105
    knob_z = 0.074

    def rect_loop(cx: float, cy: float, sx: float, sy: float) -> list[tuple[float, float]]:
        hx = sx / 2
        hy = sy / 2
        return [
            (cx - hx, cy - hy),
            (cx + hx, cy - hy),
            (cx + hx, cy + hy),
            (cx - hx, cy + hy),
        ]

    control_panel_outer = rect_loop(0.0, front_panel_height / 2, canopy_width - 2 * shell_t, front_panel_height)
    button_x_positions = (0.058, 0.094, 0.130, 0.166)
    button_z_positions = (0.048, 0.055, 0.062, 0.069)
    control_panel_holes = [
        rect_loop(button_x, button_z, button_size, button_size)
        for button_x, button_z in zip(button_x_positions, button_z_positions)
    ]

    control_panel_geom = ExtrudeWithHolesGeometry(
        control_panel_outer,
        control_panel_holes,
        front_panel_t,
        cap=True,
        center=True,
        closed=True,
    )
    control_panel_geom.rotate_x(pi / 2)
    control_panel_mesh: Mesh = mesh_from_geometry(
        control_panel_geom,
        Path(ASSETS.asset_root) / "range_hood_front_panel.obj",
    )

    body = model.part("body")
    body.visual(
        Box((canopy_width, canopy_depth, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - shell_t / 2)),
        material=stainless,
        name="canopy_top",
    )
    body.visual(
        Box((canopy_width, shell_t, canopy_height)),
        origin=Origin(xyz=(0.0, canopy_depth / 2 - shell_t / 2, canopy_height / 2)),
        material=stainless,
        name="back_panel",
    )
    body.visual(
        Box((shell_t, canopy_depth, canopy_height)),
        origin=Origin(xyz=(-canopy_width / 2 + shell_t / 2, 0.0, canopy_height / 2)),
        material=stainless,
        name="left_panel",
    )
    body.visual(
        Box((shell_t, canopy_depth, canopy_height)),
        origin=Origin(xyz=(canopy_width / 2 - shell_t / 2, 0.0, canopy_height / 2)),
        material=stainless,
        name="right_panel",
    )
    body.visual(
        control_panel_mesh,
        origin=Origin(xyz=(0.0, -canopy_depth / 2 + front_panel_t / 2, 0.0)),
        material=stainless,
        name="front_panel",
    )
    body.visual(
        Box((0.76, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -canopy_depth / 2 - 0.008, 0.106)),
        material=trim_dark,
        name="front_rail",
    )
    body.visual(
        Box((0.006, 0.14, 0.11)),
        origin=Origin(xyz=(canopy_width / 2 + 0.003, -0.11, 0.075)),
        material=trim_dark,
        name="right_trim",
    )
    body.visual(
        Box((chimney_width, chimney_panel_t, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_y - chimney_depth / 2 + chimney_panel_t / 2,
                canopy_height + chimney_height / 2 - 0.004,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    body.visual(
        Box((chimney_width, chimney_panel_t, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_y + chimney_depth / 2 - chimney_panel_t / 2,
                canopy_height + chimney_height / 2 - 0.004,
            )
        ),
        material=stainless,
        name="chimney_back",
    )
    body.visual(
        Box((chimney_panel_t, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                -chimney_width / 2 + chimney_panel_t / 2,
                chimney_y,
                canopy_height + chimney_height / 2 - 0.004,
            )
        ),
        material=stainless,
        name="chimney_left",
    )
    body.visual(
        Box((chimney_panel_t, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                chimney_width / 2 - chimney_panel_t / 2,
                chimney_y,
                canopy_height + chimney_height / 2 - 0.004,
            )
        ),
        material=stainless,
        name="chimney_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((canopy_width, canopy_depth, canopy_height + chimney_height)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_height + chimney_height) / 2)),
    )

    for index, (button_x, button_z) in enumerate(zip(button_x_positions, button_z_positions), start=1):
        button = model.part(f"button_{index}")
        button.visual(
            Box((button_size, button_depth, button_size)),
            material=control_black,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((button_size, button_depth, button_size)),
            mass=0.02,
        )
        model.articulation(
            f"button_{index}_press",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, button_y, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.08,
                lower=0.0,
                upper=button_travel,
            ),
        )

    knob = model.part("knob")
    knob.visual(
        Cylinder(radius=knob_radius, length=knob_length),
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
        material=control_black,
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=trim_dark,
        name="knob_hub",
    )
    knob.visual(
        Box((0.004, 0.004, 0.012)),
        origin=Origin(xyz=(knob_length / 2, 0.0, 0.009)),
        material=knob_marker,
        name="knob_indicator",
    )
    knob.inertial = Inertial.from_geometry(
        Cylinder(radius=knob_radius, length=knob_length),
        mass=0.05,
        origin=Origin(rpy=(0.0, pi / 2, 0.0)),
    )
    model.articulation(
        "knob_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=knob,
        origin=Origin(xyz=(canopy_width / 2 + 0.006 + knob_length / 2, knob_y, knob_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 5)]
    button_joints = [object_model.get_articulation(f"button_{index}_press") for index in range(1, 5)]
    knob = object_model.get_part("knob")
    knob_joint = object_model.get_articulation("knob_turn")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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
        "articulation_count",
        len(object_model.articulations) == 5,
        details=f"Expected 5 articulations, found {len(object_model.articulations)}.",
    )

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is None:
        ctx.fail("body_aabb_exists", "Body AABB could not be resolved.")
    else:
        body_width = body_aabb[1][0] - body_aabb[0][0]
        body_depth = body_aabb[1][1] - body_aabb[0][1]
        body_height = body_aabb[1][2] - body_aabb[0][2]
        ctx.check(
            "body_realistic_dimensions",
            0.88 <= body_width <= 0.94 and 0.49 <= body_depth <= 0.53 and 0.86 <= body_height <= 0.92,
            details=(
                f"Expected a realistic chimney hood envelope; got "
                f"{body_width:.3f} x {body_depth:.3f} x {body_height:.3f} m."
            ),
        )

    for joint in button_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_is_prismatic",
            joint.joint_type == ArticulationType.PRISMATIC and joint.axis == (0.0, 1.0, 0.0),
            details=f"{joint.name} should be a +Y prismatic button press joint.",
        )
        ctx.check(
            f"{joint.name}_travel",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and abs(limits.lower - 0.0) < 1e-9
            and 0.0035 <= limits.upper <= 0.0045,
            details=f"{joint.name} should have about 4 mm of inward travel.",
        )

    ctx.check(
        "knob_joint_is_continuous",
        knob_joint.joint_type == ArticulationType.CONTINUOUS and knob_joint.axis == (1.0, 0.0, 0.0),
        details="The side knob should rotate continuously about its own X axis.",
    )

    button_positions = [ctx.part_world_position(button) for button in buttons]
    if any(position is None for position in button_positions):
        ctx.fail("button_positions_exist", "Could not resolve all button world positions.")
    else:
        button_positions = [position for position in button_positions if position is not None]
        xs = [position[0] for position in button_positions]
        ys = [position[1] for position in button_positions]
        zs = [position[2] for position in button_positions]
        ctx.check(
            "buttons_form_rising_front_row",
            xs == sorted(xs) and zs == sorted(zs) and all(y < -0.23 for y in ys),
            details=f"Buttons should form an upward-slanting row on the front edge, got {button_positions}.",
        )

    knob_position = ctx.part_world_position(knob)
    if knob_position is None:
        ctx.fail("knob_position_exists", "Could not resolve the knob world position.")
    else:
        ctx.check(
            "knob_on_right_trim",
            knob_position[0] > 0.45 and -0.20 < knob_position[1] < -0.02 and 0.03 < knob_position[2] < 0.11,
            details=f"Knob should sit isolated on the right-side trim, got {knob_position}.",
        )

    for index, (button, joint) in enumerate(zip(buttons, button_joints), start=1):
        limits = joint.motion_limits
        ctx.expect_contact(button, body, name=f"button_{index}_rest_contact")
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.expect_contact(button, body, name=f"button_{index}_lower_contact")
                ctx.fail_if_parts_overlap_in_current_pose(name=f"button_{index}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"button_{index}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.expect_contact(button, body, name=f"button_{index}_upper_contact")
                ctx.fail_if_parts_overlap_in_current_pose(name=f"button_{index}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"button_{index}_upper_no_floating")

    ctx.expect_contact(knob, body, name="knob_rest_contact")
    with ctx.pose({knob_joint: pi / 2}):
        ctx.expect_contact(knob, body, name="knob_quarter_turn_contact")
        ctx.fail_if_parts_overlap_in_current_pose(name="knob_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="knob_quarter_turn_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
