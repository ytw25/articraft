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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_bluetooth_speaker")

    fabric = model.material("fabric", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.13, 0.14, 0.15, 1.0))
    light_button = model.material("light_button", rgba=(0.82, 0.84, 0.86, 1.0))

    body_radius = 0.067
    body_length = 0.276
    endcap_radius = 0.074
    endcap_length = 0.022
    hinge_z = 0.086
    total_length = body_length + 2.0 * endcap_length

    body = model.part("body")
    body.visual(
        Cylinder(radius=body_radius, length=body_length),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fabric,
        name="main_grille",
    )

    for side in (-1.0, 1.0):
        body.visual(
            Cylinder(radius=endcap_radius, length=endcap_length),
            origin=Origin(
                xyz=(side * (body_length * 0.5 + endcap_length * 0.5), 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rubber,
            name=f"{'left' if side < 0.0 else 'right'}_endcap",
        )
        body.visual(
            Cylinder(radius=0.050, length=0.006),
            origin=Origin(
                xyz=(side * (body_length * 0.5 + endcap_length + 0.003), 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_trim,
            name=f"{'left' if side < 0.0 else 'right'}_radiator_disc",
        )
        body.visual(
            Box((0.018, 0.022, 0.040)),
            origin=Origin(
                xyz=(side * (body_length * 0.5 + 0.010), 0.0, 0.059),
            ),
            material=rubber,
            name=f"{'left' if side < 0.0 else 'right'}_bracket_riser",
        )
        body.visual(
            Box((0.014, 0.024, 0.028)),
            origin=Origin(
                xyz=(side * (total_length * 0.5 + 0.007), 0.0, hinge_z),
            ),
            material=rubber,
            name=f"{'left' if side < 0.0 else 'right'}_bracket_cheek",
        )

    body.visual(
        Box((0.154, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=dark_trim,
        name="control_pad",
    )
    body.visual(
        Box((0.060, 0.020, 0.010)),
        origin=Origin(xyz=(-0.076, 0.0, -0.066)),
        material=rubber,
        name="left_foot",
    )
    body.visual(
        Box((0.060, 0.020, 0.010)),
        origin=Origin(xyz=(0.076, 0.0, -0.066)),
        material=rubber,
        name="right_foot",
    )
    body.inertial = Inertial.from_geometry(
        Box((total_length + 0.010, 0.160, 0.190)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    handle = model.part("handle")
    handle_path = [
        (-0.143, 0.0, 0.000),
        (-0.143, 0.0, 0.022),
        (-0.139, 0.0, 0.055),
        (-0.126, 0.0, 0.082),
        (-0.105, 0.0, 0.101),
        (-0.060, 0.0, 0.108),
        (0.060, 0.0, 0.108),
        (0.105, 0.0, 0.101),
        (0.126, 0.0, 0.082),
        (0.139, 0.0, 0.055),
        (0.143, 0.0, 0.022),
        (0.143, 0.0, 0.000),
    ]
    handle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                handle_path,
                radius=0.0075,
                samples_per_segment=20,
                radial_segments=18,
                cap_ends=True,
            ),
            "speaker_handle_loop",
        ),
        material=rubber,
        name="handle_loop",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.143, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="left_pivot_barrel",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.143, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="right_pivot_barrel",
    )
    handle.visual(
        Cylinder(radius=0.011, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.104), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="grip_sleeve",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.320, 0.030, 0.135)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.10,
            upper=1.10,
        ),
    )

    button_specs = [
        ("button_power", -0.056, 0.011),
        ("button_minus", -0.028, 0.010),
        ("button_play", 0.000, 0.013),
        ("button_plus", 0.028, 0.010),
        ("button_pair", 0.056, 0.011),
    ]
    for button_name, x_pos, radius in button_specs:
        button = model.part(button_name)
        button.visual(
            Cylinder(radius=radius, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=light_button,
            name="button_cap",
        )
        button.visual(
            Cylinder(radius=radius * 0.72, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=light_button,
            name="button_stem_stub",
        )
        button.inertial = Inertial.from_geometry(
            Cylinder(radius=radius, length=0.006),
            mass=0.01,
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
        )

        model.articulation(
            f"body_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, 0.076)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.03,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    handle_joint = object_model.get_articulation("body_to_handle")

    button_names = [
        "button_power",
        "button_minus",
        "button_play",
        "button_plus",
        "button_pair",
    ]
    buttons = [object_model.get_part(name) for name in button_names]
    button_joints = [object_model.get_articulation(f"body_to_{name}") for name in button_names]

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
        "handle hinge axis follows speaker length",
        tuple(handle_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={handle_joint.axis}",
    )
    ctx.expect_contact(
        handle,
        body,
        name="handle barrels contact the side brackets",
    )

    rest_grip = ctx.part_element_world_aabb(handle, elem="grip_sleeve")
    with ctx.pose({handle_joint: handle_joint.motion_limits.lower}):
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            min_gap=0.040,
            positive_elem="grip_sleeve",
            negative_elem="main_grille",
            name="swung handle stays above the cylindrical body",
        )
        swung_grip = ctx.part_element_world_aabb(handle, elem="grip_sleeve")

    rest_grip_center_y = None if rest_grip is None else 0.5 * (rest_grip[0][1] + rest_grip[1][1])
    swung_grip_center_y = None if swung_grip is None else 0.5 * (swung_grip[0][1] + swung_grip[1][1])
    ctx.check(
        "handle rotates away from the center plane",
        rest_grip_center_y is not None
        and swung_grip_center_y is not None
        and abs(swung_grip_center_y - rest_grip_center_y) > 0.060,
        details=f"rest_y={rest_grip_center_y}, swung_y={swung_grip_center_y}",
    )

    for button, button_joint in zip(buttons, button_joints):
        ctx.check(
            f"{button.name} uses a vertical press axis",
            tuple(button_joint.axis) == (0.0, 0.0, -1.0),
            details=f"axis={button_joint.axis}",
        )
        ctx.expect_contact(
            button,
            body,
            elem_a="button_cap",
            elem_b="control_pad",
            name=f"{button.name} is seated on the top control pad",
        )

        rest_button = ctx.part_element_world_aabb(button, elem="button_cap")
        with ctx.pose({button_joint: button_joint.motion_limits.upper}):
            pressed_button = ctx.part_element_world_aabb(button, elem="button_cap")

        ctx.check(
            f"{button.name} presses downward",
            rest_button is not None
            and pressed_button is not None
            and pressed_button[1][2] < rest_button[1][2] - 0.0015,
            details=f"rest={rest_button}, pressed={pressed_button}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
