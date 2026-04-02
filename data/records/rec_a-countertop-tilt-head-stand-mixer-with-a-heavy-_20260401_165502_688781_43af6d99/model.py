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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_head_stand_mixer")

    body_paint = model.material("body_paint", rgba=(0.74, 0.12, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.90, 0.90, 0.92, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.13, 1.0))

    def xy_section(
        width_x: float,
        width_y: float,
        radius: float,
        z: float,
        *,
        cx: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(cx + x, y, z) for x, y in rounded_rect_profile(width_x, width_y, radius)]

    def yz_section(
        width_y: float,
        height_z: float,
        radius: float,
        x: float,
        *,
        z_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z_center + z) for z, y in rounded_rect_profile(height_z, width_y, radius)]

    pedestal = model.part("pedestal")

    base_geom = ExtrudeGeometry(rounded_rect_profile(0.32, 0.22, 0.055), 0.035)
    pedestal.visual(
        mesh_from_geometry(base_geom, "pedestal_base"),
        origin=Origin(xyz=(0.03, 0.0, 0.0175)),
        material=body_paint,
        name="base_shell",
    )

    column_geom = section_loft(
        [
            xy_section(0.115, 0.12, 0.030, 0.035, cx=-0.075),
            xy_section(0.090, 0.10, 0.026, 0.185, cx=-0.072),
            xy_section(0.050, 0.070, 0.020, 0.288, cx=-0.055),
        ]
    )
    pedestal.visual(
        mesh_from_geometry(column_geom, "pedestal_column"),
        material=body_paint,
        name="column_shell",
    )

    cradle_points = [
        (-0.006, -0.055, 0.145),
        (0.020, -0.076, 0.136),
        (0.070, -0.095, 0.119),
        (0.130, -0.090, 0.111),
        (0.170, -0.055, 0.109),
        (0.180, 0.000, 0.108),
        (0.170, 0.055, 0.109),
        (0.130, 0.090, 0.111),
        (0.070, 0.095, 0.119),
        (0.020, 0.076, 0.136),
        (-0.006, 0.055, 0.145),
    ]
    cradle_geom = tube_from_spline_points(
        cradle_points,
        radius=0.007,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    pedestal.visual(
        mesh_from_geometry(cradle_geom, "cradle_frame"),
        material=body_paint,
        name="cradle_frame",
    )
    for side, y_sign in (("left", -1.0), ("right", 1.0)):
        brace_geom = tube_from_spline_points(
            [
                (-0.040, 0.030 * y_sign, 0.190),
                (-0.024, 0.045 * y_sign, 0.172),
                (-0.006, 0.055 * y_sign, 0.145),
            ],
            radius=0.006,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        )
        pedestal.visual(
            mesh_from_geometry(brace_geom, f"cradle_brace_{side}"),
            material=body_paint,
            name=f"cradle_brace_{side}",
        )
    pedestal.visual(
        Cylinder(radius=0.028, length=0.066),
        origin=Origin(xyz=(0.090, 0.0, 0.068)),
        material=body_paint,
        name="bowl_support_pillar",
    )
    pedestal.visual(
        Cylinder(radius=0.066, length=0.006),
        origin=Origin(xyz=(0.090, 0.0, 0.104)),
        material=body_paint,
        name="bowl_support_plate",
    )
    for side, y_pos in (("left", -0.036), ("right", 0.036)):
        pedestal.visual(
            Box((0.026, 0.020, 0.015)),
            origin=Origin(xyz=(-0.050, y_pos, 0.2945)),
            material=body_paint,
            name=f"hinge_cheek_{side}",
        )
        pedestal.visual(
            Cylinder(radius=0.017, length=0.022),
            origin=Origin(xyz=(-0.050, y_pos, 0.312), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=body_paint,
            name=f"hinge_lug_{side}",
        )

    pedestal.inertial = Inertial.from_geometry(
        Box((0.32, 0.22, 0.32)),
        mass=8.5,
        origin=Origin(xyz=(0.02, 0.0, 0.16)),
    )

    bowl = model.part("bowl")
    bowl_geom = LatheGeometry.from_shell_profiles(
        [
            (0.000, 0.000),
            (0.034, 0.006),
            (0.078, 0.032),
            (0.104, 0.086),
            (0.113, 0.145),
            (0.118, 0.151),
        ],
        [
            (0.000, 0.006),
            (0.028, 0.012),
            (0.072, 0.036),
            (0.096, 0.089),
            (0.103, 0.146),
        ],
        segments=64,
        lip_samples=10,
    )
    bowl.visual(
        mesh_from_geometry(bowl_geom, "mixing_bowl"),
        material=steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.118, length=0.151),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0755)),
    )

    model.articulation(
        "pedestal_to_bowl",
        ArticulationType.FIXED,
        parent=pedestal,
        child=bowl,
        origin=Origin(xyz=(0.090, 0.0, 0.107)),
    )

    head = model.part("head")
    head_geom = section_loft(
        [
            yz_section(0.074, 0.090, 0.022, 0.030, z_center=0.045),
            yz_section(0.145, 0.160, 0.045, 0.125, z_center=0.054),
            yz_section(0.140, 0.152, 0.042, 0.235, z_center=0.042),
            yz_section(0.100, 0.108, 0.028, 0.330, z_center=0.022),
        ]
    )
    head.visual(
        mesh_from_geometry(head_geom, "motor_head"),
        material=body_paint,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.050),
        origin=Origin(xyz=(0.000, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_paint,
        name="head_hinge_barrel",
    )
    head.visual(
        Box((0.015, 0.030, 0.030)),
        origin=Origin(xyz=(0.0225, 0.0, 0.0)),
        material=body_paint,
        name="hinge_web",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.026),
        origin=Origin(xyz=(0.140, 0.0, -0.040)),
        material=dark_trim,
        name="attachment_nose",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.036),
        origin=Origin(xyz=(0.140, 0.0, -0.010)),
        material=dark_trim,
        name="drive_collar",
    )
    head.visual(
        Box((0.030, 0.012, 0.018)),
        origin=Origin(xyz=(0.098, 0.074, 0.008)),
        material=dark_trim,
        name="lever_mount_boss",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.32, 0.15, 0.17)),
        mass=4.6,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_head",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=head,
        origin=Origin(xyz=(-0.050, 0.0, 0.312)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(63.0),
        ),
    )

    beater = model.part("beater")
    beater.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="beater_coupler",
    )
    beater.visual(
        Cylinder(radius=0.007, length=0.057),
        origin=Origin(xyz=(0.0, 0.0, -0.0465)),
        material=steel,
        name="beater_shaft",
    )
    beater.visual(
        Box((0.050, 0.006, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
        material=steel,
        name="beater_top_bar",
    )
    beater.visual(
        Box((0.008, 0.006, 0.058)),
        origin=Origin(xyz=(-0.018, 0.0, -0.102)),
        material=steel,
        name="beater_inner_leg",
    )
    beater.visual(
        Box((0.008, 0.006, 0.058)),
        origin=Origin(xyz=(0.018, 0.0, -0.102)),
        material=steel,
        name="beater_outer_leg",
    )
    beater.visual(
        Box((0.040, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.129)),
        material=steel,
        name="beater_bottom_bar",
    )
    beater.visual(
        Box((0.030, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.101), rpy=(0.0, math.radians(28.0), 0.0)),
        material=steel,
        name="beater_diagonal",
    )
    beater.inertial = Inertial.from_geometry(
        Box((0.06, 0.02, 0.16)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.140, 0.0, -0.053)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="lever_pivot",
    )
    speed_lever.visual(
        Box((0.028, 0.010, 0.010)),
        origin=Origin(xyz=(0.014, 0.011, 0.004)),
        material=dark_trim,
        name="lever_handle",
    )
    speed_lever.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.031, 0.013, 0.0115), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="lever_grip",
    )
    speed_lever.inertial = Inertial.from_geometry(
        Box((0.04, 0.02, 0.02)),
        mass=0.05,
        origin=Origin(xyz=(0.016, 0.010, 0.006)),
    )

    model.articulation(
        "head_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=head,
        child=speed_lever,
        origin=Origin(xyz=(0.105, 0.080, 0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=math.radians(-18.0),
            upper=math.radians(22.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    speed_lever = object_model.get_part("speed_lever")
    head_tilt = object_model.get_articulation("pedestal_to_head")
    beater_spin = object_model.get_articulation("head_to_beater")
    lever_pivot = object_model.get_articulation("head_to_speed_lever")

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

    ctx.expect_contact(
        bowl,
        pedestal,
        elem_b="bowl_support_plate",
        name="bowl sits on the support plate",
    )
    ctx.expect_overlap(
        bowl,
        pedestal,
        axes="xy",
        elem_b="cradle_frame",
        min_overlap=0.10,
        name="bowl stays nested in the front cradle",
    )

    with ctx.pose({head_tilt: 0.0}):
        ctx.expect_within(
            beater,
            bowl,
            axes="xy",
            margin=0.012,
            name="beater stays centered within the bowl opening",
        )
        ctx.expect_overlap(
            beater,
            bowl,
            axes="xy",
            min_overlap=0.015,
            name="beater hangs down into the bowl footprint",
        )

    closed_head_shell = ctx.part_element_world_aabb(head, elem="head_shell")
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        opened_head_shell = ctx.part_element_world_aabb(head, elem="head_shell")
        lifted_beater = ctx.part_world_aabb(beater)
        bowl_aabb = ctx.part_world_aabb(bowl)
    ctx.check(
        "head tilts upward",
        closed_head_shell is not None
        and opened_head_shell is not None
        and opened_head_shell[1][2] > closed_head_shell[1][2] + 0.06,
        details=f"closed={closed_head_shell}, opened={opened_head_shell}",
    )
    ctx.check(
        "open head lifts the beater clear of the bowl rim",
        lifted_beater is not None
        and bowl_aabb is not None
        and lifted_beater[0][2] > bowl_aabb[1][2] + 0.012,
        details=f"beater={lifted_beater}, bowl={bowl_aabb}",
    )

    closed_lever = ctx.part_element_world_aabb(speed_lever, elem="lever_handle")
    with ctx.pose({lever_pivot: lever_pivot.motion_limits.upper}):
        opened_lever = ctx.part_element_world_aabb(speed_lever, elem="lever_handle")
    ctx.check(
        "speed lever swings on its side pivot",
        closed_lever is not None
        and opened_lever is not None
        and (
            abs((opened_lever[0][0] + opened_lever[1][0]) - (closed_lever[0][0] + closed_lever[1][0])) > 0.004
            or abs((opened_lever[0][2] + opened_lever[1][2]) - (closed_lever[0][2] + closed_lever[1][2])) > 0.008
        ),
        details=f"closed={closed_lever}, opened={opened_lever}",
    )

    ctx.check(
        "head hinge axis is lateral",
        all(abs(a - b) < 1e-9 for a, b in zip(head_tilt.axis, (0.0, -1.0, 0.0))),
        details=f"axis={head_tilt.axis}",
    )
    ctx.check(
        "beater spindle is a continuous vertical drive",
        beater_spin.articulation_type == ArticulationType.CONTINUOUS
        and all(abs(a - b) < 1e-9 for a, b in zip(beater_spin.axis, (0.0, 0.0, 1.0))),
        details=(
            f"type={beater_spin.articulation_type}, axis={beater_spin.axis}, "
            f"limits={beater_spin.motion_limits}"
        ),
    )
    ctx.check(
        "speed lever uses an outward side pivot",
        all(abs(a - b) < 1e-9 for a, b in zip(lever_pivot.axis, (0.0, 1.0, 0.0))),
        details=f"axis={lever_pivot.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
