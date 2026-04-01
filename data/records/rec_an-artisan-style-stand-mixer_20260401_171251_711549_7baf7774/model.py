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
    CylinderGeometry,
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
    model = ArticulatedObject(name="artisan_stand_mixer")

    enamel = model.material("enamel", rgba=(0.77, 0.12, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.90, 0.90, 0.92, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.12, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.36, 0.235, 0.055), 0.045),
            "base_plate",
        ),
        origin=Origin(xyz=(0.078, 0.0, 0.0225)),
        material=enamel,
        name="base_plate",
    )
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry(rounded_rect_profile(0.12, 0.135, 0.03), 0.028),
            "bowl_pedestal",
        ),
        origin=Origin(xyz=(0.110, 0.0, 0.058)),
        material=enamel,
        name="bowl_pedestal",
    )
    base.visual(
        mesh_from_geometry(
            section_loft(
                [
                    [(x, y, 0.0) for x, y in rounded_rect_profile(0.11, 0.13, 0.032)],
                    [(x, y, 0.17) for x, y in rounded_rect_profile(0.09, 0.11, 0.030)],
                    [(x, y, 0.34) for x, y in rounded_rect_profile(0.065, 0.085, 0.026)],
                ]
            ),
            "mixer_neck",
        ),
        origin=Origin(xyz=(-0.065, 0.0, 0.044)),
        material=enamel,
        name="neck_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.235, 0.39)),
        mass=7.5,
        origin=Origin(xyz=(0.04, 0.0, 0.195)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                [
                    (0.022, 0.0),
                    (0.055, 0.010),
                    (0.102, 0.060),
                    (0.108, 0.145),
                    (0.112, 0.162),
                ],
                [
                    (0.000, 0.005),
                    (0.048, 0.014),
                    (0.095, 0.060),
                    (0.101, 0.156),
                ],
                segments=56,
                end_cap="round",
                lip_samples=8,
            ),
            "mixing_bowl",
        ),
        material=steel,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.112, length=0.162),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.120, 0.0, 0.072)),
    )

    head = model.part("head")

    def yz_section(width: float, height: float, radius: float, x_pos: float) -> list[tuple[float, float, float]]:
        z_bias = 0.09 - 0.05 * (x_pos / 0.355)
        return [(x_pos, y, z + z_bias) for z, y in rounded_rect_profile(height, width, radius)]

    head.visual(
        mesh_from_geometry(
            section_loft(
                [
                    yz_section(0.086, 0.102, 0.032, 0.0),
                    yz_section(0.150, 0.178, 0.055, 0.150),
                    yz_section(0.128, 0.150, 0.050, 0.285),
                    yz_section(0.086, 0.102, 0.036, 0.355),
                ]
            ),
            "head_shell",
        ),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Box((0.055, 0.070, 0.026)),
        origin=Origin(xyz=(0.022, 0.0, 0.013)),
        material=enamel,
        name="hinge_block",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.355, 0.178, 0.178)),
        mass=4.6,
        origin=Origin(xyz=(0.1775, 0.0, 0.045)),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.042, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=dark_trim,
        name="hub_shell",
    )
    hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.042, length=0.052),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
    )

    whisk = model.part("whisk")
    whisk_geom = CylinderGeometry(radius=0.0055, height=0.045).translate(0.0, 0.0, -0.0225)
    whisk_geom.merge(CylinderGeometry(radius=0.0085, height=0.026).translate(0.0, 0.0, -0.050))
    whisk_geom.merge(CylinderGeometry(radius=0.0125, height=0.030).translate(0.0, 0.0, -0.074))
    for i in range(8):
        angle = i * math.pi / 8.0
        c = math.cos(angle)
        s = math.sin(angle)
        whisk_geom.merge(
            tube_from_spline_points(
                [
                    (0.007 * c, 0.007 * s, -0.057),
                    (0.020 * c, 0.020 * s, -0.076),
                    (0.034 * c, 0.034 * s, -0.104),
                    (0.046 * c, 0.046 * s, -0.133),
                    (0.0, 0.0, -0.156),
                    (-0.046 * c, -0.046 * s, -0.133),
                    (-0.034 * c, -0.034 * s, -0.104),
                    (-0.020 * c, -0.020 * s, -0.076),
                    (-0.007 * c, -0.007 * s, -0.057),
                ],
                radius=0.0017,
                samples_per_segment=16,
                radial_segments=18,
            )
        )
    whisk.visual(
        mesh_from_geometry(whisk_geom, "balloon_whisk"),
        material=steel,
        name="whisk_shell",
    )
    whisk.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.160),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.065, 0.0, 0.384)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "head_to_hub",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=hub,
        origin=Origin(xyz=(0.182, 0.0, -0.027)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=18.0),
    )
    model.articulation(
        "hub_to_whisk",
        ArticulationType.FIXED,
        parent=hub,
        child=whisk,
        origin=Origin(xyz=(0.0, 0.0, -0.052)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    hub = object_model.get_part("hub")
    whisk = object_model.get_part("whisk")
    tilt_joint = object_model.get_articulation("base_to_head")

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

    ctx.expect_overlap(
        bowl,
        base,
        axes="xy",
        min_overlap=0.10,
        elem_b="bowl_pedestal",
        name="bowl footprint sits on the mixer base",
    )
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.004,
        max_penetration=0.001,
        negative_elem="bowl_pedestal",
        name="bowl seats onto the pedestal without floating",
    )

    with ctx.pose({tilt_joint: 0.0}):
        ctx.expect_overlap(
            whisk,
            bowl,
            axes="xy",
            min_overlap=0.08,
            name="closed whisk stays centered over the bowl",
        )
        ctx.expect_within(
            hub,
            head,
            axes="xy",
            margin=0.002,
            inner_elem="hub_shell",
            outer_elem="head_shell",
            name="hub remains centered under the head shell",
        )

    whisk_rest = ctx.part_world_position(whisk)
    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper or math.radians(60.0)}):
        whisk_raised = ctx.part_world_position(whisk)
        ctx.expect_gap(
            whisk,
            bowl,
            axis="z",
            min_gap=0.045,
            name="tilted head lifts the whisk clear of the bowl",
        )

    ctx.check(
        "head tilt raises the whisk",
        whisk_rest is not None
        and whisk_raised is not None
        and whisk_raised[2] > whisk_rest[2] + 0.09,
        details=f"rest={whisk_rest}, raised={whisk_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
