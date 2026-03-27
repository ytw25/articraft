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
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _superellipse_section(
    x_pos: float,
    half_width: float,
    half_height: float,
    z_center: float,
    *,
    exponent: float = 2.8,
    segments: int = 28,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    power = 2.0 / exponent
    for index in range(segments):
        angle = (2.0 * math.pi * index) / segments
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        y_pos = half_width * math.copysign(abs(cos_a) ** power, cos_a)
        z_pos = z_center + half_height * math.copysign(abs(sin_a) ** power, sin_a)
        points.append((x_pos, y_pos, z_pos))
    return points


def _build_head_shell():
    sections = [
        _superellipse_section(0.015, 0.046, 0.046, 0.020),
        _superellipse_section(0.050, 0.061, 0.058, 0.029),
        _superellipse_section(0.102, 0.059, 0.056, 0.031),
        _superellipse_section(0.150, 0.046, 0.043, 0.026),
        _superellipse_section(0.188, 0.029, 0.029, 0.015),
    ]
    return mesh_from_geometry(
        repair_loft(section_loft(sections)),
        ASSETS.mesh_dir / "artisan_mixer_head_shell.obj",
    )


def _build_bowl_shell():
    profile = [
        (0.0, 0.000),
        (0.030, 0.000),
        (0.057, 0.008),
        (0.084, 0.034),
        (0.102, 0.071),
        (0.108, 0.095),
        (0.100, 0.095),
        (0.095, 0.087),
        (0.082, 0.053),
        (0.057, 0.018),
        (0.028, 0.007),
        (0.0, 0.007),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=56),
        ASSETS.mesh_dir / "artisan_mixer_bowl_shell.obj",
    )


def _build_bowl_handle():
    handle_points = [
        (0.036, 0.094, 0.042),
        (0.044, 0.121, 0.056),
        (0.051, 0.135, 0.075),
        (0.050, 0.127, 0.091),
        (0.040, 0.098, 0.094),
    ]
    return mesh_from_geometry(
        tube_from_spline_points(
            handle_points,
            radius=0.005,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        ASSETS.mesh_dir / "artisan_mixer_bowl_handle.obj",
    )


def _build_whisk():
    loop_points = [
        (0.0, 0.0, -0.092),
        (0.015, 0.0, -0.106),
        (0.023, 0.0, -0.120),
        (0.0, 0.0, -0.136),
        (-0.023, 0.0, -0.120),
        (-0.015, 0.0, -0.106),
    ]
    whisk_geom = tube_from_spline_points(
        loop_points,
        radius=0.0018,
        samples_per_segment=18,
        closed_spline=True,
        radial_segments=12,
        cap_ends=False,
    )
    for angle in (math.pi / 4.0, math.pi / 2.0, 3.0 * math.pi / 4.0):
        whisk_geom.merge(
            tube_from_spline_points(
                loop_points,
                radius=0.0018,
                samples_per_segment=18,
                closed_spline=True,
                radial_segments=12,
                cap_ends=False,
            ).rotate_z(angle)
        )
    whisk_geom.translate(0.095, 0.0, 0.0)
    return mesh_from_geometry(
        whisk_geom,
        ASSETS.mesh_dir / "artisan_mixer_whisk.obj",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer", assets=ASSETS)

    cream = model.material("cream_enamel", rgba=(0.93, 0.90, 0.83, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.77, 0.80, 1.0))
    polished = model.material("polished_steel", rgba=(0.83, 0.85, 0.87, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.206, 0.155, 0.028)),
        origin=Origin(xyz=(-0.012, 0.0, 0.014)),
        material=cream,
        name="footprint_shell",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.155),
        origin=Origin(xyz=(0.073, 0.0, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="front_toe",
    )
    base.visual(
        Box((0.072, 0.084, 0.132)),
        origin=Origin(xyz=(-0.065, 0.0, 0.096)),
        material=cream,
        name="rear_column",
    )
    base.visual(
        Cylinder(radius=0.041, length=0.084),
        origin=Origin(xyz=(-0.065, 0.0, 0.163), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cream,
        name="column_cap",
    )
    base.visual(
        Box((0.160, 0.088, 0.022)),
        origin=Origin(xyz=(-0.005, 0.0, 0.066)),
        material=cream,
        name="support_arm",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.012),
        origin=Origin(xyz=(0.035, 0.0, 0.077)),
        material=polished,
        name="platform",
    )
    base.visual(
        Box((0.012, 0.008, 0.040)),
        origin=Origin(xyz=(-0.073, 0.036, 0.220)),
        material=cream,
        name="left_hinge_yoke",
    )
    base.visual(
        Box((0.012, 0.008, 0.040)),
        origin=Origin(xyz=(-0.073, -0.036, 0.220)),
        material=cream,
        name="right_hinge_yoke",
    )
    base.visual(
        Box((0.012, 0.050, 0.040)),
        origin=Origin(xyz=(-0.073, 0.0, 0.220)),
        material=cream,
        name="hinge_backstrap",
    )
    base.visual(
        Box((0.022, 0.010, 0.012)),
        origin=Origin(xyz=(-0.021, -0.046, 0.158), rpy=(0.15, 0.0, 0.0)),
        material=charcoal,
        name="tilt_lock",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.240, 0.160, 0.250)),
        mass=7.2,
        origin=Origin(xyz=(-0.010, 0.0, 0.120)),
    )

    bowl = model.part("bowl")
    bowl.visual(_build_bowl_shell(), material=stainless, name="bowl_shell")
    bowl.visual(
        Cylinder(radius=0.046, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=polished,
        name="bowl_foot",
    )
    bowl.visual(_build_bowl_handle(), material=polished, name="bowl_handle")
    bowl.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.037, 0.090, 0.043), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="handle_lower_mount",
    )
    bowl.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.040, 0.092, 0.093), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="handle_upper_mount",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.108, length=0.095),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.0475)),
    )

    head = model.part("head")
    head.visual(_build_head_shell(), material=cream, name="head_shell")
    head.visual(
        Box((0.126, 0.082, 0.012)),
        origin=Origin(xyz=(0.099, 0.0, -0.017)),
        material=charcoal,
        name="underside_trim",
    )
    head.visual(
        Cylinder(radius=0.015, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="hinge_boss",
    )
    head.visual(
        Box((0.014, 0.112, 0.020)),
        origin=Origin(xyz=(0.046, 0.0, -0.006)),
        material=polished,
        name="trim_band",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.173, 0.0, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="attachment_hub",
    )
    head.visual(
        Box((0.030, 0.032, 0.028)),
        origin=Origin(xyz=(0.157, 0.0, -0.006)),
        material=polished,
        name="hub_mount",
    )
    head.visual(
        Box((0.028, 0.008, 0.012)),
        origin=Origin(xyz=(0.071, 0.056, 0.024), rpy=(0.0, 0.0, -0.18)),
        material=charcoal,
        name="speed_lever",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.062),
        origin=Origin(xyz=(0.095, 0.0, -0.067)),
        material=polished,
        name="whisk_shaft",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.026),
        origin=Origin(xyz=(0.095, 0.0, -0.024)),
        material=polished,
        name="tool_hub",
    )
    head.visual(_build_whisk(), material=polished, name="whisk")
    head.inertial = Inertial.from_geometry(
        Box((0.210, 0.120, 0.120)),
        mass=3.1,
        origin=Origin(xyz=(0.104, 0.0, 0.028)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.035, 0.0, 0.083)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.052, 0.0, 0.230)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    head_tilt = object_model.get_articulation("base_to_head")

    platform = base.get_visual("platform")
    left_hinge_yoke = base.get_visual("left_hinge_yoke")
    right_hinge_yoke = base.get_visual("right_hinge_yoke")
    bowl_shell = bowl.get_visual("bowl_shell")
    bowl_foot = bowl.get_visual("bowl_foot")
    bowl_handle = bowl.get_visual("bowl_handle")
    head_boss = head.get_visual("hinge_boss")
    whisk = head.get_visual("whisk")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=bowl_foot,
        negative_elem=platform,
        name="bowl sits on the round pedestal",
    )
    ctx.expect_overlap(bowl, base, axes="xy", min_overlap=0.05)
    ctx.expect_within(
        bowl,
        base,
        axes="xy",
        inner_elem=bowl_foot,
        outer_elem=platform,
    )
    ctx.expect_gap(
        bowl,
        base,
        axis="y",
        min_gap=0.03,
        positive_elem=bowl_handle,
        negative_elem=platform,
        name="side handle projects beyond the pedestal",
    )
    ctx.expect_contact(head, base, elem_a=head_boss, elem_b=left_hinge_yoke)
    ctx.expect_contact(head, base, elem_a=head_boss, elem_b=right_hinge_yoke)
    ctx.expect_overlap(head, bowl, axes="xy", min_overlap=0.055)
    ctx.expect_origin_distance(head, bowl, axes="y", max_dist=0.02)
    ctx.expect_within(
        head,
        bowl,
        axes="xy",
        inner_elem=whisk,
        outer_elem=bowl_shell,
    )
    ctx.expect_gap(
        head,
        bowl,
        axis="z",
        min_gap=-0.090,
        max_gap=-0.030,
        positive_elem=whisk,
        negative_elem=bowl_shell,
        name="whisk descends into the bowl at rest",
    )
    with ctx.pose({head_tilt: 1.0}):
        ctx.expect_contact(head, base, elem_a=head_boss, elem_b=left_hinge_yoke)
        ctx.expect_contact(head, base, elem_a=head_boss, elem_b=right_hinge_yoke)
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            min_gap=0.040,
            positive_elem=whisk,
            negative_elem=bowl_shell,
            name="whisk clears the bowl when the head is tilted up",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
