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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _superellipse_xy(
    width: float,
    depth: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    exponent: float = 2.6,
    samples: int = 40,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_d = depth * 0.5
    pts: list[tuple[float, float, float]] = []
    for idx in range(samples):
        angle = (2.0 * math.pi * idx) / samples
        c = math.cos(angle)
        s = math.sin(angle)
        x = center_x + math.copysign(abs(c) ** (2.0 / exponent), c) * half_w
        y = center_y + math.copysign(abs(s) ** (2.0 / exponent), s) * half_d
        pts.append((x, y, z))
    return pts


def _superellipse_yz(
    x: float,
    width: float,
    height: float,
    *,
    center_y: float = 0.0,
    center_z: float = 0.0,
    exponent: float = 2.4,
    samples: int = 36,
) -> list[tuple[float, float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    pts: list[tuple[float, float, float]] = []
    for idx in range(samples):
        angle = (2.0 * math.pi * idx) / samples
        c = math.cos(angle)
        s = math.sin(angle)
        y = center_y + math.copysign(abs(c) ** (2.0 / exponent), c) * half_w
        z = center_z + math.copysign(abs(s) ** (2.0 / exponent), s) * half_h
        pts.append((x, y, z))
    return pts


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_base_foot_mesh():
    return section_loft(
        [
            _superellipse_xy(0.246, 0.172, 0.000, exponent=2.8),
            _superellipse_xy(0.240, 0.168, 0.010, center_x=0.002, exponent=2.8),
            _superellipse_xy(0.226, 0.158, 0.022, center_x=0.006, exponent=2.7),
            _superellipse_xy(0.192, 0.138, 0.033, center_x=0.012, exponent=2.5),
        ]
    )


def _build_column_mesh():
    return section_loft(
        [
            _superellipse_xy(0.110, 0.098, 0.024, center_x=-0.079, exponent=2.3),
            _superellipse_xy(0.098, 0.090, 0.100, center_x=-0.071, exponent=2.3),
            _superellipse_xy(0.076, 0.078, 0.176, center_x=-0.070, exponent=2.2),
            _superellipse_xy(0.058, 0.068, 0.214, center_x=-0.082, exponent=2.1),
            _superellipse_xy(0.042, 0.058, 0.246, center_x=-0.092, exponent=2.0),
        ]
    )


def _build_head_shell_mesh():
    return section_loft(
        [
            _superellipse_yz(0.056, 0.066, 0.060, center_z=0.040, exponent=2.0),
            _superellipse_yz(0.090, 0.100, 0.116, center_z=0.034, exponent=2.4),
            _superellipse_yz(0.136, 0.118, 0.126, center_z=0.028, exponent=2.6),
            _superellipse_yz(0.176, 0.104, 0.108, center_z=0.020, exponent=2.5),
            _superellipse_yz(0.208, 0.076, 0.076, center_z=0.013, exponent=2.3),
            _superellipse_yz(0.228, 0.046, 0.048, center_z=0.010, exponent=2.0),
        ]
    )


def _build_bowl_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.036, 0.000),
            (0.054, 0.015),
            (0.069, 0.046),
            (0.078, 0.080),
            (0.084, 0.106),
        ],
        [
            (0.000, 0.006),
            (0.026, 0.010),
            (0.047, 0.025),
            (0.061, 0.054),
            (0.072, 0.086),
            (0.079, 0.102),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )


def _build_bowl_handle_mesh():
    return tube_from_spline_points(
        [
            (0.036, 0.056, 0.078),
            (0.084, 0.104, 0.075),
            (0.100, 0.109, 0.056),
            (0.097, 0.108, 0.034),
            (0.078, 0.092, 0.016),
            (0.036, 0.060, 0.018),
        ],
        radius=0.005,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )


def _build_beater_blade_mesh():
    outer_profile = [
        (-0.010, -0.022),
        (-0.018, -0.030),
        (-0.024, -0.046),
        (-0.026, -0.072),
        (-0.018, -0.094),
        (0.000, -0.106),
        (0.018, -0.094),
        (0.026, -0.072),
        (0.024, -0.046),
        (0.018, -0.030),
        (0.010, -0.022),
        (0.000, -0.026),
    ]
    inner_profile = [
        (-0.005, -0.034),
        (-0.012, -0.050),
        (-0.015, -0.072),
        (-0.009, -0.086),
        (0.000, -0.092),
        (0.009, -0.086),
        (0.015, -0.072),
        (0.012, -0.050),
        (0.005, -0.034),
        (0.000, -0.040),
    ]
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        height=0.006,
        center=True,
    )
    geom.rotate_x(math.pi * 0.5)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer", assets=ASSETS)

    enamel = model.material("enamel_red", rgba=(0.64, 0.10, 0.11, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    polished = model.material("polished_trim", rgba=(0.87, 0.88, 0.90, 1.0))
    dark = model.material("dark_gray", rgba=(0.17, 0.17, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        _save_mesh("stand_mixer_base_foot.obj", _build_base_foot_mesh()),
        material=enamel,
        name="base_foot",
    )
    base.visual(
        _save_mesh("stand_mixer_column.obj", _build_column_mesh()),
        material=enamel,
        name="rear_column",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.012),
        origin=Origin(xyz=(0.032, 0.000, 0.049)),
        material=enamel,
        name="pedestal_plate",
    )
    base.visual(
        Box((0.028, 0.032, 0.032)),
        origin=Origin(xyz=(-0.078, 0.044, 0.247)),
        material=polished,
        name="left_hinge_ear",
    )
    base.visual(
        Box((0.028, 0.032, 0.032)),
        origin=Origin(xyz=(-0.078, -0.044, 0.247)),
        material=polished,
        name="right_hinge_ear",
    )
    base.visual(
        Box((0.038, 0.072, 0.022)),
        origin=Origin(xyz=(-0.095, 0.000, 0.236)),
        material=polished,
        name="hinge_bridge",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.250, 0.180, 0.280)),
        mass=8.0,
        origin=Origin(xyz=(-0.020, 0.000, 0.140)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        _save_mesh("stand_mixer_bowl_shell.obj", _build_bowl_shell_mesh()),
        material=stainless,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=stainless,
        name="bowl_foot",
    )
    bowl.visual(
        _save_mesh("stand_mixer_bowl_handle.obj", _build_bowl_handle_mesh()),
        material=stainless,
        name="bowl_handle",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.076, length=0.104),
        mass=1.2,
        origin=Origin(xyz=(0.000, 0.000, 0.052)),
    )

    head = model.part("head")
    head.visual(
        _save_mesh("stand_mixer_head_shell.obj", _build_head_shell_mesh()),
        origin=Origin(xyz=(0.012, 0.000, 0.000)),
        material=enamel,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.056),
        origin=Origin(xyz=(0.012, 0.000, 0.000), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=polished,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.060, 0.028, 0.022)),
        origin=Origin(xyz=(0.042, 0.000, 0.023)),
        material=enamel,
        name="rear_yoke",
    )
    head.visual(
        Cylinder(radius=0.045, length=0.010),
        origin=Origin(xyz=(0.157, 0.000, -0.010), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=polished,
        name="trim_band",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.203, 0.000, -0.014), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=polished,
        name="attachment_hub",
    )
    head.visual(
        Sphere(radius=0.019),
        origin=Origin(xyz=(0.213, 0.000, -0.014)),
        material=polished,
        name="front_nose",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.022),
        origin=Origin(xyz=(0.096, 0.000, -0.039)),
        material=polished,
        name="planetary_hub",
    )
    head.visual(
        Box((0.010, 0.028, 0.006)),
        origin=Origin(xyz=(0.060, -0.036, 0.018), rpy=(0.000, 0.150, -0.350)),
        material=dark,
        name="speed_lever",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.220, 0.120, 0.130)),
        mass=4.5,
        origin=Origin(xyz=(0.097, 0.000, 0.000)),
    )

    beater = model.part("beater")
    beater.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, -0.005)),
        material=polished,
        name="beater_collar",
    )
    beater.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(0.000, 0.000, -0.018)),
        material=polished,
        name="beater_stem",
    )
    beater.visual(
        _save_mesh("stand_mixer_beater_blade.obj", _build_beater_blade_mesh()),
        origin=Origin(xyz=(0.000, 0.000, 0.006)),
        material=polished,
        name="beater_blade",
    )
    beater.visual(
        Box((0.012, 0.006, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, -0.022)),
        material=polished,
        name="beater_yoke",
    )
    beater.inertial = Inertial.from_geometry(
        Box((0.074, 0.014, 0.166)),
        mass=0.35,
        origin=Origin(xyz=(0.000, 0.000, -0.083)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.032, 0.000, 0.055)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.064, 0.000, 0.247)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-1.05,
            upper=0.0,
        ),
    )
    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.096, 0.000, -0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    head_tilt = object_model.get_articulation("base_to_head")
    beater_spin = object_model.get_articulation("head_to_beater")

    pedestal_plate = base.get_visual("pedestal_plate")
    bowl_foot = bowl.get_visual("bowl_foot")
    hinge_barrel = head.get_visual("hinge_barrel")
    left_hinge_ear = base.get_visual("left_hinge_ear")
    planetary_hub = head.get_visual("planetary_hub")
    beater_stem = beater.get_visual("beater_stem")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_coplanar_surfaces(bowl, base, reason="bowl foot seats flush on the pedestal plate")

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.fail_if_articulation_overlaps(max_pose_samples=128, overlap_tol=0.003, overlap_volume_tol=0.0)
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(bowl, base, axes="xy", min_overlap=0.040)
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem=bowl_foot,
        negative_elem=pedestal_plate,
    )
    ctx.expect_overlap(head, bowl, axes="xy", min_overlap=0.040)
    ctx.expect_contact(head, base, elem_a=hinge_barrel, elem_b=left_hinge_ear)
    ctx.expect_within(beater, bowl, axes="xy")
    ctx.expect_origin_distance(beater, bowl, axes="xy", max_dist=0.012)
    ctx.expect_contact(head, beater, elem_a=planetary_hub, elem_b=beater_stem)
    with ctx.pose({beater_spin: 1.57}):
        ctx.expect_within(beater, bowl, axes="xy")
        ctx.expect_origin_distance(beater, bowl, axes="xy", max_dist=0.012)
    with ctx.pose({head_tilt: -0.95}):
        ctx.expect_gap(beater, bowl, axis="z", min_gap=0.020)
        ctx.expect_contact(head, beater, elem_a=planetary_hub, elem_b=beater_stem)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
