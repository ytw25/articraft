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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _build_foot_shell() -> MeshGeometry:
    return superellipse_side_loft(
        [
            (-0.14, 0.0, 0.020, 0.140),
            (-0.06, 0.0, 0.031, 0.174),
            (0.02, 0.0, 0.040, 0.180),
            (0.10, 0.0, 0.034, 0.168),
            (0.16, 0.0, 0.018, 0.124),
        ],
        exponents=2.5,
        segments=56,
    )


def _build_pedestal_core() -> MeshGeometry:
    return superellipse_side_loft(
        [
            (-0.06, 0.018, 0.086, 0.090),
            (-0.01, 0.020, 0.098, 0.108),
            (0.05, 0.020, 0.090, 0.094),
        ],
        exponents=2.4,
        segments=44,
    )


def _build_column_shell() -> MeshGeometry:
    return sweep_profile_along_spline(
        [
            (0.0, -0.058, 0.052),
            (0.0, -0.082, 0.120),
            (0.0, -0.094, 0.205),
            (0.0, -0.085, 0.278),
        ],
        profile=rounded_rect_profile(0.074, 0.088, radius=0.020, corner_segments=8),
        samples_per_segment=18,
        cap_profile=True,
    )


def _build_bowl_stem() -> MeshGeometry:
    return sweep_profile_along_spline(
        [
            (0.0, 0.018, 0.040),
            (0.0, 0.036, 0.054),
            (0.0, 0.055, 0.068),
        ],
        profile=rounded_rect_profile(0.058, 0.048, radius=0.012, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
    )


def _build_bowl_shell() -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [
            (0.038, 0.000),
            (0.046, 0.008),
            (0.058, 0.024),
            (0.084, 0.060),
            (0.106, 0.094),
            (0.110, 0.104),
        ],
        [
            (0.000, 0.000),
            (0.030, 0.003),
            (0.040, 0.020),
            (0.070, 0.058),
            (0.092, 0.094),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )


def _build_bowl_handle() -> MeshGeometry:
    return sweep_profile_along_spline(
        [
            (0.102, 0.018, 0.078),
            (0.118, 0.028, 0.086),
            (0.142, 0.018, 0.075),
            (0.150, -0.008, 0.054),
            (0.126, -0.018, 0.036),
            (0.102, -0.010, 0.030),
        ],
        profile=rounded_rect_profile(0.010, 0.016, radius=0.0035, corner_segments=6),
        samples_per_segment=12,
        cap_profile=True,
    )


def _build_head_shell() -> MeshGeometry:
    return superellipse_side_loft(
        [
            (0.028, 0.026, 0.070, 0.068),
            (0.070, 0.018, 0.082, 0.108),
            (0.122, -0.004, 0.084, 0.146),
            (0.170, -0.026, 0.070, 0.150),
            (0.208, -0.040, 0.038, 0.112),
            (0.236, -0.044, -0.004, 0.072),
        ],
        exponents=2.8,
        segments=60,
    )


def _build_whisk_mesh() -> MeshGeometry:
    whisk_loop = tube_from_spline_points(
        [
            (0.010, 0.0, -0.014),
            (0.020, 0.0, -0.030),
            (0.026, 0.0, -0.045),
            (0.024, 0.0, -0.058),
            (0.014, 0.0, -0.068),
            (0.000, 0.0, -0.072),
            (-0.014, 0.0, -0.068),
            (-0.024, 0.0, -0.058),
            (-0.026, 0.0, -0.045),
            (-0.020, 0.0, -0.030),
            (-0.010, 0.0, -0.014),
        ],
        radius=0.0028,
        samples_per_segment=10,
        radial_segments=12,
        cap_ends=True,
    )
    return _merge_meshes(
        whisk_loop,
        whisk_loop.copy().rotate_z(math.pi / 2.0),
        whisk_loop.copy().rotate_z(math.pi / 4.0),
        whisk_loop.copy().rotate_z(3.0 * math.pi / 4.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer", assets=ASSETS)

    enamel = model.material("enamel", rgba=(0.72, 0.14, 0.16, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.83, 1.0))
    polished = model.material("polished", rgba=(0.86, 0.87, 0.89, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.67, 0.69, 0.72, 1.0))

    base = model.part("base")
    base.visual(_save_mesh("mixer_foot_shell.obj", _build_foot_shell()), material=enamel, name="foot_shell")
    base.visual(_save_mesh("mixer_pedestal_core.obj", _build_pedestal_core()), material=enamel, name="pedestal_core")
    base.visual(_save_mesh("mixer_column_shell.obj", _build_column_shell()), material=enamel, name="column_shell")
    base.visual(_save_mesh("mixer_bowl_stem.obj", _build_bowl_stem()), material=enamel, name="bowl_stem")
    base.visual(
        Cylinder(radius=0.046, length=0.006),
        origin=Origin(xyz=(0.0, 0.055, 0.069)),
        material=polished,
        name="bowl_seat",
    )
    base.visual(
        Box((0.022, 0.038, 0.045)),
        origin=Origin(xyz=(0.052, -0.085, 0.255)),
        material=enamel,
        name="right_hinge_cheek",
    )
    base.visual(
        Box((0.022, 0.038, 0.045)),
        origin=Origin(xyz=(-0.052, -0.085, 0.255)),
        material=enamel,
        name="left_hinge_cheek",
    )
    base.visual(
        Box((0.128, 0.052, 0.032)),
        origin=Origin(xyz=(0.0, -0.083, 0.247)),
        material=enamel,
        name="hinge_bridge",
    )
    base.visual(
        Box((0.038, 0.026, 0.010)),
        origin=Origin(xyz=(0.0, -0.060, 0.200)),
        material=dark_trim,
        name="tilt_stop",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.078),
        origin=Origin(xyz=(0.0, -0.085, 0.275), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="hinge_pin",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.22, 0.34, 0.32)),
        mass=10.8,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    bowl = model.part("bowl")
    bowl.visual(_save_mesh("mixer_bowl_shell.obj", _build_bowl_shell()), material=stainless, name="bowl_shell")
    bowl.visual(
        Cylinder(radius=0.033, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=stainless,
        name="bowl_foot",
    )
    bowl.visual(
        Cylinder(radius=0.034, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=stainless,
        name="bowl_floor",
    )
    bowl.visual(_save_mesh("mixer_bowl_handle.obj", _build_bowl_handle()), material=stainless, name="bowl_handle")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.11),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    head = model.part("head")
    head.visual(_save_mesh("mixer_head_shell.obj", _build_head_shell()), material=enamel, name="head_shell")
    head.visual(
        Box((0.072, 0.052, 0.034)),
        origin=Origin(xyz=(0.0, 0.040, 0.046)),
        material=enamel,
        name="rear_housing",
    )
    head.visual(
        Box((0.078, 0.130, 0.026)),
        origin=Origin(xyz=(0.0, 0.128, -0.010)),
        material=enamel,
        name="underside_spine",
    )
    head.visual(
        Box((0.046, 0.072, 0.080)),
        origin=Origin(xyz=(0.0, 0.119, -0.037)),
        material=enamel,
        name="lower_web",
    )
    head.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.0, 0.208, -0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="trim_band",
    )
    head.visual(
        Cylinder(radius=0.034, length=0.046),
        origin=Origin(xyz=(0.0, 0.242, -0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="front_nose",
    )
    head.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(0.0, 0.270, -0.028), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="hub_cap",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, 0.155, -0.105)),
        material=polished,
        name="beater_socket",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.024),
        origin=Origin(xyz=(0.0, 0.155, -0.088)),
        material=enamel,
        name="socket_mount",
    )
    head.visual(
        Box((0.032, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.082, -0.066)),
        material=dark_trim,
        name="rest_heel",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.17, 0.30, 0.17)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.126, 0.008)),
    )

    beater = model.part("beater")
    beater.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=satin_metal,
        name="shaft",
    )
    beater.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=satin_metal,
        name="collar",
    )
    beater.visual(_save_mesh("mixer_whisk.obj", _build_whisk_mesh()), material=satin_metal, name="whisk")
    beater.inertial = Inertial.from_geometry(
        Cylinder(radius=0.04, length=0.09),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    model.articulation(
        "bowl_mount",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.055, 0.072)),
    )
    model.articulation(
        "tilt_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, -0.085, 0.275)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.15),
    )
    model.articulation(
        "beater_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.0, 0.155, -0.114)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")
    tilt_head = object_model.get_articulation("tilt_head")
    beater_spin = object_model.get_articulation("beater_spin")

    bowl_seat = base.get_visual("bowl_seat")
    tilt_stop = base.get_visual("tilt_stop")
    bowl_foot = bowl.get_visual("bowl_foot")
    bowl_floor = bowl.get_visual("bowl_floor")
    socket = head.get_visual("beater_socket")
    rest_heel = head.get_visual("rest_heel")
    collar = beater.get_visual("collar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # The tilt-head hinge is mostly buried inside the cast shell, so use a slightly
    # looser origin-distance sensor than the compact default.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.035)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(bowl, base, axes="xy", min_overlap=0.01)
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem=bowl_foot,
        negative_elem=bowl_seat,
    )
    ctx.expect_overlap(head, bowl, axes="xy", min_overlap=0.015)
    ctx.expect_gap(
        head,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rest_heel,
        negative_elem=tilt_stop,
    )
    ctx.expect_gap(
        head,
        beater,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=socket,
        negative_elem=collar,
    )
    ctx.expect_within(beater, bowl, axes="xy")
    ctx.expect_gap(
        beater,
        bowl,
        axis="z",
        min_gap=0.004,
        negative_elem=bowl_floor,
    )
    with ctx.pose({beater_spin: math.pi / 2.0}):
        ctx.expect_within(beater, bowl, axes="xy")
        ctx.expect_gap(
            head,
            beater,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=socket,
            negative_elem=collar,
        )
    with ctx.pose({tilt_head: 1.0}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.025,
            positive_elem=rest_heel,
            negative_elem=tilt_stop,
        )
        ctx.expect_gap(beater, bowl, axis="z", min_gap=0.050)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
