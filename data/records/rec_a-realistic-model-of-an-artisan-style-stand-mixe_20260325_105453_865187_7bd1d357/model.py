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
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _superellipse_component(value: float, radius: float, exponent: float) -> float:
    return radius * math.copysign(abs(value) ** (2.0 / exponent), value)


def _xy_superellipse_loop(
    width: float,
    depth: float,
    z: float,
    *,
    center_y: float = 0.0,
    exponent: float = 2.8,
    samples: int = 32,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    half_w = width * 0.5
    half_d = depth * 0.5
    for i in range(samples):
        angle = (2.0 * math.pi * i) / samples
        points.append(
            (
                _superellipse_component(math.cos(angle), half_w, exponent),
                center_y + _superellipse_component(math.sin(angle), half_d, exponent),
                z,
            )
        )
    return points


def _xz_superellipse_loop(
    y: float,
    width: float,
    height: float,
    z_center: float,
    *,
    exponent: float = 2.6,
    samples: int = 32,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    half_w = width * 0.5
    half_h = height * 0.5
    for i in range(samples):
        angle = (2.0 * math.pi * i) / samples
        points.append(
            (
                _superellipse_component(math.cos(angle), half_w, exponent),
                y,
                z_center + _superellipse_component(math.sin(angle), half_h, exponent),
            )
        )
    return points


def _mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_foot_mesh():
    return _mesh(
        "artisan_stand_mixer_foot.obj",
        repair_loft(
            [
                _xy_superellipse_loop(0.230, 0.210, 0.000, center_y=0.025, exponent=2.9),
                _xy_superellipse_loop(0.222, 0.206, 0.010, center_y=0.024, exponent=3.0),
                _xy_superellipse_loop(0.176, 0.134, 0.022, center_y=0.010, exponent=3.1),
            ]
        ),
    )


def _build_pedestal_mesh():
    return _mesh(
        "artisan_stand_mixer_pedestal.obj",
        repair_loft(
            [
                _xz_superellipse_loop(-0.058, 0.100, 0.048, 0.046, exponent=2.5),
                _xz_superellipse_loop(-0.044, 0.092, 0.084, 0.066, exponent=2.6),
                _xz_superellipse_loop(-0.032, 0.088, 0.106, 0.075, exponent=2.7),
                _xz_superellipse_loop(-0.024, 0.084, 0.090, 0.080, exponent=2.8),
            ]
        ),
    )


def _build_head_shell_mesh():
    return _mesh(
        "artisan_stand_mixer_head_shell.obj",
        repair_loft(
            [
                _xz_superellipse_loop(0.006, 0.074, 0.046, 0.024, exponent=2.6),
                _xz_superellipse_loop(0.020, 0.098, 0.078, 0.041, exponent=2.6),
                _xz_superellipse_loop(0.044, 0.110, 0.092, 0.052, exponent=2.7),
                _xz_superellipse_loop(0.068, 0.104, 0.090, 0.054, exponent=2.8),
                _xz_superellipse_loop(0.089, 0.084, 0.070, 0.042, exponent=2.8),
                _xz_superellipse_loop(0.104, 0.052, 0.046, 0.028, exponent=2.7),
            ]
        ),
    )


def _build_bowl_mesh():
    return _mesh(
        "artisan_stand_mixer_bowl.obj",
        LatheGeometry(
            [
                (0.000, 0.000),
                (0.026, 0.000),
                (0.038, 0.004),
                (0.060, 0.020),
                (0.080, 0.050),
                (0.092, 0.080),
                (0.096, 0.084),
                (0.089, 0.082),
                (0.082, 0.050),
                (0.058, 0.018),
                (0.034, 0.003),
                (0.000, 0.003),
            ],
            segments=56,
        ),
    )


def _build_bowl_handle_mesh():
    return _mesh(
        "artisan_stand_mixer_bowl_handle.obj",
        tube_from_spline_points(
            [
                (0.082, 0.018, 0.060),
                (0.108, 0.022, 0.068),
                (0.124, 0.010, 0.058),
                (0.124, -0.010, 0.042),
                (0.108, -0.018, 0.028),
                (0.066, -0.018, 0.030),
            ],
            radius=0.0045,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
    )


def _build_beater_mesh():
    beater = ExtrudeWithHolesGeometry(
        [
            (-0.013, 0.000),
            (-0.018, -0.015),
            (-0.014, -0.040),
            (-0.008, -0.060),
            (-0.012, -0.074),
            (0.000, -0.080),
            (0.012, -0.074),
            (0.008, -0.060),
            (0.014, -0.040),
            (0.018, -0.015),
            (0.013, 0.000),
            (0.004, 0.000),
            (0.003, 0.010),
            (-0.003, 0.010),
            (-0.004, 0.000),
        ],
        [
            [
                (-0.007, -0.008),
                (-0.011, -0.028),
                (-0.008, -0.058),
                (0.000, -0.068),
                (0.008, -0.058),
                (0.011, -0.028),
                (0.007, -0.008),
                (0.002, -0.008),
                (0.001, 0.003),
                (-0.001, 0.003),
                (-0.002, -0.008),
            ]
        ],
        height=0.004,
        center=True,
    )
    beater.rotate_x(math.pi / 2.0)
    return _mesh("artisan_stand_mixer_beater.obj", beater)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer", assets=ASSETS)

    body_paint = model.material("body_paint", rgba=(0.78, 0.18, 0.15, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.79, 0.81, 1.0))
    chrome = model.material("chrome", rgba=(0.90, 0.90, 0.92, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.20, 1.0))

    base = model.part("base")
    base.visual(_build_foot_mesh(), material=body_paint, name="foot_shell")
    base.visual(_build_pedestal_mesh(), material=body_paint, name="pedestal_shell")
    base.visual(
        Cylinder(radius=0.054, length=0.010),
        origin=Origin(xyz=(0.000, 0.042, 0.027)),
        material=chrome,
        name="bowl_plate",
    )
    base.visual(
        Box((0.088, 0.040, 0.008)),
        origin=Origin(xyz=(0.000, -0.028, 0.124)),
        material=body_paint,
        name="pedestal_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.230, 0.210, 0.130)),
        mass=4.3,
        origin=Origin(xyz=(0.000, 0.020, 0.065)),
    )

    bowl = model.part("bowl")
    bowl.visual(_build_bowl_mesh(), material=stainless, name="bowl_shell")
    bowl.visual(_build_bowl_handle_mesh(), material=stainless, name="bowl_handle")
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.096, length=0.084),
        mass=1.2,
        origin=Origin(xyz=(0.000, 0.000, 0.042)),
    )

    head = model.part("head")
    head.visual(
        Box((0.078, 0.022, 0.006)),
        origin=Origin(xyz=(0.000, 0.010, 0.003)),
        material=body_paint,
        name="head_seat",
    )
    head.visual(_build_head_shell_mesh(), material=body_paint, name="head_shell")
    head.visual(
        Cylinder(radius=0.043, length=0.006),
        origin=Origin(xyz=(0.000, 0.079, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="trim_band",
    )
    head.visual(
        Cylinder(radius=0.020, length=0.032),
        origin=Origin(xyz=(0.000, 0.103, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hub_housing",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.000, 0.121, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hub_cover",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.000, 0.103, -0.004)),
        material=chrome,
        name="attachment_shaft",
    )
    head.visual(
        _build_beater_mesh(),
        origin=Origin(xyz=(0.000, 0.103, -0.006)),
        material=chrome,
        name="flat_beater",
    )
    head.visual(
        Box((0.010, 0.026, 0.010)),
        origin=Origin(xyz=(-0.056, 0.045, 0.054)),
        material=charcoal,
        name="speed_lever",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.110, 0.130, 0.100)),
        mass=3.1,
        origin=Origin(xyz=(0.000, 0.055, 0.050)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.000, 0.042, 0.032)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.000, -0.028, 0.128)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    head_tilt = object_model.get_articulation("base_to_head")

    bowl_mount = base.get_visual("bowl_plate")
    pedestal_cap = base.get_visual("pedestal_cap")
    bowl_shell = bowl.get_visual("bowl_shell")
    bowl_handle = bowl.get_visual("bowl_handle")
    head_seat = head.get_visual("head_seat")
    head_shell = head.get_visual("head_shell")
    flat_beater = head.get_visual("flat_beater")
    speed_lever = head.get_visual("speed_lever")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(bowl, base, axes="x", max_dist=0.002)
    ctx.expect_origin_distance(head, bowl, axes="x", max_dist=0.002)
    ctx.expect_contact(bowl, base, elem_a=bowl_shell, elem_b=bowl_mount)
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=bowl_shell,
        negative_elem=bowl_mount,
        name="bowl_seated_on_mount",
    )
    ctx.expect_contact(head, base, elem_a=head_seat, elem_b=pedestal_cap)
    ctx.expect_gap(
        head,
        base,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0,
        positive_elem=head_seat,
        negative_elem=pedestal_cap,
        name="head_closed_on_hinge_seat",
    )
    ctx.expect_overlap(head, bowl, axes="xy", min_overlap=0.020)
    ctx.expect_within(head, bowl, axes="xy", inner_elem=flat_beater, outer_elem=bowl_shell)
    ctx.expect_contact(bowl, bowl, elem_a=bowl_handle, elem_b=bowl_shell)
    ctx.expect_contact(head, head, elem_a=speed_lever, elem_b=head_shell)

    with ctx.pose({head_tilt: 0.95}):
        ctx.expect_gap(
            head,
            bowl,
            axis="z",
            min_gap=0.035,
            positive_elem=flat_beater,
            negative_elem=bowl_shell,
            name="beater_clears_bowl_when_head_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
