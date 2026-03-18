from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    Inertial,
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
HERE = ASSETS.asset_root
MESH_DIR = HERE / "meshes"
HALF_PI = pi / 2.0


def _mesh_path(name: str) -> Path:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    return MESH_DIR / name


def _build_shell_mesh(width: float, depth: float, bottom: float, height: float):
    half_depth = depth / 2.0
    top = bottom + height
    sections = [
        (-half_depth, bottom + 0.010, top - 0.010, width * 0.94),
        (-depth * 0.30, bottom + 0.004, top - 0.004, width * 0.975),
        (-depth * 0.12, bottom + 0.001, top - 0.001, width * 0.995),
        (0.0, bottom, top, width),
        (depth * 0.12, bottom + 0.001, top - 0.001, width * 0.995),
        (depth * 0.30, bottom + 0.004, top - 0.004, width * 0.975),
        (half_depth, bottom + 0.010, top - 0.010, width * 0.94),
    ]
    shell_geom = superellipse_side_loft(
        sections,
        exponents=3.5,
        segments=72,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(shell_geom, _mesh_path("suitcase_shell.obj"))


def _build_carry_handle_mesh() -> object:
    carry_handle_geom = tube_from_spline_points(
        [
            (-0.056, 0.018, 0.648),
            (-0.045, 0.018, 0.676),
            (0.045, 0.018, 0.676),
            (0.056, 0.018, 0.648),
        ],
        radius=0.0065,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(carry_handle_geom, _mesh_path("carry_handle.obj"))


def _build_pull_grip_mesh(rod_spacing: float) -> object:
    half_spacing = rod_spacing / 2.0
    grip_profile = rounded_rect_profile(0.018, 0.012, radius=0.0035, corner_segments=8)
    grip_geom = sweep_profile_along_spline(
        [
            (-half_spacing, 0.0, 0.118),
            (-half_spacing + 0.014, 0.0, 0.136),
            (-0.040, 0.0, 0.151),
            (0.040, 0.0, 0.151),
            (half_spacing - 0.014, 0.0, 0.136),
            (half_spacing, 0.0, 0.118),
        ],
        profile=grip_profile,
        samples_per_segment=20,
        cap_profile=True,
    )
    return mesh_from_geometry(grip_geom, _mesh_path("pull_handle_grip.obj"))


def build_object_model() -> ArticulatedObject:
    width = 0.382
    depth = 0.242
    body_bottom = 0.055
    body_height = 0.600
    body_top = body_bottom + body_height
    wheel_radius = 0.032
    wheel_thickness = 0.038
    wheel_x = 0.156
    wheel_y = 0.116
    wheel_z = wheel_radius
    handle_guide_y = 0.102
    handle_joint_z = 0.700
    guide_radius = 0.009
    guide_length = 0.126
    rod_radius = 0.0065
    rod_length = 0.240
    rod_spacing = 0.190

    model = ArticulatedObject(name="rolling_suitcase", assets=ASSETS)

    body_shell = model.material("body_shell", rgba=(0.16, 0.18, 0.21, 1.0))
    zipper_trim = model.material("zipper_trim", rgba=(0.08, 0.08, 0.09, 1.0))
    corner_guard = model.material("corner_guard", rgba=(0.11, 0.11, 0.12, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    hub = model.material("hub", rgba=(0.58, 0.60, 0.63, 1.0))

    case_body = model.part("case_body")
    case_body.visual(
        _build_shell_mesh(width=width, depth=depth, bottom=body_bottom, height=body_height),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=body_shell,
    )
    case_body.inertial = Inertial.from_geometry(
        Box((width, depth, body_height)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.0, body_bottom + body_height / 2.0)),
    )

    for x_pos, rib_width, rib_height, rib_center_z in (
        (0.0, 0.054, 0.420, 0.325),
        (-0.105, 0.032, 0.465, 0.308),
        (0.105, 0.032, 0.465, 0.308),
    ):
        case_body.visual(
            Box((rib_width, 0.010, rib_height)),
            origin=Origin(xyz=(x_pos, -0.119, rib_center_z)),
            material=zipper_trim,
        )

    for x_pos, strip_height, center_z in (
        (-0.182, 0.468, 0.326),
        (0.182, 0.468, 0.326),
    ):
        case_body.visual(
            Box((0.015, 0.014, strip_height)),
            origin=Origin(xyz=(x_pos, 0.0, center_z)),
            material=zipper_trim,
        )
    case_body.visual(
        Box((0.252, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.642)),
        material=zipper_trim,
    )
    case_body.visual(
        Box((0.198, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=zipper_trim,
    )

    for y_pos in (-0.112, 0.112):
        for z_pos in (0.107, 0.605):
            for x_pos in (-0.164, 0.164):
                case_body.visual(
                    Box((0.046, 0.028, 0.054)),
                    origin=Origin(xyz=(x_pos, y_pos, z_pos)),
                    material=corner_guard,
                )

    for x_pos in (-0.110, 0.110):
        case_body.visual(
            Box((0.032, 0.068, 0.055)),
            origin=Origin(xyz=(x_pos, -0.088, 0.0275)),
            material=corner_guard,
        )

    for x_pos in (-wheel_x, wheel_x):
        case_body.visual(
            Box((0.074, 0.082, 0.112)),
            origin=Origin(xyz=(x_pos, 0.108, 0.056)),
            material=corner_guard,
        )

    case_body.visual(
        Box((0.242, 0.042, 0.068)),
        origin=Origin(xyz=(0.0, 0.106, 0.034)),
        material=corner_guard,
    )

    for x_pos in (-0.095, 0.095):
        case_body.visual(
            Box((0.030, 0.048, 0.058)),
            origin=Origin(xyz=(x_pos, handle_guide_y, 0.604)),
            material=corner_guard,
        )
        case_body.visual(
            Cylinder(radius=guide_radius, length=guide_length),
            origin=Origin(xyz=(x_pos, handle_guide_y, 0.648), rpy=(0.0, 0.0, 0.0)),
            material=aluminum,
        )
    case_body.visual(
        Box((0.222, 0.026, 0.050)),
        origin=Origin(xyz=(0.0, handle_guide_y, 0.676)),
        material=corner_guard,
    )

    for x_pos in (-0.055, 0.055):
        case_body.visual(
            Box((0.026, 0.030, 0.018)),
            origin=Origin(xyz=(x_pos, 0.018, 0.647)),
            material=corner_guard,
        )
    case_body.visual(
        _build_carry_handle_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=corner_guard,
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_thickness),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=rubber,
    )
    left_wheel.visual(
        Cylinder(radius=0.021, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=hub,
    )
    for x_pos in (-0.010, 0.010):
        left_wheel.visual(
            Cylinder(radius=0.024, length=0.006),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
            material=zipper_trim,
        )
    left_wheel.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=hub,
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_thickness),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=wheel_radius, length=wheel_thickness),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=rubber,
    )
    right_wheel.visual(
        Cylinder(radius=0.021, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=hub,
    )
    for x_pos in (-0.010, 0.010):
        right_wheel.visual(
            Cylinder(radius=0.024, length=0.006),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
            material=zipper_trim,
        )
    right_wheel.visual(
        Cylinder(radius=0.008, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
        material=hub,
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=wheel_radius, length=wheel_thickness),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, HALF_PI, 0.0)),
    )

    pull_handle = model.part("pull_handle")
    for x_pos in (-rod_spacing / 2.0, rod_spacing / 2.0):
        pull_handle.visual(
            Cylinder(radius=rod_radius, length=rod_length),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)),
            material=aluminum,
        )
        pull_handle.visual(
            Cylinder(radius=0.010, length=0.036),
            origin=Origin(xyz=(x_pos, 0.0, -0.102), rpy=(0.0, 0.0, 0.0)),
            material=zipper_trim,
        )
    pull_handle.visual(
        Box((0.180, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=zipper_trim,
    )
    pull_handle.visual(
        _build_pull_grip_mesh(rod_spacing=rod_spacing),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=corner_guard,
    )
    pull_handle.visual(
        Box((0.024, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=zipper_trim,
    )
    pull_handle.inertial = Inertial.from_geometry(
        Box((0.220, 0.028, 0.280)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="case_body",
        child="left_wheel",
        origin=Origin(xyz=(-wheel_x, wheel_y, wheel_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=25.0,
        ),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent="case_body",
        child="right_wheel",
        origin=Origin(xyz=(wheel_x, wheel_y, wheel_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=25.0,
        ),
    )
    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent="case_body",
        child="pull_handle",
        origin=Origin(xyz=(0.0, handle_guide_y, handle_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.60,
            lower=0.0,
            upper=0.26,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.allow_overlap(
        "case_body",
        "left_wheel",
        reason="wheel sits slightly recessed into the molded rear corner housing",
    )
    ctx.allow_overlap(
        "case_body",
        "right_wheel",
        reason="wheel sits slightly recessed into the molded rear corner housing",
    )
    ctx.allow_overlap(
        "case_body",
        "pull_handle",
        reason="telescoping rods intentionally nest into the suitcase guide sleeves when retracted",
    )
    ctx.check_no_overlaps(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("left_wheel", "case_body", axes="yz", min_overlap=0.02)
    ctx.expect_aabb_overlap("right_wheel", "case_body", axes="yz", min_overlap=0.02)
    ctx.expect_origin_distance("left_wheel", "right_wheel", axes="yz", max_dist=0.002)
    ctx.expect_aabb_contact("pull_handle", "case_body")
    ctx.expect_aabb_overlap("pull_handle", "case_body", axes="xy", min_overlap=0.01)
    ctx.expect_joint_motion_axis(
        "handle_slide",
        "pull_handle",
        world_axis="z",
        direction="positive",
        min_delta=0.10,
    )

    with ctx.pose(handle_slide=0.13):
        ctx.expect_aabb_overlap("pull_handle", "case_body", axes="xy", min_overlap=0.01)

    with ctx.pose(handle_slide=0.26):
        ctx.expect_aabb_overlap("pull_handle", "case_body", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_gap(
            "pull_handle",
            "case_body",
            axis="z",
            max_gap=0.14,
            max_penetration=0.0,
        )

    with ctx.pose(left_wheel_spin=1.10, right_wheel_spin=-1.70):
        ctx.expect_aabb_overlap("left_wheel", "case_body", axes="yz", min_overlap=0.02)
        ctx.expect_aabb_overlap("right_wheel", "case_body", axes="yz", min_overlap=0.02)
        ctx.expect_origin_distance("left_wheel", "right_wheel", axes="yz", max_dist=0.002)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
