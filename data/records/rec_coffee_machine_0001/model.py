from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _translated_profile(profile, dx=0.0, dy=0.0):
    return [(x + dx, y + dy) for x, y in profile]


def _rounded_plate_mesh(filename, width, depth, thickness, radius):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius, corner_segments=10),
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _slotted_panel_mesh(
    filename,
    width,
    depth,
    thickness,
    corner_radius,
    slot_width,
    slot_count,
):
    outer = rounded_rect_profile(width, depth, corner_radius, corner_segments=8)
    margin = max(corner_radius * 1.5, 0.012)
    slot_depth = depth - 2.0 * margin
    slot_radius = min(slot_width * 0.45, slot_depth * 0.24)
    slot_profile = rounded_rect_profile(
        slot_width,
        slot_depth,
        slot_radius,
        corner_segments=6,
    )

    if slot_count <= 1:
        x_positions = [0.0]
    else:
        usable_width = width - 2.0 * margin
        pitch = usable_width / (slot_count - 1)
        x_positions = [-0.5 * usable_width + i * pitch for i in range(slot_count)]

    hole_profiles = [_translated_profile(slot_profile, x, 0.0) for x in x_positions]
    geom = ExtrudeWithHolesGeometry(
        outer,
        hole_profiles,
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _body_shell_mesh():
    sections = [
        (-0.145, 0.0, 0.332, 0.244),
        (-0.102, 0.0, 0.346, 0.252),
        (-0.032, 0.0, 0.358, 0.258),
        (0.050, 0.0, 0.352, 0.256),
        (0.108, 0.0, 0.338, 0.244),
        (0.135, 0.0, 0.326, 0.232),
    ]
    geom = superellipse_side_loft(
        sections,
        exponents=3.1,
        segments=56,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("coffee_machine_body.obj"))


def _steam_wand_mesh():
    geom = tube_from_spline_points(
        [
            (0.097, 0.132, 0.214),
            (0.101, 0.132, 0.204),
            (0.103, 0.132, 0.182),
            (0.099, 0.134, 0.145),
            (0.090, 0.136, 0.112),
        ],
        radius=0.0046,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("coffee_machine_steam_wand.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_coffee_machine", assets=ASSETS)

    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.84, 0.86, 0.89, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.20, 0.22, 0.82))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    display_blue = model.material("display_blue", rgba=(0.34, 0.56, 0.86, 0.68))
    water_tint = model.material("water_tint", rgba=(0.79, 0.89, 0.96, 0.34))
    water_window = model.material("water_window", rgba=(0.71, 0.84, 0.94, 0.48))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=brushed_steel)
    body.visual(
        _slotted_panel_mesh(
            "coffee_machine_top_grille.obj",
            width=0.172,
            depth=0.114,
            thickness=0.003,
            corner_radius=0.012,
            slot_width=0.016,
            slot_count=6,
        ),
        origin=Origin(xyz=(0.0, -0.024, 0.3565)),
        material=polished_steel,
    )
    body.visual(
        Box((0.248, 0.266, 0.022)),
        origin=Origin(xyz=(0.0, -0.004, 0.011)),
        material=black_plastic,
    )
    for x_sign in (-1.0, 1.0):
        body.visual(
            Box((0.026, 0.038, 0.008)),
            origin=Origin(xyz=(0.092 * x_sign, -0.096, 0.004)),
            material=rubber,
        )
        body.visual(
            Box((0.026, 0.038, 0.008)),
            origin=Origin(xyz=(0.092 * x_sign, 0.088, 0.004)),
            material=rubber,
        )
    body.visual(
        Box((0.208, 0.014, 0.032)),
        origin=Origin(xyz=(0.0, 0.129, 0.041)),
        material=polished_steel,
    )
    body.visual(
        Box((0.170, 0.075, 0.180)),
        origin=Origin(xyz=(0.0, 0.092, 0.136)),
        material=black_plastic,
    )
    body.visual(
        Box((0.150, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.126, 0.222)),
        material=polished_steel,
    )
    body.visual(
        Box((0.190, 0.012, 0.104)),
        origin=Origin(xyz=(0.0, 0.128, 0.290)),
        material=smoked_glass,
    )
    body.visual(
        Box((0.050, 0.004, 0.024)),
        origin=Origin(xyz=(0.0, 0.135, 0.316)),
        material=display_blue,
    )
    for button_x in (-0.060, -0.020, 0.020):
        body.visual(
            Cylinder(radius=0.011, length=0.014),
            origin=Origin(xyz=(button_x, 0.135, 0.286), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=polished_steel,
        )
    body.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=(0.086, 0.136, 0.286), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
    )
    body.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.086, 0.147, 0.286), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
    )
    body.visual(
        Box((0.110, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, 0.112, 0.205)),
        material=black_plastic,
    )
    body.visual(
        Cylinder(radius=0.024, length=0.038),
        origin=Origin(xyz=(0.0, 0.130, 0.206), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
    )
    body.visual(
        Box((0.048, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.132, 0.176)),
        material=polished_steel,
    )
    for spout_x in (-0.019, 0.019):
        body.visual(
            Cylinder(radius=0.0045, length=0.040),
            origin=Origin(xyz=(spout_x, 0.132, 0.154)),
            material=polished_steel,
        )
    body.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.097, 0.132, 0.214), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=polished_steel,
    )
    body.visual(_steam_wand_mesh(), material=polished_steel)
    body.visual(
        Box((0.006, 0.120, 0.014)),
        origin=Origin(xyz=(-0.066, -0.024, 0.363)),
        material=polished_steel,
    )
    body.visual(
        Box((0.006, 0.120, 0.014)),
        origin=Origin(xyz=(0.066, -0.024, 0.363)),
        material=polished_steel,
    )
    body.visual(
        Box((0.138, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.031, 0.363)),
        material=polished_steel,
    )
    body.inertial = Inertial.from_geometry(
        Box((0.260, 0.285, 0.360)),
        mass=7.8,
        origin=Origin(xyz=(0.0, -0.005, 0.180)),
    )

    reservoir = model.part("reservoir")
    reservoir.visual(
        Box((0.082, 0.080, 0.250)),
        origin=Origin(xyz=(0.0, -0.040, 0.0)),
        material=water_tint,
    )
    reservoir.visual(
        Box((0.086, 0.084, 0.014)),
        origin=Origin(xyz=(0.0, -0.040, 0.131)),
        material=black_plastic,
    )
    reservoir.visual(
        Box((0.008, 0.032, 0.160)),
        origin=Origin(xyz=(0.036, -0.020, 0.0)),
        material=water_window,
    )
    reservoir.inertial = Inertial.from_geometry(
        Box((0.086, 0.084, 0.264)),
        mass=0.8,
        origin=Origin(xyz=(0.0, -0.040, 0.007)),
    )

    reservoir_lid = model.part("reservoir_lid")
    reservoir_lid.visual(
        _rounded_plate_mesh(
            "coffee_machine_reservoir_lid.obj",
            width=0.086,
            depth=0.080,
            thickness=0.008,
            radius=0.010,
        ),
        origin=Origin(xyz=(0.0, 0.040, 0.004)),
        material=black_plastic,
    )
    reservoir_lid.visual(
        Cylinder(radius=0.005, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished_steel,
    )
    reservoir_lid.visual(
        Box((0.030, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.074, 0.008)),
        material=polished_steel,
    )
    reservoir_lid.inertial = Inertial.from_geometry(
        Box((0.086, 0.080, 0.012)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.040, 0.004)),
    )

    drip_tray = model.part("drip_tray")
    drip_tray.visual(
        Box((0.184, 0.092, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=black_plastic,
    )
    drip_tray.visual(
        _slotted_panel_mesh(
            "coffee_machine_drip_grate.obj",
            width=0.162,
            depth=0.072,
            thickness=0.004,
            corner_radius=0.008,
            slot_width=0.020,
            slot_count=5,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=polished_steel,
    )
    drip_tray.visual(
        Box((0.188, 0.014, 0.036)),
        origin=Origin(xyz=(0.0, 0.039, 0.018)),
        material=polished_steel,
    )
    drip_tray.visual(
        Box((0.150, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.046, 0.023)),
        material=black_plastic,
    )
    drip_tray.inertial = Inertial.from_geometry(
        Box((0.188, 0.092, 0.036)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    model.articulation(
        "body_to_reservoir",
        ArticulationType.FIXED,
        parent="body",
        child="reservoir",
        origin=Origin(xyz=(0.079, -0.143, 0.180)),
    )
    model.articulation(
        "reservoir_lid_hinge",
        ArticulationType.REVOLUTE,
        parent="reservoir",
        child="reservoir_lid",
        origin=Origin(xyz=(0.0, -0.082, 0.131)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "drip_tray_slide",
        ArticulationType.PRISMATIC,
        parent="body",
        child="drip_tray",
        origin=Origin(xyz=(0.0, 0.086, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=0.0,
            upper=0.055,
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
        "body",
        "reservoir_lid",
        reason="rear fill lid nests over the reservoir shoulder and chassis edge when closed",
    )
    ctx.check_no_overlaps(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("reservoir", "body")
    ctx.expect_aabb_overlap("reservoir", "body", axes="xz", min_overlap=0.06)
    ctx.expect_aabb_contact("reservoir_lid", "reservoir")
    ctx.expect_aabb_overlap("reservoir_lid", "reservoir", axes="xy", min_overlap=0.03)
    ctx.expect_origin_distance("reservoir_lid", "reservoir", axes="x", max_dist=0.01)
    ctx.expect_joint_motion_axis(
        "reservoir_lid_hinge",
        "reservoir_lid",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    ctx.expect_aabb_contact("drip_tray", "body")
    ctx.expect_aabb_overlap("drip_tray", "body", axes="xy", min_overlap=0.04)
    ctx.expect_origin_distance("drip_tray", "body", axes="x", max_dist=0.01)
    ctx.expect_joint_motion_axis(
        "drip_tray_slide",
        "drip_tray",
        world_axis="y",
        direction="positive",
        min_delta=0.03,
    )

    with ctx.pose({"drip_tray_slide": 0.055}):
        ctx.expect_aabb_contact("drip_tray", "body")
        ctx.expect_aabb_overlap("drip_tray", "body", axes="xz", min_overlap=0.02)
        ctx.expect_origin_distance("drip_tray", "body", axes="x", max_dist=0.01)

    with ctx.pose({"reservoir_lid_hinge": 1.15}):
        ctx.expect_aabb_overlap("reservoir_lid", "reservoir", axes="x", min_overlap=0.05)
        ctx.expect_aabb_gap(
            "reservoir_lid",
            "reservoir",
            axis="z",
            max_gap=0.09,
            max_penetration=0.016,
        )
        ctx.expect_origin_distance("reservoir_lid", "reservoir", axes="x", max_dist=0.015)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
