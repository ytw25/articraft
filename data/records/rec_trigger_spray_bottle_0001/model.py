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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    Material,
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


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, rgba=rgba)
    except TypeError:
        return Material(name=name, color=rgba)


def _mesh_path(name: str) -> Path:
    try:
        return ASSETS.mesh_path(name)
    except AttributeError:
        mesh_dir = HERE / "meshes"
        mesh_dir.mkdir(parents=True, exist_ok=True)
        return mesh_dir / name


def _write_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, _mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trigger_spray_bottle", assets=ASSETS)

    bottle_shell = _material("translucent_cleaner_blue", (0.73, 0.87, 0.96, 0.36))
    cleaner_liquid = _material("cleaner_liquid_blue", (0.30, 0.60, 0.86, 0.52))
    sprayer_white = _material("sprayer_white", (0.96, 0.96, 0.95, 1.0))
    label_white = _material("label_white", (0.98, 0.98, 0.97, 1.0))
    charcoal = _material("charcoal_plastic", (0.18, 0.18, 0.20, 1.0))
    accent_red = _material("selector_red", (0.82, 0.15, 0.14, 1.0))
    tube_natural = _material("natural_tube", (0.90, 0.92, 0.90, 0.78))

    body_sections = [
        (-0.038, 0.0, 0.170, 0.066),
        (-0.022, 0.0, 0.197, 0.081),
        (-0.008, 0.0, 0.219, 0.090),
        (0.010, 0.0, 0.223, 0.092),
        (0.024, 0.0, 0.206, 0.087),
        (0.034, 0.0, 0.186, 0.076),
    ]
    body_mesh = _write_mesh(
        "spray_bottle_body.obj",
        superellipse_side_loft(
            body_sections,
            exponents=[3.2, 3.1, 3.0, 3.0, 3.1, 3.2],
            segments=72,
            cap=True,
            closed=True,
        ),
    )

    liquid_sections = [
        (-0.032, 0.006, 0.120, 0.056),
        (-0.018, 0.006, 0.128, 0.069),
        (-0.006, 0.006, 0.134, 0.078),
        (0.008, 0.006, 0.136, 0.079),
        (0.020, 0.006, 0.129, 0.074),
        (0.030, 0.006, 0.120, 0.064),
    ]
    liquid_mesh = _write_mesh(
        "spray_bottle_liquid.obj",
        superellipse_side_loft(
            liquid_sections,
            exponents=[3.2, 3.1, 3.0, 3.0, 3.1, 3.2],
            segments=64,
            cap=True,
            closed=True,
        ),
    )

    shoulder_mesh = _write_mesh(
        "spray_bottle_shoulder.obj",
        LatheGeometry(
            [
                (0.0, 0.176),
                (0.034, 0.176),
                (0.033, 0.182),
                (0.030, 0.188),
                (0.027, 0.194),
                (0.024, 0.200),
                (0.021, 0.206),
                (0.020, 0.214),
                (0.0, 0.214),
            ],
            segments=48,
        ),
    )

    dip_tube_mesh = _write_mesh(
        "spray_bottle_dip_tube.obj",
        tube_from_spline_points(
            [
                (0.0, 0.0, 0.250),
                (0.0, -0.004, 0.182),
                (0.0, -0.010, 0.084),
                (0.0, -0.008, 0.018),
            ],
            radius=0.0023,
            samples_per_segment=18,
            radial_segments=14,
            cap_ends=True,
        ),
    )

    head_shell_mesh = _write_mesh(
        "sprayer_head_shell.obj",
        ExtrudeGeometry(
            rounded_rect_profile(0.050, 0.034, 0.007, corner_segments=8),
            0.050,
            cap=True,
            center=True,
            closed=True,
        ).rotate_x(pi / 2.0),
    )

    trigger_mesh = _write_mesh(
        "sprayer_trigger.obj",
        sweep_profile_along_spline(
            [
                (0.0, 0.0, 0.0),
                (0.0, 0.002, -0.008),
                (0.0, 0.005, -0.020),
                (0.0, 0.008, -0.033),
                (0.0, 0.011, -0.047),
            ],
            profile=rounded_rect_profile(0.015, 0.009, 0.0024, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
        ),
    )

    bottle = model.part("bottle")
    bottle.visual(
        body_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bottle_shell,
        name="body_shell",
    )
    bottle.visual(
        liquid_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=cleaner_liquid,
        name="liquid_fill",
    )
    bottle.visual(
        shoulder_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=bottle_shell,
        name="bottle_shoulder",
    )
    bottle.visual(
        Cylinder(radius=0.0185, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.233), rpy=(0.0, 0.0, 0.0)),
        material=bottle_shell,
        name="neck",
    )
    bottle.visual(
        Cylinder(radius=0.0245, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.248), rpy=(0.0, 0.0, 0.0)),
        material=sprayer_white,
        name="closure_cap",
    )
    bottle.visual(
        Cylinder(radius=0.0265, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.260), rpy=(0.0, 0.0, 0.0)),
        material=sprayer_white,
        name="cap_top_ring",
    )
    bottle.visual(
        Cylinder(radius=0.036, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003), rpy=(0.0, 0.0, 0.0)),
        material=bottle_shell,
        name="base_ring",
    )
    bottle.visual(
        Box((0.055, 0.0025, 0.086)),
        origin=Origin(xyz=(0.0, 0.032, 0.104), rpy=(0.0, 0.0, 0.0)),
        material=label_white,
        name="front_label_panel",
    )
    bottle.visual(
        Box((0.004, 0.024, 0.080)),
        origin=Origin(xyz=(0.041, 0.002, 0.110), rpy=(0.0, 0.0, 0.0)),
        material=charcoal,
        name="right_grip_strip",
    )
    bottle.visual(
        Box((0.004, 0.024, 0.080)),
        origin=Origin(xyz=(-0.041, 0.002, 0.110), rpy=(0.0, 0.0, 0.0)),
        material=charcoal,
        name="left_grip_strip",
    )
    bottle.visual(
        dip_tube_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tube_natural,
        name="dip_tube",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.100, 0.080, 0.265)),
        mass=0.70,
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
    )

    sprayer_head = model.part("sprayer_head")
    sprayer_head.visual(
        head_shell_mesh,
        origin=Origin(xyz=(0.0, 0.012, 0.028), rpy=(0.0, 0.0, 0.0)),
        material=sprayer_white,
        name="head_shell",
    )
    sprayer_head.visual(
        Cylinder(radius=0.0135, length=0.050),
        origin=Origin(xyz=(0.0, 0.045, 0.028), rpy=(pi / 2.0, 0.0, 0.0)),
        material=sprayer_white,
        name="nozzle_barrel",
    )
    sprayer_head.visual(
        Cylinder(radius=0.0155, length=0.010),
        origin=Origin(xyz=(0.0, 0.020, 0.028), rpy=(pi / 2.0, 0.0, 0.0)),
        material=sprayer_white,
        name="nozzle_collar",
    )
    sprayer_head.visual(
        Cylinder(radius=0.0120, length=0.022),
        origin=Origin(xyz=(0.0, -0.018, 0.028), rpy=(pi / 2.0, 0.0, 0.0)),
        material=sprayer_white,
        name="rear_cap",
    )
    sprayer_head.visual(
        Box((0.028, 0.018, 0.026)),
        origin=Origin(xyz=(0.0, -0.004, 0.013), rpy=(0.0, 0.0, 0.0)),
        material=sprayer_white,
        name="rear_stem",
    )
    sprayer_head.visual(
        Box((0.032, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.008, 0.004), rpy=(0.0, 0.0, 0.0)),
        material=sprayer_white,
        name="base_bridge",
    )
    sprayer_head.inertial = Inertial.from_geometry(
        Box((0.070, 0.102, 0.050)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.018, 0.024)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        trigger_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=charcoal,
        name="trigger_body",
    )
    trigger.visual(
        Cylinder(radius=0.0035, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="pivot_boss",
    )
    trigger.visual(
        Box((0.018, 0.010, 0.011)),
        origin=Origin(xyz=(0.0, 0.012, -0.044), rpy=(0.0, 0.0, 0.0)),
        material=charcoal,
        name="finger_pad",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.026, 0.018, 0.065)),
        mass=0.05,
        origin=Origin(xyz=(0.0, -0.002, -0.030)),
    )

    nozzle_tip = model.part("nozzle_tip")
    nozzle_tip.visual(
        Cylinder(radius=0.0105, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="selector_body",
    )
    nozzle_tip.visual(
        Cylinder(radius=0.0125, length=0.003),
        origin=Origin(xyz=(0.0, 0.0135, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=accent_red,
        name="selector_face",
    )
    nozzle_tip.visual(
        Box((0.010, 0.004, 0.009)),
        origin=Origin(xyz=(0.007, 0.006, 0.008), rpy=(0.0, 0.0, 0.0)),
        material=accent_red,
        name="selector_tab",
    )
    nozzle_tip.visual(
        Cylinder(radius=0.0025, length=0.004),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="spray_orifice",
    )
    nozzle_tip.inertial = Inertial.from_geometry(
        Box((0.024, 0.020, 0.020)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.008, 0.002)),
    )

    model.articulation(
        "bottle_to_head",
        ArticulationType.FIXED,
        parent="bottle",
        child="sprayer_head",
        origin=Origin(xyz=(0.0, 0.0, 0.262)),
    )
    model.articulation(
        "trigger_hinge",
        ArticulationType.REVOLUTE,
        parent="sprayer_head",
        child="trigger",
        origin=Origin(xyz=(0.0, 0.060, 0.018)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
            lower=0.0,
            upper=0.58,
        ),
    )
    model.articulation(
        "nozzle_selector",
        ArticulationType.REVOLUTE,
        parent="sprayer_head",
        child="nozzle_tip",
        origin=Origin(xyz=(0.0, 0.068, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.35,
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
        "sprayer_head",
        "trigger",
        reason="trigger pivot rides in a tight molded shroud and generated hinge hulls are conservative",
    )
    ctx.allow_overlap(
        "sprayer_head",
        "nozzle_tip",
        reason="the rotating nozzle selector nests into the front collar with intended close contact",
    )
    ctx.check_no_overlaps(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap_xy("sprayer_head", "bottle", min_overlap=0.018)
    ctx.expect_xy_distance("sprayer_head", "bottle", max_dist=0.040)
    ctx.expect_aabb_gap_z("sprayer_head", "bottle", max_gap=0.003, max_penetration=0.0)

    ctx.expect_joint_motion_axis(
        "trigger_hinge",
        "trigger",
        world_axis="y",
        direction="negative",
        min_delta=0.006,
    )

    with ctx.pose(trigger_hinge=0.0):
        ctx.expect_aabb_overlap_xy("trigger", "sprayer_head", min_overlap=0.004)
        ctx.expect_xy_distance("trigger", "sprayer_head", max_dist=0.062)
        ctx.expect_xy_distance("trigger", "bottle", max_dist=0.065)

    with ctx.pose(trigger_hinge=0.58):
        ctx.expect_aabb_overlap_xy("trigger", "sprayer_head", min_overlap=0.002)
        ctx.expect_xy_distance("trigger", "sprayer_head", max_dist=0.062)
        ctx.expect_xy_distance("trigger", "bottle", max_dist=0.065)

    with ctx.pose(nozzle_selector=0.0):
        ctx.expect_aabb_overlap_xy("nozzle_tip", "sprayer_head", min_overlap=0.002)
        ctx.expect_xy_distance("nozzle_tip", "sprayer_head", max_dist=0.080)
        ctx.expect_xy_distance("nozzle_tip", "bottle", max_dist=0.090)

    with ctx.pose(nozzle_selector=1.35):
        ctx.expect_aabb_overlap_xy("nozzle_tip", "sprayer_head", min_overlap=0.002)
        ctx.expect_xy_distance("nozzle_tip", "sprayer_head", max_dist=0.080)
        ctx.expect_xy_distance("nozzle_tip", "bottle", max_dist=0.090)

    with ctx.pose(trigger_hinge=0.58, nozzle_selector=1.35):
        ctx.expect_xy_distance("nozzle_tip", "trigger", max_dist=0.110)
        ctx.expect_xy_distance("sprayer_head", "bottle", max_dist=0.040)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
