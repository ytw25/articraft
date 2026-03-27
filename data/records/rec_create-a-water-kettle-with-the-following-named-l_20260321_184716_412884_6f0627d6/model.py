from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    CapsuleGeometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    mesh_from_geometry,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


BODY_RADIUS = 0.078
BODY_HEIGHT = 0.236
LID_HINGE_X = 0.053
LID_HINGE_Z = BODY_HEIGHT


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_body_mesh():
    profile = [
        (0.0, 0.0),
        (0.071, 0.0),
        (0.074, 0.010),
        (0.076, 0.055),
        (0.078, 0.145),
        (0.077, 0.198),
        (0.075, 0.223),
        (0.073, BODY_HEIGHT),
        (0.0, BODY_HEIGHT),
    ]
    return _mesh("kettle_body.obj", LatheGeometry(profile, segments=64))


def _build_spout_mesh():
    spout = ConeGeometry(radius=0.023, height=0.062, radial_segments=28, closed=True)
    spout.scale(1.20, 0.68, 0.82)
    spout.rotate_y(math.pi / 2.0)
    return _mesh("kettle_spout.obj", spout)


def _build_handle_mesh():
    profile = rounded_rect_profile(0.028, 0.020, radius=0.007, corner_segments=8)
    handle = sweep_profile_along_spline(
        [
            (0.070, 0.0, 0.207),
            (0.100, 0.0, 0.190),
            (0.123, 0.0, 0.146),
            (0.127, 0.0, 0.088),
            (0.113, 0.0, 0.037),
            (0.072, 0.0, 0.015),
        ],
        profile=profile,
        samples_per_segment=16,
        cap_profile=True,
    )
    return _mesh("kettle_handle.obj", handle)


def _build_lid_dome_mesh():
    dome = DomeGeometry(
        radius=(0.071, 0.071, 0.016),
        radial_segments=40,
        height_segments=14,
        closed=True,
    )
    return _mesh("kettle_lid_dome.obj", dome)


def _build_button_mesh():
    button = CapsuleGeometry(radius=0.0085, length=0.020, radial_segments=20, height_segments=10)
    button.rotate_y(math.pi / 2.0)
    button.scale(1.12, 1.70, 0.92)
    return _mesh("kettle_button.obj", button)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="water_kettle", assets=ASSETS)

    gloss_black = model.material("gloss_black", rgba=(0.07, 0.07, 0.08, 1.0))
    matte_black = model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.19, 1.0))
    smoke = model.material("smoke", rgba=(0.55, 0.62, 0.68, 0.42))
    switch_clear = model.material("switch_clear", rgba=(0.82, 0.86, 0.90, 0.42))
    window_tint = model.material("window_tint", rgba=(0.83, 0.86, 0.88, 0.48))

    body_mesh = _build_body_mesh()
    spout_mesh = _build_spout_mesh()
    handle_mesh = _build_handle_mesh()
    lid_dome_mesh = _build_lid_dome_mesh()
    button_mesh = _build_button_mesh()

    base = model.part("base_link")
    base.visual(body_mesh, material=gloss_black)
    base.visual(
        Cylinder(radius=BODY_RADIUS, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=charcoal,
        name="base_ring",
    )
    base.visual(
        spout_mesh,
        origin=Origin(xyz=(-0.099, 0.0, 0.228), rpy=(0.0, 0.0, 0.02)),
        material=gloss_black,
        name="spout",
    )
    base.visual(handle_mesh, material=gloss_black, name="handle")
    base.visual(
        Box((0.024, 0.044, 0.024)),
        origin=Origin(xyz=(0.071, 0.0, 0.214)),
        material=gloss_black,
        name="upper_handle_mount",
    )
    base.visual(
        Box((0.020, 0.030, 0.038)),
        origin=Origin(xyz=(0.069, 0.0, 0.024)),
        material=gloss_black,
        name="lower_handle_mount",
    )
    base.visual(
        Box((0.024, 0.048, 0.012)),
        origin=Origin(xyz=(0.083, 0.0, 0.206)),
        material=matte_black,
        name="button_recess",
    )
    base.visual(
        Box((0.052, 0.006, 0.170)),
        origin=Origin(xyz=(0.0, 0.074, 0.108)),
        material=window_tint,
        name="water_window",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.074, 0.190), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=window_tint,
        name="window_cap_top",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.074, 0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=window_tint,
        name="window_cap_bottom",
    )
    base.visual(
        Box((0.024, 0.034, 0.014)),
        origin=Origin(xyz=(0.062, 0.0, 0.232)),
        material=gloss_black,
        name="lid_hinge_saddle",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_RADIUS, length=BODY_HEIGHT),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.072, length=0.006),
        origin=Origin(xyz=(-0.049, 0.0, 0.003)),
        material=smoke,
        name="lid_disc",
    )
    lid.visual(
        lid_dome_mesh,
        origin=Origin(xyz=(-0.049, 0.0, 0.002)),
        material=smoke,
        name="lid_dome",
    )
    lid.visual(
        Box((0.080, 0.028, 0.010)),
        origin=Origin(xyz=(-0.016, 0.0, 0.006)),
        material=gloss_black,
        name="lid_bridge",
    )
    lid.visual(
        Box((0.020, 0.034, 0.010)),
        origin=Origin(xyz=(0.004, 0.0, 0.006)),
        material=gloss_black,
        name="lid_hinge_leaf",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.072, length=0.016),
        mass=0.08,
        origin=Origin(xyz=(-0.049, 0.0, 0.008)),
    )

    lid_release_button = model.part("lid_release_button")
    lid_release_button.visual(
        button_mesh,
        origin=Origin(xyz=(0.012, 0.0, 0.008)),
        material=matte_black,
        name="button_top",
    )
    lid_release_button.visual(
        Box((0.022, 0.030, 0.010)),
        origin=Origin(xyz=(0.011, 0.0, 0.005)),
        material=matte_black,
        name="button_body",
    )
    lid_release_button.inertial = Inertial.from_geometry(
        Box((0.022, 0.030, 0.010)),
        mass=0.012,
        origin=Origin(xyz=(0.011, 0.0, 0.005)),
    )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Box((0.014, 0.022, 0.050)),
        origin=Origin(xyz=(0.008, 0.0, -0.024)),
        material=switch_clear,
        name="switch_stem",
    )
    power_switch.visual(
        Cylinder(radius=0.011, length=0.014),
        origin=Origin(xyz=(0.008, 0.0, -0.048), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=switch_clear,
        name="switch_tip",
    )
    power_switch.inertial = Inertial.from_geometry(
        Box((0.014, 0.022, 0.050)),
        mass=0.015,
        origin=Origin(xyz=(0.008, 0.0, -0.024)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="base_link",
        child="lid",
        origin=Origin(xyz=(LID_HINGE_X, 0.0, LID_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.55),
    )
    model.articulation(
        "lid_release_button_hinge",
        ArticulationType.REVOLUTE,
        parent="base_link",
        child="lid_release_button",
        origin=Origin(xyz=(0.092, 0.0, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-0.28, upper=0.10),
    )
    model.articulation(
        "power_switch_hinge",
        ArticulationType.REVOLUTE,
        parent="base_link",
        child="power_switch",
        origin=Origin(xyz=(0.086, 0.0, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-0.45, upper=0.58),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    part_names = {part.name for part in object_model.parts}
    assert part_names == {"base_link", "lid", "lid_release_button", "power_switch"}
    assert len(object_model.articulations) == 3
    articulation_names = {articulation.name for articulation in object_model.articulations}
    assert articulation_names == {"lid_hinge", "lid_release_button_hinge", "power_switch_hinge"}
    for articulation in object_model.articulations:
        assert articulation.parent == "base_link"
        assert articulation.articulation_type == ArticulationType.REVOLUTE

    lid_pos = ctx.part_world_position("lid")
    button_pos = ctx.part_world_position("lid_release_button")
    switch_pos = ctx.part_world_position("power_switch")
    assert lid_pos[0] > 0.04
    assert button_pos[0] > 0.08
    assert switch_pos[0] > 0.08
    assert lid_pos[2] > 0.22
    assert button_pos[2] > 0.18
    assert switch_pos[2] < 0.08
    assert lid_pos[2] > button_pos[2] > switch_pos[2]

    ctx.expect_aabb_overlap("lid", "base_link", axes="xy", min_overlap=0.11)
    ctx.expect_aabb_contact("lid", "base_link")
    ctx.expect_aabb_contact("lid_release_button", "base_link")
    ctx.expect_aabb_contact("power_switch", "base_link")
    ctx.expect_joint_motion_axis("lid_hinge", "lid", world_axis="z", direction="positive", min_delta=0.02)
    ctx.expect_joint_motion_axis(
        "lid_release_button_hinge",
        "lid_release_button",
        world_axis="z",
        direction="negative",
        min_delta=0.003,
    )
    ctx.expect_joint_motion_axis(
        "power_switch_hinge",
        "power_switch",
        world_axis="x",
        direction="negative",
        min_delta=0.004,
    )

    with ctx.pose(lid_hinge=1.35):
        ctx.expect_aabb_contact("lid", "base_link")

    with ctx.pose(lid_release_button_hinge=-0.20):
        ctx.expect_aabb_contact("lid_release_button", "base_link")

    with ctx.pose(power_switch_hinge=0.45):
        ctx.expect_aabb_contact("power_switch", "base_link")

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
