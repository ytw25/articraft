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
    Cylinder,
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, MESH_DIR / name)


def _rect_loop_x(x: float, half_y: float, z_bottom: float, z_top: float):
    return [
        (x, -half_y, z_bottom),
        (x, half_y, z_bottom),
        (x, half_y, z_top),
        (x, -half_y, z_top),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="water_kettle", assets=ASSETS)

    black_plastic = model.material("black_plastic", rgba=(0.08, 0.08, 0.09, 1.0))
    charcoal = model.material("charcoal", rgba=(0.13, 0.13, 0.14, 1.0))
    smoky_glass = model.material("smoky_glass", rgba=(0.76, 0.85, 0.92, 0.34))
    water_window = model.material("water_window", rgba=(0.84, 0.87, 0.90, 0.56))
    clear_plastic = model.material("clear_plastic", rgba=(0.88, 0.91, 0.95, 0.40))

    shell_profile = [
        (0.0, 0.0),
        (0.061, 0.0),
        (0.068, 0.008),
        (0.071, 0.035),
        (0.072, 0.120),
        (0.074, 0.184),
        (0.074, 0.191),
        (0.071, 0.198),
        (0.064, 0.206),
        (0.058, 0.206),
        (0.056, 0.198),
        (0.056, 0.018),
        (0.050, 0.010),
        (0.0, 0.010),
    ]
    shell_geom = LatheGeometry(shell_profile, segments=72)
    handle_geom = sweep_profile_along_spline(
        [
            (0.0, 0.071, 0.168),
            (0.030, 0.114, 0.145),
            (0.032, 0.122, 0.095),
            (0.026, 0.114, 0.048),
            (0.0, 0.080, 0.016),
        ],
        profile=rounded_rect_profile(0.026, 0.018, radius=0.006, corner_segments=8),
        samples_per_segment=18,
        cap_profile=True,
    )
    spout_geom = section_loft(
        (
            _rect_loop_x(-0.071, 0.017, 0.181, 0.193),
            _rect_loop_x(-0.091, 0.008, 0.184, 0.199),
        )
    )
    lid_dome = _save_mesh(
        "kettle_lid_dome.obj",
        DomeGeometry(radius=(0.065, 0.060, 0.010), radial_segments=40, height_segments=12),
    )
    shell_mesh = _save_mesh("kettle_shell.obj", shell_geom)
    handle_mesh = _save_mesh("kettle_handle.obj", handle_geom)
    spout_mesh = _save_mesh("kettle_spout.obj", spout_geom)

    base = model.part("base_link")
    base.visual(shell_mesh, material=black_plastic)
    base.visual(handle_mesh, material=black_plastic)
    base.visual(spout_mesh, material=black_plastic)
    base.visual(
        Box((0.036, 0.054, 0.022)),
        origin=Origin(xyz=(0.0, 0.095, 0.164), rpy=(-0.42, 0.0, 0.0)),
        material=black_plastic,
    )
    base.visual(
        Cylinder(radius=0.061, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=charcoal,
    )
    base.visual(
        Box((0.024, 0.003, 0.112)),
        origin=Origin(xyz=(0.069, 0.0, 0.092), rpy=(0.0, math.radians(7.0), 0.0)),
        material=water_window,
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.074, length=0.190),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.071, length=0.006),
        origin=Origin(xyz=(0.0, -0.059, 0.003)),
        material=black_plastic,
    )
    lid.visual(
        lid_dome,
        origin=Origin(xyz=(0.0, -0.060, 0.006)),
        material=smoky_glass,
    )
    lid.visual(
        Box((0.036, 0.052, 0.012)),
        origin=Origin(xyz=(0.0, -0.022, 0.008), rpy=(0.12, 0.0, 0.0)),
        material=black_plastic,
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.071, length=0.014),
        mass=0.18,
        origin=Origin(xyz=(0.0, -0.052, 0.007)),
    )

    button = model.part("lid_release_button")
    button.visual(
        Box((0.026, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, -0.012, 0.004)),
        material=black_plastic,
    )
    button.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.0, -0.005, 0.010)),
        material=charcoal,
    )
    button.inertial = Inertial.from_geometry(
        Box((0.026, 0.022, 0.010)),
        mass=0.025,
        origin=Origin(xyz=(0.0, -0.012, 0.004)),
    )

    switch = model.part("power_switch")
    switch.visual(
        Box((0.014, 0.018, 0.038)),
        origin=Origin(xyz=(0.0, 0.006, -0.016)),
        material=black_plastic,
    )
    switch.visual(
        Box((0.016, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.006, -0.036)),
        material=clear_plastic,
    )
    switch.inertial = Inertial.from_geometry(
        Box((0.016, 0.020, 0.042)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.006, -0.018)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent="base_link",
        child="lid",
        origin=Origin(xyz=(0.0, 0.062, 0.184)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.18),
    )
    model.articulation(
        "lid_release_button_hinge",
        ArticulationType.REVOLUTE,
        parent="base_link",
        child="lid_release_button",
        origin=Origin(xyz=(0.0, 0.104, 0.173)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-0.35, upper=0.0),
    )
    model.articulation(
        "power_switch_hinge",
        ArticulationType.REVOLUTE,
        parent="base_link",
        child="power_switch",
        origin=Origin(xyz=(0.0, 0.082, 0.056)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-0.40, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    ctx.allow_overlap("lid", "lid_release_button", reason="release button nests under the lid shroud near the handle")
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    expected_parts = {"base_link", "lid", "lid_release_button", "power_switch"}
    if {part.name for part in object_model.parts} != expected_parts:
        raise AssertionError("Model must expose exactly the requested kettle link names.")

    expected_joint_names = {"lid_hinge", "lid_release_button_hinge", "power_switch_hinge"}
    if {joint.name for joint in object_model.articulations} != expected_joint_names:
        raise AssertionError("Model must expose the three revolute joints for lid, button, and switch.")
    for joint in object_model.articulations:
        if joint.parent != "base_link":
            raise AssertionError(f"Joint {joint.name} should attach directly to base_link.")
        if joint.child not in expected_parts - {"base_link"}:
            raise AssertionError(f"Joint {joint.name} drives an unexpected child link.")
        if joint.articulation_type != ArticulationType.REVOLUTE:
            raise AssertionError(f"Joint {joint.name} must be revolute.")

    ctx.expect_aabb_contact("lid", "base_link")
    ctx.expect_aabb_overlap("lid", "base_link", axes="xy", min_overlap=0.09)
    ctx.expect_joint_motion_axis(
        "lid_hinge",
        "lid",
        world_axis="z",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "lid_hinge",
        "lid",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )

    ctx.expect_aabb_contact("lid_release_button", "base_link")
    ctx.expect_joint_motion_axis(
        "lid_release_button_hinge",
        "lid_release_button",
        world_axis="z",
        direction="positive",
        min_delta=0.002,
    )

    ctx.expect_aabb_contact("power_switch", "base_link")
    ctx.expect_joint_motion_axis(
        "power_switch_hinge",
        "power_switch",
        world_axis="z",
        direction="positive",
        min_delta=0.004,
    )

    lid_pos = ctx.part_world_position("lid")
    button_pos = ctx.part_world_position("lid_release_button")
    switch_pos = ctx.part_world_position("power_switch")
    if button_pos[1] < 0.08 or button_pos[2] < 0.16:
        raise AssertionError("Lid release button should sit high on the handle top.")
    if switch_pos[1] < 0.07 or switch_pos[2] > 0.10:
        raise AssertionError("Power switch should sit low near the handle root.")
    if button_pos[2] <= switch_pos[2] + 0.06:
        raise AssertionError("Button and power switch should be clearly separated vertically on the handle.")
    if lid_pos[2] <= button_pos[2]:
        raise AssertionError("Closed lid should sit above the release button.")

    with ctx.pose(lid_release_button_hinge=-0.30):
        ctx.expect_aabb_contact("lid_release_button", "base_link")
    with ctx.pose(power_switch_hinge=0.30):
        ctx.expect_aabb_contact("power_switch", "base_link")

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
