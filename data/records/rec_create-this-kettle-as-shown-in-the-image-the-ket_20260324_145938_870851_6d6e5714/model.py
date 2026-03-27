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
    CapsuleGeometry,
    Cylinder,
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)

BODY_HEIGHT = 0.226
BODY_RADIUS = 0.084
LID_HINGE_X = 0.056
TOP_Z = 0.228


def _spout_loop(x: float, z_center: float, half_width: float, half_height: float) -> list[tuple[float, float, float]]:
    return [
        (x, 0.0, z_center + half_height),
        (x, 0.55 * half_width, z_center + 0.78 * half_height),
        (x, half_width, z_center + 0.20 * half_height),
        (x, 0.82 * half_width, z_center - 0.38 * half_height),
        (x, 0.0, z_center - half_height),
        (x, -0.82 * half_width, z_center - 0.38 * half_height),
        (x, -half_width, z_center + 0.20 * half_height),
        (x, -0.55 * half_width, z_center + 0.78 * half_height),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="electric_kettle", assets=ASSETS)

    gloss_black = model.material("gloss_black", rgba=(0.10, 0.10, 0.11, 1.0))
    trim_black = model.material("trim_black", rgba=(0.16, 0.16, 0.17, 1.0))
    smoke = model.material("smoke", rgba=(0.72, 0.75, 0.77, 0.55))
    silver = model.material("silver", rgba=(0.76, 0.77, 0.79, 1.0))

    body = model.part("body")

    body_profile = [
        (0.0, 0.0),
        (0.076, 0.0),
        (0.080, 0.010),
        (0.083, 0.055),
        (0.084, 0.132),
        (0.082, 0.188),
        (0.078, 0.214),
        (0.072, BODY_HEIGHT),
        (0.0, BODY_HEIGHT),
    ]
    body_shell = mesh_from_geometry(
        LatheGeometry(body_profile, segments=56),
        ASSETS.mesh_path("kettle_body_shell.obj"),
    )
    body.visual(body_shell, material=gloss_black, name="body_shell")

    body.visual(
        Cylinder(radius=BODY_RADIUS, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=trim_black,
        name="base_ring",
    )
    body.visual(
        Cylinder(radius=0.072, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.226)),
        material=trim_black,
        name="lid_seat",
    )

    handle_shell = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.066, -0.008, 0.202),
                (0.106, -0.008, 0.208),
                (0.134, -0.010, 0.158),
                (0.134, -0.008, 0.084),
                (0.098, -0.006, 0.024),
            ],
            profile=rounded_rect_profile(0.018, 0.030, radius=0.005),
            samples_per_segment=18,
            cap_profile=True,
        ),
        ASSETS.mesh_path("kettle_handle_shell.obj"),
    )
    body.visual(handle_shell, material=gloss_black, name="handle_shell")

    spout_shell = mesh_from_geometry(
        section_loft(
            [
                _spout_loop(-0.070, 0.181, 0.020, 0.026),
                _spout_loop(-0.082, 0.185, 0.018, 0.024),
                _spout_loop(-0.096, 0.188, 0.015, 0.021),
                _spout_loop(-0.110, 0.188, 0.011, 0.016),
                _spout_loop(-0.122, 0.185, 0.008, 0.012),
            ]
        ),
        ASSETS.mesh_path("kettle_spout_shell.obj"),
    )
    body.visual(spout_shell, material=gloss_black, name="spout_shell")

    water_window_geom = CapsuleGeometry(radius=0.020, length=0.088)
    water_window_geom.scale(0.72, 0.28, 1.0).translate(0.0, 0.080, 0.110)
    body.visual(
        mesh_from_geometry(water_window_geom, ASSETS.mesh_path("kettle_water_window.obj")),
        material=smoke,
        name="water_window",
    )

    body.visual(
        Box((0.032, 0.026, 0.010)),
        origin=Origin(xyz=(0.104, -0.008, 0.221)),
        material=trim_black,
        name="button_seat",
    )
    body.visual(
        Box((0.022, 0.014, 0.006)),
        origin=Origin(xyz=(0.104, -0.008, 0.218)),
        material=trim_black,
        name="button_stop",
    )
    body.visual(
        Box((0.008, 0.020, 0.016)),
        origin=Origin(xyz=(0.080, 0.024, 0.031)),
        material=trim_black,
        name="switch_mount",
    )
    body.visual(
        Box((0.024, 0.012, 0.004)),
        origin=Origin(xyz=(0.094, 0.024, 0.026)),
        material=trim_black,
        name="switch_floor",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.268, 0.180, 0.240)),
        mass=1.2,
        origin=Origin(xyz=(0.014, 0.0, 0.120)),
    )

    lid = model.part("lid")
    lid_shell_geom = DomeGeometry(radius=(0.068, 0.068, 0.017), radial_segments=40, height_segments=14)
    lid_shell_geom.translate(-0.066, 0.0, 0.0)
    lid.visual(
        mesh_from_geometry(lid_shell_geom, ASSETS.mesh_path("kettle_lid_shell.obj")),
        material=gloss_black,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.066, length=0.003),
        origin=Origin(xyz=(-0.066, 0.0, 0.0015)),
        material=trim_black,
        name="lid_flange",
    )
    lid.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(-0.122, 0.0, 0.009)),
        material=gloss_black,
        name="lid_tip",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.068, length=0.018),
        mass=0.18,
        origin=Origin(xyz=(-0.066, 0.0, 0.009)),
    )

    button = model.part("lid_release_button")
    button_cap_geom = CapsuleGeometry(radius=0.010, length=0.010)
    button_cap_geom.rotate_x(math.pi / 2.0).scale(0.85, 1.0, 0.60)
    button.visual(
        mesh_from_geometry(button_cap_geom, ASSETS.mesh_path("kettle_button_cap.obj")),
        material=trim_black,
        name="button_cap",
    )
    button.inertial = Inertial.from_geometry(
        Box((0.024, 0.022, 0.012)),
        mass=0.02,
        origin=Origin(),
    )

    switch = model.part("power_switch")
    switch.visual(
        Box((0.024, 0.010, 0.006)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=trim_black,
        name="switch_arm",
    )
    switch.visual(
        Box((0.012, 0.012, 0.006)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=silver,
        name="switch_tip",
    )
    switch.inertial = Inertial.from_geometry(
        Box((0.036, 0.014, 0.008)),
        mass=0.03,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(LID_HINGE_X, 0.0, TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.05),
    )
    model.articulation(
        "body_to_lid_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.104, -0.008, 0.232)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.3, lower=0.0, upper=0.005),
    )
    model.articulation(
        "body_to_power_switch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=switch,
        origin=Origin(xyz=(0.084, 0.024, 0.031)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    button = object_model.get_part("lid_release_button")
    switch = object_model.get_part("power_switch")

    lid_hinge = object_model.get_articulation("body_to_lid")
    button_slide = object_model.get_articulation("body_to_lid_release_button")
    switch_hinge = object_model.get_articulation("body_to_power_switch")

    lid_seat = body.get_visual("lid_seat")
    spout_shell = body.get_visual("spout_shell")
    button_seat = body.get_visual("button_seat")
    button_stop = body.get_visual("button_stop")
    switch_mount = body.get_visual("switch_mount")
    switch_floor = body.get_visual("switch_floor")

    lid_flange = lid.get_visual("lid_flange")
    lid_tip = lid.get_visual("lid_tip")
    button_cap = button.get_visual("button_cap")
    switch_arm = switch.get_visual("switch_arm")
    switch_tip = switch.get_visual("switch_tip")

    ctx.allow_overlap(button, body, reason="the lid-release button nests into its handle recess when pressed")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lid_flange,
        negative_elem=lid_seat,
    )
    with ctx.pose({lid_hinge: 1.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.040,
            positive_elem=lid_tip,
            negative_elem=lid_seat,
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            min_gap=0.025,
            positive_elem=lid_tip,
            negative_elem=spout_shell,
        )

    ctx.expect_gap(
        button,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=button_cap,
        negative_elem=button_seat,
    )
    ctx.expect_gap(
        button,
        body,
        axis="z",
        min_gap=0.004,
        positive_elem=button_cap,
        negative_elem=button_stop,
    )
    with ctx.pose({button_slide: 0.005}):
        ctx.expect_gap(
            button,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=button_cap,
            negative_elem=button_stop,
        )

    ctx.expect_gap(
        switch,
        body,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=switch_arm,
        negative_elem=switch_mount,
    )
    ctx.expect_gap(
        switch,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=switch_tip,
        negative_elem=switch_floor,
    )
    with ctx.pose({switch_hinge: 0.35}):
        ctx.expect_gap(
            switch,
            body,
            axis="z",
            min_gap=0.006,
            positive_elem=switch_tip,
            negative_elem=switch_floor,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
