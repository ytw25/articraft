from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
import math

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

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
    repair_loft,
    section_loft,
)


ASSETS = AssetContext.from_script(__file__)

POLE_HEIGHT = 5.40
POLE_MOUNT_Z = 3.18
ARM_TIP_X = 0.98
PAN_CENTER_Z = -0.345
TILT_PIVOT_Z = 0.225


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _octagon_section(z: float, across_flats: float) -> list[tuple[float, float, float]]:
    radius = across_flats / (2.0 * math.cos(math.pi / 8.0))
    return [
        (
            radius * math.cos((math.pi / 8.0) + (index * math.pi / 4.0)),
            radius * math.sin((math.pi / 8.0) + (index * math.pi / 4.0)),
            z,
        )
        for index in range(8)
    ]


def _build_pole_mesh():
    return repair_loft(
        section_loft(
            [
                _octagon_section(0.00, 0.176),
                _octagon_section(1.90, 0.154),
                _octagon_section(3.80, 0.122),
                _octagon_section(POLE_HEIGHT, 0.094),
            ]
        )
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="street_camera_pole", assets=ASSETS)

    galvanized = model.material("galvanized", rgba=(0.57, 0.59, 0.61, 1.0))
    dark_paint = model.material("dark_paint", rgba=(0.20, 0.21, 0.23, 1.0))
    camera_white = model.material("camera_white", rgba=(0.85, 0.86, 0.88, 1.0))
    lens_black = model.material("lens_black", rgba=(0.07, 0.07, 0.08, 1.0))

    pole = model.part("pole")
    pole.visual(
        _save_mesh("street_camera_pole_shell.obj", _build_pole_mesh()),
        material=galvanized,
        name="pole_shell",
    )
    pole.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=POLE_HEIGHT),
        mass=138.0,
        origin=Origin(xyz=(0.0, 0.0, POLE_HEIGHT / 2.0)),
    )

    bracket = model.part("bracket")
    bracket.visual(
        Cylinder(radius=0.112, length=0.22),
        material=dark_paint,
        name="collar_band",
    )
    bracket.visual(
        Box((0.038, 0.058, 0.22)),
        origin=Origin(xyz=(0.0, 0.113, 0.0)),
        material=dark_paint,
        name="left_clamp_lug",
    )
    bracket.visual(
        Box((0.038, 0.058, 0.22)),
        origin=Origin(xyz=(0.0, -0.113, 0.0)),
        material=dark_paint,
        name="right_clamp_lug",
    )
    bracket.visual(
        Cylinder(radius=0.012, length=0.27),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_paint,
        name="clamp_bolt",
    )
    bracket.visual(
        Box((0.93, 0.10, 0.08)),
        origin=Origin(xyz=(0.50, 0.0, 0.025)),
        material=dark_paint,
        name="arm_beam",
    )
    bracket.visual(
        Box((0.18, 0.13, 0.10)),
        origin=Origin(xyz=(0.12, 0.0, 0.005)),
        material=dark_paint,
        name="weld_block",
    )
    bracket.visual(
        Box((0.30, 0.060, 0.060)),
        origin=Origin(xyz=(0.18, 0.0, -0.080), rpy=(0.0, math.radians(29.0), 0.0)),
        material=dark_paint,
        name="under_brace",
    )
    bracket.visual(
        Box((0.030, 0.120, 0.330)),
        origin=Origin(xyz=(ARM_TIP_X, 0.0, -0.180)),
        material=dark_paint,
        name="drop_plate",
    )
    bracket.visual(
        Cylinder(radius=0.026, length=0.12),
        origin=Origin(xyz=(ARM_TIP_X, 0.0, PAN_CENTER_Z)),
        material=dark_paint,
        name="mount_spigot",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((1.12, 0.26, 0.34)),
        mass=20.0,
        origin=Origin(xyz=(0.50, 0.0, -0.03)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.041, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_paint,
        name="bearing_collar",
    )
    pan_head.visual(
        Cylinder(radius=0.074, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=dark_paint,
        name="rotation_base",
    )
    pan_head.visual(
        Box((0.12, 0.048, 0.124)),
        origin=Origin(xyz=(0.055, 0.0, 0.164)),
        material=dark_paint,
        name="yoke_neck",
    )
    pan_head.visual(
        Cylinder(radius=0.009, length=0.145),
        origin=Origin(xyz=(0.055, 0.0, TILT_PIVOT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_paint,
        name="pivot_pin",
    )
    pan_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.08, length=0.30),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    tilt_bracket = model.part("tilt_bracket")
    tilt_bracket.visual(
        Cylinder(radius=0.017, length=0.160),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_paint,
        name="pivot_tube",
    )
    tilt_bracket.visual(
        Box((0.24, 0.012, 0.20)),
        origin=Origin(xyz=(0.120, 0.074, -0.110)),
        material=dark_paint,
        name="left_plate",
    )
    tilt_bracket.visual(
        Box((0.24, 0.012, 0.20)),
        origin=Origin(xyz=(0.120, -0.074, -0.110)),
        material=dark_paint,
        name="right_plate",
    )
    tilt_bracket.visual(
        Box((0.16, 0.148, 0.014)),
        origin=Origin(xyz=(0.180, 0.0, -0.214)),
        material=dark_paint,
        name="cradle_tray",
    )
    tilt_bracket.visual(
        Box((0.040, 0.148, 0.036)),
        origin=Origin(xyz=(0.120, 0.0, -0.155)),
        material=dark_paint,
        name="rear_bridge",
    )
    tilt_bracket.visual(
        Box((0.036, 0.148, 0.036)),
        origin=Origin(xyz=(0.235, 0.0, -0.170)),
        material=dark_paint,
        name="front_bridge",
    )
    tilt_bracket.inertial = Inertial.from_geometry(
        Box((0.40, 0.18, 0.25)),
        mass=4.5,
        origin=Origin(xyz=(0.14, 0.0, -0.11)),
    )

    camera = model.part("camera")
    camera.visual(
        Box((0.095, 0.10, 0.022)),
        origin=Origin(xyz=(0.035, 0.0, 0.011)),
        material=camera_white,
        name="camera_shoe",
    )
    camera.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.020, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="rear_cap",
    )
    camera.visual(
        Cylinder(radius=0.045, length=0.235),
        origin=Origin(xyz=(0.152, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="camera_body",
    )
    camera.visual(
        Cylinder(radius=0.034, length=0.078),
        origin=Origin(xyz=(0.3085, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=camera_white,
        name="camera_nose",
    )
    camera.visual(
        Cylinder(radius=0.022, length=0.026),
        origin=Origin(xyz=(0.3605, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_black,
        name="lens_barrel",
    )
    camera.visual(
        Box((0.110, 0.086, 0.016)),
        origin=Origin(xyz=(0.270, 0.0, 0.104)),
        material=camera_white,
        name="sunshade",
    )
    camera.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.34),
        mass=3.3,
        origin=Origin(xyz=(0.18, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "pole_to_bracket",
        ArticulationType.FIXED,
        parent=pole,
        child=bracket,
        origin=Origin(xyz=(0.0, 0.0, POLE_MOUNT_Z)),
    )
    model.articulation(
        "pan_rotate",
        ArticulationType.CONTINUOUS,
        parent=bracket,
        child=pan_head,
        origin=Origin(xyz=(ARM_TIP_X, 0.0, PAN_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5),
    )
    model.articulation(
        "tilt_pitch",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_bracket,
        origin=Origin(xyz=(0.055, 0.0, TILT_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "camera_mount",
        ArticulationType.FIXED,
        parent=tilt_bracket,
        child=camera,
        origin=Origin(xyz=(0.150, 0.0, -0.207)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pole = object_model.get_part("pole")
    bracket = object_model.get_part("bracket")
    pan_head = object_model.get_part("pan_head")
    tilt_bracket = object_model.get_part("tilt_bracket")
    camera = object_model.get_part("camera")
    pan_rotate = object_model.get_articulation("pan_rotate")
    tilt_pitch = object_model.get_articulation("tilt_pitch")

    pole_shell = pole.get_visual("pole_shell")
    collar_band = bracket.get_visual("collar_band")
    arm_beam = bracket.get_visual("arm_beam")
    drop_plate = bracket.get_visual("drop_plate")
    mount_spigot = bracket.get_visual("mount_spigot")
    bearing_collar = pan_head.get_visual("bearing_collar")
    rotation_base = pan_head.get_visual("rotation_base")
    pivot_pin = pan_head.get_visual("pivot_pin")
    pivot_tube = tilt_bracket.get_visual("pivot_tube")
    cradle_tray = tilt_bracket.get_visual("cradle_tray")
    camera_body = camera.get_visual("camera_body")
    camera_shoe = camera.get_visual("camera_shoe")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.warn_if_articulation_origin_near_geometry(tol=0.07)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(bracket, pan_head, reason="mount spigot nests inside the rotating bearing collar below the drop plate")
    ctx.allow_overlap(pan_head, tilt_bracket, reason="tilt bracket pivot tube wraps the horizontal pivot pin")
    ctx.allow_overlap(
        bracket,
        tilt_bracket,
        reason="the drop-mount plate nests between the tilt bracket cheeks and pivot tube in side-pan poses",
    )
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(bracket, pole, axes="xy", max_dist=0.001)
    ctx.expect_overlap(bracket, pole, axes="xy", min_overlap=0.01)
    ctx.expect_contact(bracket, pole, elem_a=collar_band, elem_b=pole_shell)
    ctx.expect_gap(
        pan_head,
        pole,
        axis="x",
        min_gap=0.80,
        positive_elem=bearing_collar,
        negative_elem=pole_shell,
    )
    ctx.expect_within(
        bracket,
        pan_head,
        axes="xy",
        inner_elem=mount_spigot,
        outer_elem=bearing_collar,
    )
    ctx.expect_overlap(tilt_bracket, pan_head, axes="xz", min_overlap=0.02)
    ctx.expect_within(
        pan_head,
        tilt_bracket,
        axes="y",
        inner_elem=pivot_pin,
        outer_elem=pivot_tube,
    )
    ctx.expect_within(
        camera,
        tilt_bracket,
        axes="y",
        inner_elem=camera_shoe,
    )
    ctx.expect_gap(
        camera,
        tilt_bracket,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=camera_shoe,
        negative_elem=cradle_tray,
    )
    ctx.expect_overlap(camera, tilt_bracket, axes="xz", min_overlap=0.04)
    with ctx.pose({pan_rotate: math.pi / 2.0}):
        ctx.expect_gap(
            camera,
            pole,
            axis="x",
            min_gap=0.75,
            positive_elem=camera_body,
            negative_elem=pole_shell,
        )
        ctx.expect_within(
            camera,
            tilt_bracket,
            axes="y",
            inner_elem=camera_shoe,
        )
    with ctx.pose({tilt_pitch: -0.35}):
        ctx.expect_gap(
            camera,
            bracket,
            axis="x",
            min_gap=0.22,
            positive_elem=camera_body,
            negative_elem=arm_beam,
        )
        ctx.expect_within(
            camera,
            tilt_bracket,
            axes="y",
            inner_elem=camera_shoe,
        )
    with ctx.pose({tilt_pitch: 0.30}):
        ctx.expect_overlap(
            pan_head,
            tilt_bracket,
            axes="xy",
            min_overlap=0.015,
        )
        ctx.expect_gap(
            pan_head,
            camera,
            axis="z",
            min_gap=0.015,
            positive_elem=rotation_base,
            negative_elem=camera_body,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
