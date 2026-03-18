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
    Inertial,
    LouverPanelGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BASE_W = 0.36
BASE_D = 0.26
BASE_H = 0.02
SHELL_W = 0.34
SHELL_D = 0.24
SHELL_H = 0.64
SHELL_RADIUS = 0.048
SHELL_TOP_Z = BASE_H + SHELL_H

DOOR_W = 0.288
DOOR_H = 0.49
DOOR_T = 0.018
DOOR_CENTER_Z = 0.33
DOOR_HINGE_X = DOOR_W / 2.0
DOOR_HINGE_Y = SHELL_D / 2.0 + 0.004

KNOB_POS = (0.052, 0.03, SHELL_TOP_Z + 0.004)
BUTTON_POS = (-0.05, 0.036, SHELL_TOP_Z + 0.004)
BUTTON_TRAVEL = 0.0025
DOOR_OPEN_ANGLE = 1.2


def _named_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    for kwargs in (
        {"name": name, "color": rgba},
        {"name": name, "rgba": rgba},
    ):
        try:
            return Material(**kwargs)
        except TypeError:
            continue
    try:
        return Material(name, color=rgba)
    except TypeError:
        return Material(name, rgba=rgba)


SHELL_MAT = _named_material("shell_white", (0.93, 0.94, 0.92, 1.0))
CHARCOAL_MAT = _named_material("charcoal_abs", (0.14, 0.15, 0.16, 1.0))
GLASS_MAT = _named_material("smoked_glass", (0.10, 0.11, 0.12, 0.9))
METAL_MAT = _named_material("brushed_aluminum", (0.63, 0.65, 0.67, 1.0))
RUBBER_MAT = _named_material("soft_rubber", (0.07, 0.08, 0.08, 1.0))


def _rounded_prism_mesh(
    name: str,
    width: float,
    depth: float,
    height: float,
    radius: float,
    *,
    center: bool,
):
    safe_radius = max(0.0, min(radius, width / 2.0 - 1e-6, depth / 2.0 - 1e-6))
    profile = rounded_rect_profile(
        width,
        depth,
        radius=safe_radius,
        corner_segments=10,
    )
    geometry = ExtrudeGeometry(profile, height, center=center)
    return mesh_from_geometry(geometry, ASSETS.mesh_path(f"{name}.obj"))


def _louver_mesh(
    name: str,
    panel_size: tuple[float, float],
    thickness: float,
    *,
    frame: float,
    slat_pitch: float,
    slat_width: float,
    slat_angle_deg: float,
    corner_radius: float,
):
    geometry = LouverPanelGeometry(
        panel_size=panel_size,
        thickness=thickness,
        frame=frame,
        slat_pitch=slat_pitch,
        slat_width=slat_width,
        slat_angle_deg=slat_angle_deg,
        corner_radius=corner_radius,
        center=True,
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path(f"{name}.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="air_purifier", assets=ASSETS)

    body = model.part("body")
    body.visual(
        _rounded_prism_mesh(
            "base_plinth",
            BASE_W,
            BASE_D,
            BASE_H,
            radius=0.055,
            center=False,
        ),
        material=CHARCOAL_MAT,
    )
    body.visual(
        _rounded_prism_mesh(
            "main_shell",
            SHELL_W,
            SHELL_D,
            SHELL_H,
            radius=SHELL_RADIUS,
            center=False,
        ),
        origin=Origin(xyz=(0.0, 0.0, BASE_H)),
        material=SHELL_MAT,
    )
    body.visual(
        _rounded_prism_mesh(
            "top_cap",
            0.322,
            0.222,
            0.012,
            radius=0.038,
            center=False,
        ),
        origin=Origin(xyz=(0.0, 0.0, SHELL_TOP_Z - 0.006)),
        material=SHELL_MAT,
    )
    body.visual(
        _louver_mesh(
            "top_exhaust",
            (0.202, 0.108),
            0.006,
            frame=0.01,
            slat_pitch=0.018,
            slat_width=0.009,
            slat_angle_deg=28.0,
            corner_radius=0.01,
        ),
        origin=Origin(xyz=(0.0, -0.03, SHELL_TOP_Z - 0.002)),
        material=CHARCOAL_MAT,
    )
    body.visual(
        _rounded_prism_mesh(
            "control_deck",
            0.162,
            0.104,
            0.006,
            radius=0.024,
            center=False,
        ),
        origin=Origin(xyz=(0.0, 0.03, SHELL_TOP_Z - 0.002)),
        material=GLASS_MAT,
    )
    body.visual(
        _louver_mesh(
            "rear_intake",
            (0.19, 0.28),
            0.006,
            frame=0.008,
            slat_pitch=0.02,
            slat_width=0.008,
            slat_angle_deg=22.0,
            corner_radius=0.008,
        ),
        origin=Origin(
            xyz=(0.0, -SHELL_D / 2.0 + 0.003, 0.305),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=CHARCOAL_MAT,
    )
    body.visual(
        Box((0.182, 0.008, 0.062)),
        origin=Origin(xyz=(0.0, SHELL_D / 2.0 - 0.003, 0.58)),
        material=GLASS_MAT,
    )
    body.visual(
        Box((0.128, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, SHELL_D / 2.0 - 0.0015, 0.536)),
        material=METAL_MAT,
    )
    body.visual(
        Cylinder(radius=0.03, length=0.004),
        origin=Origin(xyz=(KNOB_POS[0], KNOB_POS[1], SHELL_TOP_Z + 0.002)),
        material=METAL_MAT,
    )
    body.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(BUTTON_POS[0], BUTTON_POS[1], SHELL_TOP_Z + 0.002)),
        material=METAL_MAT,
    )
    body.inertial = Inertial.from_geometry(
        Box((BASE_W, BASE_D, SHELL_TOP_Z + 0.008)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, (SHELL_TOP_Z + 0.008) / 2.0)),
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        _rounded_prism_mesh(
            "filter_door_panel",
            DOOR_W,
            DOOR_T,
            DOOR_H,
            radius=0.022,
            center=True,
        ),
        origin=Origin(xyz=(-DOOR_W / 2.0, DOOR_T / 2.0, 0.0)),
        material=SHELL_MAT,
    )
    filter_door.visual(
        _louver_mesh(
            "filter_door_grille",
            (0.214, 0.274),
            0.006,
            frame=0.007,
            slat_pitch=0.022,
            slat_width=0.009,
            slat_angle_deg=26.0,
            corner_radius=0.008,
        ),
        origin=Origin(
            xyz=(-DOOR_W / 2.0, DOOR_T - 0.002, -0.055),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=CHARCOAL_MAT,
    )
    filter_door.visual(
        Box((0.018, 0.012, 0.095)),
        origin=Origin(xyz=(-DOOR_W + 0.025, DOOR_T + 0.004, 0.02)),
        material=METAL_MAT,
    )
    filter_door.inertial = Inertial.from_geometry(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        mass=1.1,
        origin=Origin(xyz=(-DOOR_W / 2.0, DOOR_T / 2.0, 0.0)),
    )

    control_knob = model.part("control_knob")
    control_knob.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=RUBBER_MAT,
    )
    control_knob.visual(
        Cylinder(radius=0.02, length=0.009),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=METAL_MAT,
    )
    control_knob.visual(
        Box((0.012, 0.005, 0.003)),
        origin=Origin(xyz=(0.01, 0.0, 0.0235)),
        material=CHARCOAL_MAT,
    )
    control_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=0.0225),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.01225)),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.0065, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=RUBBER_MAT,
    )
    power_button.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=CHARCOAL_MAT,
    )
    power_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.012),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    model.articulation(
        "filter_door_hinge",
        ArticulationType.REVOLUTE,
        parent="body",
        child="filter_door",
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "control_knob_spin",
        ArticulationType.CONTINUOUS,
        parent="body",
        child="control_knob",
        origin=Origin(xyz=KNOB_POS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=6.0,
        ),
    )
    model.articulation(
        "power_button_slide",
        ArticulationType.PRISMATIC,
        parent="body",
        child="power_button",
        origin=Origin(xyz=BUTTON_POS),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.2,
            lower=0.0,
            upper=BUTTON_TRAVEL,
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
        "power_button",
        "body",
        reason="the spring-loaded button stem rides inside an unmodeled actuator well under the solid control deck visual",
    )
    ctx.check_no_overlaps(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_xy_distance("filter_door", "body", max_dist=0.2)
    ctx.expect_joint_motion_axis(
        "filter_door_hinge",
        "filter_door",
        world_axis="y",
        direction="positive",
        min_delta=0.05,
    )

    ctx.expect_aabb_overlap_xy("control_knob", "body", min_overlap=0.03)
    ctx.expect_xy_distance("control_knob", "body", max_dist=0.09)
    ctx.expect_aabb_gap_z(
        "control_knob",
        "body",
        max_gap=0.03,
        max_penetration=0.002,
    )

    ctx.expect_aabb_overlap_xy("power_button", "body", min_overlap=0.02)
    ctx.expect_xy_distance("power_button", "body", max_dist=0.09)
    ctx.expect_aabb_gap_z(
        "power_button",
        "body",
        max_gap=0.02,
        max_penetration=0.008,
    )
    ctx.expect_joint_motion_axis(
        "power_button_slide",
        "power_button",
        world_axis="z",
        direction="negative",
        min_delta=0.002,
    )

    with ctx.pose(filter_door_hinge=DOOR_OPEN_ANGLE):
        ctx.expect_xy_distance("filter_door", "body", max_dist=0.31)

    with ctx.pose(control_knob_spin=math.pi / 2.0):
        ctx.expect_aabb_overlap_xy("control_knob", "body", min_overlap=0.03)
        ctx.expect_aabb_gap_z(
            "control_knob",
            "body",
            max_gap=0.03,
            max_penetration=0.002,
        )

    with ctx.pose(power_button_slide=BUTTON_TRAVEL):
        ctx.expect_aabb_gap_z(
            "power_button",
            "body",
            max_gap=0.02,
            max_penetration=0.01,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
