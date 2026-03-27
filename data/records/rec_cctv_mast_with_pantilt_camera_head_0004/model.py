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
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _annular_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    radial_segments: int = 72,
):
    outer = CylinderGeometry(radius=outer_radius, height=length, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=length + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner)


def _add_lock_hardware(
    part,
    *,
    prefix: str,
    y_pos: float,
    z_pos: float,
    material,
) -> None:
    part.visual(
        Box((0.034, 0.020, 0.050)),
        origin=Origin(xyz=(0.0, y_pos, z_pos)),
        material=material,
        name=f"{prefix}_lock_block",
    )
    part.visual(
        Box((0.018, 0.012, 0.030)),
        origin=Origin(xyz=(0.025, y_pos, z_pos + 0.018), rpy=(0.0, 0.35, 0.0)),
        material=material,
        name=f"{prefix}_lock_ear",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.040),
        origin=Origin(xyz=(0.0, y_pos - 0.006, z_pos - 0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_lock_pin",
    )


def _build_pan_head(part, *, bearing_mesh, metal_dark, metal_mid) -> None:
    part.visual(
        bearing_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=metal_mid,
        name="bearing_collar",
    )
    part.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=metal_dark,
        name="drive_can",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=metal_mid,
        name="turntable_disk",
    )
    part.visual(
        Box((0.040, 0.024, 0.090)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=metal_dark,
        name="yoke_pedestal",
    )
    part.visual(
        Box((0.016, 0.016, 0.052)),
        origin=Origin(xyz=(-0.028, 0.0, 0.125)),
        material=metal_mid,
        name="left_yoke_arm",
    )
    part.visual(
        Box((0.016, 0.016, 0.052)),
        origin=Origin(xyz=(0.028, 0.0, 0.125)),
        material=metal_mid,
        name="right_yoke_arm",
    )
    part.visual(
        Box((0.072, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=metal_mid,
        name="rear_yoke_bridge",
    )
    part.visual(
        Box((0.020, 0.028, 0.014)),
        origin=Origin(xyz=(0.0, -0.010, 0.044)),
        material=metal_dark,
        name="service_box",
    )


def _build_tilt_camera(part, *, housing, lens_glass, metal_mid) -> None:
    part.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(-0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mid,
        name="left_trunnion_hub",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mid,
        name="right_trunnion_hub",
    )
    part.visual(
        Box((0.068, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.011, 0.0)),
        material=metal_mid,
        name="hub_bridge",
    )
    part.visual(
        Box((0.068, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=metal_mid,
        name="camera_saddle",
    )
    part.visual(
        Box((0.064, 0.058, 0.036)),
        origin=Origin(xyz=(0.0, 0.057, 0.012)),
        material=housing,
        name="camera_body",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.032),
        origin=Origin(xyz=(0.0, 0.091, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="lens_barrel",
    )
    part.visual(
        Box((0.048, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, 0.079, 0.031)),
        material=metal_mid,
        name="visor",
    )
    part.visual(
        Box((0.036, 0.012, 0.016)),
        origin=Origin(xyz=(0.0, 0.033, 0.010)),
        material=metal_mid,
        name="rear_module",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_camera_mast", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.26, 0.28, 0.30, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.55, 0.58, 0.62, 1.0))
    steel_light = model.material("steel_light", rgba=(0.74, 0.77, 0.80, 1.0))
    clamp_black = model.material("clamp_black", rgba=(0.11, 0.12, 0.13, 1.0))
    camera_grey = model.material("camera_grey", rgba=(0.22, 0.24, 0.26, 1.0))
    glass = model.material("glass", rgba=(0.36, 0.50, 0.58, 0.48))

    mast_socket_mesh = _save_mesh(
        "mast_socket.obj",
        _annular_shell(outer_radius=0.074, inner_radius=0.064, length=0.018),
    )
    outer_tube_mesh = _save_mesh(
        "outer_tube_shell.obj",
        _annular_shell(outer_radius=0.063, inner_radius=0.056, length=0.940),
    )
    outer_guide_sleeve_mesh = _save_mesh(
        "outer_guide_sleeve.obj",
        _annular_shell(outer_radius=0.056, inner_radius=0.0515, length=0.150),
    )
    outer_clamp_ring_mesh = _save_mesh(
        "outer_clamp_ring.obj",
        _annular_shell(outer_radius=0.073, inner_radius=0.063, length=0.056),
    )
    mid_tube_mesh = _save_mesh(
        "mid_tube_shell.obj",
        _annular_shell(outer_radius=0.050, inner_radius=0.045, length=0.840),
    )
    mid_guide_sleeve_mesh = _save_mesh(
        "mid_guide_sleeve.obj",
        _annular_shell(outer_radius=0.045, inner_radius=0.0420, length=0.126),
    )
    mid_clamp_ring_mesh = _save_mesh(
        "mid_clamp_ring.obj",
        _annular_shell(outer_radius=0.060, inner_radius=0.050, length=0.052),
    )
    inner_tube_mesh = _save_mesh(
        "inner_tube_shell.obj",
        _annular_shell(outer_radius=0.041, inner_radius=0.036, length=0.740),
    )
    top_mount_shoulder_mesh = _save_mesh(
        "top_mount_shoulder.obj",
        _annular_shell(outer_radius=0.052, inner_radius=0.041, length=0.020),
    )
    collar_base_mesh = _save_mesh(
        "crossarm_collar_base.obj",
        _annular_shell(outer_radius=0.056, inner_radius=0.043, length=0.014),
    )
    collar_shell_mesh = _save_mesh(
        "crossarm_collar_shell.obj",
        _annular_shell(outer_radius=0.055, inner_radius=0.043, length=0.110),
    )
    pan_bearing_mesh = _save_mesh(
        "pan_bearing_collar.obj",
        _annular_shell(outer_radius=0.036, inner_radius=0.020, length=0.022),
    )

    base_plate = model.part("base_plate")
    base_plate.visual(
        Box((0.350, 0.350, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=steel_dark,
        name="flange_plate",
    )
    base_plate.visual(
        mast_socket_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=steel_dark,
        name="mast_socket",
    )
    for x_pos in (-0.122, 0.122):
        for y_pos in (-0.122, 0.122):
            base_plate.visual(
                Cylinder(radius=0.015, length=0.008),
                origin=Origin(xyz=(x_pos, y_pos, 0.030)),
                material=steel_mid,
                name=f"anchor_boss_{'p' if x_pos > 0 else 'n'}x_{'p' if y_pos > 0 else 'n'}y",
            )
    base_plate.visual(
        Box((0.094, 0.022, 0.036)),
        origin=Origin(xyz=(0.0, 0.058, 0.044)),
        material=steel_dark,
        name="front_gusset",
    )
    base_plate.visual(
        Box((0.094, 0.022, 0.036)),
        origin=Origin(xyz=(0.0, -0.058, 0.044)),
        material=steel_dark,
        name="rear_gusset",
    )
    base_plate.visual(
        Box((0.022, 0.094, 0.036)),
        origin=Origin(xyz=(0.058, 0.0, 0.044)),
        material=steel_dark,
        name="right_gusset",
    )
    base_plate.visual(
        Box((0.022, 0.094, 0.036)),
        origin=Origin(xyz=(-0.058, 0.0, 0.044)),
        material=steel_dark,
        name="left_gusset",
    )
    base_plate.inertial = Inertial.from_geometry(
        Box((0.350, 0.350, 0.072)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    mast_outer = model.part("mast_outer")
    mast_outer.visual(
        outer_tube_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
        material=steel_light,
        name="outer_tube",
    )
    mast_outer.visual(
        outer_guide_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.812)),
        material=steel_mid,
        name="outer_guide_sleeve",
    )
    mast_outer.visual(
        outer_clamp_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.861)),
        material=clamp_black,
        name="outer_clamp_ring",
    )
    _add_lock_hardware(mast_outer, prefix="outer", y_pos=0.071, z_pos=0.861, material=clamp_black)
    mast_outer.inertial = Inertial.from_geometry(
        Cylinder(radius=0.073, length=0.940),
        mass=15.5,
        origin=Origin(xyz=(0.0, 0.0, 0.470)),
    )

    mast_mid = model.part("mast_mid")
    mast_mid.visual(
        mid_tube_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        material=steel_light,
        name="mid_tube",
    )
    mast_mid.visual(
        mid_guide_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.692)),
        material=steel_mid,
        name="mid_guide_sleeve",
    )
    mast_mid.visual(
        mid_clamp_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.792)),
        material=clamp_black,
        name="mid_clamp_ring",
    )
    _add_lock_hardware(mast_mid, prefix="mid", y_pos=0.061, z_pos=0.792, material=clamp_black)
    mast_mid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.060, length=0.840),
        mass=10.5,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
    )

    mast_inner = model.part("mast_inner")
    mast_inner.visual(
        inner_tube_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        material=steel_light,
        name="inner_tube",
    )
    mast_inner.visual(
        top_mount_shoulder_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.690)),
        material=steel_mid,
        name="top_mount_shoulder",
    )
    mast_inner.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.742)),
        material=clamp_black,
        name="top_cap",
    )
    mast_inner.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.740),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
    )

    crossarm_assembly = model.part("crossarm_assembly")
    crossarm_assembly.visual(
        collar_base_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=clamp_black,
        name="collar_base",
    )
    crossarm_assembly.visual(
        collar_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=clamp_black,
        name="collar_shell",
    )
    crossarm_assembly.visual(
        Box((0.056, 0.024, 0.118)),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=steel_dark,
        name="upright_web",
    )
    crossarm_assembly.visual(
        Box((0.118, 0.010, 0.066)),
        origin=Origin(xyz=(0.0, 0.018, 0.072), rpy=(0.0, 0.62, 0.0)),
        material=steel_dark,
        name="front_gusset",
    )
    crossarm_assembly.visual(
        Box((0.118, 0.010, 0.066)),
        origin=Origin(xyz=(0.0, -0.018, 0.072), rpy=(0.0, 0.62, 0.0)),
        material=steel_dark,
        name="rear_gusset",
    )
    crossarm_assembly.visual(
        Box((0.780, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.144)),
        material=steel_mid,
        name="crossarm_beam",
    )
    crossarm_assembly.visual(
        Box((0.094, 0.078, 0.014)),
        origin=Origin(xyz=(-0.338, 0.0, 0.170)),
        material=steel_dark,
        name="left_mount_pad",
    )
    crossarm_assembly.visual(
        Box((0.094, 0.078, 0.014)),
        origin=Origin(xyz=(0.338, 0.0, 0.170)),
        material=steel_dark,
        name="right_mount_pad",
    )
    crossarm_assembly.inertial = Inertial.from_geometry(
        Box((0.780, 0.120, 0.200)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
    )

    left_pan_head = model.part("left_pan_head")
    _build_pan_head(left_pan_head, bearing_mesh=pan_bearing_mesh, metal_dark=steel_dark, metal_mid=steel_mid)
    left_pan_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.092),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
    )

    left_tilt_camera = model.part("left_tilt_camera")
    _build_tilt_camera(left_tilt_camera, housing=camera_grey, lens_glass=glass, metal_mid=steel_mid)
    left_tilt_camera.inertial = Inertial.from_geometry(
        Box((0.090, 0.120, 0.060)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.050, 0.012)),
    )

    right_pan_head = model.part("right_pan_head")
    _build_pan_head(right_pan_head, bearing_mesh=pan_bearing_mesh, metal_dark=steel_dark, metal_mid=steel_mid)
    right_pan_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.092),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
    )

    right_tilt_camera = model.part("right_tilt_camera")
    _build_tilt_camera(right_tilt_camera, housing=camera_grey, lens_glass=glass, metal_mid=steel_mid)
    right_tilt_camera.inertial = Inertial.from_geometry(
        Box((0.090, 0.120, 0.060)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.050, 0.012)),
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.FIXED,
        parent=base_plate,
        child=mast_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
    )
    model.articulation(
        "outer_to_mid",
        ArticulationType.PRISMATIC,
        parent=mast_outer,
        child=mast_mid,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.260),
    )
    model.articulation(
        "mid_to_inner",
        ArticulationType.PRISMATIC,
        parent=mast_mid,
        child=mast_inner,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.220),
    )
    model.articulation(
        "inner_to_crossarm",
        ArticulationType.FIXED,
        parent=mast_inner,
        child=crossarm_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
    )
    model.articulation(
        "crossarm_to_left_pan",
        ArticulationType.CONTINUOUS,
        parent=crossarm_assembly,
        child=left_pan_head,
        origin=Origin(xyz=(-0.338, 0.0, 0.177)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5),
    )
    model.articulation(
        "left_pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=left_pan_head,
        child=left_tilt_camera,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.75, upper=0.55),
    )
    model.articulation(
        "crossarm_to_right_pan",
        ArticulationType.CONTINUOUS,
        parent=crossarm_assembly,
        child=right_pan_head,
        origin=Origin(xyz=(0.338, 0.0, 0.177)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5),
    )
    model.articulation(
        "right_pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=right_pan_head,
        child=right_tilt_camera,
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.75, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_plate = object_model.get_part("base_plate")
    mast_outer = object_model.get_part("mast_outer")
    mast_mid = object_model.get_part("mast_mid")
    mast_inner = object_model.get_part("mast_inner")
    crossarm_assembly = object_model.get_part("crossarm_assembly")
    left_pan_head = object_model.get_part("left_pan_head")
    left_tilt_camera = object_model.get_part("left_tilt_camera")
    right_pan_head = object_model.get_part("right_pan_head")
    right_tilt_camera = object_model.get_part("right_tilt_camera")

    outer_to_mid = object_model.get_articulation("outer_to_mid")
    mid_to_inner = object_model.get_articulation("mid_to_inner")
    left_pan = object_model.get_articulation("crossarm_to_left_pan")
    left_tilt = object_model.get_articulation("left_pan_to_tilt")
    right_pan = object_model.get_articulation("crossarm_to_right_pan")
    right_tilt = object_model.get_articulation("right_pan_to_tilt")

    flange_plate = base_plate.get_visual("flange_plate")
    mast_socket = base_plate.get_visual("mast_socket")
    outer_tube = mast_outer.get_visual("outer_tube")
    outer_guide_sleeve = mast_outer.get_visual("outer_guide_sleeve")
    mid_tube = mast_mid.get_visual("mid_tube")
    mid_guide_sleeve = mast_mid.get_visual("mid_guide_sleeve")
    inner_tube = mast_inner.get_visual("inner_tube")
    top_mount_shoulder = mast_inner.get_visual("top_mount_shoulder")
    collar_base = crossarm_assembly.get_visual("collar_base")
    crossarm_beam = crossarm_assembly.get_visual("crossarm_beam")
    left_mount_pad = crossarm_assembly.get_visual("left_mount_pad")
    right_mount_pad = crossarm_assembly.get_visual("right_mount_pad")
    left_bearing_collar = left_pan_head.get_visual("bearing_collar")
    left_left_yoke_arm = left_pan_head.get_visual("left_yoke_arm")
    left_right_yoke_arm = left_pan_head.get_visual("right_yoke_arm")
    right_bearing_collar = right_pan_head.get_visual("bearing_collar")
    right_left_yoke_arm = right_pan_head.get_visual("left_yoke_arm")
    right_right_yoke_arm = right_pan_head.get_visual("right_yoke_arm")
    left_left_trunnion_hub = left_tilt_camera.get_visual("left_trunnion_hub")
    left_right_trunnion_hub = left_tilt_camera.get_visual("right_trunnion_hub")
    right_left_trunnion_hub = right_tilt_camera.get_visual("left_trunnion_hub")
    right_right_trunnion_hub = right_tilt_camera.get_visual("right_trunnion_hub")
    left_camera_body = left_tilt_camera.get_visual("camera_body")
    right_camera_body = right_tilt_camera.get_visual("camera_body")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.060)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(left_pan_head, left_tilt_camera, reason="tilt trunnion hubs nest inside the yoke cheeks")
    ctx.allow_overlap(right_pan_head, right_tilt_camera, reason="tilt trunnion hubs nest inside the yoke cheeks")
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_gap(
        mast_outer,
        base_plate,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=outer_tube,
        negative_elem=mast_socket,
        name="outer_mast_seated_on_base_socket",
    )
    ctx.expect_within(
        mast_mid,
        mast_outer,
        axes="xy",
        inner_elem=mid_tube,
        outer_elem=outer_tube,
        name="mid_section_nests_within_outer_section",
    )
    ctx.expect_within(
        mast_mid,
        mast_outer,
        axes="xy",
        inner_elem=mid_tube,
        outer_elem=outer_guide_sleeve,
        name="mid_section_runs_in_outer_guide_sleeve",
    )
    ctx.expect_within(
        mast_inner,
        mast_mid,
        axes="xy",
        inner_elem=inner_tube,
        outer_elem=mid_tube,
        name="inner_section_nests_within_mid_section",
    )
    ctx.expect_within(
        mast_inner,
        mast_mid,
        axes="xy",
        inner_elem=inner_tube,
        outer_elem=mid_guide_sleeve,
        name="inner_section_runs_in_mid_guide_sleeve",
    )
    ctx.expect_contact(
        crossarm_assembly,
        mast_inner,
        elem_a=collar_base,
        elem_b=top_mount_shoulder,
        name="crossarm_collar_seated_on_top_section_shoulder",
    )
    ctx.expect_overlap(
        crossarm_assembly,
        mast_inner,
        axes="xy",
        elem_a=collar_base,
        elem_b=top_mount_shoulder,
        min_overlap=0.02,
        name="crossarm_collar_is_centered_on_mast",
    )
    ctx.expect_contact(
        left_pan_head,
        crossarm_assembly,
        elem_a=left_bearing_collar,
        elem_b=left_mount_pad,
        name="left_pan_head_seated_on_crossarm_pad",
    )
    ctx.expect_overlap(
        left_pan_head,
        crossarm_assembly,
        axes="xy",
        elem_a=left_bearing_collar,
        elem_b=left_mount_pad,
        min_overlap=0.03,
        name="left_pan_head_has_full_mount_footprint",
    )
    ctx.expect_contact(
        right_pan_head,
        crossarm_assembly,
        elem_a=right_bearing_collar,
        elem_b=right_mount_pad,
        name="right_pan_head_seated_on_crossarm_pad",
    )
    ctx.expect_overlap(
        right_pan_head,
        crossarm_assembly,
        axes="xy",
        elem_a=right_bearing_collar,
        elem_b=right_mount_pad,
        min_overlap=0.03,
        name="right_pan_head_has_full_mount_footprint",
    )
    ctx.expect_contact(
        left_tilt_camera,
        left_pan_head,
        elem_a=left_left_trunnion_hub,
        elem_b=left_left_yoke_arm,
        name="left_outer_yoke_cheek_captures_trunnion_hub",
    )
    ctx.expect_contact(
        left_tilt_camera,
        left_pan_head,
        elem_a=left_right_trunnion_hub,
        elem_b=left_right_yoke_arm,
        name="left_inner_yoke_cheek_captures_trunnion_hub",
    )
    ctx.expect_contact(
        right_tilt_camera,
        right_pan_head,
        elem_a=right_left_trunnion_hub,
        elem_b=right_left_yoke_arm,
        name="right_inner_yoke_cheek_captures_trunnion_hub",
    )
    ctx.expect_contact(
        right_tilt_camera,
        right_pan_head,
        elem_a=right_right_trunnion_hub,
        elem_b=right_right_yoke_arm,
        name="right_outer_yoke_cheek_captures_trunnion_hub",
    )
    ctx.expect_gap(
        mast_inner,
        left_pan_head,
        axis="x",
        min_gap=0.240,
        positive_elem=inner_tube,
        negative_elem=left_bearing_collar,
        name="left_pan_head_is_outboard_of_mast",
    )
    ctx.expect_gap(
        right_pan_head,
        mast_inner,
        axis="x",
        min_gap=0.240,
        positive_elem=right_bearing_collar,
        negative_elem=inner_tube,
        name="right_pan_head_is_outboard_of_mast",
    )

    with ctx.pose({outer_to_mid: 0.260, mid_to_inner: 0.220}):
        ctx.expect_within(
            mast_mid,
            mast_outer,
            axes="xy",
            inner_elem=mid_tube,
            outer_elem=outer_guide_sleeve,
            name="mid_section_stays_guided_when_extended",
        )
        ctx.expect_within(
            mast_inner,
            mast_mid,
            axes="xy",
            inner_elem=inner_tube,
            outer_elem=mid_guide_sleeve,
            name="inner_section_stays_guided_when_extended",
        )
        ctx.expect_gap(
            crossarm_assembly,
            base_plate,
            axis="z",
            min_gap=1.520,
            positive_elem=crossarm_beam,
            negative_elem=flange_plate,
            name="extended_mast_lifts_crossarm_well_above_base",
        )

    with ctx.pose(
        {
            outer_to_mid: 0.180,
            mid_to_inner: 0.170,
            left_pan: 0.90,
            left_tilt: -0.70,
            right_pan: -1.10,
            right_tilt: 0.40,
        }
    ):
        ctx.expect_gap(
            left_tilt_camera,
            crossarm_assembly,
            axis="z",
            min_gap=0.010,
            positive_elem=left_camera_body,
            negative_elem=crossarm_beam,
            name="left_camera_clears_crossarm_in_working_pose",
        )
        ctx.expect_gap(
            right_tilt_camera,
            crossarm_assembly,
            axis="z",
            min_gap=0.010,
            positive_elem=right_camera_body,
            negative_elem=crossarm_beam,
            name="right_camera_clears_crossarm_in_working_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
