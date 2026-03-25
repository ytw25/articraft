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
    BoxGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _cyl_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _build_heatpipe_mesh(index: int, x_offset: float):
    geom = tube_from_spline_points(
        [
            (x_offset, 0.022, -0.080),
            (x_offset, 0.022, -0.030),
            (x_offset, 0.022, 0.028),
            (x_offset, 0.024, 0.052),
            (x_offset, 0.038, 0.064),
            (x_offset, 0.052, 0.052),
            (x_offset, 0.053, 0.028),
            (x_offset, 0.053, -0.030),
            (x_offset, 0.053, -0.080),
        ],
        radius=0.0032,
        samples_per_segment=18,
        radial_segments=16,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_dir / f"heatpipe_{index}.obj")


def _build_blade_mesh():
    geom = (
        BoxGeometry((0.026, 0.0022, 0.014))
        .translate(0.028, 0.0, 0.0)
        .rotate_z(math.radians(28.0))
        .rotate_x(math.radians(16.0))
    )
    return mesh_from_geometry(geom, ASSETS.mesh_dir / "fan_blade.obj")


def _build_clip_mesh(side: str):
    x = -0.056 if side == "left" else 0.056
    outer = -0.062 if side == "left" else 0.062
    geom = wire_from_points(
        [
            (x, -0.010, 0.040),
            (outer, -0.004, 0.040),
            (outer, 0.015, 0.040),
            (x, 0.021, 0.047),
            (x, 0.021, -0.047),
            (outer, 0.015, -0.040),
            (outer, -0.004, -0.040),
            (x, -0.010, -0.040),
        ],
        radius=0.0014,
        radial_segments=10,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.003,
        corner_segments=6,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_dir / f"fan_clip_{side}.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cpu_fan_cooler", assets=ASSETS)

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    copper = model.material("copper", rgba=(0.77, 0.46, 0.28, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.62, 0.64, 0.68, 1.0))
    nickel = model.material("nickel", rgba=(0.69, 0.71, 0.74, 1.0))

    blade_mesh = _build_blade_mesh()
    left_clip_mesh = _build_clip_mesh("left")
    right_clip_mesh = _build_clip_mesh("right")
    heatpipe_specs = [
        ("heatpipe_outer_left", -0.029, _build_heatpipe_mesh(1, -0.029)),
        ("heatpipe_inner_left", -0.011, _build_heatpipe_mesh(2, -0.011)),
        ("heatpipe_inner_right", 0.011, _build_heatpipe_mesh(3, 0.011)),
        ("heatpipe_outer_right", 0.029, _build_heatpipe_mesh(4, 0.029)),
    ]

    heatsink = model.part("heatsink")
    heatsink.inertial = Inertial.from_geometry(
        Box((0.130, 0.090, 0.155)),
        mass=0.82,
        origin=Origin(xyz=(0.0, 0.036, -0.018)),
    )

    heatsink.visual(
        Box((0.045, 0.045, 0.010)),
        origin=Origin(xyz=(0.0, 0.0375, -0.089)),
        material=nickel,
        name="base_plate",
    )
    heatsink.visual(
        Box((0.042, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, 0.0375, -0.069)),
        material=nickel,
        name="mount_block",
    )
    heatsink.visual(
        Box((0.050, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, 0.0375, -0.050)),
        material=nickel,
        name="transition_block",
    )

    fin_width = 0.122
    fin_depth = 0.050
    fin_thickness = 0.0011
    fin_start_z = -0.038
    fin_pitch = 0.0040
    for index in range(24):
        z = fin_start_z + index * fin_pitch
        width_scale = 0.965 if index in (0, 1, 22, 23) else 1.0
        depth_scale = 0.94 if index in (0, 23) else 1.0
        heatsink.visual(
            Box((fin_width * width_scale, fin_depth * depth_scale, fin_thickness)),
            origin=Origin(xyz=(0.0, 0.0375, z)),
            material=aluminum,
            name=f"fin_{index:02d}",
        )

    heatsink.visual(
        Box((0.118, 0.046, 0.004)),
        origin=Origin(xyz=(0.0, 0.0375, 0.056)),
        material=dark_plastic,
        name="top_cap",
    )

    frame_side = 0.110
    frame_depth = 0.025
    frame_border = 0.010
    opening = frame_side - 2.0 * frame_border
    side_center = frame_side / 2.0 - frame_border / 2.0

    fan_frame = model.part("fan_frame")
    fan_frame.inertial = Inertial.from_geometry(
        Box((frame_side, frame_depth, frame_side)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    fan_frame.visual(
        Box((frame_side, frame_depth, frame_border)),
        origin=Origin(xyz=(0.0, 0.0, side_center)),
        material=black_plastic,
        name="frame_top",
    )
    fan_frame.visual(
        Box((frame_side, frame_depth, frame_border)),
        origin=Origin(xyz=(0.0, 0.0, -side_center)),
        material=black_plastic,
        name="frame_bottom",
    )
    fan_frame.visual(
        Box((frame_border, frame_depth, opening)),
        origin=Origin(xyz=(-side_center, 0.0, 0.0)),
        material=black_plastic,
        name="frame_left",
    )
    fan_frame.visual(
        Box((frame_border, frame_depth, opening)),
        origin=Origin(xyz=(side_center, 0.0, 0.0)),
        material=black_plastic,
        name="frame_right",
    )

    for x in (-0.050, 0.050):
        for z in (-0.050, 0.050):
            fan_frame.visual(
                Cylinder(radius=0.0055, length=frame_depth),
                origin=_cyl_origin(x, 0.0, z),
                material=dark_plastic,
                name=f"corner_pad_{'l' if x < 0.0 else 'r'}_{'b' if z < 0.0 else 't'}",
            )

    fan_frame.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=_cyl_origin(0.0, 0.0103, 0.0),
        material=dark_plastic,
        name="rear_motor_housing",
    )
    fan_frame.visual(
        Cylinder(radius=0.008, length=0.002),
        origin=_cyl_origin(0.0, 0.0114, 0.0),
        material=nickel,
        name="bearing_cap",
    )

    spoke_centers = [
        (0.023, 0.0102, 0.023, math.pi / 4.0),
        (-0.023, 0.0102, 0.023, 3.0 * math.pi / 4.0),
        (-0.023, 0.0102, -0.023, -3.0 * math.pi / 4.0),
        (0.023, 0.0102, -0.023, -math.pi / 4.0),
    ]
    for index, (x, y, z, yaw) in enumerate(spoke_centers):
        fan_frame.visual(
            Box((0.055, 0.0026, 0.0050)),
            origin=Origin(xyz=(x, y, z), rpy=(0.0, yaw, 0.0)),
            material=dark_plastic,
            name=f"stator_spoke_{index}",
        )

    heatsink.visual(left_clip_mesh, material=spring_steel, name="left_clip")
    heatsink.visual(right_clip_mesh, material=spring_steel, name="right_clip")

    for part_name, x_offset, mesh in heatpipe_specs:
        heatpipe = model.part(part_name)
        heatpipe.inertial = Inertial.from_geometry(
            Box((0.008, 0.034, 0.150)),
            mass=0.035,
            origin=Origin(xyz=(0.0, 0.0155, 0.064)),
        )
        heatpipe.visual(
            mesh,
            origin=Origin(xyz=(-x_offset, -0.022, 0.072)),
            material=copper,
            name=part_name,
        )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.044, length=0.020),
        mass=0.075,
        origin=_cyl_origin(0.0, 0.0, 0.0),
    )
    rotor.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=_cyl_origin(0.0, 0.0, 0.0),
        material=black_plastic,
        name="hub",
    )
    rotor.visual(
        Cylinder(radius=0.012, length=0.0014),
        origin=_cyl_origin(0.0, -0.0098, 0.0),
        material=spring_steel,
        name="hub_badge",
    )
    for blade_index in range(7):
        rotor.visual(
            blade_mesh,
            origin=Origin(rpy=(0.0, blade_index * (2.0 * math.pi / 7.0), 0.0)),
            material=black_plastic,
            name=f"blade_{blade_index}",
        )

    model.articulation(
        "heatsink_to_fan_frame",
        ArticulationType.FIXED,
        parent="heatsink",
        child="fan_frame",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    for part_name, x_offset, _ in heatpipe_specs:
        model.articulation(
            f"heatsink_to_{part_name}",
            ArticulationType.FIXED,
            parent="heatsink",
            child=part_name,
            origin=Origin(xyz=(x_offset, 0.022, -0.072)),
        )

    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent="fan_frame",
        child="rotor",
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=65.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("fan_frame", "heatsink")
    ctx.expect_aabb_overlap("fan_frame", "heatsink", axes="xz", min_overlap=0.100)
    ctx.expect_aabb_contact("heatpipe_outer_left", "heatsink")
    ctx.expect_aabb_contact("heatpipe_outer_right", "heatsink")
    ctx.expect_aabb_overlap("heatpipe_inner_left", "heatsink", axes="z", min_overlap=0.130)
    ctx.expect_aabb_overlap("heatpipe_inner_right", "heatsink", axes="z", min_overlap=0.130)
    ctx.expect_origin_distance("rotor", "fan_frame", axes="xz", max_dist=0.002)
    ctx.expect_aabb_overlap("rotor", "fan_frame", axes="xz", min_overlap=0.074)

    with ctx.pose(fan_spin=math.pi / 3.0):
        ctx.expect_origin_distance("rotor", "fan_frame", axes="xz", max_dist=0.002)
        ctx.expect_aabb_overlap("rotor", "fan_frame", axes="xz", min_overlap=0.073)

    with ctx.pose(fan_spin=math.pi):
        ctx.expect_origin_distance("rotor", "fan_frame", axes="xz", max_dist=0.002)
        ctx.expect_aabb_overlap("rotor", "fan_frame", axes="xz", min_overlap=0.074)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
