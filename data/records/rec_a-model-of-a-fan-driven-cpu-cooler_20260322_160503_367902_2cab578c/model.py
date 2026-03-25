from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
MESH_DIR = ASSETS.mesh_dir


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cpu_cooler", assets=ASSETS)

    # Materials
    nickel = model.material("nickel", rgba=(0.8, 0.8, 0.85, 1.0))
    aluminum = model.material("aluminum", rgba=(0.75, 0.75, 0.75, 1.0))
    copper = model.material("copper", rgba=(0.72, 0.45, 0.20, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.1, 0.1, 0.1, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.2, 0.2, 0.2, 1.0))

    # 1. Base Part
    base = model.part("base")
    base_plate_size = (0.04, 0.04, 0.008)
    base.visual(Box(base_plate_size), origin=Origin(xyz=(0, 0, 0.004)), material=nickel, name="base_plate")
    base.inertial = Inertial.from_geometry(Box(base_plate_size), mass=0.2, origin=Origin(xyz=(0, 0, 0.004)))

    # 2. Fin Stack Part
    fin_stack = model.part("fin_stack")
    fin_box_size = (0.12, 0.05, 0.10)
    
    # Heat pipes (now in fin_stack to connect to base)
    pipe_radius = 0.003
    pipe_positions = [(-0.012, 0, 0), (0.012, 0, 0), (-0.004, 0, 0), (0.004, 0, 0)]
    for i, pos in enumerate(pipe_positions):
        # Pipes start embedded in base plate
        pipe_height = 0.14
        pipe_geom = CylinderGeometry(radius=pipe_radius, height=pipe_height)
        pipe_mesh = mesh_from_geometry(pipe_geom, MESH_DIR / f"pipe_{i}.obj")
        # Origin is z=0.008 relative to base, pipes start at z=0.004 in world
        # So in fin_stack frame, pipes are at z=pipe_height/2 - 0.004
        fin_stack.visual(pipe_mesh, origin=Origin(xyz=(pos[0], pos[1], pipe_height/2 - 0.004)), material=copper, name=f"heat_pipe_{i}")

    # Fins as a single block
    # Fins start at world z=0.03 -> fin_stack z = 0.022
    fin_stack.visual(
        Box(fin_box_size),
        origin=Origin(xyz=(0, 0, 0.022 + fin_box_size[2]/2)),
        material=aluminum,
        name="fin_block",
    )
    fin_stack.inertial = Inertial.from_geometry(Box(fin_box_size), mass=0.4, origin=Origin(xyz=(0, 0, 0.072)))

    model.articulation(
        "base_to_fins",
        ArticulationType.FIXED,
        parent="base",
        child="fin_stack",
        origin=Origin(xyz=(0, 0, 0.008)),
    )

    # 3. Fan Frame Part
    fan_frame = model.part("fan_frame")
    fan_size = 0.12
    fan_thickness = 0.025
    
    # Create a frame with a hole
    outer_profile = rounded_rect_profile(fan_size, fan_size, radius=0.005)
    inner_hole = [[0.058 * math.cos(a), 0.058 * math.sin(a)] for a in [i * 2 * math.pi / 32 for i in range(32)]]
    
    frame_geom = ExtrudeWithHolesGeometry(
        outer_profile=outer_profile,
        hole_profiles=[inner_hole],
        height=fan_thickness,
    )
    frame_geom.rotate_x(math.pi / 2)
    frame_mesh = mesh_from_geometry(frame_geom, MESH_DIR / "fan_frame.obj")
    
    # Joint fins_to_fan_frame at (0, -0.025, 0.072) relative to fin_stack origin
    fan_y_local = -fan_thickness / 2
    fan_frame.visual(frame_mesh, origin=Origin(xyz=(0, fan_y_local, 0)), material=black_plastic, name="frame_body")
    
    # Corner mounts
    mount_size = 0.01
    for sx in [-1, 1]:
        for sz in [-1, 1]:
            fan_frame.visual(
                Box((mount_size, fan_thickness, mount_size)),
                origin=Origin(xyz=(sx * (fan_size / 2 - mount_size / 2), fan_y_local, sz * (fan_size / 2 - mount_size / 2))),
                material=black_plastic,
                name=f"mount_{sx}_{sz}",
            )

    fan_frame.inertial = Inertial.from_geometry(Box((fan_size, fan_thickness, fan_size)), mass=0.1, origin=Origin(xyz=(0, fan_y_local, 0)))

    model.articulation(
        "fins_to_fan_frame",
        ArticulationType.FIXED,
        parent="fin_stack",
        child="fan_frame",
        origin=Origin(xyz=(0, -0.025, 0.072)),
    )

    # 4. Fan Rotor Part
    fan_rotor = model.part("fan_rotor")
    hub_radius = 0.02
    hub_length = 0.02
    
    # Hub
    hub_geom = CylinderGeometry(radius=hub_radius, height=hub_length)
    hub_geom.rotate_x(math.pi / 2)
    
    # Blades
    num_blades = 7
    blade_length = 0.035
    blade_width = 0.025
    blade_thickness = 0.002
    
    rotor_geom = hub_geom.copy()
    for i in range(num_blades):
        angle = i * 2 * math.pi / num_blades
        blade = BoxGeometry((blade_width, blade_thickness, blade_length))
        blade.rotate_x(math.radians(45))
        blade.translate(0, 0, hub_radius + blade_length / 2)
        blade.rotate_y(angle)
        rotor_geom.merge(blade)
    
    rotor_mesh = mesh_from_geometry(rotor_geom, MESH_DIR / "fan_rotor.obj")
    fan_rotor.visual(rotor_mesh, origin=Origin(xyz=(0, 0, 0)), material=black_plastic, name="blades")
    fan_rotor.inertial = Inertial.from_geometry(Cylinder(radius=0.055, length=0.02), mass=0.05, origin=Origin(xyz=(0, 0, 0)))

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent="fan_frame",
        child="fan_rotor",
        origin=Origin(xyz=(0, fan_y_local, 0)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=100.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    # Allow overlap between base and fins due to heat pipes embedding
    ctx.allow_overlap("base", "fin_stack", reason="heat pipes embedded in base plate")
    ctx.warn_if_overlaps(max_pose_samples=64, ignore_adjacent=True, ignore_fixed=True)

    # Base is at the bottom
    base_aabb = ctx.part_world_aabb("base")
    # base_aabb is (min_xyz, max_xyz)
    assert base_aabb[0][2] < 0.01

    # Fin stack is above base plate (check the blocks specifically)
    ctx.expect_aabb_gap(
        "fin_stack", "base",
        axis="z",
        min_gap=0.01,
        positive_elem="fin_block",
        negative_elem="base_plate"
    )
    
    # Fan is on the side
    ctx.expect_aabb_overlap("fan_frame", "fin_stack", axes="xz")
    
    # Rotor is inside the frame
    ctx.expect_aabb_overlap("fan_rotor", "fan_frame", axes="xz")
    
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
