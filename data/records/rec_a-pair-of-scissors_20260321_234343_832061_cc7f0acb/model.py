from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from pathlib import Path
import math
from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
    Mesh,
    mesh_from_geometry,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    BoxGeometry,
    CylinderGeometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scissors", assets=ASSETS)

    # Materials
    steel = model.material("steel", rgba=(0.75, 0.75, 0.8, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.15, 0.15, 0.15, 1.0))
    pivot_metal = model.material("pivot_metal", rgba=(0.9, 0.9, 0.9, 1.0))

    # Dimensions
    BLADE_LENGTH = 0.14
    PIVOT_RADIUS = 0.009
    METAL_THICKNESS = 0.0025
    HANDLE_THICKNESS = 0.012
    
    # Upper Part (Left handle / Finger loop)
    upper_part = model.part("upper_part")
    
    # Metal blade (tip at +Y, cutting edge at X=0)
    upper_blade_profile = [
        (0.0, 0.0),
        (-0.018, 0.0),
        (-0.003, BLADE_LENGTH),
        (0.0, BLADE_LENGTH),
    ]
    ub_geom = ExtrudeGeometry(upper_blade_profile, METAL_THICKNESS)
    ub_mesh = mesh_from_geometry(ub_geom, ASSETS.mesh_path("upper_blade.obj"))
    upper_part.visual(ub_mesh, origin=Origin(xyz=(0, 0, METAL_THICKNESS/2)), material=steel)
    
    # Metal tang (extends into handle)
    upper_tang_profile = [
        (0.0, 0.0),
        (-0.015, 0.0),
        (-0.018, -0.04),
        (-0.005, -0.04),
    ]
    ut_geom = ExtrudeGeometry(upper_tang_profile, METAL_THICKNESS)
    ut_mesh = mesh_from_geometry(ut_geom, ASSETS.mesh_path("upper_tang.obj"))
    upper_part.visual(ut_mesh, origin=Origin(xyz=(0, 0, METAL_THICKNESS/2)), material=steel)
    
    # Pivot disc
    up_disc_geom = CylinderGeometry(radius=PIVOT_RADIUS, height=METAL_THICKNESS)
    up_disc_mesh = mesh_from_geometry(up_disc_geom, ASSETS.mesh_path("upper_pivot_disc.obj"))
    upper_part.visual(up_disc_mesh, origin=Origin(xyz=(0, 0, METAL_THICKNESS/2)), material=steel)

    # Plastic handle loop (Large)
    upper_outer_pts = [
        (-0.005, -0.03),
        (-0.040, -0.04),
        (-0.060, -0.08),
        (-0.045, -0.12),
        (-0.010, -0.13),
        (0.010, -0.10),
        (0.005, -0.04),
    ]
    upper_outer = sample_catmull_rom_spline_2d(upper_outer_pts, closed=True)
    upper_inner_pts = [
        (-0.015, -0.05),
        (-0.035, -0.06),
        (-0.045, -0.08),
        (-0.035, -0.10),
        (-0.015, -0.11),
        (-0.005, -0.09),
        (-0.005, -0.06),
    ]
    upper_inner = sample_catmull_rom_spline_2d(upper_inner_pts, closed=True)
    uh_geom = ExtrudeWithHolesGeometry(upper_outer, [upper_inner], HANDLE_THICKNESS)
    uh_mesh = mesh_from_geometry(uh_geom, ASSETS.mesh_path("upper_handle.obj"))
    # Offset handle in Z to avoid overlap with lower handle
    upper_part.visual(uh_mesh, origin=Origin(xyz=(0, 0, METAL_THICKNESS/2 + HANDLE_THICKNESS/2)), material=plastic_black)

    # Lower Part (Right handle / Thumb loop)
    lower_part = model.part("lower_part")
    
    # Metal blade (tip at +Y, cutting edge at X=0)
    lower_blade_profile = [
        (0.0, 0.0),
        (0.018, 0.0),
        (0.003, BLADE_LENGTH),
        (0.0, BLADE_LENGTH),
    ]
    lb_geom = ExtrudeGeometry(lower_blade_profile, METAL_THICKNESS)
    lb_mesh = mesh_from_geometry(lb_geom, ASSETS.mesh_path("lower_blade.obj"))
    lower_part.visual(lb_mesh, origin=Origin(xyz=(0, 0, -METAL_THICKNESS/2)), material=steel)
    
    # Metal tang
    lower_tang_profile = [
        (0.0, 0.0),
        (0.015, 0.0),
        (0.018, -0.04),
        (0.005, -0.04),
    ]
    lt_geom = ExtrudeGeometry(lower_tang_profile, METAL_THICKNESS)
    lt_mesh = mesh_from_geometry(lt_geom, ASSETS.mesh_path("lower_tang.obj"))
    lower_part.visual(lt_mesh, origin=Origin(xyz=(0, 0, -METAL_THICKNESS/2)), material=steel)
    
    # Pivot disc
    lp_disc_geom = CylinderGeometry(radius=PIVOT_RADIUS, height=METAL_THICKNESS)
    lp_disc_mesh = mesh_from_geometry(lp_disc_geom, ASSETS.mesh_path("lower_pivot_disc.obj"))
    lower_part.visual(lp_disc_mesh, origin=Origin(xyz=(0, 0, -METAL_THICKNESS/2)), material=steel)

    # Plastic handle loop (Small)
    lower_outer_pts = [
        (0.005, -0.03),
        (0.040, -0.04),
        (0.050, -0.07),
        (0.040, -0.10),
        (0.010, -0.11),
        (-0.005, -0.08),
        (-0.005, -0.04),
    ]
    lower_outer = sample_catmull_rom_spline_2d(lower_outer_pts, closed=True)
    lower_inner_pts = [
        (0.010, -0.05),
        (0.028, -0.055),
        (0.035, -0.07),
        (0.028, -0.085),
        (0.010, -0.09),
        (0.002, -0.075),
        (0.002, -0.055),
    ]
    lower_inner = sample_catmull_rom_spline_2d(lower_inner_pts, closed=True)
    lh_geom = ExtrudeWithHolesGeometry(lower_outer, [lower_inner], HANDLE_THICKNESS)
    lh_mesh = mesh_from_geometry(lh_geom, ASSETS.mesh_path("lower_handle.obj"))
    # Offset handle in Z to avoid overlap with upper handle
    lower_part.visual(lh_mesh, origin=Origin(xyz=(0, 0, -METAL_THICKNESS/2 - HANDLE_THICKNESS/2)), material=plastic_black)

    # Pivot Pin
    pin_geom = CylinderGeometry(radius=0.0035, height=METAL_THICKNESS * 3)
    pin_mesh = mesh_from_geometry(pin_geom, ASSETS.mesh_path("pivot_pin.obj"))
    # Attach pin to lower part
    lower_part.visual(pin_mesh, origin=Origin(xyz=(0, 0, 0)), material=pivot_metal)

    # Articulation
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent="lower_part",
        child="upper_part",
        origin=Origin(xyz=(0, 0, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=0.0, upper=math.radians(40)),
    )

    # Inertials
    upper_part.inertial = Inertial.from_geometry(Box((0.06, 0.25, 0.015)), mass=0.04)
    lower_part.inertial = Inertial.from_geometry(Box((0.06, 0.25, 0.015)), mass=0.04)

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    
    # Allow overlap at the pivot where they are stacked
    ctx.allow_overlap("upper_part", "lower_part", reason="Stacked blades share pivot origin")
    
    ctx.check_articulation_overlaps(max_pose_samples=16)
    ctx.warn_if_overlaps(max_pose_samples=16, ignore_adjacent=True, ignore_fixed=True)
    
    # Semantic checks
    # Rotating pivot opens the scissors. 
    # For the upper_part (left handle), the AABB center moves in positive X as the handle swings out.
    ctx.expect_joint_motion_axis("pivot", "upper_part", world_axis="x", direction="positive", min_delta=0.01)
    
    # Check that handles are below pivot and blades are above
    ctx.expect_aabb_gap("upper_part", "lower_part", axis="z", max_gap=0.01, max_penetration=0.01)
    
    return ctx.report()

object_model = build_object_model()
# >>> USER_CODE_END

object_model = build_object_model()
