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
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scissors", assets=ASSETS)

    # Materials
    # Brushed steel for the blades
    steel = model.material("steel", rgba=(0.7, 0.72, 0.75, 1.0))
    # Matte black plastic for the handles
    plastic_black = model.material("plastic_black", rgba=(0.15, 0.15, 0.15, 1.0))
    # A bit of shiny metal for the pivot screw
    chrome = model.material("chrome", rgba=(0.9, 0.9, 0.9, 1.0))

    # Dimensions (meters)
    BLADE_LENGTH = 0.12
    BLADE_WIDTH_PIVOT = 0.016
    BLADE_THICKNESS = 0.003
    PIVOT_RADIUS = 0.0045
    HANDLE_THICKNESS = 0.012

    # --- Blade Geometry ---
    # Scissor blades are usually flat on the "inner" side and beveled on the "outer" side.
    # We'll model the profile such that one edge is the cutting edge.
    # Profile in XY, will be extruded along Z.
    # The cutting edge will be along the X axis (y=0).
    blade_profile = [
        (0.0, 0.0),                     # Pivot center-ish, cutting edge start
        (BLADE_LENGTH, 0.0),            # Tip, cutting edge end
        (BLADE_LENGTH, 0.002),          # Tip back
        (0.0, BLADE_WIDTH_PIVOT),       # Pivot back
    ]
    blade_geom = ExtrudeGeometry(blade_profile, BLADE_THICKNESS)

    # Connector from pivot area to the handle loops
    connector_profile = [
        (0.0, 0.0),
        (0.0, BLADE_WIDTH_PIVOT),
        (-0.025, 0.015),
        (-0.025, -0.005),
    ]
    connector_geom = ExtrudeGeometry(connector_profile, BLADE_THICKNESS)

    # --- Handle Geometries ---
    # Thumb loop (smaller, upper blade)
    thumb_outer = rounded_rect_profile(0.045, 0.035, 0.015)
    thumb_inner = rounded_rect_profile(0.032, 0.022, 0.008)
    thumb_handle_geom = ExtrudeWithHolesGeometry(
        thumb_outer, [thumb_inner], height=HANDLE_THICKNESS
    ).translate(-0.045, 0.015, 0)

    # Finger loop (larger, lower blade)
    finger_outer = rounded_rect_profile(0.075, 0.04, 0.018)
    finger_inner = rounded_rect_profile(0.06, 0.025, 0.01)
    finger_handle_geom = ExtrudeWithHolesGeometry(
        finger_outer, [finger_inner], height=HANDLE_THICKNESS
    ).translate(-0.055, -0.018, 0)

    # --- Parts ---

    # 1. Lower Blade (Finger side) - The "Base" part
    lower_blade = model.part("lower_blade")
    
    # Metal part for lower blade
    # It's the "bottom" blade, so its flat side (inner) faces +Z.
    # We extrude from 0 to -BLADE_THICKNESS.
    lower_metal_geom = blade_geom.clone().merge(connector_geom.clone()).translate(0, 0, -BLADE_THICKNESS)
    lower_metal_mesh = mesh_from_geometry(lower_metal_geom, MESH_DIR / "lower_metal.obj")
    lower_blade.visual(lower_metal_mesh, material=steel, name="metal")
    
    # Plastic handle for lower blade
    # Centered vertically relative to the metal
    lower_handle_mesh = mesh_from_geometry(finger_handle_geom, MESH_DIR / "lower_handle.obj")
    lower_blade.visual(
        lower_handle_mesh, 
        material=plastic_black, 
        origin=Origin(xyz=(0, 0, -HANDLE_THICKNESS/2)), 
        name="handle"
    )
    
    # Pivot pin (visualized on lower blade)
    pivot_pin = Cylinder(radius=PIVOT_RADIUS, length=BLADE_THICKNESS * 2.5)
    lower_blade.visual(
        pivot_pin, 
        material=chrome, 
        origin=Origin(xyz=(0, 0, 0)), 
        name="pivot_bolt"
    )
    
    lower_blade.inertial = Inertial.from_geometry(Box((0.2, 0.06, 0.015)), mass=0.08)

    # 2. Upper Blade (Thumb side) - The "Moving" part
    upper_blade = model.part("upper_blade")
    
    # Metal part for upper blade
    # It's the "top" blade, so its flat side (inner) faces -Z.
    # We mirror the profile in Y and extrude from 0 to +BLADE_THICKNESS.
    upper_metal_geom = (
        blade_geom.clone().scale(1, -1, 1)
        .merge(connector_geom.clone().scale(1, -1, 1))
    )
    upper_metal_mesh = mesh_from_geometry(upper_metal_geom, MESH_DIR / "upper_metal.obj")
    upper_blade.visual(upper_metal_mesh, material=steel, name="metal")
    
    # Plastic handle for upper blade
    upper_handle_mesh = mesh_from_geometry(thumb_handle_geom, MESH_DIR / "upper_handle.obj")
    upper_blade.visual(
        upper_handle_mesh, 
        material=plastic_black, 
        origin=Origin(xyz=(0, 0, -HANDLE_THICKNESS/2 + BLADE_THICKNESS)), 
        name="handle"
    )
    
    upper_blade.inertial = Inertial.from_geometry(Box((0.18, 0.05, 0.015)), mass=0.06)

    # --- Articulation ---
    # The joint is at the pivot (0,0,0).
    # Since lower_blade metal is [ -BLADE_THICKNESS, 0 ] 
    # and upper_blade metal is [ 0, BLADE_THICKNESS ] (in its local frame),
    # they meet at Z=0.
    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent="lower_blade",
        child="upper_blade",
        origin=Origin(xyz=(0, 0, 0)), 
        axis=(0, 0, 1),
        motion_limits=MotionLimits(
            effort=5.0, velocity=10.0, lower=0.0, upper=math.radians(60)
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Broad sensors
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    # Allowances: Scissor blades cross/touch at the pivot and along the edge.
    ctx.allow_overlap("upper_blade", "lower_blade", reason="blades must touch to cut")

    # Joint clearance check
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)
    # Global overlap check
    ctx.warn_if_overlaps(max_pose_samples=64, ignore_adjacent=True, ignore_fixed=True)

    # --- Semantic checks ---
    
    # 1. Proportions: Check total length (~20cm)
    lower_aabb = ctx.part_world_aabb("lower_blade")
    # AABB is (min, max) where each is (x, y, z)
    total_len = lower_aabb[1][0] - lower_aabb[0][0]
    # Handle is at ~ -0.09, blade at 0.12. Total should be ~0.21.
    assert 0.18 < total_len < 0.25, f"Unexpected total length: {total_len}"

    # 2. Attachment: Handles should be firmly attached to the metal
    # We check that the handle and metal visuals within the same part overlap or are very close.
    # Metal is [-0.025, 0.12], Handle is [-0.0925, -0.0175] (approx)
    # Along X, Metal is "positive" relative to Handle.
    ctx.expect_aabb_gap("lower_blade", "lower_blade", axis="x", 
                       max_gap=0.001, max_penetration=0.01,
                       positive_elem="metal", negative_elem="handle")
    ctx.expect_aabb_gap("upper_blade", "upper_blade", axis="x", 
                       max_gap=0.001, max_penetration=0.01,
                       positive_elem="metal", negative_elem="handle")

    # 3. Motion: Opening moves the thumb handle away from the finger handle in Y.
    # Rest pose (pivot=0): handles are near each other.
    # Open pose (pivot=60 deg): upper handle moves in +Y.
    with ctx.pose(pivot=math.radians(60)):
        ctx.expect_joint_motion_axis("pivot", "upper_blade", world_axis="y", direction="positive")
        
        # Also check that the tips of the blades are separated in Y
        # Upper blade tip is at (0.12, 0, 0) in local, rotated by 60 deg -> Y = 0.12 * sin(60) = 0.10
        # Lower blade tip is at (0.12, 0, 0) in local -> Y = 0
        upper_aabb = ctx.part_world_aabb("upper_blade")
        center_y = (upper_aabb[0][1] + upper_aabb[1][1]) / 2
        assert center_y > 0.02, f"Upper blade should have moved up in Y when open, got center_y={center_y}"

    return ctx.report()


object_model = build_object_model()


# >>> USER_CODE_END

object_model = build_object_model()
