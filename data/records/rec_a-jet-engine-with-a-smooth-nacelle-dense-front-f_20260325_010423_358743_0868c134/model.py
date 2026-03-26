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
    LatheGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jet_engine", assets=ASSETS)

    # Materials
    nacelle_mat = model.material("nacelle_metal", rgba=(0.8, 0.8, 0.85, 1.0))
    blade_mat = model.material("blade_metal", rgba=(0.3, 0.3, 0.35, 1.0))
    core_mat = model.material("core_metal", rgba=(0.2, 0.2, 0.22, 1.0))
    exhaust_mat = model.material("exhaust_metal", rgba=(0.4, 0.35, 0.35, 1.0))

    # 1. Nacelle (Static part)
    # Revolved profile for the outer shell
    # Y is the engine axis (forward to back)
    # Shifted by -0.35 to center the engine at Y=0
    nacelle_profile = [
        (1.1, -1.8 - 0.35),   # Front intake lip outer
        (1.25, -1.5 - 0.35),
        (1.4, -0.5 - 0.35),
        (1.45, 0.5 - 0.35),
        (1.3, 1.5 - 0.35),
        (1.1, 2.0 - 0.35),    # Rear exhaust outer
        (1.05, 2.0 - 0.35),   # Rear exhaust inner lip
        (1.2, 1.5 - 0.35),
        (1.35, 0.5 - 0.35),
        (1.3, -0.5 - 0.35),
        (1.15, -1.5 - 0.35),
        (1.0, -1.75 - 0.35),  # Front intake lip inner
    ]
    nacelle_geom = LatheGeometry(nacelle_profile, segments=64)
    # Rotate LatheGeometry (z-axis by default) to Y-axis
    # Use -pi/2 to map +z to +y (keeping front-to-back profile)
    nacelle_geom.rotate_x(-math.pi / 2)
    
    nacelle = model.part("nacelle")
    nacelle.visual(
        mesh_from_geometry(nacelle_geom, ASSETS.mesh_path("nacelle.obj")),
        material=nacelle_mat,
        name="nacelle_shell"
    )

    # Add core struts to connect nacelle to rotor (stators)
    # These will be at the rear of the fan, around y = -1.5
    num_struts = 4
    strut_y = -1.1 - 0.35
    strut_r_inner = 0.2 # Ensure deep contact with central_shaft
    strut_r_outer = 1.3
    strut_mid_r = (strut_r_inner + strut_r_outer) / 2
    strut_len = strut_r_outer - strut_r_inner
    for k in range(num_struts):
        s_angle = 2 * math.pi * k / num_struts
        nacelle.visual(
            Box((strut_len, 0.1, 0.05)),
            origin=Origin(
                xyz=(strut_mid_r * math.cos(s_angle), strut_y, strut_mid_r * math.sin(s_angle)),
                rpy=(0, -s_angle, 0)
            ),
            material=nacelle_mat,
            name=f"core_strut_{k}"
        )

    nacelle.inertial = Inertial.from_geometry(Cylinder(radius=1.45, length=3.8), mass=2000.0)

    # 2. Rotor (Rotating part: Fan, Hub, Turbine)
    rotor = model.part("rotor")
    
    # Central Shaft for connectivity
    # Extends from spinner tip area to turbine area
    rotor.visual(
        Cylinder(radius=0.35, length=3.0),
        origin=Origin(xyz=(0, -0.2, 0), rpy=(math.pi/2, 0, 0)),
        material=core_mat,
        name="central_shaft"
    )

    # Hub / Spinner (Front cone)
    spinner_profile = [
        (0.0, -1.8 - 0.35),
        (0.1, -1.78 - 0.35),
        (0.25, -1.7 - 0.35),
        (0.35, -1.5 - 0.35),
        (0.4, -1.2 - 0.35),
        (0.4, 1.5 - 0.35),    # Shaft extends back
        (0.0, 1.5 - 0.35),
    ]
    spinner_geom = LatheGeometry(spinner_profile, segments=32).rotate_x(-math.pi / 2)
    rotor.visual(
        mesh_from_geometry(spinner_geom, ASSETS.mesh_path("spinner.obj")),
        material=core_mat,
        name="spinner"
    )

    # Fan Blades
    num_fan_blades = 24
    fan_radius_inner = 0.1 # Ensure deep overlap with spinner
    fan_radius_outer = 1.05
    blade_width = 0.3
    blade_thickness = 0.02
    
    # Positioned behind the spinner tip
    y_pos = -1.4 - 0.35
    mid_r = (fan_radius_inner + fan_radius_outer) / 2
    blade_height = fan_radius_outer - fan_radius_inner
    
    for i in range(num_fan_blades):
        angle = 2 * math.pi * i / num_fan_blades
        # Twist angle (aerodynamic)
        twist = math.radians(30)
        
        # Using Box(height, width, thickness) and rpy=(twist, -angle, 0)
        # to ensure local X is radial, local Y is axial, local Z is tangential.
        blade_origin = Origin(
            xyz=(mid_r * math.cos(angle), y_pos, mid_r * math.sin(angle)),
            rpy=(twist, -angle, 0) 
        )
        
        rotor.visual(
            Box((blade_height, blade_width, blade_thickness)),
            origin=blade_origin,
            material=blade_mat,
            name=f"fan_blade_{i}"
        )

    # Turbine details (Simplified rear stages)
    for stage_y_orig in [0.5, 1.0, 1.4]:
        stage_y = stage_y_orig - 0.35
        stage_radius = 0.6 - (stage_y_orig * 0.1)
        # Turbine disk
        rotor.visual(
            Cylinder(radius=stage_radius, length=0.05),
            origin=Origin(xyz=(0, stage_y, 0), rpy=(math.pi/2, 0, 0)),
            material=core_mat,
            name=f"turbine_disk_{stage_y_orig}"
        )
        # Small turbine blades
        num_turbine_blades = 40
        t_height = 0.2
        t_width = 0.05
        t_thickness = 0.01
        for j in range(num_turbine_blades):
            t_angle = 2 * math.pi * j / num_turbine_blades
            # Overlap with disk: stage_radius is the disk radius.
            # Inward extent is t_height/2 = 0.1. 
            # We want t_blade_r - 0.1 < stage_radius.
            t_blade_r = stage_radius + 0.05 
            rotor.visual(
                Box((t_height, t_width, t_thickness)),
                origin=Origin(
                    xyz=(t_blade_r * math.cos(t_angle), stage_y, t_blade_r * math.sin(t_angle)),
                    rpy=(math.radians(45), -t_angle, 0)
                ),
                material=blade_mat,
                name=f"turbine_blade_{stage_y_orig}_{j}"
            )

    rotor.inertial = Inertial.from_geometry(Cylinder(radius=1.1, length=3.3), mass=800.0)

    # 3. Exhaust Cone
    exhaust_cone_profile = [
        (0.4, 1.5 - 0.35),
        (0.3, 2.2 - 0.35),
        (0.0, 2.5 - 0.35),
    ]
    exhaust_cone_geom = LatheGeometry(exhaust_cone_profile, segments=32).rotate_x(-math.pi / 2)
    rotor.visual(
        mesh_from_geometry(exhaust_cone_geom, ASSETS.mesh_path("exhaust_cone.obj")),
        material=exhaust_mat,
        name="exhaust_cone"
    )

    # Articulation
    model.articulation(
        "fan_rotation",
        ArticulationType.CONTINUOUS,
        parent=nacelle,
        child=rotor,
        origin=Origin(xyz=(0, -1.45, 0)),
        axis=(0, 1, 0), # Y-axis rotation
        motion_limits=MotionLimits(effort=10000.0, velocity=100.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    nacelle = object_model.get_part("nacelle")
    rotor = object_model.get_part("rotor")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    
    # We allow overlap because the rotor is inside the nacelle
    ctx.allow_overlap(nacelle, rotor, reason="rotor is nested inside nacelle shell")

    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=8)
    ctx.warn_if_overlaps(max_pose_samples=8, ignore_adjacent=True, ignore_fixed=True)

    # Hero features
    ctx.expect_origin_distance(rotor, nacelle, axes="xz", max_dist=0.01)
    
    # Verify fan blades are present and attached to the hub
    spinner = rotor.get_visual("spinner")
    for i in range(24):
        blade = rotor.get_visual(f"fan_blade_{i}")
        ctx.expect_contact(rotor, rotor, elem_a=blade, elem_b=spinner)

    # Verify turbine detail is present and attached
    for stage_y in [0.5, 1.0, 1.4]:
        disk = rotor.get_visual(f"turbine_disk_{stage_y}")
        blade = rotor.get_visual(f"turbine_blade_{stage_y}_0")
        ctx.expect_contact(rotor, rotor, elem_a=blade, elem_b=disk)

    return ctx.report()


object_model = build_object_model()


# >>> USER_CODE_END

object_model = build_object_model()
