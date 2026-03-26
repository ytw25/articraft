from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    BoxGeometry,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jet_engine", assets=ASSETS)

    aerospace_metal = model.material("aerospace_metal", rgba=(0.85, 0.85, 0.88, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.25, 0.25, 0.28, 1.0))
    titanium = model.material("titanium", rgba=(0.45, 0.45, 0.48, 1.0))

    # Casing / Stator (Root Part)
    casing = model.part("casing")

    # Nacelle CCW profile (R, Z)
    nacelle_profile = [
        (0.85, 0.0),
        (0.865, -0.02),
        (0.88, 0.0),
        (0.95, 0.5),
        (1.00, 1.5),
        (0.98, 1.95),
        (0.95, 2.0),
        (0.92, 1.9),
        (0.91, 1.8),
        (0.91, 1.6), # Cylindrical section for the fan
        (0.80, 1.0),
        (0.80, 0.5),
    ]
    nacelle_geom = LatheGeometry(nacelle_profile, segments=64)
    casing.visual(
        mesh_from_geometry(nacelle_geom, ASSETS.mesh_dir / "nacelle.obj"),
        material=aerospace_metal,
        name="nacelle_shell",
    )

    core_rear_profile = [
        (0.018, 0.1),
        (0.4, 0.1),
        (0.4, 0.39),
        (0.018, 0.39),
    ]
    core_rear_geom = LatheGeometry(core_rear_profile, segments=32)
    casing.visual(
        mesh_from_geometry(core_rear_geom, ASSETS.mesh_dir / "core_rear.obj"),
        material=dark_metal,
        name="core_rear",
    )

    core_front_profile = [
        (0.018, 0.81),
        (0.4, 0.81),
        (0.4, 1.5),
        (0.25, 1.748),
        (0.018, 1.748),
    ]
    core_front_geom = LatheGeometry(core_front_profile, segments=32)
    casing.visual(
        mesh_from_geometry(core_front_geom, ASSETS.mesh_dir / "core_front.obj"),
        material=dark_metal,
        name="core_front",
    )

    casing_shaft = CylinderGeometry(radius=0.02, height=1.752, radial_segments=16)
    casing_shaft.translate(0.0, 0.0, 1.752 / 2)
    casing.visual(
        mesh_from_geometry(casing_shaft, ASSETS.mesh_dir / "casing_shaft.obj"),
        material=dark_metal,
        name="casing_shaft",
    )

    front_struts = MeshGeometry()
    for i in range(6):
        angle = i * (2 * math.pi / 6)
        strut = BoxGeometry((0.52, 0.04, 0.1))
        strut.translate(0.64, 0.0, 0.0)
        strut.rotate_z(angle)
        strut.translate(0.0, 0.0, 1.5)
        front_struts.merge(strut)
    casing.visual(
        mesh_from_geometry(front_struts, ASSETS.mesh_dir / "front_struts.obj"),
        material=dark_metal,
        name="front_struts",
    )

    rear_struts = MeshGeometry()
    for i in range(6):
        angle = i * (2 * math.pi / 6)
        strut = BoxGeometry((0.51, 0.04, 0.1))
        strut.translate(0.585, 0.0, 0.0)
        strut.rotate_z(angle)
        strut.translate(0.0, 0.0, 0.3)
        rear_struts.merge(strut)
    casing.visual(
        mesh_from_geometry(rear_struts, ASSETS.mesh_dir / "rear_struts.obj"),
        material=dark_metal,
        name="rear_struts",
    )
    
    casing.inertial = Inertial.from_geometry(
        Cylinder(radius=1.0, length=2.0), mass=500.0, origin=Origin(xyz=(0, 0, 1.0))
    )

    # Fan
    fan = model.part("fan")
    spinner_geom = ConeGeometry(radius=0.25, height=0.4, radial_segments=32)
    spinner_geom.translate(0.0, 0.0, 0.20)  # base at relative Z=0.0
    fan.visual(
        mesh_from_geometry(spinner_geom, ASSETS.mesh_dir / "spinner.obj"),
        material=titanium,
        name="spinner",
    )

    fan_blades_geom = MeshGeometry()
    for i in range(36):
        angle = i * (2 * math.pi / 36)
        blade = BoxGeometry((0.70, 0.015, 0.12))
        blade.translate(0.53, 0.0, 0.0)
        blade.rotate_x(math.radians(35))
        blade.translate(0.0, 0.0, 0.08) # shift to clear the core below
        blade.rotate_z(angle)
        fan_blades_geom.merge(blade)
    fan.visual(
        mesh_from_geometry(fan_blades_geom, ASSETS.mesh_dir / "fan_blades.obj"),
        material=titanium,
        name="fan_blades",
    )
    fan.inertial = Inertial.from_geometry(Cylinder(radius=0.9, length=0.2), mass=50.0)

    # Turbine
    turbine = model.part("turbine")
    turbine_hub_profile = [
        (0.018, -0.198),
        (0.4, -0.198),
        (0.4, 0.198),
        (0.018, 0.198),
    ]
    turbine_hub_geom = LatheGeometry(turbine_hub_profile, segments=32)
    turbine.visual(
        mesh_from_geometry(turbine_hub_geom, ASSETS.mesh_dir / "turbine_hub.obj"),
        material=dark_metal,
        name="turbine_hub",
    )

    turbine_geom = MeshGeometry()
    for stage, z_offset in enumerate([-0.1, 0.0, 0.1]):
        radius_inner = 0.38
        radius_outer = 0.78
        length = radius_outer - radius_inner
        for i in range(18):
            angle = i * (2 * math.pi / 18) + (stage * 0.1)
            blade = BoxGeometry((length, 0.02, 0.06))
            blade.translate(radius_inner + length / 2, 0.0, 0.0)
            blade.rotate_x(math.radians(45))
            blade.rotate_z(angle)
            blade.translate(0.0, 0.0, z_offset)
            turbine_geom.merge(blade)
    turbine.visual(
        mesh_from_geometry(turbine_geom, ASSETS.mesh_dir / "turbine_blades.obj"),
        material=dark_metal,
        name="turbine_blades",
    )
    turbine.inertial = Inertial.from_geometry(Cylinder(radius=0.8, length=0.4), mass=80.0)

    # Articulations
    model.articulation(
        "casing_to_fan",
        ArticulationType.CONTINUOUS,
        parent=casing,
        child=fan,
        origin=Origin(xyz=(0.0, 0.0, 1.75)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=100.0),
    )

    model.articulation(
        "casing_to_turbine",
        ArticulationType.CONTINUOUS,
        parent=casing,
        child=turbine,
        origin=Origin(xyz=(0.0, 0.0, 0.6)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200.0, velocity=100.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.warn_if_articulation_origin_near_geometry(tol=0.025)
    ctx.warn_if_part_geometry_disconnected()
    
    casing = object_model.get_part("casing")
    fan = object_model.get_part("fan")
    turbine = object_model.get_part("turbine")
    
    nacelle_shell = casing.get_visual("nacelle_shell")
    fan_blades = fan.get_visual("fan_blades")
    turbine_blades = turbine.get_visual("turbine_blades")
    
    # Allow intentional overlaps for the central shaft connecting the components
    ctx.allow_overlap(casing, fan, reason="shaft nested in spinner")
    ctx.allow_overlap(casing, turbine, reason="shaft nested in hub")
    
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    fan_joint = object_model.get_articulation("casing_to_fan")
    turbine_joint = object_model.get_articulation("casing_to_turbine")

    # Fan and turbine are seated within casing radially
    ctx.expect_within(fan, casing, axes="xy")
    ctx.expect_within(turbine, casing, axes="xy")
    
    # Prove the specific prompt claim: fan blades and turbine blades are within the nacelle shell
    ctx.expect_within(fan, casing, inner_elem=fan_blades, outer_elem=nacelle_shell, axes="xy")
    ctx.expect_within(turbine, casing, inner_elem=turbine_blades, outer_elem=nacelle_shell, axes="xy")
    
    # Fan and turbine overlap with casing longitudinally
    ctx.expect_overlap(fan, casing, axes="z", min_overlap=0.2)
    ctx.expect_overlap(turbine, casing, axes="z", min_overlap=0.3)
    
    with ctx.pose({fan_joint: 1.0, turbine_joint: -1.0}):
        ctx.expect_within(fan, casing, axes="xy")
        ctx.expect_within(turbine, casing, axes="xy")

    return ctx.report()


object_model = build_object_model()


# >>> USER_CODE_END

object_model = build_object_model()
