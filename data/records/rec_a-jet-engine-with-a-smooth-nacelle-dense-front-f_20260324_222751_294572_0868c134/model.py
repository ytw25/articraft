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
    Cylinder,
    Inertial,
    LatheGeometry,
    Material,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="jet_engine", assets=ASSETS)

    titanium = Material(name="titanium", rgba=(0.72, 0.74, 0.78, 1.0))
    dark_metal = Material(name="dark_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    fan_metal = Material(name="fan_metal", rgba=(0.56, 0.60, 0.66, 1.0))
    shadow = Material(name="shadow", rgba=(0.16, 0.17, 0.19, 1.0))

    nacelle = model.part("nacelle")
    fan = model.part("fan")
    core = model.part("core")

    nacelle_outer = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.22, -0.06),
            (0.29, -0.01),
            (0.33, 0.08),
            (0.34, 0.28),
            (0.32, 0.58),
            (0.27, 0.83),
            (0.23, 0.96),
        ],
        inner_profile=[
            (0.17, -0.045),
            (0.23, 0.005),
            (0.26, 0.09),
            (0.27, 0.28),
            (0.255, 0.56),
            (0.22, 0.80),
            (0.18, 0.95),
        ],
        segments=64,
        start_cap="round",
        end_cap="flat",
        lip_samples=10,
    )
    nacelle.visual(
        mesh_from_geometry(nacelle_outer, ASSETS.mesh_path("nacelle_outer.obj")),
        material=titanium,
        name="nacelle_outer",
    )
    nacelle.visual(
        Cylinder(radius=0.18, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.965)),
        material=dark_metal,
        name="exhaust_ring",
    )
    nacelle.inertial = Inertial.from_geometry(Cylinder(radius=0.34, length=1.02), mass=110.0)

    fan.visual(
        Cylinder(radius=0.024, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=shadow,
        name="center_shaft",
    )
    fan.visual(
        Cylinder(radius=0.08, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="fan_hub",
    )
    fan.visual(
        Cylinder(radius=0.17, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=shadow,
        name="fan_annulus",
    )
    fan.visual(
        Cylinder(radius=0.175, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_metal,
        name="fan_disk",
    )
    fan.visual(
        Cylinder(radius=0.232, length=0.01),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=dark_metal,
        name="mount_ring",
    )
    for idx in range(18):
        angle = 2.0 * math.pi * idx / 18.0
        fan.visual(
            Box((0.095, 0.018, 0.004)),
            origin=Origin(
                xyz=(0.122 * math.cos(angle), 0.122 * math.sin(angle), 0.02),
                rpy=(0.0, 0.24, angle),
            ),
            material=fan_metal,
            name=f"fan_blade_{idx:02d}",
        )
    fan.inertial = Inertial.from_geometry(Cylinder(radius=0.18, length=0.10), mass=28.0)

    core.visual(
        Cylinder(radius=0.024, length=0.72),
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
        material=shadow,
        name="core_shaft",
    )
    core.visual(Cylinder(radius=0.11, length=0.03), origin=Origin(xyz=(0.0, 0.0, 0.32)), material=dark_metal, name="turbine_stage_front")
    core.visual(Cylinder(radius=0.095, length=0.03), origin=Origin(xyz=(0.0, 0.0, 0.48)), material=dark_metal, name="turbine_stage_mid")
    core.visual(Cylinder(radius=0.08, length=0.03), origin=Origin(xyz=(0.0, 0.0, 0.66)), material=dark_metal, name="turbine_stage_rear")
    core.visual(Cylinder(radius=0.17, length=0.016), origin=Origin(xyz=(0.0, 0.0, 0.28)), material=dark_metal, name="stator_ring")
    core.visual(Cylinder(radius=0.11, length=0.012), origin=Origin(xyz=(0.0, 0.0, 0.32)), material=dark_metal, name="front_disk")
    core.visual(Cylinder(radius=0.096, length=0.012), origin=Origin(xyz=(0.0, 0.0, 0.48)), material=dark_metal, name="mid_disk")
    core.visual(Cylinder(radius=0.082, length=0.012), origin=Origin(xyz=(0.0, 0.0, 0.66)), material=dark_metal, name="rear_disk")
    for idx in range(6):
        angle = 2.0 * math.pi * idx / 6.0
        core.visual(
            Box((0.11, 0.014, 0.03)),
            origin=Origin(xyz=(0.165 * math.cos(angle), 0.165 * math.sin(angle), 0.28), rpy=(0.0, 0.0, angle)),
            material=dark_metal,
            name=f"stator_vane_{idx:02d}",
        )
    for stage_name, z_pos, radius, count, twist in (
        ("front", 0.32, 0.087, 12, 0.30),
        ("mid", 0.48, 0.074, 14, 0.26),
        ("rear", 0.66, 0.061, 16, 0.24),
    ):
        for idx in range(count):
            angle = 2.0 * math.pi * idx / count
            core.visual(
                Box((0.046, 0.010, 0.004)),
                origin=Origin(xyz=(radius * math.cos(angle), radius * math.sin(angle), z_pos), rpy=(0.0, 0.20, angle + twist)),
                material=fan_metal,
                name=f"{stage_name}_blade_{idx:02d}",
            )
    core.inertial = Inertial.from_geometry(Cylinder(radius=0.12, length=0.72), mass=42.0)

    model.articulation(
        "nacelle_to_fan",
        ArticulationType.FIXED,
        parent=nacelle,
        child=fan,
        origin=Origin(),
    )
    model.articulation(
        "fan_to_core",
        ArticulationType.FIXED,
        parent=fan,
        child=core,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    nacelle = object_model.get_part("nacelle")
    fan = object_model.get_part("fan")
    core = object_model.get_part("core")
    nacelle_outer = nacelle.get_visual("nacelle_outer")
    fan_hub = fan.get_visual("fan_hub")
    fan_annulus = fan.get_visual("fan_annulus")
    center_shaft = fan.get_visual("center_shaft")
    fan_disk = fan.get_visual("fan_disk")
    mount_ring = fan.get_visual("mount_ring")
    front_blade = fan.get_visual("fan_blade_00")
    turbine_stage_front = core.get_visual("turbine_stage_front")
    turbine_stage_mid = core.get_visual("turbine_stage_mid")
    turbine_stage_rear = core.get_visual("turbine_stage_rear")
    core_shaft = core.get_visual("core_shaft")
    stator_ring = core.get_visual("stator_ring")
    front_disk = core.get_visual("front_disk")
    mid_disk = core.get_visual("mid_disk")
    rear_disk = core.get_visual("rear_disk")
    mid_blade = core.get_visual("mid_blade_00")
    rear_blade = core.get_visual("rear_blade_00")
    opposite_front_blade = fan.get_visual("fan_blade_09")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual connectivity gate for floating/disconnected subassemblies inside one part.
    ctx.check_part_geometry_connected()
    # Default articulated-joint clearance sensor; keep it non-blocking unless prompt-specific checks justify tighter gating.
    ctx.warn_if_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(fan, nacelle, axes="xy", inner_elem=fan_hub, outer_elem=nacelle_outer)
    ctx.expect_within(fan, nacelle, axes="xy", inner_elem=fan_annulus, outer_elem=nacelle_outer)
    ctx.expect_within(core, nacelle, axes="xy", inner_elem=turbine_stage_front, outer_elem=nacelle_outer)
    ctx.expect_within(core, nacelle, axes="xy", inner_elem=turbine_stage_mid, outer_elem=nacelle_outer)
    ctx.expect_within(core, nacelle, axes="xy", inner_elem=turbine_stage_rear, outer_elem=nacelle_outer)
    ctx.expect_within(fan, nacelle, axes="xy", inner_elem=front_blade, outer_elem=nacelle_outer)
    ctx.expect_within(fan, nacelle, axes="xy", inner_elem=opposite_front_blade, outer_elem=nacelle_outer)
    ctx.expect_within(core, nacelle, axes="xy", inner_elem=mid_blade, outer_elem=nacelle_outer)
    ctx.expect_within(core, nacelle, axes="xy", inner_elem=rear_blade, outer_elem=nacelle_outer)
    ctx.expect_within(core, nacelle, axes="xy", inner_elem=stator_ring, outer_elem=nacelle_outer)
    ctx.expect_overlap(fan, fan, axes="xy", min_overlap=0.01, elem_a=center_shaft, elem_b=fan_hub)
    ctx.expect_overlap(core, core, axes="xy", min_overlap=0.01, elem_a=core_shaft, elem_b=turbine_stage_front)
    ctx.expect_overlap(fan, fan, axes="xy", min_overlap=0.018, elem_a=fan_disk, elem_b=front_blade)
    ctx.expect_overlap(core, core, axes="xy", min_overlap=0.012, elem_a=mid_disk, elem_b=mid_blade)
    ctx.expect_overlap(core, core, axes="xy", min_overlap=0.012, elem_a=rear_disk, elem_b=rear_blade)
    ctx.expect_gap(fan, fan, axis="z", positive_elem=fan_hub, negative_elem=fan_annulus, min_gap=-0.05, max_gap=0.01)
    ctx.expect_gap(core, core, axis="z", positive_elem=turbine_stage_mid, negative_elem=turbine_stage_front, min_gap=0.12, max_gap=0.18)
    ctx.expect_gap(core, core, axis="z", positive_elem=turbine_stage_rear, negative_elem=turbine_stage_mid, min_gap=0.12, max_gap=0.18)
    ctx.expect_overlap(fan, core, axes="xy", min_overlap=0.03, elem_a=center_shaft, elem_b=core_shaft)
    ctx.expect_contact(fan, nacelle, elem_a=mount_ring, elem_b=nacelle_outer)
    ctx.expect_within(core, nacelle, axes="xy", inner_elem=core_shaft, outer_elem=nacelle_outer)
    ctx.expect_within(core, nacelle, axes="xy", inner_elem=turbine_stage_rear, outer_elem=nacelle_outer)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
