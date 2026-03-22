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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)
MESH_DIR = ASSETS.mesh_dir


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="graphics_card", assets=ASSETS)

    # Materials
    pcb_mat = model.material("pcb_black", color=(0.05, 0.05, 0.05, 1.0))
    metal_mat = model.material("brushed_metal", color=(0.6, 0.6, 0.65, 1.0))
    shroud_mat = model.material("shroud_matte", color=(0.12, 0.12, 0.14, 1.0))
    fan_mat = model.material("fan_blade", color=(0.2, 0.2, 0.2, 0.9))
    accent_mat = model.material("accent_silver", color=(0.8, 0.8, 0.8, 1.0))

    # Dimensions
    L = 0.30  # Total length
    H = 0.13  # Total height
    W = 0.05  # Total width (thickness)
    PCB_L = 0.28
    PCB_H = 0.11
    PCB_W = 0.002

    # 1. PCB
    pcb = model.part("pcb")
    pcb.visual(
        Box((PCB_L, PCB_W, PCB_H)),
        origin=Origin(xyz=(0, 0, 0)),
        material=pcb_mat,
        name="pcb_board",
    )
    # PCIe Connector
    pcb.visual(
        Box((0.06, 0.003, 0.01)),
        origin=Origin(xyz=(-0.08, 0, -0.06)),
        material=accent_mat,
        name="pcie_connector",
    )
    # Backplate
    pcb.visual(
        Box((PCB_L, 0.002, PCB_H)),
        origin=Origin(xyz=(0, -0.002, 0)),
        material=shroud_mat,
        name="backplate",
    )
    # Add some "electronic components" to the back of the PCB
    for i in range(5):
        for j in range(3):
            pcb.visual(
                Box((0.01, 0.002, 0.01)),
                origin=Origin(xyz=(-0.1 + i * 0.04, -0.003, -0.03 + j * 0.03)),
                material=shroud_mat,
            )
    pcb.inertial = Inertial.from_geometry(Box((PCB_L, PCB_W, PCB_H)), mass=0.5)

    # 2. Bracket (PCIe)
    # Joint origin at the edge of the PCB
    # Bracket plate is at -0.15 in world, joint is at -0.14.
    # So child-local translate is -0.01.
    bracket_geom = BoxGeometry((0.002, 0.02, 0.16)).translate(-0.01, 0.01, 0.015)
    bracket_tab = BoxGeometry((0.02, 0.02, 0.002)).translate(0, 0.01, 0.095)
    bracket_geom.merge(bracket_tab)
    bracket_mesh = mesh_from_geometry(
        bracket_geom, str(MESH_DIR / "pcie_bracket.obj")
    )

    bracket = model.part("bracket")
    bracket.visual(bracket_mesh, material=metal_mat)
    bracket.inertial = Inertial.from_geometry(Box((0.02, 0.02, 0.16)), mass=0.1)

    model.articulation(
        "pcb_to_bracket",
        ArticulationType.FIXED,
        parent="pcb",
        child="bracket",
        origin=Origin(xyz=(-0.14, 0, 0)),
    )

    # 3. Shroud and Heatsink
    shroud = model.part("shroud")

    # Create shroud front with fan holes
    fan_positions = [-0.09, 0.0, 0.09]
    outer_rect = rounded_rect_profile(L, H, radius=0.01)
    fan_holes = []
    for x_pos in fan_positions:
        # Hole radius 0.045
        hole = superellipse_profile(0.09, 0.09, segments=32)
        hole = [(px + x_pos, py) for px, py in hole]
        fan_holes.append(hole)

    shroud_front_geom = ExtrudeWithHolesGeometry(outer_rect, fan_holes, height=0.01)
    # Extruded along Z, centered. Rotate to face +Y and translate to front.
    shroud_front_geom.rotate_x(math.pi / 2).translate(0, 0.04, 0)

    # Shroud side walls
    shroud_sides = BoxGeometry((L, 0.035, 0.01)).translate(0, 0.0175, 0.06)
    shroud_sides.merge(BoxGeometry((L, 0.035, 0.01)).translate(0, 0.0175, -0.06))
    shroud_sides.merge(BoxGeometry((0.01, 0.035, H)).translate(0.145, 0.0175, 0))
    shroud_sides.merge(BoxGeometry((0.01, 0.035, H)).translate(-0.145, 0.0175, 0))

    shroud_front_geom.merge(shroud_sides)
    shroud_mesh = mesh_from_geometry(shroud_front_geom, str(MESH_DIR / "shroud_v3.obj"))
    shroud.visual(shroud_mesh, material=shroud_mat)

    # Heatsink fins (visible through the shroud)
    for i in range(30):
        shroud.visual(
            Box((0.001, 0.03, 0.10)),
            origin=Origin(xyz=(-0.12 + i * 0.008, 0.015, 0)),
            material=metal_mat,
        )

    shroud.inertial = Inertial.from_geometry(Box((L, 0.04, H)), mass=0.8)
    model.articulation(
        "pcb_to_shroud",
        ArticulationType.FIXED,
        parent="pcb",
        child="shroud",
        origin=Origin(xyz=(0, 0, 0)),
    )

    # 4. Fans
    for i, x_pos in enumerate(fan_positions):
        fan_name = f"fan_{i}"
        fan_part = model.part(fan_name)

        # Fan geometry: Hub + Blades
        hub_r = 0.015
        hub_h = 0.008
        hub_geom = CylinderGeometry(radius=hub_r, height=hub_h).rotate_x(math.pi / 2)

        # Blades (radius ~0.043, fits in 0.045 hole)
        blade_count = 9
        for b in range(blade_count):
            angle = 2 * math.pi * b / blade_count
            blade = BoxGeometry((0.03, 0.002, 0.025)).translate(0.028, 0, 0)
            blade.rotate_x(0.5)
            blade.rotate_y(angle)
            hub_geom.merge(blade)

        fan_mesh = mesh_from_geometry(hub_geom, str(MESH_DIR / f"fan_v3_{i}.obj"))
        fan_part.visual(fan_mesh, material=fan_mat)
        fan_part.inertial = Inertial.from_geometry(
            Cylinder(radius=0.045, length=0.01), mass=0.05
        )

        model.articulation(
            f"shroud_to_{fan_name}",
            ArticulationType.CONTINUOUS,
            parent="shroud",
            child=fan_part,
            # Place at y=0.055 (well outside the shroud front face max y=0.045)
            origin=Origin(xyz=(x_pos, 0.055, 0)),
            axis=(0, 1, 0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # The fans are nested in the shroud cutouts (conceptually)
    # and the heatsink fins are very close.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=32)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=32, ignore_adjacent=True, ignore_fixed=True)

    # Semantic checks
    # 1. Bracket is at the negative X end of the PCB
    ctx.expect_aabb_overlap("bracket", "pcb", axes="yz")
    # Allow some overlap for the bracket mounting
    ctx.expect_aabb_gap("pcb", "bracket", axis="x", max_gap=0.02, max_penetration=0.02)

    # 2. Shroud is offset from PCB along Y (thickness)
    ctx.expect_aabb_gap("shroud", "pcb", axis="y", max_gap=0.01, max_penetration=0.01)

    # 3. Fans are on the front face of the shroud
    for i in range(3):
        # Center alignment check
        ctx.expect_aabb_overlap(f"fan_{i}", "shroud", axes="xz")
        # Depth check
        ctx.expect_aabb_gap(f"fan_{i}", "shroud", axis="y", max_gap=0.02, max_penetration=0.01)

    # 4. PCIe connector is at the bottom
    ctx.expect_aabb_gap(
        "pcb",
        "pcb",
        axis="z",
        positive_elem="pcb_board",
        negative_elem="pcie_connector",
        max_gap=0.01,
        max_penetration=0.01,
    )

    return ctx.report()


object_model = build_object_model()
# >>> USER_CODE_END

object_model = build_object_model()
