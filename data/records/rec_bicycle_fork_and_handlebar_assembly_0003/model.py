from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    AssetContext,
    Box,
    Cylinder,
    LatheGeometry,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _rounded_section(center: tuple[float, float, float], width: float, depth: float) -> list[tuple[float, float, float]]:
    cx, cy, cz = center
    profile = rounded_rect_profile(
        width,
        depth,
        radius=min(width, depth) * 0.24,
        corner_segments=6,
    )
    return [(cx + x, cy + y, cz) for x, y in profile]


def _fork_blade_mesh(side: float, mesh_name: str):
    sections = [
        _rounded_section((side * 0.026, 0.004, 0.305), 0.023, 0.028),
        _rounded_section((side * 0.030, 0.010, 0.240), 0.019, 0.024),
        _rounded_section((side * 0.036, 0.020, 0.170), 0.016, 0.020),
        _rounded_section((side * 0.043, 0.030, 0.090), 0.013, 0.015),
        _rounded_section((side * 0.050, 0.039, 0.014), 0.010, 0.010),
    ]
    geom = repair_loft(section_loft(sections))
    return mesh_from_geometry(geom, ASSETS.mesh_path(mesh_name))


def _crown_bridge_mesh():
    geom = tube_from_spline_points(
        [
            (-0.037, 0.013, 0.300),
            (-0.020, 0.019, 0.308),
            (0.000, 0.022, 0.312),
            (0.020, 0.019, 0.308),
            (0.037, 0.013, 0.300),
        ],
        radius=0.0055,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("crown_bridge.obj"))


def _steerer_mesh():
    geom = LatheGeometry(
        [
            (0.0, -0.072),
            (0.019, -0.072),
            (0.018, -0.040),
            (0.014, -0.005),
            (0.014, 0.072),
            (0.0, 0.072),
        ],
        segments=56,
    )
    geom.translate(0.0, 0.0, 0.373)
    return mesh_from_geometry(geom, ASSETS.mesh_path("steerer.obj"))


def _drop_side_mesh(side: float, mesh_name: str):
    geom = tube_from_spline_points(
        [
            (side * 0.055, 0.091, 0.425),
            (side * 0.115, 0.095, 0.435),
            (side * 0.180, 0.092, 0.420),
            (side * 0.225, 0.072, 0.360),
            (side * 0.210, 0.035, 0.295),
            (side * 0.175, 0.005, 0.265),
        ],
        radius=0.011,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(mesh_name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="road_bike_fork_cockpit", assets=ASSETS)

    carbon = model.material("carbon", rgba=(0.13, 0.13, 0.14, 1.0))
    alloy = model.material("alloy", rgba=(0.22, 0.22, 0.24, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))

    assembly = model.part("assembly")

    assembly.visual(_steerer_mesh(), material=carbon, name="steerer")
    assembly.visual(
        Box((0.068, 0.032, 0.020)),
        origin=Origin(xyz=(0.0, 0.002, 0.302)),
        material=carbon,
        name="crown",
    )
    assembly.visual(_crown_bridge_mesh(), material=carbon, name="crown_bridge")
    assembly.visual(_fork_blade_mesh(-1.0, "left_blade.obj"), material=carbon, name="left_blade")
    assembly.visual(_fork_blade_mesh(1.0, "right_blade.obj"), material=carbon, name="right_blade")
    assembly.visual(
        Box((0.018, 0.012, 0.016)),
        origin=Origin(xyz=(-0.051, 0.039, 0.007)),
        material=alloy,
        name="left_dropout",
    )
    assembly.visual(
        Box((0.018, 0.012, 0.016)),
        origin=Origin(xyz=(0.051, 0.039, 0.007)),
        material=alloy,
        name="right_dropout",
    )
    assembly.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.444)),
        material=alloy,
        name="top_cap",
    )
    assembly.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.4485)),
        material=steel,
        name="top_cap_bolt",
    )

    assembly.visual(
        Cylinder(radius=0.024, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=alloy,
        name="stem_clamp",
    )
    assembly.visual(
        Box((0.038, 0.090, 0.026)),
        origin=Origin(xyz=(0.0, 0.048, 0.425)),
        material=alloy,
        name="stem_body",
    )
    assembly.visual(
        Cylinder(radius=0.022, length=0.050),
        origin=Origin(xyz=(0.0, 0.091, 0.425), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=alloy,
        name="bar_clamp",
    )
    assembly.visual(
        Box((0.018, 0.014, 0.052)),
        origin=Origin(xyz=(0.0, 0.097, 0.425)),
        material=alloy,
        name="faceplate",
    )
    assembly.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.0, 0.108, 0.438), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="upper_bolt",
    )
    assembly.visual(
        Cylinder(radius=0.004, length=0.008),
        origin=Origin(xyz=(0.0, 0.108, 0.412), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lower_bolt",
    )

    assembly.visual(
        Box((0.120, 0.016, 0.018)),
        origin=Origin(xyz=(0.0, 0.091, 0.425)),
        material=black,
        name="bar_center",
    )
    assembly.visual(_drop_side_mesh(-1.0, "left_drop.obj"), material=black, name="left_drop")
    assembly.visual(_drop_side_mesh(1.0, "right_drop.obj"), material=black, name="right_drop")

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    assembly = object_model.get_part("assembly")
    steerer = assembly.get_visual("steerer")
    crown_bridge = assembly.get_visual("crown_bridge")
    left_blade = assembly.get_visual("left_blade")
    right_blade = assembly.get_visual("right_blade")
    left_dropout = assembly.get_visual("left_dropout")
    right_dropout = assembly.get_visual("right_dropout")
    top_cap = assembly.get_visual("top_cap")
    top_cap_bolt = assembly.get_visual("top_cap_bolt")
    stem_clamp = assembly.get_visual("stem_clamp")
    bar_clamp = assembly.get_visual("bar_clamp")
    faceplate = assembly.get_visual("faceplate")
    upper_bolt = assembly.get_visual("upper_bolt")
    lower_bolt = assembly.get_visual("lower_bolt")
    bar_center = assembly.get_visual("bar_center")
    left_drop = assembly.get_visual("left_drop")
    right_drop = assembly.get_visual("right_drop")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(assembly, assembly, axes="xy", min_overlap=0.001, elem_a=stem_clamp, elem_b=steerer)
    ctx.expect_gap(
        assembly,
        assembly,
        axis="z",
        max_gap=0.012,
        max_penetration=0.0,
        positive_elem=top_cap,
        negative_elem=stem_clamp,
    )
    ctx.expect_contact(assembly, assembly, elem_a=top_cap_bolt, elem_b=top_cap)

    ctx.expect_within(assembly, assembly, axes="yz", inner_elem=bar_center, outer_elem=bar_clamp)
    ctx.expect_overlap(assembly, assembly, axes="xz", min_overlap=0.012, elem_a=bar_center, elem_b=faceplate)
    ctx.expect_gap(
        assembly,
        assembly,
        axis="x",
        min_gap=0.02,
        positive_elem=right_drop,
        negative_elem=faceplate,
    )
    ctx.expect_gap(
        assembly,
        assembly,
        axis="x",
        min_gap=0.02,
        positive_elem=faceplate,
        negative_elem=left_drop,
    )
    ctx.expect_gap(
        assembly,
        assembly,
        axis="z",
        min_gap=0.09,
        positive_elem=bar_center,
        negative_elem=crown_bridge,
    )

    ctx.expect_contact(assembly, assembly, elem_a=upper_bolt, elem_b=faceplate)
    ctx.expect_contact(assembly, assembly, elem_a=lower_bolt, elem_b=faceplate)
    ctx.expect_gap(
        assembly,
        assembly,
        axis="z",
        min_gap=0.014,
        positive_elem=upper_bolt,
        negative_elem=lower_bolt,
    )

    ctx.expect_gap(
        assembly,
        assembly,
        axis="x",
        min_gap=0.08,
        positive_elem=right_dropout,
        negative_elem=left_dropout,
    )
    ctx.expect_gap(
        assembly,
        assembly,
        axis="z",
        max_gap=0.008,
        max_penetration=0.004,
        positive_elem=left_blade,
        negative_elem=left_dropout,
    )
    ctx.expect_gap(
        assembly,
        assembly,
        axis="z",
        max_gap=0.008,
        max_penetration=0.004,
        positive_elem=right_blade,
        negative_elem=right_dropout,
    )
    ctx.expect_overlap(assembly, assembly, axes="yz", min_overlap=0.004, elem_a=crown_bridge, elem_b=left_blade)
    ctx.expect_overlap(assembly, assembly, axes="yz", min_overlap=0.004, elem_a=crown_bridge, elem_b=right_blade)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
