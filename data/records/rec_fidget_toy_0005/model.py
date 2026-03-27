from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

COLLAR_LENGTH = 0.006
COLLAR_RADIUS = 0.0125
END_CLEARANCE = 0.004
INTER_RING_GAP = 0.005
RING_WIDTH = 0.018
TRACK_LENGTH = RING_WIDTH
SHAFT_RADIUS = 0.0072
TRACK_RADIUS = 0.0082
RING_INNER_RADIUS = 0.0080
RING_OUTER_EDGE_RADIUS = 0.0186
RING_OUTER_CROWN_RADIUS = 0.0202
RING_MID_BAND_HALF = 0.0048
RING_Z_POSITIONS = (-0.023, 0.0, 0.023)
ROD_LENGTH = (2.0 * COLLAR_LENGTH) + (3.0 * RING_WIDTH) + (2.0 * END_CLEARANCE) + (2.0 * INTER_RING_GAP)
COLLAR_OFFSET = (ROD_LENGTH * 0.5) - (COLLAR_LENGTH * 0.5)


def _save_mesh(name: str, geometry):
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _ring_band_mesh():
    outer_profile = [
        (RING_OUTER_EDGE_RADIUS, -(RING_WIDTH * 0.5)),
        (RING_OUTER_CROWN_RADIUS, -RING_MID_BAND_HALF),
        (RING_OUTER_CROWN_RADIUS, RING_MID_BAND_HALF),
        (RING_OUTER_EDGE_RADIUS, RING_WIDTH * 0.5),
    ]
    inner_profile = [
        (RING_INNER_RADIUS, -(RING_WIDTH * 0.5)),
        (RING_INNER_RADIUS, RING_WIDTH * 0.5),
    ]
    return _save_mesh(
        "magnetic_ring_band.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=88,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _add_cylindrical_segment(part, *, name: str, radius: float, length: float, center_z: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(0.0, 0.0, center_z)),
        material=material,
        name=name,
    )


def _add_ring(model: ArticulatedObject, core, *, index: int, z_pos: float, band_mesh, material):
    ring = model.part(f"ring_{index}")
    ring.visual(band_mesh, material=material, name="band")
    ring.inertial = Inertial.from_geometry(
        Cylinder(radius=RING_OUTER_CROWN_RADIUS, length=RING_WIDTH),
        mass=0.040,
    )
    model.articulation(
        f"core_to_ring_{index}",
        ArticulationType.CONTINUOUS,
        parent=core,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, z_pos)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=35.0),
    )
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnetic_ring_fidget", assets=ASSETS)

    shaft_black = model.material("shaft_black", rgba=(0.16, 0.17, 0.19, 1.0))
    track_steel = model.material("track_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    bronze = model.material("bronze", rgba=(0.67, 0.46, 0.28, 1.0))
    titanium = model.material("titanium", rgba=(0.60, 0.63, 0.68, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.31, 0.35, 0.39, 1.0))

    band_mesh = _ring_band_mesh()

    core = model.part("core")
    _add_cylindrical_segment(
        core,
        name="collar_lower",
        radius=COLLAR_RADIUS,
        length=COLLAR_LENGTH,
        center_z=-COLLAR_OFFSET,
        material=track_steel,
    )
    _add_cylindrical_segment(
        core,
        name="neck_lower",
        radius=SHAFT_RADIUS,
        length=END_CLEARANCE,
        center_z=-(ROD_LENGTH * 0.5) + COLLAR_LENGTH + (END_CLEARANCE * 0.5),
        material=shaft_black,
    )
    _add_cylindrical_segment(
        core,
        name="track_1",
        radius=TRACK_RADIUS,
        length=TRACK_LENGTH,
        center_z=RING_Z_POSITIONS[0],
        material=track_steel,
    )
    _add_cylindrical_segment(
        core,
        name="spacer_1",
        radius=SHAFT_RADIUS,
        length=INTER_RING_GAP,
        center_z=-0.0115,
        material=shaft_black,
    )
    _add_cylindrical_segment(
        core,
        name="track_2",
        radius=TRACK_RADIUS,
        length=TRACK_LENGTH,
        center_z=RING_Z_POSITIONS[1],
        material=track_steel,
    )
    _add_cylindrical_segment(
        core,
        name="spacer_2",
        radius=SHAFT_RADIUS,
        length=INTER_RING_GAP,
        center_z=0.0115,
        material=shaft_black,
    )
    _add_cylindrical_segment(
        core,
        name="track_3",
        radius=TRACK_RADIUS,
        length=TRACK_LENGTH,
        center_z=RING_Z_POSITIONS[2],
        material=track_steel,
    )
    _add_cylindrical_segment(
        core,
        name="neck_upper",
        radius=SHAFT_RADIUS,
        length=END_CLEARANCE,
        center_z=(ROD_LENGTH * 0.5) - COLLAR_LENGTH - (END_CLEARANCE * 0.5),
        material=shaft_black,
    )
    _add_cylindrical_segment(
        core,
        name="collar_upper",
        radius=COLLAR_RADIUS,
        length=COLLAR_LENGTH,
        center_z=COLLAR_OFFSET,
        material=track_steel,
    )
    core.inertial = Inertial.from_geometry(
        Cylinder(radius=COLLAR_RADIUS, length=ROD_LENGTH),
        mass=0.14,
    )

    _add_ring(model, core, index=1, z_pos=RING_Z_POSITIONS[0], band_mesh=band_mesh, material=bronze)
    _add_ring(model, core, index=2, z_pos=RING_Z_POSITIONS[1], band_mesh=band_mesh, material=titanium)
    _add_ring(model, core, index=3, z_pos=RING_Z_POSITIONS[2], band_mesh=band_mesh, material=gunmetal)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    core = object_model.get_part("core")
    ring_1 = object_model.get_part("ring_1")
    ring_2 = object_model.get_part("ring_2")
    ring_3 = object_model.get_part("ring_3")
    spin_1 = object_model.get_articulation("core_to_ring_1")
    spin_2 = object_model.get_articulation("core_to_ring_2")
    spin_3 = object_model.get_articulation("core_to_ring_3")

    collar_lower = core.get_visual("collar_lower")
    collar_upper = core.get_visual("collar_upper")
    track_1 = core.get_visual("track_1")
    track_2 = core.get_visual("track_2")
    track_3 = core.get_visual("track_3")
    band_1 = ring_1.get_visual("band")
    band_2 = ring_2.get_visual("band")
    band_3 = ring_3.get_visual("band")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    for ring in (ring_1, ring_2, ring_3):
        ctx.allow_overlap(ring, core, reason="ring band lightly preloads its bearing track so the spinner does not float off the core")
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

    for ring, band, track in (
        (ring_1, band_1, track_1),
        (ring_2, band_2, track_2),
        (ring_3, band_3, track_3),
    ):
        ctx.expect_origin_distance(ring, core, axes="xy", max_dist=0.0005)
        ctx.expect_overlap(ring, core, axes="xy", min_overlap=TRACK_RADIUS * 1.9, elem_a=band, elem_b=track)
        ctx.expect_within(core, ring, axes="xy", inner_elem=track, outer_elem=band)

    ctx.expect_gap(
        ring_1,
        core,
        axis="z",
        min_gap=0.0035,
        max_gap=0.0045,
        positive_elem=band_1,
        negative_elem=collar_lower,
    )
    ctx.expect_gap(
        ring_2,
        ring_1,
        axis="z",
        min_gap=0.0045,
        max_gap=0.0055,
        positive_elem=band_2,
        negative_elem=band_1,
    )
    ctx.expect_gap(
        ring_3,
        ring_2,
        axis="z",
        min_gap=0.0045,
        max_gap=0.0055,
        positive_elem=band_3,
        negative_elem=band_2,
    )
    ctx.expect_gap(
        core,
        ring_3,
        axis="z",
        min_gap=0.0035,
        max_gap=0.0045,
        positive_elem=collar_upper,
        negative_elem=band_3,
    )
    ctx.expect_within(core, core, axes="xy", inner_elem=track_1, outer_elem=collar_lower)
    ctx.expect_within(core, core, axes="xy", inner_elem=track_3, outer_elem=collar_upper)

    with ctx.pose({spin_1: 1.3, spin_2: -2.2, spin_3: 3.7}):
        ctx.expect_within(core, ring_1, axes="xy", inner_elem=track_1, outer_elem=band_1)
        ctx.expect_within(core, ring_2, axes="xy", inner_elem=track_2, outer_elem=band_2)
        ctx.expect_within(core, ring_3, axes="xy", inner_elem=track_3, outer_elem=band_3)
        ctx.expect_gap(
            ring_2,
            ring_1,
            axis="z",
            min_gap=0.0045,
            max_gap=0.0055,
            positive_elem=band_2,
            negative_elem=band_1,
        )
        ctx.expect_gap(
            ring_3,
            ring_2,
            axis="z",
            min_gap=0.0045,
            max_gap=0.0055,
            positive_elem=band_3,
            negative_elem=band_2,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
