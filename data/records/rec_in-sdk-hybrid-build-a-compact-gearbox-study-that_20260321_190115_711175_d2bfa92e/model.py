from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
import math

import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    BevelGearPair,
    Box,
    CrossedGearPair,
    HerringboneGear,
    HyperbolicGearPair,
    Inertial,
    Origin,
    PlanetaryGearset,
    RackGear,
    RingGear,
    SpurGear,
    TestContext,
    TestReport,
    Worm,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BASE_LENGTH = 0.50
BASE_WIDTH = 0.26
BASE_THICKNESS = 0.016
MODULE_PAD_THICKNESS = 0.006
GEAR_UNIT_SCALE = 0.001

MODULE_POSES = {
    "spur_gallery": (-0.155, 0.070, BASE_THICKNESS),
    "planetary_module": (0.000, 0.000, BASE_THICKNESS),
    "rack_worm_module": (-0.155, -0.075, BASE_THICKNESS),
    "bevel_module": (0.155, 0.070, BASE_THICKNESS),
    "skew_gallery": (0.155, -0.075, BASE_THICKNESS),
}

MODULE_BOUNDS = {
    "spur_gallery": (0.162, 0.095, 0.068),
    "planetary_module": (0.140, 0.110, 0.088),
    "rack_worm_module": (0.160, 0.095, 0.064),
    "bevel_module": (0.110, 0.090, 0.070),
    "skew_gallery": (0.150, 0.090, 0.068),
}


def _solid(shape: cq.Workplane | cq.Shape) -> cq.Shape:
    return shape.val() if isinstance(shape, cq.Workplane) else shape


def _mm(value: float) -> float:
    return value * 1000.0


def _compound(*shapes: cq.Workplane | cq.Shape) -> cq.Shape:
    return cq.Compound.makeCompound([_solid(shape) for shape in shapes if shape is not None])


def _moved(
    shape: cq.Workplane | cq.Shape,
    *,
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotate: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Shape:
    body = cq.Workplane(obj=_solid(shape))
    rx, ry, rz = rotate
    if rx:
        body = body.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), rx)
    if ry:
        body = body.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), ry)
    if rz:
        body = body.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), rz)
    x, y, z = xyz
    if x or y or z:
        body = body.translate((x, y, z))
    return body.val()


def _rounded_plate(length: float, width: float, thickness: float, corner: float) -> cq.Shape:
    return (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .edges("|Z")
        .fillet(corner)
        .translate((0.0, 0.0, thickness / 2.0))
        .val()
    )


def _box(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Shape:
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .translate((x, y, z + height / 2.0))
        .val()
    )


def _cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    axis: str = "z",
) -> cq.Shape:
    body = cq.Workplane("XY").circle(radius).extrude(length)
    if axis == "x":
        body = body.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    elif axis == "y":
        body = body.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    return body.translate((x, y, z)).val()


def _spur_gallery_geometry() -> tuple[cq.Shape, cq.Shape, cq.Shape]:
    pad = _rounded_plate(0.162, 0.095, MODULE_PAD_THICKNESS, 0.006)
    rear_beam = _box(0.138, 0.012, 0.012, y=-0.026, z=MODULE_PAD_THICKNESS)
    pedestals = [
        _cylinder(0.0065, 0.018, x=x, y=0.0, z=MODULE_PAD_THICKNESS)
        for x in (-0.050, 0.000, 0.050)
    ]
    structure = _compound(pad, rear_beam, *pedestals)

    shafts = _compound(
        *[
            _cylinder(0.0032, 0.036, x=x, y=0.0, z=MODULE_PAD_THICKNESS)
            for x in (-0.050, 0.000, 0.050)
        ]
    )

    spur = SpurGear(module=_mm(0.002), teeth_number=18, width=_mm(0.010), bore_d=_mm(0.006))
    helical = SpurGear(
        module=_mm(0.002),
        teeth_number=20,
        width=_mm(0.010),
        helix_angle=28.0,
        bore_d=_mm(0.006),
    )
    herringbone = HerringboneGear(
        module=_mm(0.002),
        teeth_number=24,
        width=_mm(0.012),
        helix_angle=20.0,
        bore_d=_mm(0.006),
    )

    gears = _compound(
        _moved(cq.Workplane("XY").gear(spur).val(), xyz=(_mm(-0.050), 0.000, _mm(0.024)), rotate=(0.0, 0.0, 14.0)),
        _moved(cq.Workplane("XY").gear(helical).val(), xyz=(0.000, 0.000, _mm(0.024)), rotate=(0.0, 0.0, -9.0)),
        _moved(cq.Workplane("XY").gear(herringbone).val(), xyz=(_mm(0.050), 0.000, _mm(0.024)), rotate=(0.0, 0.0, 18.0)),
    )
    return structure, shafts, gears


def _planetary_module_geometry() -> tuple[cq.Shape, cq.Shape, cq.Shape]:
    pad = _rounded_plate(0.140, 0.110, MODULE_PAD_THICKNESS, 0.006)
    carrier_dish = _moved(cq.Workplane("XY").circle(0.040).extrude(0.004).val(), xyz=(-0.022, 0.000, MODULE_PAD_THICKNESS))
    center_hub = _cylinder(0.008, 0.020, x=-0.022, y=0.000, z=MODULE_PAD_THICKNESS)

    planet_posts = []
    for angle_deg in (0.0, 120.0, 240.0):
        angle = math.radians(angle_deg)
        planet_posts.append(
            _cylinder(
                0.0032,
                0.019,
                x=-0.022 + 0.024 * math.cos(angle),
                y=0.024 * math.sin(angle),
                z=MODULE_PAD_THICKNESS,
            )
        )

    ring_mount = _compound(
        _box(0.010, 0.014, 0.050, x=0.046, y=-0.018, z=MODULE_PAD_THICKNESS),
        _box(0.010, 0.014, 0.050, x=0.046, y=0.034, z=MODULE_PAD_THICKNESS),
        _box(0.010, 0.066, 0.010, x=0.046, y=0.008, z=0.046),
        _box(0.020, 0.012, 0.012, x=0.036, y=0.008, z=MODULE_PAD_THICKNESS + 0.004),
    )
    structure = _compound(pad, carrier_dish, center_hub, *planet_posts, ring_mount)

    planetary = PlanetaryGearset(
        module=_mm(0.002),
        sun_teeth_number=12,
        planet_teeth_number=9,
        width=_mm(0.008),
        rim_width=_mm(0.004),
        n_planets=3,
    )
    ring = RingGear(
        module=_mm(0.002),
        teeth_number=42,
        width=_mm(0.008),
        rim_width=_mm(0.004),
    )

    planetary_gears = _moved(planetary.build(), xyz=(_mm(-0.022), 0.000, _mm(0.026)), rotate=(0.0, 0.0, 18.0))
    ring_display = _moved(ring.build(), xyz=(_mm(0.046), _mm(0.008), _mm(0.042)), rotate=(0.0, 90.0, 0.0))
    return structure, planetary_gears, ring_display


def _rack_worm_module_geometry() -> tuple[cq.Shape, cq.Shape, cq.Shape]:
    pad = _rounded_plate(0.160, 0.095, MODULE_PAD_THICKNESS, 0.006)
    rack_bed = _box(0.096, 0.020, 0.008, x=-0.032, y=0.000, z=MODULE_PAD_THICKNESS)
    pinion_post = _cylinder(0.007, 0.018, x=-0.026, y=0.023, z=MODULE_PAD_THICKNESS)
    worm_cradle = _compound(
        _box(0.012, 0.016, 0.018, x=0.028, y=0.030, z=MODULE_PAD_THICKNESS),
        _box(0.012, 0.016, 0.018, x=0.062, y=0.030, z=MODULE_PAD_THICKNESS),
        _box(0.046, 0.008, 0.010, x=0.045, y=0.030, z=MODULE_PAD_THICKNESS + 0.010),
        _cylinder(0.0065, 0.018, x=0.045, y=0.000, z=MODULE_PAD_THICKNESS),
    )
    structure = _compound(pad, rack_bed, pinion_post, worm_cradle)

    rack = RackGear(module=_mm(0.002), length=_mm(0.090), width=_mm(0.008), height=_mm(0.006))
    pinion = SpurGear(module=_mm(0.002), teeth_number=18, width=_mm(0.008), bore_d=_mm(0.006))
    worm_wheel = SpurGear(module=_mm(0.002), teeth_number=20, width=_mm(0.010), bore_d=_mm(0.006))
    worm = Worm(
        module=_mm(0.002),
        lead_angle=20.0,
        n_threads=2,
        length=_mm(0.050),
        bore_d=_mm(0.006),
    )

    gears = _compound(
        _moved(rack.build(), xyz=(_mm(-0.032), 0.000, _mm(0.016))),
        _moved(
            cq.Workplane("XY").gear(pinion).val(),
            xyz=(_mm(-0.026), pinion.r0 + _mm(0.003), _mm(0.028)),
            rotate=(0.0, 0.0, 90.0),
        ),
        _moved(
            cq.Workplane("XY").gear(worm_wheel).val(),
            xyz=(_mm(0.045), 0.000, _mm(0.028)),
            rotate=(0.0, 0.0, 8.0),
        ),
    )
    shafts = _moved(
        worm.build(),
        xyz=(_mm(0.045), worm_wheel.r0 + _mm(0.010), _mm(0.028)),
        rotate=(0.0, 90.0, 0.0),
    )
    return structure, shafts, gears


def _bevel_module_geometry() -> tuple[cq.Shape, cq.Shape]:
    pad = _rounded_plate(0.110, 0.090, MODULE_PAD_THICKNESS, 0.006)
    corner_frame = _compound(
        _box(0.016, 0.070, 0.028, x=-0.010, y=0.000, z=MODULE_PAD_THICKNESS),
        _box(0.060, 0.016, 0.028, x=0.012, y=-0.010, z=MODULE_PAD_THICKNESS),
        _cylinder(0.0032, 0.026, x=0.000, y=0.000, z=MODULE_PAD_THICKNESS),
        _cylinder(0.0032, 0.030, x=0.000, y=0.000, z=0.024, axis="x"),
    )
    structure = _compound(pad, corner_frame)

    pair = BevelGearPair(
        module=_mm(0.002),
        gear_teeth=24,
        pinion_teeth=16,
        face_width=_mm(0.008),
        axis_angle=90.0,
    )
    gears = _moved(pair.build(), xyz=(0.000, 0.000, _mm(0.029)), rotate=(0.0, 0.0, 32.0))
    return structure, gears


def _skew_gallery_geometry() -> tuple[cq.Shape, cq.Shape]:
    pad = _rounded_plate(0.150, 0.090, MODULE_PAD_THICKNESS, 0.006)
    supports = _compound(
        _box(0.048, 0.016, 0.016, x=-0.038, y=0.000, z=MODULE_PAD_THICKNESS),
        _box(0.048, 0.016, 0.016, x=0.040, y=0.000, z=MODULE_PAD_THICKNESS),
        _box(0.018, 0.060, 0.014, x=-0.038, y=0.000, z=MODULE_PAD_THICKNESS),
        _box(0.018, 0.060, 0.014, x=0.040, y=0.000, z=MODULE_PAD_THICKNESS),
    )
    structure = _compound(pad, supports)

    crossed = CrossedGearPair(
        module=_mm(0.002),
        gear1_teeth_number=18,
        gear2_teeth_number=18,
        gear1_width=_mm(0.008),
        gear2_width=_mm(0.008),
        shaft_angle=90.0,
        gear1_helix_angle=30.0,
    )
    hyperbolic = HyperbolicGearPair(
        module=_mm(0.002),
        gear1_teeth_number=20,
        width=_mm(0.008),
        shaft_angle=60.0,
    )
    gears = _compound(
        _moved(crossed.build(), xyz=(_mm(-0.038), 0.000, _mm(0.038)), rotate=(0.0, 0.0, 22.0)),
        _moved(hyperbolic.build(), xyz=(_mm(0.040), 0.000, _mm(0.040)), rotate=(0.0, 0.0, -18.0)),
    )
    return structure, gears


def _base_geometry() -> cq.Shape:
    base = _rounded_plate(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, 0.014)
    top_recess = _moved(
        cq.Workplane("XY")
        .box(BASE_LENGTH - 0.040, BASE_WIDTH - 0.040, 0.004)
        .edges("|Z")
        .fillet(0.010)
        .val(),
        xyz=(0.0, 0.0, BASE_THICKNESS - 0.002),
    )
    center_spine = _box(0.350, 0.028, 0.004, z=BASE_THICKNESS)
    return cq.Workplane(obj=base).cut(top_recess).union(cq.Workplane(obj=center_spine)).val()


def _set_part_inertial(part, size: tuple[float, float, float], mass: float) -> None:
    sx, sy, sz = size
    part.inertial = Inertial.from_geometry(
        Box((sx, sy, sz)),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, sz / 2.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_gearbox_study", assets=ASSETS)

    graphite = model.material("graphite", rgba=(0.20, 0.22, 0.25, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.34, 0.36, 0.40, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.75, 0.79, 1.0))
    oxide = model.material("oxide", rgba=(0.14, 0.15, 0.17, 1.0))
    brass = model.material("brass", rgba=(0.78, 0.63, 0.28, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_geometry(), "base_plate.obj", assets=ASSETS), material=graphite)
    _set_part_inertial(base, (BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS), mass=5.0)

    spur_gallery = model.part("spur_gallery")
    spur_structure, spur_shafts, spur_gears = _spur_gallery_geometry()
    spur_gallery.visual(mesh_from_cadquery(spur_structure, "spur_gallery_structure.obj", assets=ASSETS), material=frame_gray)
    spur_gallery.visual(mesh_from_cadquery(spur_shafts, "spur_gallery_shafts.obj", assets=ASSETS), material=oxide)
    spur_gallery.visual(
        mesh_from_cadquery(spur_gears, "spur_gallery_gears.obj", assets=ASSETS, unit_scale=GEAR_UNIT_SCALE),
        material=steel,
    )
    _set_part_inertial(spur_gallery, MODULE_BOUNDS["spur_gallery"], mass=0.7)

    planetary_module = model.part("planetary_module")
    planetary_structure, planetary_gears, ring_display = _planetary_module_geometry()
    planetary_module.visual(
        mesh_from_cadquery(planetary_structure, "planetary_structure.obj", assets=ASSETS),
        material=frame_gray,
    )
    planetary_module.visual(
        mesh_from_cadquery(planetary_gears, "planetary_gears.obj", assets=ASSETS, unit_scale=GEAR_UNIT_SCALE),
        material=steel,
    )
    planetary_module.visual(
        mesh_from_cadquery(ring_display, "planetary_ring_display.obj", assets=ASSETS, unit_scale=GEAR_UNIT_SCALE),
        material=brass,
    )
    _set_part_inertial(planetary_module, MODULE_BOUNDS["planetary_module"], mass=0.9)

    rack_worm_module = model.part("rack_worm_module")
    rack_structure, worm_drive, rack_worm_gears = _rack_worm_module_geometry()
    rack_worm_module.visual(
        mesh_from_cadquery(rack_structure, "rack_worm_structure.obj", assets=ASSETS),
        material=frame_gray,
    )
    rack_worm_module.visual(
        mesh_from_cadquery(worm_drive, "rack_worm_drive.obj", assets=ASSETS, unit_scale=GEAR_UNIT_SCALE),
        material=brass,
    )
    rack_worm_module.visual(
        mesh_from_cadquery(rack_worm_gears, "rack_worm_gears.obj", assets=ASSETS, unit_scale=GEAR_UNIT_SCALE),
        material=steel,
    )
    _set_part_inertial(rack_worm_module, MODULE_BOUNDS["rack_worm_module"], mass=0.8)

    bevel_module = model.part("bevel_module")
    bevel_structure, bevel_gears = _bevel_module_geometry()
    bevel_module.visual(mesh_from_cadquery(bevel_structure, "bevel_structure.obj", assets=ASSETS), material=frame_gray)
    bevel_module.visual(
        mesh_from_cadquery(bevel_gears, "bevel_gears.obj", assets=ASSETS, unit_scale=GEAR_UNIT_SCALE),
        material=steel,
    )
    _set_part_inertial(bevel_module, MODULE_BOUNDS["bevel_module"], mass=0.5)

    skew_gallery = model.part("skew_gallery")
    skew_structure, skew_gears = _skew_gallery_geometry()
    skew_gallery.visual(mesh_from_cadquery(skew_structure, "skew_structure.obj", assets=ASSETS), material=frame_gray)
    skew_gallery.visual(
        mesh_from_cadquery(skew_gears, "skew_gears.obj", assets=ASSETS, unit_scale=GEAR_UNIT_SCALE),
        material=steel,
    )
    _set_part_inertial(skew_gallery, MODULE_BOUNDS["skew_gallery"], mass=0.6)

    for child_name, pose in MODULE_POSES.items():
        model.articulation(
            f"base_to_{child_name}",
            ArticulationType.FIXED,
            parent="base",
            child=child_name,
            origin=Origin(xyz=pose),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=32,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    expected_parts = {
        "base",
        "spur_gallery",
        "planetary_module",
        "rack_worm_module",
        "bevel_module",
        "skew_gallery",
    }
    expected_joints = {
        "base_to_spur_gallery",
        "base_to_planetary_module",
        "base_to_rack_worm_module",
        "base_to_bevel_module",
        "base_to_skew_gallery",
    }
    assert {part.name for part in object_model.parts} == expected_parts, "scene should expose one base and five study bays covering the requested gear families"
    assert {joint.name for joint in object_model.articulations} == expected_joints, "each study bay should mount explicitly to the common base"

    for module_name in (
        "spur_gallery",
        "planetary_module",
        "rack_worm_module",
        "bevel_module",
        "skew_gallery",
    ):
        ctx.expect_aabb_overlap(module_name, "base", axes="xy", min_overlap=0.045)
        module_pos = ctx.part_world_position(module_name)
        assert abs(module_pos[2] - BASE_THICKNESS) < 1e-9, f"{module_name} should mount on the top surface datum of the base plate"

    spur_pos = ctx.part_world_position("spur_gallery")
    planetary_pos = ctx.part_world_position("planetary_module")
    rack_pos = ctx.part_world_position("rack_worm_module")
    bevel_pos = ctx.part_world_position("bevel_module")
    skew_pos = ctx.part_world_position("skew_gallery")

    assert spur_pos[0] < -0.10 and spur_pos[1] > 0.02, "spur, helical, and herringbone gallery should occupy the front-left bay"
    assert abs(planetary_pos[0]) < 0.03 and abs(planetary_pos[1]) < 0.03, "ring gear and planetary display should anchor the center of the study"
    assert rack_pos[0] < -0.10 and rack_pos[1] < -0.02, "rack and worm study should occupy the rear-left bay"
    assert bevel_pos[0] > 0.10 and bevel_pos[1] > 0.02, "bevel gear pair should sit in the front-right bay"
    assert skew_pos[0] > 0.10 and skew_pos[1] < -0.02, "crossed and hyperbolic gears should sit in the rear-right bay"
    assert abs(spur_pos[0] - rack_pos[0]) < 0.01, "left-side studies should align into a readable column"
    assert abs(bevel_pos[0] - skew_pos[0]) < 0.01, "right-side studies should align into a readable column"
    assert abs(spur_pos[1] - bevel_pos[1]) < 0.01, "front-row studies should share a common presentation line"
    assert abs(rack_pos[1] - skew_pos[1]) < 0.01, "rear-row studies should share a common presentation line"
    assert spur_pos[0] < planetary_pos[0] < bevel_pos[0], "planetary module should separate the left and right galleries"
    assert rack_pos[1] < planetary_pos[1] < spur_pos[1], "planetary module should sit between the front and rear rows"
    assert abs(spur_pos[0] - bevel_pos[0]) > 0.25, "front studies should remain widely separated so their gear silhouettes stay unobscured"
    assert abs(spur_pos[1] - rack_pos[1]) > 0.10, "left-side studies should have visible front-to-rear spacing"

    base_pos = ctx.part_world_position("base")
    assert abs(base_pos[0]) < 1e-9 and abs(base_pos[1]) < 1e-9, "base should remain centered to frame the whole scene"

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
