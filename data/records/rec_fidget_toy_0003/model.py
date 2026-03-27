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
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

BODY_THICKNESS = 0.0084
BODY_HALF = BODY_THICKNESS * 0.5
LIP_THICKNESS = 0.0007
BEARING_THICKNESS = BODY_THICKNESS - 2.0 * LIP_THICKNESS

LOBE_CENTER_RADIUS = 0.034
LOBE_LENGTH = 0.048
LOBE_WIDTH = 0.034
LOBE_CORNER_RADIUS = 0.0062

CENTER_COLLAR_RADIUS = 0.0245
CENTER_BEARING_OUTER_RADIUS = 0.0187
CENTER_BEARING_INNER_RADIUS = 0.0074
CENTER_INNER_RACE_OUTER_RADIUS = 0.0072
CENTER_CAPTURE_INNER_RADIUS = 0.0170
CENTER_CAPTURE_OUTER_RADIUS = 0.0218

OUTER_BEARING_OUTER_RADIUS = 0.0113
OUTER_BEARING_INNER_RADIUS = 0.0064
OUTER_CAPTURE_INNER_RADIUS = 0.0100
OUTER_CAPTURE_OUTER_RADIUS = 0.0137

WEIGHT_RADIUS = 0.0038
WEIGHT_OFFSET_LOCAL_X = 0.0165

HUB_AXLE_RADIUS = 0.0058
INNER_RACE_BORE_RADIUS = 0.00572
HUB_CAP_RADIUS = 0.0134
HUB_CAP_THICKNESS = 0.0032
HUB_FACE_CLEARANCE = 0.0006

RADIAL_SEGMENTS = 72


def _lobe_angle(index: int) -> float:
    return index * (2.0 * math.pi / 3.0)


def _polar(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _lobe_center(index: int) -> tuple[float, float]:
    return _polar(LOBE_CENTER_RADIUS, _lobe_angle(index))


def _local_to_world(
    x_local: float,
    y_local: float,
    angle: float,
    center_xy: tuple[float, float],
) -> tuple[float, float]:
    cx, cy = center_xy
    c = math.cos(angle)
    s = math.sin(angle)
    return (
        cx + c * x_local - s * y_local,
        cy + s * x_local + c * y_local,
    )


def _weight_center(index: int) -> tuple[float, float]:
    return _local_to_world(
        WEIGHT_OFFSET_LOCAL_X,
        0.0,
        _lobe_angle(index),
        _lobe_center(index),
    )


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _ring_geometry(outer_radius: float, inner_radius: float, thickness: float):
    return boolean_difference(
        CylinderGeometry(radius=outer_radius, height=thickness, radial_segments=RADIAL_SEGMENTS),
        CylinderGeometry(radius=inner_radius, height=thickness + 0.002, radial_segments=RADIAL_SEGMENTS),
    )


def _hole_cutter(radius: float, center_xy: tuple[float, float]):
    return CylinderGeometry(
        radius=radius,
        height=BODY_THICKNESS + 0.004,
        radial_segments=RADIAL_SEGMENTS,
    ).translate(center_xy[0], center_xy[1], 0.0)


def _lobe_plate_geometry(index: int):
    cx, cy = _lobe_center(index)
    return ExtrudeGeometry.centered(
        rounded_rect_profile(
            LOBE_LENGTH,
            LOBE_WIDTH,
            LOBE_CORNER_RADIUS,
            corner_segments=8,
        ),
        BODY_THICKNESS,
    ).rotate_z(_lobe_angle(index)).translate(cx, cy, 0.0)


def _build_spinner_body_geometry():
    body_geom = CylinderGeometry(
        radius=CENTER_COLLAR_RADIUS,
        height=BODY_THICKNESS,
        radial_segments=RADIAL_SEGMENTS,
    )
    for index in range(3):
        body_geom = boolean_union(body_geom, _lobe_plate_geometry(index))

    body_geom = boolean_difference(body_geom, _hole_cutter(CENTER_BEARING_OUTER_RADIUS, (0.0, 0.0)))
    for index in range(3):
        body_geom = boolean_difference(
            body_geom,
            _hole_cutter(OUTER_BEARING_OUTER_RADIUS, _lobe_center(index)),
        )
        body_geom = boolean_difference(
            body_geom,
            _hole_cutter(WEIGHT_RADIUS, _weight_center(index)),
        )
    return body_geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tri_spinner_fidget", assets=ASSETS)

    body_polymer = model.material("body_polymer", rgba=(0.17, 0.18, 0.22, 1.0))
    hub_black = model.material("hub_black", rgba=(0.07, 0.07, 0.08, 1.0))
    bearing_steel = model.material("bearing_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    plug_metal = model.material("plug_metal", rgba=(0.80, 0.77, 0.71, 1.0))

    spinner_body_mesh = _save_mesh("tri_spinner_body.obj", _build_spinner_body_geometry())
    center_lip_mesh = _save_mesh(
        "tri_spinner_center_capture_lip.obj",
        _ring_geometry(CENTER_CAPTURE_OUTER_RADIUS, CENTER_CAPTURE_INNER_RADIUS, LIP_THICKNESS),
    )
    outer_lip_mesh = _save_mesh(
        "tri_spinner_outer_capture_lip.obj",
        _ring_geometry(OUTER_CAPTURE_OUTER_RADIUS, OUTER_CAPTURE_INNER_RADIUS, LIP_THICKNESS),
    )
    center_bearing_mesh = _save_mesh(
        "tri_spinner_center_bearing_ring.obj",
        _ring_geometry(CENTER_BEARING_OUTER_RADIUS, CENTER_BEARING_INNER_RADIUS, BEARING_THICKNESS),
    )
    outer_bearing_mesh = _save_mesh(
        "tri_spinner_outer_bearing_ring.obj",
        _ring_geometry(OUTER_BEARING_OUTER_RADIUS, OUTER_BEARING_INNER_RADIUS, BEARING_THICKNESS),
    )
    center_inner_race_mesh = _save_mesh(
        "tri_spinner_center_inner_race.obj",
        _ring_geometry(CENTER_INNER_RACE_OUTER_RADIUS, INNER_RACE_BORE_RADIUS, BEARING_THICKNESS),
    )

    center_hub = model.part("center_hub")
    center_hub.visual(
        Cylinder(radius=HUB_CAP_RADIUS, length=HUB_CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BODY_HALF + HUB_FACE_CLEARANCE + 0.5 * HUB_CAP_THICKNESS)),
        material=hub_black,
        name="top_cap",
    )
    center_hub.visual(
        Cylinder(radius=HUB_CAP_RADIUS, length=HUB_CAP_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, -(BODY_HALF + HUB_FACE_CLEARANCE + 0.5 * HUB_CAP_THICKNESS))),
        material=hub_black,
        name="bottom_cap",
    )
    center_hub.visual(
        Cylinder(
            radius=HUB_AXLE_RADIUS,
            length=BODY_THICKNESS + 2.0 * HUB_FACE_CLEARANCE + 2.0 * HUB_CAP_THICKNESS,
        ),
        material=hub_black,
        name="axle",
    )
    center_hub.inertial = Inertial.from_geometry(
        Cylinder(
            radius=HUB_CAP_RADIUS,
            length=BODY_THICKNESS + 2.0 * HUB_FACE_CLEARANCE + 2.0 * HUB_CAP_THICKNESS,
        ),
        mass=0.022,
    )

    spinner_body = model.part("spinner_body")
    spinner_body.visual(spinner_body_mesh, material=body_polymer, name="body_plate")
    spinner_body.visual(
        center_lip_mesh,
        origin=Origin(xyz=(0.0, 0.0, BODY_HALF - 0.5 * LIP_THICKNESS)),
        material=body_polymer,
        name="top_center_lip",
    )
    spinner_body.visual(
        center_bearing_mesh,
        material=bearing_steel,
        name="center_bearing_outer_ring",
    )
    spinner_body.visual(
        center_lip_mesh,
        origin=Origin(xyz=(0.0, 0.0, -(BODY_HALF - 0.5 * LIP_THICKNESS))),
        material=body_polymer,
        name="bottom_center_lip",
    )
    for index in range(3):
        cx, cy = _lobe_center(index)
        spinner_body.visual(
            outer_lip_mesh,
            origin=Origin(xyz=(cx, cy, BODY_HALF - 0.5 * LIP_THICKNESS)),
            material=body_polymer,
            name=f"top_outer_lip_{index}",
        )
        spinner_body.visual(
            outer_lip_mesh,
            origin=Origin(xyz=(cx, cy, -(BODY_HALF - 0.5 * LIP_THICKNESS))),
            material=body_polymer,
            name=f"bottom_outer_lip_{index}",
        )
    spinner_body.inertial = Inertial.from_geometry(
        Box((0.116, 0.116, BODY_THICKNESS)),
        mass=0.058,
    )

    for index in range(3):
        outer_bearing = model.part(f"outer_bearing_{index}")
        outer_bearing.visual(outer_bearing_mesh, material=bearing_steel, name="outer_ring")
        outer_bearing.inertial = Inertial.from_geometry(
            Cylinder(radius=OUTER_BEARING_OUTER_RADIUS, length=BEARING_THICKNESS),
            mass=0.004,
        )

        plug_weight = model.part(f"plug_weight_{index}")
        plug_weight.visual(
            Cylinder(radius=WEIGHT_RADIUS, length=BODY_THICKNESS),
            material=plug_metal,
            name="plug_weight",
        )
        plug_weight.inertial = Inertial.from_geometry(
            Cylinder(radius=WEIGHT_RADIUS, length=BODY_THICKNESS),
            mass=0.002,
        )

    center_inner_race = model.part("center_inner_race")
    center_inner_race.visual(center_inner_race_mesh, material=bearing_steel, name="inner_race")
    center_inner_race.inertial = Inertial.from_geometry(
        Cylinder(radius=CENTER_INNER_RACE_OUTER_RADIUS, length=BEARING_THICKNESS),
        mass=0.003,
    )

    model.articulation(
        "hub_to_spinner",
        ArticulationType.CONTINUOUS,
        parent=center_hub,
        child=spinner_body,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=60.0),
    )
    for index in range(3):
        cx, cy = _lobe_center(index)
        wx, wy = _weight_center(index)
        model.articulation(
            f"spinner_to_outer_bearing_{index}",
            ArticulationType.FIXED,
            parent=spinner_body,
            child=f"outer_bearing_{index}",
            origin=Origin(xyz=(cx, cy, 0.0)),
        )
        model.articulation(
            f"spinner_to_plug_weight_{index}",
            ArticulationType.FIXED,
            parent=spinner_body,
            child=f"plug_weight_{index}",
            origin=Origin(xyz=(wx, wy, 0.0)),
        )
    model.articulation(
        "hub_to_center_inner_race",
        ArticulationType.FIXED,
        parent=center_hub,
        child=center_inner_race,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    center_hub = object_model.get_part("center_hub")
    center_inner_race = object_model.get_part("center_inner_race")
    spinner_body = object_model.get_part("spinner_body")
    spin = object_model.get_articulation("hub_to_spinner")

    top_cap = center_hub.get_visual("top_cap")
    bottom_cap = center_hub.get_visual("bottom_cap")
    axle = center_hub.get_visual("axle")
    inner_race = center_inner_race.get_visual("inner_race")
    top_center_lip = spinner_body.get_visual("top_center_lip")
    bottom_center_lip = spinner_body.get_visual("bottom_center_lip")
    center_ring = spinner_body.get_visual("center_bearing_outer_ring")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.018)
    ctx.warn_if_part_geometry_disconnected()
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
    ctx.allow_overlap(center_hub, center_inner_race, reason="press-fit axle seats the stationary inner race")

    ctx.expect_origin_distance(spinner_body, center_hub, axes="xy", max_dist=0.0005)
    ctx.expect_origin_distance(center_inner_race, center_hub, axes="xy", max_dist=0.0005)
    ctx.expect_contact(center_hub, center_inner_race, elem_a=axle, elem_b=inner_race)
    ctx.expect_within(center_inner_race, spinner_body, axes="xy", inner_elem=inner_race, outer_elem=center_ring)
    ctx.expect_within(center_hub, spinner_body, axes="xy", inner_elem=axle, outer_elem=center_ring)
    ctx.expect_contact(spinner_body, spinner_body, elem_a=center_ring, elem_b=top_center_lip)
    ctx.expect_contact(spinner_body, spinner_body, elem_a=center_ring, elem_b=bottom_center_lip)
    ctx.expect_gap(
        center_hub,
        spinner_body,
        axis="z",
        min_gap=0.0004,
        max_gap=0.0009,
        positive_elem=top_cap,
        negative_elem=top_center_lip,
    )
    ctx.expect_gap(
        spinner_body,
        center_hub,
        axis="z",
        min_gap=0.0004,
        max_gap=0.0009,
        positive_elem=bottom_center_lip,
        negative_elem=bottom_cap,
    )

    for index in range(3):
        outer_bearing = object_model.get_part(f"outer_bearing_{index}")
        plug_weight = object_model.get_part(f"plug_weight_{index}")
        outer_ring = outer_bearing.get_visual("outer_ring")
        plug_visual = plug_weight.get_visual("plug_weight")
        top_outer_lip = spinner_body.get_visual(f"top_outer_lip_{index}")
        bottom_outer_lip = spinner_body.get_visual(f"bottom_outer_lip_{index}")

        ctx.expect_contact(outer_bearing, spinner_body)
        ctx.expect_within(outer_bearing, spinner_body, axes="xy")
        ctx.expect_contact(outer_bearing, spinner_body, elem_a=outer_ring, elem_b=top_outer_lip)
        ctx.expect_contact(outer_bearing, spinner_body, elem_a=outer_ring, elem_b=bottom_outer_lip)
        ctx.expect_contact(plug_weight, spinner_body, elem_a=plug_visual)
        ctx.expect_within(plug_weight, spinner_body, axes="xy")

    with ctx.pose({spin: math.pi / 3.0}):
        ctx.expect_origin_distance(spinner_body, center_hub, axes="xy", max_dist=0.0005)
        ctx.expect_gap(
            center_hub,
            spinner_body,
            axis="z",
            min_gap=0.0004,
            max_gap=0.0009,
            positive_elem=top_cap,
            negative_elem=top_center_lip,
        )
        ctx.expect_gap(
            spinner_body,
            center_hub,
            axis="z",
            min_gap=0.0004,
            max_gap=0.0009,
            positive_elem=bottom_center_lip,
            negative_elem=bottom_cap,
        )
        for index in range(3):
            outer_bearing = object_model.get_part(f"outer_bearing_{index}")
            plug_weight = object_model.get_part(f"plug_weight_{index}")
            ctx.expect_contact(outer_bearing, spinner_body)
            ctx.expect_contact(plug_weight, spinner_body)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
