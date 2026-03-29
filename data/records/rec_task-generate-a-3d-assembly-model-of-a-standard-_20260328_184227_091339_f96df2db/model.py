from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    SphereGeometry,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    wire_from_points,
)


CUBE_SIZE = 0.057
CUBE_HALF = CUBE_SIZE * 0.5
MODULE = CUBE_SIZE / 3.0
SEAM_GAP = 0.0012
SLIDING_CLEARANCE = 0.0001

CENTER_CAP_SIZE = 0.0178
CENTER_CAP_DEPTH = 0.0060
CENTER_NECK_SIZE = 0.0108
CENTER_NECK_DEPTH = 0.0080
CENTER_WING_LENGTH = 0.0156
CENTER_WING_WIDTH = 0.0056
CENTER_WING_DEPTH = 0.0036
CENTER_SOCKET_RADIUS = 0.01405
CENTER_CAVITY_RADIUS = 0.0047
CENTER_CAVITY_DEPTH = 0.0106

EDGE_BODY_SIZE = 0.0176
CORNER_BODY_SIZE = 0.0174
TILE_DEPTH = 0.0011
TILE_SPAN = 0.0180

EDGE_FOOT_RADIUS = CENTER_SOCKET_RADIUS - SLIDING_CLEARANCE
CORNER_FOOT_RADIUS = CENTER_SOCKET_RADIUS - SLIDING_CLEARANCE
EDGE_BAND = 0.0028
EDGE_CUT = 0.0042
CORNER_CUT = 0.0038

CORE_ARM_RADIUS = 0.0031
CORE_ARM_LENGTH = 0.020
CORE_SHAFT_RADIUS = 0.0019
CORE_SHAFT_LENGTH = 0.010
CORE_COLLAR_RADIUS = 0.0036
CORE_COLLAR_THICKNESS = 0.0018


FACE_TO_COORDS: dict[str, tuple[int, int, int]] = {
    "u": (0, 0, 1),
    "d": (0, 0, -1),
    "f": (0, 1, 0),
    "b": (0, -1, 0),
    "r": (1, 0, 0),
    "l": (-1, 0, 0),
}

EDGE_TO_COORDS: dict[str, tuple[int, int, int]] = {
    "ur": (1, 0, 1),
    "uf": (0, 1, 1),
    "ul": (-1, 0, 1),
    "ub": (0, -1, 1),
    "dr": (1, 0, -1),
    "df": (0, 1, -1),
    "dl": (-1, 0, -1),
    "db": (0, -1, -1),
    "fr": (1, 1, 0),
    "fl": (-1, 1, 0),
    "br": (1, -1, 0),
    "bl": (-1, -1, 0),
}

CORNER_TO_COORDS: dict[str, tuple[int, int, int]] = {
    "urf": (1, 1, 1),
    "ufl": (-1, 1, 1),
    "ulb": (-1, -1, 1),
    "ubr": (1, -1, 1),
    "dfr": (1, 1, -1),
    "dlf": (-1, 1, -1),
    "dbl": (-1, -1, -1),
    "drb": (1, -1, -1),
}


def _axis_index(coords: tuple[int, int, int]) -> int:
    for index, value in enumerate(coords):
        if value != 0:
            return index
    raise ValueError("No non-zero axis in coordinate tuple")


def _nonzero_faces(coords: tuple[int, int, int]) -> list[str]:
    faces: list[str] = []
    for face, face_coords in FACE_TO_COORDS.items():
        for coord, face_coord in zip(coords, face_coords):
            if face_coord != 0 and coord == face_coord:
                faces.append(face)
                break
    return faces


def _coord_point(
    coords: tuple[int, int, int],
    *,
    nonzero_abs: float,
    zero_abs: float = 0.0,
) -> tuple[float, float, float]:
    point = []
    for value in coords:
        if value == 0:
            point.append(zero_abs)
        else:
            point.append(value * nonzero_abs)
    return tuple(point)


def _center_for_face(face: str, axial_abs: float) -> tuple[float, float, float]:
    coords = FACE_TO_COORDS[face]
    return tuple(value * axial_abs for value in coords)


def _size_for_axis(axis_index: int, depth: float, span_a: float, span_b: float) -> tuple[float, float, float]:
    size = [0.0, 0.0, 0.0]
    tangents = [idx for idx in range(3) if idx != axis_index]
    size[axis_index] = depth
    size[tangents[0]] = span_a
    size[tangents[1]] = span_b
    return tuple(size)


def _rpy_for_axis(axis_index: int) -> tuple[float, float, float]:
    if axis_index == 0:
        return (0.0, math.pi / 2.0, 0.0)
    if axis_index == 1:
        return (math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _box_geom(size: tuple[float, float, float], center: tuple[float, float, float]) -> MeshGeometry:
    return BoxGeometry(size).translate(*center)


def _cyl_geom(
    radius: float,
    length: float,
    center: tuple[float, float, float],
    *,
    axis_index: int,
) -> MeshGeometry:
    geom = CylinderGeometry(radius=radius, height=length, radial_segments=32)
    if axis_index == 0:
        geom.rotate_y(math.pi / 2.0)
    elif axis_index == 1:
        geom.rotate_x(math.pi / 2.0)
    geom.translate(*center)
    return geom


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _subtract_box(
    geom: MeshGeometry,
    *,
    min_xyz: tuple[float, float, float],
    max_xyz: tuple[float, float, float],
) -> MeshGeometry:
    size = (
        max_xyz[0] - min_xyz[0],
        max_xyz[1] - min_xyz[1],
        max_xyz[2] - min_xyz[2],
    )
    center = (
        0.5 * (min_xyz[0] + max_xyz[0]),
        0.5 * (min_xyz[1] + max_xyz[1]),
        0.5 * (min_xyz[2] + max_xyz[2]),
    )
    return boolean_difference(geom, _box_geom(size, center))


def _sphere_sector(
    radius: float,
    *,
    min_x: float | None = None,
    max_x: float | None = None,
    min_y: float | None = None,
    max_y: float | None = None,
    min_z: float | None = None,
    max_z: float | None = None,
) -> MeshGeometry:
    geom = SphereGeometry(radius=radius, width_segments=48, height_segments=30)
    extent = 0.08
    if min_x is not None:
        geom = _subtract_box(
            geom,
            min_xyz=(-extent, -extent, -extent),
            max_xyz=(min_x, extent, extent),
        )
    if max_x is not None:
        geom = _subtract_box(
            geom,
            min_xyz=(max_x, -extent, -extent),
            max_xyz=(extent, extent, extent),
        )
    if min_y is not None:
        geom = _subtract_box(
            geom,
            min_xyz=(-extent, -extent, -extent),
            max_xyz=(extent, min_y, extent),
        )
    if max_y is not None:
        geom = _subtract_box(
            geom,
            min_xyz=(-extent, max_y, -extent),
            max_xyz=(extent, extent, extent),
        )
    if min_z is not None:
        geom = _subtract_box(
            geom,
            min_xyz=(-extent, -extent, -extent),
            max_xyz=(extent, extent, min_z),
        )
    if max_z is not None:
        geom = _subtract_box(
            geom,
            min_xyz=(-extent, -extent, max_z),
            max_xyz=(extent, extent, extent),
        )
    return geom


def _center_shell_mesh(face: str) -> MeshGeometry:
    coords = FACE_TO_COORDS[face]
    axis_index = _axis_index(coords)
    sign = coords[axis_index]
    axial_cap = CUBE_HALF - CENTER_CAP_DEPTH * 0.5
    axial_neck = CUBE_HALF - CENTER_CAP_DEPTH - CENTER_NECK_DEPTH * 0.5 + 0.0001
    axial_wing = CUBE_HALF - CENTER_CAP_DEPTH - CENTER_NECK_DEPTH - CENTER_WING_DEPTH * 0.5 + 0.0001

    cap_center = [0.0, 0.0, 0.0]
    cap_center[axis_index] = sign * axial_cap
    neck_center = [0.0, 0.0, 0.0]
    neck_center[axis_index] = sign * axial_neck
    wing_center = [0.0, 0.0, 0.0]
    wing_center[axis_index] = sign * axial_wing
    tangents = [idx for idx in range(3) if idx != axis_index]

    wing_a_size = [0.0, 0.0, 0.0]
    wing_a_size[axis_index] = CENTER_WING_DEPTH
    wing_a_size[tangents[0]] = CENTER_WING_LENGTH
    wing_a_size[tangents[1]] = CENTER_WING_WIDTH

    wing_b_size = [0.0, 0.0, 0.0]
    wing_b_size[axis_index] = CENTER_WING_DEPTH
    wing_b_size[tangents[0]] = CENTER_WING_WIDTH
    wing_b_size[tangents[1]] = CENTER_WING_LENGTH

    outer = _merge_geometries(
        [
            _box_geom(
                _size_for_axis(axis_index, CENTER_CAP_DEPTH, CENTER_CAP_SIZE, CENTER_CAP_SIZE),
                tuple(cap_center),
            ),
            _box_geom(
                _size_for_axis(axis_index, CENTER_NECK_DEPTH, CENTER_NECK_SIZE, CENTER_NECK_SIZE),
                tuple(neck_center),
            ),
            _box_geom(tuple(wing_a_size), tuple(wing_center)),
            _box_geom(tuple(wing_b_size), tuple(wing_center)),
        ]
    )

    cavity_center = [0.0, 0.0, 0.0]
    cavity_center[axis_index] = sign * (
        CUBE_HALF - CENTER_CAP_DEPTH - (CENTER_CAVITY_DEPTH * 0.5) - 0.0011
    )

    outer = boolean_difference(
        outer,
        _cyl_geom(
            CENTER_CAVITY_RADIUS,
            CENTER_CAVITY_DEPTH,
            tuple(cavity_center),
            axis_index=axis_index,
        ),
    )
    outer = boolean_difference(
        outer,
        SphereGeometry(radius=CENTER_SOCKET_RADIUS, width_segments=48, height_segments=30),
    )
    return outer


def _edge_foot_mesh(coords: tuple[int, int, int]) -> MeshGeometry:
    limits: dict[str, float | None] = {
        "min_x": None,
        "max_x": None,
        "min_y": None,
        "max_y": None,
        "min_z": None,
        "max_z": None,
    }
    for axis_name, value in zip(("x", "y", "z"), coords):
        if value == 0:
            limits[f"min_{axis_name}"] = -EDGE_BAND
            limits[f"max_{axis_name}"] = EDGE_BAND
        elif value > 0:
            limits[f"min_{axis_name}"] = EDGE_CUT
        else:
            limits[f"max_{axis_name}"] = -EDGE_CUT
    return _sphere_sector(EDGE_FOOT_RADIUS, **limits)


def _corner_foot_mesh(coords: tuple[int, int, int]) -> MeshGeometry:
    limits: dict[str, float | None] = {
        "min_x": None,
        "max_x": None,
        "min_y": None,
        "max_y": None,
        "min_z": None,
        "max_z": None,
    }
    for axis_name, value in zip(("x", "y", "z"), coords):
        if value > 0:
            limits[f"min_{axis_name}"] = CORNER_CUT
        else:
            limits[f"max_{axis_name}"] = -CORNER_CUT
    return _sphere_sector(CORNER_FOOT_RADIUS, **limits)


def _spring_mesh(face: str) -> MeshGeometry:
    coords = FACE_TO_COORDS[face]
    axis_index = _axis_index(coords)
    sign = coords[axis_index]
    tangents = [idx for idx in range(3) if idx != axis_index]
    coil_radius = 0.00225
    wire_radius = 0.00038
    start_axial = 0.0121
    end_axial = 0.0182
    turns = 4.5
    samples = 54
    points: list[tuple[float, float, float]] = []
    for index in range(samples + 1):
        t = index / samples
        angle = turns * 2.0 * math.pi * t
        axial = start_axial + (end_axial - start_axial) * t
        local_u = coil_radius * math.cos(angle)
        local_v = coil_radius * math.sin(angle)
        world = [0.0, 0.0, 0.0]
        world[axis_index] = sign * axial
        world[tangents[0]] = local_u
        world[tangents[1]] = local_v
        points.append(tuple(world))
    return wire_from_points(
        points,
        radius=wire_radius,
        radial_segments=12,
        closed_path=False,
        cap_ends=True,
        corner_mode="miter",
    )


def _add_segment(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_face_tile(part, face: str, *, slot_coords: tuple[int, int, int], material) -> None:
    face_coords = FACE_TO_COORDS[face]
    axis_index = _axis_index(face_coords)
    sign = face_coords[axis_index]
    center = [coord * MODULE for coord in slot_coords]
    center[axis_index] = sign * (CUBE_HALF - TILE_DEPTH * 0.5)
    size = [TILE_SPAN, TILE_SPAN, TILE_SPAN]
    size[axis_index] = TILE_DEPTH
    part.visual(
        Box(tuple(size)),
        origin=Origin(xyz=tuple(center)),
        material=material,
        name=f"tile_{face}",
    )


def _add_center_piece(model, face: str, *, body_material, screw_material, spring_material, tile_material) -> None:
    part_name = f"center_{face}"
    part = model.part(part_name)
    face_coords = FACE_TO_COORDS[face]
    axis_index = _axis_index(face_coords)
    sign = face_coords[axis_index]
    rpy = _rpy_for_axis(axis_index)
    tangents = [idx for idx in range(3) if idx != axis_index]

    axial_cap = CUBE_HALF - CENTER_CAP_DEPTH * 0.5
    axial_neck = CUBE_HALF - CENTER_CAP_DEPTH - CENTER_NECK_DEPTH * 0.5 + 0.0001
    axial_wing = CUBE_HALF - CENTER_CAP_DEPTH - CENTER_NECK_DEPTH - CENTER_WING_DEPTH * 0.5 + 0.0001

    cap_center = [0.0, 0.0, 0.0]
    cap_center[axis_index] = sign * axial_cap
    part.visual(
        Box(_size_for_axis(axis_index, CENTER_CAP_DEPTH, CENTER_CAP_SIZE, CENTER_CAP_SIZE)),
        origin=Origin(xyz=tuple(cap_center)),
        material=body_material,
        name="body_cap",
    )

    neck_center = [0.0, 0.0, 0.0]
    neck_center[axis_index] = sign * axial_neck
    part.visual(
        Box(_size_for_axis(axis_index, CENTER_NECK_DEPTH, CENTER_NECK_SIZE, CENTER_NECK_SIZE)),
        origin=Origin(xyz=tuple(neck_center)),
        material=body_material,
        name="body_neck",
    )

    wing_center = [0.0, 0.0, 0.0]
    wing_center[axis_index] = sign * axial_wing
    wing_a_size = [CENTER_WING_WIDTH, CENTER_WING_WIDTH, CENTER_WING_WIDTH]
    wing_a_size[axis_index] = CENTER_WING_DEPTH
    wing_a_size[tangents[0]] = CENTER_WING_LENGTH
    wing_a_size[tangents[1]] = CENTER_WING_WIDTH
    wing_b_size = [CENTER_WING_WIDTH, CENTER_WING_WIDTH, CENTER_WING_WIDTH]
    wing_b_size[axis_index] = CENTER_WING_DEPTH
    wing_b_size[tangents[0]] = CENTER_WING_WIDTH
    wing_b_size[tangents[1]] = CENTER_WING_LENGTH
    part.visual(
        Box(tuple(wing_a_size)),
        origin=Origin(xyz=tuple(wing_center)),
        material=body_material,
        name="body_wing_a",
    )
    part.visual(
        Box(tuple(wing_b_size)),
        origin=Origin(xyz=tuple(wing_center)),
        material=body_material,
        name="body_wing_b",
    )

    cavity_center = [0.0, 0.0, 0.0]
    cavity_center[axis_index] = sign * (
        CUBE_HALF - CENTER_CAP_DEPTH - (CENTER_CAVITY_DEPTH * 0.5) - 0.0011
    )
    for tangent_index, tangent_axis in enumerate(tangents):
        for tangent_sign in (-1.0, 1.0):
            rim_center = list(cavity_center)
            rim_center[tangent_axis] = tangent_sign * (CENTER_CAVITY_RADIUS + 0.0010)
            rim_size = [0.0022, 0.0022, 0.0022]
            rim_size[axis_index] = CENTER_CAVITY_DEPTH * 0.72
            rim_size[tangent_axis] = 0.0016
            rim_size[tangents[1 - tangent_index]] = 0.0030
            part.visual(
                Box(tuple(rim_size)),
                origin=Origin(xyz=tuple(rim_center)),
                material=body_material,
                name=f"cavity_rim_{tangent_index}_{'p' if tangent_sign > 0 else 'n'}",
            )

    _add_face_tile(part, face, slot_coords=(0, 0, 0), material=tile_material)

    spring = mesh_from_geometry(_spring_mesh(face), f"{part_name}_spring")
    part.visual(spring, material=spring_material, name="spring")

    screw_shaft_center = [0.0, 0.0, 0.0]
    screw_shaft_center[axis_index] = sign * 0.0174
    part.visual(
        Cylinder(radius=0.00125, length=0.0064),
        origin=Origin(xyz=tuple(screw_shaft_center), rpy=rpy),
        material=screw_material,
        name="screw_shaft",
    )

    screw_head_center = [0.0, 0.0, 0.0]
    screw_head_center[axis_index] = sign * 0.0209
    part.visual(
        Cylinder(radius=0.00285, length=0.0020),
        origin=Origin(xyz=tuple(screw_head_center), rpy=rpy),
        material=screw_material,
        name="screw_head",
    )

    washer_center = [0.0, 0.0, 0.0]
    washer_center[axis_index] = sign * 0.0193
    part.visual(
        Cylinder(radius=0.0031, length=0.0010),
        origin=Origin(xyz=tuple(washer_center), rpy=rpy),
        material=screw_material,
        name="screw_washer",
    )

    inertial_center = [0.0, 0.0, 0.0]
    inertial_center[axis_index] = sign * 0.020
    part.inertial = Inertial.from_geometry(
        Box(_size_for_axis(axis_index, 0.018, 0.018, 0.018)),
        mass=0.004,
        origin=Origin(xyz=tuple(inertial_center)),
    )

    model.articulation(
        f"core_to_{part_name}",
        ArticulationType.REVOLUTE,
        parent="core_spider",
        child=part,
        origin=Origin(),
        axis=tuple(face_coords),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=8.0,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )


def _add_edge_piece(model, edge_name: str, *, body_material, tile_materials: dict[str, object]) -> None:
    coords = EDGE_TO_COORDS[edge_name]
    part_name = f"edge_{edge_name}"
    part = model.part(part_name)
    body_center = _coord_point(coords, nonzero_abs=MODULE)
    part.visual(
        Box((EDGE_BODY_SIZE, EDGE_BODY_SIZE, EDGE_BODY_SIZE)),
        origin=Origin(xyz=body_center),
        material=body_material,
        name="body_shell",
    )

    for face in _nonzero_faces(coords):
        _add_face_tile(part, face, slot_coords=coords, material=tile_materials[face])

    stem_start = _coord_point(coords, nonzero_abs=0.0142)
    stem_end = _coord_point(coords, nonzero_abs=0.0101)
    _add_segment(part, stem_start, stem_end, radius=0.00225, material=body_material, name="anchor_stem")

    crossbar_center = _coord_point(coords, nonzero_abs=0.0094)
    missing_axis = [idx for idx, value in enumerate(coords) if value == 0][0]
    crossbar_size = [0.0038, 0.0038, 0.0038]
    crossbar_size[missing_axis] = 0.0096
    part.visual(
        Box(tuple(crossbar_size)),
        origin=Origin(xyz=crossbar_center),
        material=body_material,
        name="anchor_bar",
    )

    foot_center = _coord_point(coords, nonzero_abs=0.0086)
    part.visual(
        Sphere(radius=0.00325),
        origin=Origin(xyz=foot_center),
        material=body_material,
        name="foot",
    )

    for axis_index, value in enumerate(coords):
        if value == 0:
            flange_center = list(foot_center)
            flange_size = [0.0024, 0.0024, 0.0024]
            flange_size[axis_index] = 0.0088
            part.visual(
                Box(tuple(flange_size)),
                origin=Origin(xyz=tuple(flange_center)),
                material=body_material,
                name=f"flange_{axis_index}",
            )
        else:
            lobe_center = [0.0, 0.0, 0.0]
            lobe_center[axis_index] = value * 0.0065
            other_axes = [idx for idx in range(3) if idx != axis_index]
            for other_axis in other_axes:
                if coords[other_axis] != 0:
                    lobe_center[other_axis] = coords[other_axis] * 0.0052
            part.visual(
                Sphere(radius=0.00225),
                origin=Origin(xyz=tuple(lobe_center)),
                material=body_material,
                name=f"lobe_{axis_index}",
            )

    part.inertial = Inertial.from_geometry(
        Box((0.019, 0.019, 0.019)),
        mass=0.0035,
        origin=Origin(xyz=body_center),
    )

    model.articulation(
        f"core_to_{part_name}",
        ArticulationType.FIXED,
        parent="core_spider",
        child=part,
        origin=Origin(),
    )


def _add_corner_piece(model, corner_name: str, *, body_material, tile_materials: dict[str, object]) -> None:
    coords = CORNER_TO_COORDS[corner_name]
    part_name = f"corner_{corner_name}"
    part = model.part(part_name)
    body_center = _coord_point(coords, nonzero_abs=MODULE)
    part.visual(
        Box((CORNER_BODY_SIZE, CORNER_BODY_SIZE, CORNER_BODY_SIZE)),
        origin=Origin(xyz=body_center),
        material=body_material,
        name="body_shell",
    )

    for face in _nonzero_faces(coords):
        _add_face_tile(part, face, slot_coords=coords, material=tile_materials[face])

    hub_center = _coord_point(coords, nonzero_abs=0.0098)
    part.visual(
        Box((0.0046, 0.0046, 0.0046)),
        origin=Origin(xyz=hub_center),
        material=body_material,
        name="hub",
    )

    for axis_index, sign in enumerate(coords):
        leg_center = list(hub_center)
        leg_center[axis_index] = sign * 0.0144
        leg_size = [0.0034, 0.0034, 0.0034]
        leg_size[axis_index] = 0.0108
        part.visual(
            Box(tuple(leg_size)),
            origin=Origin(xyz=tuple(leg_center)),
            material=body_material,
            name=f"leg_{axis_index}",
        )

    part.visual(
        Sphere(radius=0.00315),
        origin=Origin(xyz=_coord_point(coords, nonzero_abs=0.0084)),
        material=body_material,
        name="foot",
    )
    for axis_index, sign in enumerate(coords):
        toe_center = [0.0, 0.0, 0.0]
        toe_center[axis_index] = sign * 0.0088
        for other_axis, other_sign in enumerate(coords):
            if other_axis != axis_index:
                toe_center[other_axis] = other_sign * 0.0078
        part.visual(
            Sphere(radius=0.0021),
            origin=Origin(xyz=tuple(toe_center)),
            material=body_material,
            name=f"toe_{axis_index}",
        )

    part.inertial = Inertial.from_geometry(
        Box((0.019, 0.019, 0.019)),
        mass=0.0038,
        origin=Origin(xyz=body_center),
    )

    model.articulation(
        f"core_to_{part_name}",
        ArticulationType.FIXED,
        parent="core_spider",
        child=part,
        origin=Origin(),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_3x3_twisty_puzzle")

    core_gray = model.material("core_gray", rgba=(0.78, 0.79, 0.80, 1.0))
    body_black = model.material("body_black", rgba=(0.08, 0.08, 0.09, 1.0))
    screw_steel = model.material("screw_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    colors = {
        "u": model.material("white_tile", rgba=(0.95, 0.96, 0.97, 1.0)),
        "d": model.material("yellow_tile", rgba=(0.94, 0.78, 0.14, 1.0)),
        "f": model.material("green_tile", rgba=(0.10, 0.56, 0.28, 1.0)),
        "b": model.material("blue_tile", rgba=(0.09, 0.29, 0.72, 1.0)),
        "r": model.material("red_tile", rgba=(0.73, 0.12, 0.11, 1.0)),
        "l": model.material("orange_tile", rgba=(0.92, 0.43, 0.07, 1.0)),
    }

    core = model.part("core_spider")
    core.visual(
        Box((0.0072, 0.0072, 0.0072)),
        origin=Origin(),
        material=core_gray,
        name="hub",
    )

    for axis_index in range(3):
        core.visual(
            Cylinder(radius=CORE_ARM_RADIUS, length=CORE_ARM_LENGTH),
            origin=Origin(rpy=_rpy_for_axis(axis_index)),
            material=core_gray,
            name=f"arm_{axis_index}",
        )

    for face, coords in FACE_TO_COORDS.items():
        axis_index = _axis_index(coords)
        sign = coords[axis_index]
        shaft_center = [0.0, 0.0, 0.0]
        shaft_center[axis_index] = sign * 0.0140
        collar_center = [0.0, 0.0, 0.0]
        collar_center[axis_index] = sign * 0.0109
        pad_center = [0.0, 0.0, 0.0]
        pad_center[axis_index] = sign * 0.0177
        rpy = _rpy_for_axis(axis_index)
        core.visual(
            Cylinder(radius=CORE_SHAFT_RADIUS, length=CORE_SHAFT_LENGTH),
            origin=Origin(xyz=tuple(shaft_center), rpy=rpy),
            material=core_gray,
            name=f"shaft_{face}",
        )
        core.visual(
            Cylinder(radius=CORE_COLLAR_RADIUS, length=CORE_COLLAR_THICKNESS),
            origin=Origin(xyz=tuple(collar_center), rpy=rpy),
            material=core_gray,
            name=f"collar_{face}",
        )
        core.visual(
            Cylinder(radius=0.0028, length=0.0016),
            origin=Origin(xyz=tuple(pad_center), rpy=rpy),
            material=core_gray,
            name=f"stop_{face}",
        )

    core.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.040)),
        mass=0.020,
        origin=Origin(),
    )

    for face in ("u", "d", "f", "b", "r", "l"):
        _add_center_piece(
            model,
            face,
            body_material=body_black,
            screw_material=screw_steel,
            spring_material=spring_steel,
            tile_material=colors[face],
        )

    for edge_name in (
        "ur",
        "uf",
        "ul",
        "ub",
        "dr",
        "df",
        "dl",
        "db",
        "fr",
        "fl",
        "br",
        "bl",
    ):
        _add_edge_piece(model, edge_name, body_material=body_black, tile_materials=colors)

    for corner_name in ("urf", "ufl", "ulb", "ubr", "dfr", "dlf", "dbl", "drb"):
        _add_corner_piece(model, corner_name, body_material=body_black, tile_materials=colors)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    core = object_model.get_part("core_spider")
    center_u = object_model.get_part("center_u")
    center_f = object_model.get_part("center_f")
    center_r = object_model.get_part("center_r")
    edge_ur = object_model.get_part("edge_ur")
    edge_uf = object_model.get_part("edge_uf")
    corner_urf = object_model.get_part("corner_urf")

    joint_u = object_model.get_articulation("core_to_center_u")
    joint_f = object_model.get_articulation("core_to_center_f")
    joint_r = object_model.get_articulation("core_to_center_r")

    expected_part_names = (
        "core_spider",
        "center_u",
        "center_d",
        "center_f",
        "center_b",
        "center_r",
        "center_l",
        "edge_ur",
        "edge_uf",
        "edge_ul",
        "edge_ub",
        "edge_dr",
        "edge_df",
        "edge_dl",
        "edge_db",
        "edge_fr",
        "edge_fl",
        "edge_br",
        "edge_bl",
        "corner_urf",
        "corner_ufl",
        "corner_ulb",
        "corner_ubr",
        "corner_dfr",
        "corner_dlf",
        "corner_dbl",
        "corner_drb",
    )
    for name in expected_part_names:
        ctx.check(f"part_present_{name}", object_model.get_part(name) is not None, f"Missing part {name}")

    ctx.check("u_axis_correct", tuple(joint_u.axis) == (0, 0, 1), f"Unexpected U axis {joint_u.axis}")
    ctx.check("f_axis_correct", tuple(joint_f.axis) == (0, 1, 0), f"Unexpected F axis {joint_f.axis}")
    ctx.check("r_axis_correct", tuple(joint_r.axis) == (1, 0, 0), f"Unexpected R axis {joint_r.axis}")

    limits_u = joint_u.motion_limits
    limits_f = joint_f.motion_limits
    limits_r = joint_r.motion_limits
    ctx.check(
        "quarter_turn_limits_u",
        limits_u is not None and abs((limits_u.upper or 0.0) - math.pi / 2.0) < 1e-6 and abs((limits_u.lower or 0.0) + math.pi / 2.0) < 1e-6,
        "U center should be limited to quarter-turn motion about its face axis",
    )
    ctx.check(
        "quarter_turn_limits_f",
        limits_f is not None and abs((limits_f.upper or 0.0) - math.pi / 2.0) < 1e-6 and abs((limits_f.lower or 0.0) + math.pi / 2.0) < 1e-6,
        "F center should be limited to quarter-turn motion about its face axis",
    )
    ctx.check(
        "quarter_turn_limits_r",
        limits_r is not None and abs((limits_r.upper or 0.0) - math.pi / 2.0) < 1e-6 and abs((limits_r.lower or 0.0) + math.pi / 2.0) < 1e-6,
        "R center should be limited to quarter-turn motion about its face axis",
    )

    ctx.expect_gap(
        edge_ur,
        center_u,
        axis="x",
        positive_elem="tile_u",
        negative_elem="tile_u",
        min_gap=0.0005,
        max_gap=0.0015,
    )
    ctx.expect_gap(
        corner_urf,
        edge_ur,
        axis="y",
        positive_elem="tile_r",
        negative_elem="tile_r",
        min_gap=0.0005,
        max_gap=0.0015,
    )
    ctx.expect_gap(
        corner_urf,
        edge_uf,
        axis="x",
        positive_elem="tile_f",
        negative_elem="tile_f",
        min_gap=0.0005,
        max_gap=0.0015,
    )
    ctx.expect_overlap(center_u, edge_ur, axes="y", elem_a="tile_u", elem_b="tile_u", min_overlap=0.014)
    ctx.expect_overlap(edge_ur, corner_urf, axes="z", elem_a="tile_r", elem_b="tile_r", min_overlap=0.014)
    ctx.expect_overlap(edge_uf, corner_urf, axes="z", elem_a="tile_f", elem_b="tile_f", min_overlap=0.014)
    ctx.expect_overlap(center_u, core, axes="xy", min_overlap=0.004)

    with ctx.pose({joint_u: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="u_quarter_turn_clearance")
    with ctx.pose({joint_f: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="f_quarter_turn_clearance")
    with ctx.pose({joint_r: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="r_quarter_turn_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
