from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    SphereGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
    tube_from_spline_points,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

PLATE_THICKNESS = 0.008
PEDESTAL_HEIGHT = 0.050
HANDLE_OFFSET_Y = 0.130


def _write_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / filename)


def _build_deck_plate_mesh():
    profile = superellipse_profile(0.280, 0.340, exponent=3.4, segments=72)
    plate = ExtrudeGeometry.from_z0(profile, PLATE_THICKNESS, cap=True, closed=True)
    return _write_mesh(plate, "faucet_deck_plate.obj")


def _build_spout_body_mesh():
    column = LatheGeometry(
        [
            (0.0, 0.000),
            (0.040, 0.000),
            (0.043, 0.007),
            (0.036, 0.024),
            (0.031, 0.054),
            (0.028, 0.078),
            (0.024, 0.086),
            (0.0, 0.086),
        ],
        segments=48,
    )

    spout = tube_from_spline_points(
        [
            (0.000, 0.000, 0.082),
            (0.018, 0.000, 0.145),
            (0.098, 0.000, 0.238),
            (0.225, 0.000, 0.235),
            (0.312, 0.000, 0.158),
            (0.345, 0.000, 0.105),
        ],
        radius=0.017,
        samples_per_segment=18,
        radial_segments=22,
        cap_ends=True,
    )

    nozzle = (
        CylinderGeometry(radius=0.020, height=0.030, radial_segments=30)
        .rotate_y(math.pi / 2.0)
        .translate(0.347, 0.000, 0.106)
    )

    column.merge(spout)
    column.merge(nozzle)
    return _write_mesh(column, "faucet_spout_body.obj")


def _build_mount_mesh():
    pedestal = LatheGeometry(
        [
            (0.0, 0.000),
            (0.030, 0.000),
            (0.033, 0.005),
            (0.028, 0.018),
            (0.024, 0.035),
            (0.021, 0.046),
            (0.019, 0.050),
            (0.0, 0.050),
        ],
        segments=40,
    )
    return _write_mesh(pedestal, "faucet_handle_mount.obj")


def _build_cross_handle_mesh():
    handle = CylinderGeometry(radius=0.021, height=0.008, radial_segments=32).translate(
        0.000, 0.000, 0.004
    )
    handle.merge(
        CylinderGeometry(radius=0.016, height=0.022, radial_segments=32).translate(
            0.000, 0.000, 0.014
        )
    )
    handle.merge(
        CylinderGeometry(radius=0.0065, height=0.094, radial_segments=24)
        .rotate_y(math.pi / 2.0)
        .translate(0.000, 0.000, 0.021)
    )
    handle.merge(
        CylinderGeometry(radius=0.0065, height=0.094, radial_segments=24)
        .rotate_x(math.pi / 2.0)
        .translate(0.000, 0.000, 0.021)
    )

    for tip in (
        (0.047, 0.000, 0.021),
        (-0.047, 0.000, 0.021),
        (0.000, 0.047, 0.021),
        (0.000, -0.047, 0.021),
    ):
        handle.merge(SphereGeometry(radius=0.009, width_segments=18, height_segments=12).translate(*tip))

    return _write_mesh(handle, "faucet_cross_handle.obj")


def _aabb_axis_bounds(aabb, axis: str) -> tuple[float, float]:
    idx = {"x": 0, "y": 1, "z": 2}[axis]
    min_attr = getattr(aabb, f"min_{axis}", None)
    max_attr = getattr(aabb, f"max_{axis}", None)
    if min_attr is not None and max_attr is not None:
        return float(min_attr), float(max_attr)
    if hasattr(aabb, "minimum") and hasattr(aabb, "maximum"):
        return float(aabb.minimum[idx]), float(aabb.maximum[idx])
    if hasattr(aabb, "min") and hasattr(aabb, "max"):
        lower = aabb.min[idx] if hasattr(aabb.min, "__getitem__") else getattr(aabb.min, axis)
        upper = aabb.max[idx] if hasattr(aabb.max, "__getitem__") else getattr(aabb.max, axis)
        return float(lower), float(upper)
    if isinstance(aabb, (tuple, list)) and len(aabb) == 2:
        return float(aabb[0][idx]), float(aabb[1][idx])
    raise AssertionError(f"Unsupported AABB representation for axis {axis!r}: {aabb!r}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="elegant_chrome_faucet", assets=ASSETS)

    chrome = model.material("chrome", rgba=(0.80, 0.83, 0.86, 1.0))
    porcelain = model.material("porcelain", rgba=(0.97, 0.97, 0.95, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    hot_marker = model.material("hot_marker", rgba=(0.82, 0.18, 0.16, 1.0))
    cold_marker = model.material("cold_marker", rgba=(0.18, 0.42, 0.82, 1.0))

    deck_mesh = _build_deck_plate_mesh()
    spout_mesh = _build_spout_body_mesh()
    mount_mesh = _build_mount_mesh()
    handle_mesh = _build_cross_handle_mesh()

    deck = model.part("deck")
    deck.visual(deck_mesh, material=chrome)
    deck.inertial = Inertial.from_geometry(
        Box((0.280, 0.340, PLATE_THICKNESS)),
        mass=2.2,
        origin=Origin(xyz=(0.000, 0.000, PLATE_THICKNESS / 2.0)),
    )

    spout_body = model.part("spout_body")
    spout_body.visual(spout_mesh, material=chrome)
    spout_body.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.356, 0.000, 0.085)),
        material=black_plastic,
        name="aerator",
    )
    spout_body.inertial = Inertial.from_geometry(
        Box((0.410, 0.060, 0.260)),
        mass=1.3,
        origin=Origin(xyz=(0.155, 0.000, 0.130)),
    )

    left_mount = model.part("left_mount")
    left_mount.visual(mount_mesh, material=chrome)
    left_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=PEDESTAL_HEIGHT),
        mass=0.35,
        origin=Origin(xyz=(0.000, 0.000, PEDESTAL_HEIGHT / 2.0)),
    )

    right_mount = model.part("right_mount")
    right_mount.visual(mount_mesh, material=chrome)
    right_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=PEDESTAL_HEIGHT),
        mass=0.35,
        origin=Origin(xyz=(0.000, 0.000, PEDESTAL_HEIGHT / 2.0)),
    )

    left_handle = model.part("left_handle")
    left_handle.visual(handle_mesh, material=chrome)
    left_handle.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.027)),
        material=porcelain,
        name="left_handle_cap",
    )
    left_handle.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.032)),
        material=hot_marker,
        name="left_handle_marker",
    )
    left_handle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.038),
        mass=0.22,
        origin=Origin(xyz=(0.000, 0.000, 0.019)),
    )

    right_handle = model.part("right_handle")
    right_handle.visual(handle_mesh, material=chrome)
    right_handle.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.000, 0.000, 0.027)),
        material=porcelain,
        name="right_handle_cap",
    )
    right_handle.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.032)),
        material=cold_marker,
        name="right_handle_marker",
    )
    right_handle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.038),
        mass=0.22,
        origin=Origin(xyz=(0.000, 0.000, 0.019)),
    )

    model.articulation(
        "deck_to_spout",
        ArticulationType.FIXED,
        parent="deck",
        child="spout_body",
        origin=Origin(xyz=(0.000, 0.000, PLATE_THICKNESS)),
    )
    model.articulation(
        "deck_to_left_mount",
        ArticulationType.FIXED,
        parent="deck",
        child="left_mount",
        origin=Origin(xyz=(0.000, HANDLE_OFFSET_Y, PLATE_THICKNESS)),
    )
    model.articulation(
        "deck_to_right_mount",
        ArticulationType.FIXED,
        parent="deck",
        child="right_mount",
        origin=Origin(xyz=(0.000, -HANDLE_OFFSET_Y, PLATE_THICKNESS)),
    )
    model.articulation(
        "left_handle_spin",
        ArticulationType.CONTINUOUS,
        parent="left_mount",
        child="left_handle",
        origin=Origin(xyz=(0.000, 0.000, PEDESTAL_HEIGHT)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=1.8, velocity=4.0),
    )
    model.articulation(
        "right_handle_spin",
        ArticulationType.CONTINUOUS,
        parent="right_mount",
        child="right_handle",
        origin=Origin(xyz=(0.000, 0.000, PEDESTAL_HEIGHT)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=1.8, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("spout_body", "deck", axes="xy", min_overlap=0.060)
    ctx.expect_aabb_gap("spout_body", "deck", axis="z", max_gap=0.001, max_penetration=0.006)

    ctx.expect_aabb_overlap("left_mount", "deck", axes="xy", min_overlap=0.055)
    ctx.expect_aabb_gap("left_mount", "deck", axis="z", max_gap=0.001, max_penetration=0.001)
    ctx.expect_aabb_overlap("right_mount", "deck", axes="xy", min_overlap=0.055)
    ctx.expect_aabb_gap("right_mount", "deck", axis="z", max_gap=0.001, max_penetration=0.001)

    ctx.expect_aabb_overlap("left_handle", "left_mount", axes="xy", min_overlap=0.030)
    ctx.expect_aabb_gap("left_handle", "left_mount", axis="z", max_gap=0.001, max_penetration=0.002)
    ctx.expect_aabb_overlap("right_handle", "right_mount", axes="xy", min_overlap=0.030)
    ctx.expect_aabb_gap("right_handle", "right_mount", axis="z", max_gap=0.001, max_penetration=0.002)

    spout_aabb = ctx.part_world_aabb("spout_body", use="visual")
    span_x = _aabb_axis_bounds(spout_aabb, "x")[1] - _aabb_axis_bounds(spout_aabb, "x")[0]
    span_y = _aabb_axis_bounds(spout_aabb, "y")[1] - _aabb_axis_bounds(spout_aabb, "y")[0]
    span_z = _aabb_axis_bounds(spout_aabb, "z")[1] - _aabb_axis_bounds(spout_aabb, "z")[0]
    if span_x <= 0.380:
        raise AssertionError(f"Spout should read elongated in x; got span_x={span_x:.3f}")
    if span_z <= 0.220:
        raise AssertionError(f"Spout should arch upward before reaching out; got span_z={span_z:.3f}")
    if span_x <= span_y * 2.5:
        raise AssertionError(
            f"Spout body should be much longer than it is wide; got span_x={span_x:.3f}, span_y={span_y:.3f}"
        )

    left_mount_pos = ctx.part_world_position("left_mount")
    right_mount_pos = ctx.part_world_position("right_mount")
    left_handle_pos = ctx.part_world_position("left_handle")
    right_handle_pos = ctx.part_world_position("right_handle")

    if not (left_mount_pos[1] > 0.10 and right_mount_pos[1] < -0.10):
        raise AssertionError(
            f"Handle mounts should sit to opposite sides of the faucet body; got {left_mount_pos=} and {right_mount_pos=}"
        )
    if abs(left_handle_pos[1] - left_mount_pos[1]) > 1e-6 or abs(right_handle_pos[1] - right_mount_pos[1]) > 1e-6:
        raise AssertionError("Handles should sit directly above their corresponding side mounts.")
    if abs(left_mount_pos[1] + right_mount_pos[1]) > 0.010:
        raise AssertionError("Side mounts should frame the spout symmetrically.")

    left_handle_aabb = ctx.part_world_aabb("left_handle", use="visual")
    right_handle_aabb = ctx.part_world_aabb("right_handle", use="visual")
    left_handle_span_x = _aabb_axis_bounds(left_handle_aabb, "x")[1] - _aabb_axis_bounds(left_handle_aabb, "x")[0]
    left_handle_span_y = _aabb_axis_bounds(left_handle_aabb, "y")[1] - _aabb_axis_bounds(left_handle_aabb, "y")[0]
    right_handle_span_x = _aabb_axis_bounds(right_handle_aabb, "x")[1] - _aabb_axis_bounds(right_handle_aabb, "x")[0]
    right_handle_span_y = _aabb_axis_bounds(right_handle_aabb, "y")[1] - _aabb_axis_bounds(right_handle_aabb, "y")[0]

    if left_handle_span_x <= 0.085 or left_handle_span_y <= 0.085:
        raise AssertionError(
            f"Left control should read as a broad classic cross-handle, got spans x={left_handle_span_x:.3f}, y={left_handle_span_y:.3f}"
        )
    if right_handle_span_x <= 0.085 or right_handle_span_y <= 0.085:
        raise AssertionError(
            f"Right control should read as a broad classic cross-handle, got spans x={right_handle_span_x:.3f}, y={right_handle_span_y:.3f}"
        )
    if abs(left_handle_span_x - left_handle_span_y) > 0.018:
        raise AssertionError("Left handle should read as a balanced four-arm cross, not an elongated lever.")
    if abs(right_handle_span_x - right_handle_span_y) > 0.018:
        raise AssertionError("Right handle should read as a balanced four-arm cross, not an elongated lever.")

    with ctx.pose(left_handle_spin=math.pi / 2.0, right_handle_spin=-math.pi / 2.0):
        ctx.expect_aabb_overlap("left_handle", "left_mount", axes="xy", min_overlap=0.030)
        ctx.expect_aabb_gap("left_handle", "left_mount", axis="z", max_gap=0.001, max_penetration=0.002)
        ctx.expect_aabb_overlap("right_handle", "right_mount", axes="xy", min_overlap=0.030)
        ctx.expect_aabb_gap("right_handle", "right_mount", axis="z", max_gap=0.001, max_penetration=0.002)

        posed_left = ctx.part_world_position("left_handle")
        posed_right = ctx.part_world_position("right_handle")
        if any(abs(a - b) > 1e-6 for a, b in zip(posed_left, left_handle_pos)):
            raise AssertionError("The left cross-handle should rotate in place without translating.")
        if any(abs(a - b) > 1e-6 for a, b in zip(posed_right, right_handle_pos)):
            raise AssertionError("The right cross-handle should rotate in place without translating.")

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
