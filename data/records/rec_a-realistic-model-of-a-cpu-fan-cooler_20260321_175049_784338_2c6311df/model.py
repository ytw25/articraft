from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

FIN_W = 0.116
FIN_D = 0.046
FIN_STACK_H = 0.100
FIN_COUNT = 22
FIN_THICK = 0.0006
FIN_END_THICK = 0.0012

PIPE_R = 0.00325
PIPE_HOLE_R = 0.0037
PIPE_POINTS = [(-0.018, 0.0), (-0.006, 0.0), (0.006, 0.0), (0.018, 0.0)]

BASE_W = 0.042
BASE_D = 0.014
BASE_T = 0.006
BASE_Z = -0.059
CONTACT_T = 0.002
CONTACT_Z = BASE_Z - (BASE_T / 2.0) - (CONTACT_T / 2.0)
PIPE_BOTTOM_Z = BASE_Z + (BASE_T / 2.0)
PIPE_TOP_Z = 0.064
PIPE_LEN = PIPE_TOP_Z - PIPE_BOTTOM_Z
PIPE_CAP_LEN = 0.004

MOUNT_BRIDGE_W = 0.064
MOUNT_BRIDGE_D = 0.008
MOUNT_BRIDGE_T = 0.002
MOUNT_BRIDGE_Z = BASE_Z + (BASE_T / 2.0) + (MOUNT_BRIDGE_T / 2.0)

FAN_SIZE = 0.120
FRAME_D = 0.026
FAN_WINDOW = 0.104
FAN_CENTER_Y = -((FIN_D + FRAME_D) / 2.0)
FRAME_HOLE_OFFSET = 0.049

MOTOR_CORE_R = 0.0105
MOTOR_CORE_LEN = 0.010
MOTOR_CORE_Y = 0.005
STRUT_Y = 0.006

HUB_R = 0.020
HUB_LEN = 0.016
BLADE_COUNT = 7
BLADE_CENTER_R = 0.032
BLADE_LENGTH = 0.030
BLADE_WIDTH = 0.014
BLADE_THICK = 0.0026
BLADE_PITCH_DEG = 110.0


def _mesh(shape: object, filename: str, *, tolerance: float, angular_tolerance: float):
    return mesh_from_cadquery(
        shape,
        filename,
        assets=ASSETS,
        tolerance=tolerance,
        angular_tolerance=angular_tolerance,
    )


def _build_fin_plate(thickness: float) -> cq.Workplane:
    plate = cq.Workplane("XY").box(FIN_W, FIN_D, thickness, centered=(True, True, False))
    hole_cutters = (
        cq.Workplane("XY")
        .pushPoints(PIPE_POINTS)
        .circle(PIPE_HOLE_R)
        .extrude(thickness + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    return plate.cut(hole_cutters)


def _build_fin_stack_shape():
    fin_shapes = []
    pitch = FIN_STACK_H / (FIN_COUNT - 1)
    for idx in range(FIN_COUNT):
        thickness = FIN_END_THICK if idx in (0, FIN_COUNT - 1) else FIN_THICK
        center_z = -FIN_STACK_H / 2.0 + idx * pitch
        fin = _build_fin_plate(thickness).translate((0.0, 0.0, center_z - (thickness / 2.0)))
        fin_shapes.append(fin.val())
    return cq.Compound.makeCompound(fin_shapes)


def _build_heat_core_shape():
    base = cq.Workplane("XY").box(BASE_W, BASE_D, BASE_T).translate((0.0, 0.0, BASE_Z))
    contact = (
        cq.Workplane("XY")
        .box(BASE_W * 0.92, BASE_D * 0.86, CONTACT_T)
        .translate((0.0, 0.0, CONTACT_Z))
    )
    pipes = (
        cq.Workplane("XY")
        .pushPoints(PIPE_POINTS)
        .circle(PIPE_R)
        .extrude(PIPE_LEN)
        .translate((0.0, 0.0, PIPE_BOTTOM_Z))
    )
    caps = (
        cq.Workplane("XY")
        .pushPoints(PIPE_POINTS)
        .circle(PIPE_R + 0.0002)
        .extrude(PIPE_CAP_LEN)
        .translate((0.0, 0.0, PIPE_TOP_Z))
    )
    return cq.Compound.makeCompound([base.val(), contact.val(), *pipes.vals(), *caps.vals()])


def _build_fan_frame_shape():
    frame = (
        cq.Workplane("XY")
        .box(FAN_SIZE, FRAME_D, FAN_SIZE)
        .edges("|Y")
        .fillet(0.004)
    )
    inner_cut = cq.Workplane("XY").box(FAN_WINDOW, FRAME_D + 0.004, FAN_WINDOW)
    frame = frame.cut(inner_cut)

    corner_holes = (
        cq.Workplane("XZ")
        .pushPoints(
            [
                (-FRAME_HOLE_OFFSET, -FRAME_HOLE_OFFSET),
                (-FRAME_HOLE_OFFSET, FRAME_HOLE_OFFSET),
                (FRAME_HOLE_OFFSET, -FRAME_HOLE_OFFSET),
                (FRAME_HOLE_OFFSET, FRAME_HOLE_OFFSET),
            ]
        )
        .circle(0.0026)
        .extrude(FRAME_D + 0.004)
        .translate((0.0, -(FRAME_D + 0.004) / 2.0, 0.0))
    )
    frame = frame.cut(corner_holes)

    motor_core = (
        cq.Workplane("XZ")
        .circle(MOTOR_CORE_R)
        .extrude(MOTOR_CORE_LEN)
        .translate((0.0, MOTOR_CORE_Y - (MOTOR_CORE_LEN / 2.0), 0.0))
    )

    base_strut = (
        cq.Workplane("XY")
        .box(0.056, 0.0028, 0.004)
        .translate((0.020, STRUT_Y, 0.020))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 45.0)
    )
    for angle in (0.0, 90.0, 180.0, 270.0):
        frame = frame.union(base_strut.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle))

    frame = frame.union(motor_core)
    return frame.val()


def _build_fan_rotor_shape():
    hub = (
        cq.Workplane("XZ")
        .circle(HUB_R)
        .extrude(HUB_LEN)
        .translate((0.0, -(HUB_LEN / 2.0), 0.0))
    )

    blade = (
        cq.Workplane("XY")
        .center(BLADE_CENTER_R, 0.0)
        .slot2D(BLADE_LENGTH, BLADE_WIDTH, angle=18.0)
        .extrude(BLADE_THICK)
        .translate((0.0, 0.0, -(BLADE_THICK / 2.0)))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), BLADE_PITCH_DEG)
    )

    rotor = hub
    for idx in range(BLADE_COUNT):
        rotor = rotor.union(
            blade.rotate(
                (0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                idx * (360.0 / BLADE_COUNT),
            )
        )
    return rotor.val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cpu_fan_cooler", assets=ASSETS)

    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    copper = model.material("copper", rgba=(0.77, 0.47, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.57, 0.59, 0.62, 1.0))
    polymer = model.material("polymer", rgba=(0.09, 0.10, 0.11, 1.0))

    fin_stack = model.part("fin_stack")
    fin_stack.visual(
        _mesh(
            _build_fin_stack_shape(),
            "fin_stack.obj",
            tolerance=0.00035,
            angular_tolerance=0.06,
        ),
        material=aluminum,
    )
    fin_stack.inertial = Inertial.from_geometry(
        Box((FIN_W, FIN_D, FIN_STACK_H)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    heat_core = model.part("heat_core")
    heat_core.visual(
        _mesh(
            _build_heat_core_shape(),
            "heat_core.obj",
            tolerance=0.0006,
            angular_tolerance=0.08,
        ),
        material=copper,
    )
    heat_core.visual(
        Box((MOUNT_BRIDGE_W, MOUNT_BRIDGE_D, MOUNT_BRIDGE_T)),
        origin=Origin(xyz=(0.0, 0.0, MOUNT_BRIDGE_Z)),
        material=steel,
    )
    heat_core.inertial = Inertial.from_geometry(
        Box((0.070, 0.020, 0.130)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    fan_frame = model.part("fan_frame")
    fan_frame.visual(
        _mesh(
            _build_fan_frame_shape(),
            "fan_frame.obj",
            tolerance=0.0007,
            angular_tolerance=0.08,
        ),
        material=polymer,
    )
    fan_frame.inertial = Inertial.from_geometry(
        Box((FAN_SIZE, FRAME_D, FAN_SIZE)),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        _mesh(
            _build_fan_rotor_shape(),
            "fan_rotor.obj",
            tolerance=0.0007,
            angular_tolerance=0.08,
        ),
        material=polymer,
    )
    fan_rotor.inertial = Inertial.from_geometry(
        Box((0.098, HUB_LEN, 0.098)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "fin_stack_to_heat_core",
        ArticulationType.FIXED,
        parent="fin_stack",
        child="heat_core",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "fin_stack_to_fan_frame",
        ArticulationType.FIXED,
        parent="fin_stack",
        child="fan_frame",
        origin=Origin(xyz=(0.0, FAN_CENTER_Y, 0.0)),
    )
    model.articulation(
        "fan_frame_to_fan_rotor",
        ArticulationType.CONTINUOUS,
        parent="fan_frame",
        child="fan_rotor",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=120.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "fin_stack",
        "heat_core",
        reason="heatpipes intentionally pass through the heatsink plates",
    )
    ctx.allow_overlap(
        "fan_frame",
        "fan_rotor",
        reason="rotor shell nests around the fixed bearing boss",
    )
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("heat_core", "fin_stack", axes="xy", max_dist=0.002)
    ctx.expect_aabb_overlap("heat_core", "fin_stack", axes="xz", min_overlap=0.040)
    ctx.expect_aabb_overlap("heat_core", "fin_stack", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_contact("heat_core", "fin_stack")

    ctx.expect_origin_distance("fan_frame", "fin_stack", axes="xz", max_dist=0.002)
    ctx.expect_aabb_overlap("fan_frame", "fin_stack", axes="xz", min_overlap=0.090)
    ctx.expect_aabb_gap("fin_stack", "fan_frame", axis="y", max_gap=0.001, max_penetration=0.002)
    ctx.expect_aabb_contact("fan_frame", "fin_stack")

    for angle in (0.0, math.pi / 2.0, math.pi):
        with ctx.pose(fan_frame_to_fan_rotor=angle):
            ctx.expect_origin_distance("fan_rotor", "fan_frame", axes="xz", max_dist=0.001)
            ctx.expect_origin_distance("fan_rotor", "fan_frame", axes="y", max_dist=0.001)
            ctx.expect_aabb_overlap("fan_rotor", "fan_frame", axes="xz", min_overlap=0.080)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
