from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    Box,
    Cylinder,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WIDTH = 0.18
HEIGHT = 0.10
FACE_THICKNESS = 0.004
FRAME = 0.012
OPENING_WIDTH = WIDTH - 2.0 * FRAME
OPENING_HEIGHT = HEIGHT - 2.0 * FRAME
DUCT_DEPTH = 0.026
DUCT_WALL = 0.003
SLAT_PITCH = 0.018
SLAT_HEIGHT = 0.009
SLAT_THICKNESS = 0.0014
SLAT_CHORD = OPENING_WIDTH + 0.004
SLAT_ANGLE_DEG = -35.0
FACE_ORIGIN_Z = 0.002
SCREW_RADIUS = 0.002
SCREW_LENGTH = 0.002
SCREW_OFFSET_X = WIDTH / 2.0 - 0.006
SCREW_OFFSET_Y = HEIGHT / 2.0 - 0.006


def _build_vent_shell_shape() -> cq.Workplane:
    front_ring = cq.Workplane("XY").box(WIDTH, HEIGHT, FACE_THICKNESS)
    front_ring = front_ring.cut(
        cq.Workplane("XY").box(
            OPENING_WIDTH,
            OPENING_HEIGHT,
            FACE_THICKNESS + 0.002,
        )
    )

    duct_outer = cq.Workplane("XY").box(OPENING_WIDTH, OPENING_HEIGHT, DUCT_DEPTH)
    duct_inner = cq.Workplane("XY").box(
        OPENING_WIDTH - 2.0 * DUCT_WALL,
        OPENING_HEIGHT - 2.0 * DUCT_WALL,
        DUCT_DEPTH + 0.004,
    )
    duct_shell = duct_outer.cut(duct_inner).translate((0.0, 0.0, -FACE_THICKNESS / 2.0 - DUCT_DEPTH / 2.0 + 0.001))

    shape = front_ring.union(duct_shell)

    y = -OPENING_HEIGHT / 2.0 + SLAT_PITCH / 2.0
    limit = OPENING_HEIGHT / 2.0 - SLAT_PITCH / 2.0
    while y <= limit + 1e-9:
        slat = cq.Workplane("XY").box(SLAT_CHORD, SLAT_HEIGHT, SLAT_THICKNESS)
        slat = slat.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), SLAT_ANGLE_DEG)
        slat = slat.translate((0.0, y, -0.001))
        shape = shape.union(slat)
        y += SLAT_PITCH

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_vent")

    plastic_white = model.material("plastic_white", color=(0.9, 0.9, 0.9))
    metal_silver = model.material("metal_silver", color=(0.7, 0.7, 0.7))

    vent_body = model.part("vent_body")
    vent_shell = _build_vent_shell_shape()
    vent_mesh = mesh_from_cadquery(vent_shell, "vent_shell_mesh")

    vent_body.visual(
        vent_mesh,
        origin=Origin(xyz=(0.0, 0.0, FACE_ORIGIN_Z)),
        material=plastic_white,
        name="vent_shell",
    )

    for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        vent_body.visual(
            Cylinder(radius=SCREW_RADIUS, length=SCREW_LENGTH),
            origin=Origin(
                xyz=(
                    sx * SCREW_OFFSET_X,
                    sy * SCREW_OFFSET_Y,
                    FACE_ORIGIN_Z + FACE_THICKNESS + SCREW_LENGTH / 2.0,
                )
            ),
            material=metal_silver,
            name=f"screw_{i}",
        )

    total_depth = FACE_THICKNESS + DUCT_DEPTH + FACE_ORIGIN_Z - 0.001
    vent_body.inertial = Inertial.from_geometry(
        Box((WIDTH, HEIGHT, total_depth)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, (FACE_ORIGIN_Z + FACE_THICKNESS - (DUCT_DEPTH - 0.001)) / 2.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    vent_body = object_model.get_part("vent_body")

    aabb = ctx.part_world_aabb(vent_body)
    if aabb:
        min_pt, max_pt = aabb
        dx = max_pt[0] - min_pt[0]
        dy = max_pt[1] - min_pt[1]
        dz = max_pt[2] - min_pt[2]

        ctx.check("width matches", abs(dx - WIDTH) < 0.001, details=f"width={dx}")
        ctx.check("height matches", abs(dy - HEIGHT) < 0.001, details=f"height={dy}")
        ctx.check("total depth matches", abs(dz - 0.033) < 0.004, details=f"depth={dz}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
