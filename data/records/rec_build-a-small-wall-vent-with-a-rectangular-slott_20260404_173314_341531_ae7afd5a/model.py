from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    Box,
    Cylinder,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    mesh_from_geometry,
)


WIDTH = 0.18
HEIGHT = 0.10
FACE_THICKNESS = 0.004
DUCT_DEPTH = 0.026
SCREW_RADIUS = 0.002
SCREW_LENGTH = 0.002
SCREW_OFFSET_X = WIDTH / 2.0 - 0.006
SCREW_OFFSET_Y = HEIGHT / 2.0 - 0.006
TOTAL_DEPTH = 0.032


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_vent")

    plastic_white = model.material("plastic_white", color=(0.9, 0.9, 0.9))
    metal_silver = model.material("metal_silver", color=(0.7, 0.7, 0.7))

    vent_body = model.part("vent_body")
    vent_mesh = mesh_from_geometry(
        VentGrilleGeometry(
            (WIDTH, HEIGHT),
            frame=0.012,
            face_thickness=FACE_THICKNESS,
            duct_depth=DUCT_DEPTH,
            duct_wall=0.003,
            slat_pitch=0.018,
            slat_width=0.009,
            slat_angle_deg=35.0,
            corner_radius=0.006,
        ),
        "vent_shell_mesh",
    )
    vent_body.visual(
        vent_mesh,
        origin=Origin(),
        material=plastic_white,
        name="vent_shell",
    )

    for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        vent_body.visual(
            Cylinder(radius=SCREW_RADIUS, length=SCREW_LENGTH),
            origin=Origin(
                xyz=(sx * SCREW_OFFSET_X, sy * SCREW_OFFSET_Y, 0.004),
            ),
            material=metal_silver,
            name=f"screw_{i}",
        )

    vent_body.inertial = Inertial.from_geometry(
        Box((WIDTH, HEIGHT, TOTAL_DEPTH)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
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
        ctx.check("total depth matches", abs(dz - TOTAL_DEPTH) < 0.004, details=f"depth={dz}")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
