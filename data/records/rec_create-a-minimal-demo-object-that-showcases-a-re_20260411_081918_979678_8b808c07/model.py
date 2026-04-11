from __future__ import annotations

from sdk import (
    ArticulatedObject,
    BezelFace,
    BezelGeometry,
    BezelRecess,
    Box,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

OUTER_SIZE = (0.096, 0.074)
DEPTH = 0.013


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_instrument_bezel_demo")
    finish = model.material("instrument_bezel_gray", rgba=(0.55, 0.57, 0.60, 1.0))

    bezel = model.part("recessed_instrument_bezel")
    bezel.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.060, 0.040),
                OUTER_SIZE,
                DEPTH,
                opening_corner_radius=0.004,
                outer_corner_radius=0.008,
                face=BezelFace(style="rounded", front_lip=0.003, fillet=0.0014),
                recess=BezelRecess(depth=0.003, inset=0.006),
            ),
            "recessed_instrument_bezel",
        ),
        material=finish,
        name="recessed_instrument_bezel",
    )
    bezel.inertial = Inertial.from_geometry(Box((OUTER_SIZE[0], OUTER_SIZE[1], DEPTH)), mass=0.10)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bezel = object_model.get_part("recessed_instrument_bezel")
    ctx.check("recessed_instrument_bezel_part_present", bezel is not None, "Expected a recessed_instrument_bezel part.")
    if bezel is None:
        return ctx.report()

    ctx.check(
        "recessed_instrument_bezel_visual_present",
        bezel.get_visual("recessed_instrument_bezel") is not None,
        "Expected a mesh-backed recessed_instrument_bezel visual.",
    )
    aabb = ctx.part_world_aabb(bezel)
    ctx.check("recessed_instrument_bezel_aabb_present", aabb is not None, "Expected a world AABB for the bezel.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("recessed_instrument_bezel_width", 0.092 <= size[0] <= 0.100, f"size={size!r}")
    ctx.check("recessed_instrument_bezel_height", 0.070 <= size[1] <= 0.078, f"size={size!r}")
    ctx.check("recessed_instrument_bezel_depth", 0.011 <= size[2] <= 0.017, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
