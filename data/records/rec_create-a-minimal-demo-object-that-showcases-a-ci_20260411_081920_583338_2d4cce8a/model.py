from __future__ import annotations

from sdk import (
    ArticulatedObject,
    BezelFace,
    BezelFlange,
    BezelGeometry,
    Box,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

OUTER_DIAMETER = 0.080


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="circular_lens_bezel_demo")
    finish = model.material("lens_bezel_silver", rgba=(0.78, 0.80, 0.83, 1.0))

    bezel = model.part("circular_lens_bezel")
    bezel.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.046, 0.046),
                (OUTER_DIAMETER, OUTER_DIAMETER),
                0.010,
                opening_shape="circle",
                outer_shape="circle",
                face=BezelFace(style="chamfered", chamfer=0.0012),
                flange=BezelFlange(width=0.004, thickness=0.002, offset=0.001),
            ),
            "circular_lens_bezel",
        ),
        material=finish,
        name="circular_lens_bezel",
    )
    bezel.inertial = Inertial.from_geometry(Box((OUTER_DIAMETER, OUTER_DIAMETER, 0.012)), mass=0.09)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bezel = object_model.get_part("circular_lens_bezel")
    ctx.check("circular_lens_bezel_part_present", bezel is not None, "Expected a circular_lens_bezel part.")
    if bezel is None:
        return ctx.report()

    ctx.check(
        "circular_lens_bezel_visual_present",
        bezel.get_visual("circular_lens_bezel") is not None,
        "Expected a mesh-backed circular_lens_bezel visual.",
    )
    aabb = ctx.part_world_aabb(bezel)
    ctx.check("circular_lens_bezel_aabb_present", aabb is not None, "Expected a world AABB for the bezel.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("circular_lens_bezel_diameter", 0.078 <= max(size[0], size[1]) <= 0.090, f"size={size!r}")
    ctx.check("circular_lens_bezel_depth", 0.010 <= size[2] <= 0.015, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
