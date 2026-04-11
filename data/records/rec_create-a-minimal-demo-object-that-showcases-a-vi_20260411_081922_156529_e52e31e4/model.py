from __future__ import annotations

from sdk import (
    ArticulatedObject,
    BezelGeometry,
    BezelVisor,
    Box,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="visor_bezel_demo")
    finish = model.material("visor_bezel_black", rgba=(0.09, 0.09, 0.10, 1.0))

    bezel = model.part("visor_bezel")
    bezel.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.078, 0.044),
                (0.112, 0.072),
                0.012,
                opening_corner_radius=0.004,
                outer_corner_radius=0.008,
                visor=BezelVisor(top_extension=0.014, side_extension=0.006, thickness=0.002),
            ),
            "visor_bezel",
        ),
        material=finish,
        name="visor_bezel",
    )
    bezel.inertial = Inertial.from_geometry(Box((0.124, 0.090, 0.014)), mass=0.12)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bezel = object_model.get_part("visor_bezel")
    ctx.check("visor_bezel_part_present", bezel is not None, "Expected a visor_bezel part.")
    if bezel is None:
        return ctx.report()

    ctx.check(
        "visor_bezel_visual_present",
        bezel.get_visual("visor_bezel") is not None,
        "Expected a mesh-backed visor_bezel visual.",
    )
    aabb = ctx.part_world_aabb(bezel)
    ctx.check("visor_bezel_aabb_present", aabb is not None, "Expected a world AABB for the bezel.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("visor_bezel_width", 0.110 <= size[0] <= 0.126, f"size={size!r}")
    ctx.check("visor_bezel_height", 0.082 <= size[1] <= 0.096, f"size={size!r}")
    ctx.check("visor_bezel_depth", 0.010 <= size[2] <= 0.014, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
