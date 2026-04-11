from __future__ import annotations

from sdk import ArticulatedObject, BezelGeometry, Box, Inertial, TestContext, TestReport, mesh_from_geometry

OUTER_SIZE = (0.110, 0.080)
DEPTH = 0.012


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_bezel_demo")
    finish = model.material("display_bezel_black", rgba=(0.10, 0.10, 0.11, 1.0))

    bezel = model.part("display_bezel")
    bezel.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.080, 0.050),
                OUTER_SIZE,
                DEPTH,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.006,
                outer_corner_radius=0.010,
            ),
            "display_bezel",
        ),
        material=finish,
        name="display_bezel",
    )
    bezel.inertial = Inertial.from_geometry(Box((OUTER_SIZE[0], OUTER_SIZE[1], DEPTH)), mass=0.11)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bezel = object_model.get_part("display_bezel")
    ctx.check("display_bezel_part_present", bezel is not None, "Expected a display_bezel part.")
    if bezel is None:
        return ctx.report()

    ctx.check(
        "display_bezel_visual_present",
        bezel.get_visual("display_bezel") is not None,
        "Expected a mesh-backed display_bezel visual.",
    )
    aabb = ctx.part_world_aabb(bezel)
    ctx.check("display_bezel_aabb_present", aabb is not None, "Expected a world AABB for the bezel.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("display_bezel_width", 0.106 <= size[0] <= 0.114, f"size={size!r}")
    ctx.check("display_bezel_height", 0.076 <= size[1] <= 0.084, f"size={size!r}")
    ctx.check("display_bezel_depth", 0.010 <= size[2] <= 0.014, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
