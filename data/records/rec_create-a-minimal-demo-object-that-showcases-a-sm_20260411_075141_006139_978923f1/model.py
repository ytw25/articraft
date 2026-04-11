from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    KnobBore,
    KnobGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

DIAMETER = 0.040
HEIGHT = 0.020


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smooth_knob_demo")
    finish = model.material("knob_gray", rgba=(0.70, 0.72, 0.76, 1.0))

    knob = model.part("smooth_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                DIAMETER,
                HEIGHT,
                body_style="cylindrical",
                edge_radius=0.0016,
                crown_radius=0.0012,
                bore=KnobBore(style="round", diameter=0.006),
            ),
            "smooth_knob",
        ),
        material=finish,
        name="smooth_knob",
    )
    knob.inertial = Inertial.from_geometry(Box((0.042, 0.042, HEIGHT)), mass=0.08)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    knob = object_model.get_part("smooth_knob")
    ctx.check("smooth_knob_part_present", knob is not None, "Expected a smooth_knob part.")
    if knob is None:
        return ctx.report()

    ctx.check(
        "smooth_knob_visual_present",
        knob.get_visual("smooth_knob") is not None,
        "Expected a mesh-backed smooth_knob visual.",
    )
    aabb = ctx.part_world_aabb(knob)
    ctx.check("smooth_knob_aabb_present", aabb is not None, "Expected a world AABB for the knob.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    ctx.check("smooth_knob_diameter", 0.036 <= max(size[0], size[1]) <= 0.044, f"size={size!r}")
    ctx.check("smooth_knob_height", 0.018 <= size[2] <= 0.024, f"size={size!r}")
    ctx.check("smooth_knob_centered", max(abs(value) for value in center) <= 0.004, f"center={center!r}")
    return ctx.report()


object_model = build_object_model()
