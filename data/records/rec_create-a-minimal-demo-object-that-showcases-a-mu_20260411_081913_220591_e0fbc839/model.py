from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    KnobGeometry,
    KnobGrip,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

DIAMETER = 0.050
HEIGHT = 0.026


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mushroom_clamp_knob_demo")
    finish = model.material("clamp_knob_red", rgba=(0.70, 0.18, 0.14, 1.0))

    knob = model.part("mushroom_clamp_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                DIAMETER,
                HEIGHT,
                body_style="mushroom",
                grip=KnobGrip(style="ribbed", count=12, depth=0.0016),
            ),
            "mushroom_clamp_knob",
        ),
        material=finish,
        name="mushroom_clamp_knob",
    )
    knob.inertial = Inertial.from_geometry(Box((0.052, 0.052, HEIGHT)), mass=0.12)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    knob = object_model.get_part("mushroom_clamp_knob")
    ctx.check("mushroom_clamp_knob_part_present", knob is not None, "Expected a mushroom_clamp_knob part.")
    if knob is None:
        return ctx.report()

    ctx.check(
        "mushroom_clamp_knob_visual_present",
        knob.get_visual("mushroom_clamp_knob") is not None,
        "Expected a mesh-backed mushroom_clamp_knob visual.",
    )
    aabb = ctx.part_world_aabb(knob)
    ctx.check("mushroom_clamp_knob_aabb_present", aabb is not None, "Expected a world AABB for the knob.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    ctx.check("mushroom_clamp_knob_diameter", 0.046 <= max(size[0], size[1]) <= 0.056, f"size={size!r}")
    ctx.check("mushroom_clamp_knob_height", 0.022 <= size[2] <= 0.032, f"size={size!r}")
    ctx.check("mushroom_clamp_knob_centered", max(abs(value) for value in center) <= 0.004, f"center={center!r}")
    return ctx.report()


object_model = build_object_model()
