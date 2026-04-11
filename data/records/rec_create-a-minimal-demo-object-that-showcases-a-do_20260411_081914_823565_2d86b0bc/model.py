from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    KnobBore,
    KnobGeometry,
    KnobIndicator,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

DIAMETER = 0.040
HEIGHT = 0.022


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="domed_pointer_knob_demo")
    finish = model.material("pointer_knob_cream", rgba=(0.85, 0.83, 0.76, 1.0))

    knob = model.part("domed_pointer_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                DIAMETER,
                HEIGHT,
                body_style="domed",
                top_diameter=0.034,
                indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=30.0),
                bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            ),
            "domed_pointer_knob",
        ),
        material=finish,
        name="domed_pointer_knob",
    )
    knob.inertial = Inertial.from_geometry(Box((0.042, 0.042, HEIGHT)), mass=0.08)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    knob = object_model.get_part("domed_pointer_knob")
    ctx.check("domed_pointer_knob_part_present", knob is not None, "Expected a domed_pointer_knob part.")
    if knob is None:
        return ctx.report()

    ctx.check(
        "domed_pointer_knob_visual_present",
        knob.get_visual("domed_pointer_knob") is not None,
        "Expected a mesh-backed domed_pointer_knob visual.",
    )
    aabb = ctx.part_world_aabb(knob)
    ctx.check("domed_pointer_knob_aabb_present", aabb is not None, "Expected a world AABB for the knob.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    ctx.check("domed_pointer_knob_diameter", 0.036 <= max(size[0], size[1]) <= 0.044, f"size={size!r}")
    ctx.check("domed_pointer_knob_height", 0.020 <= size[2] <= 0.028, f"size={size!r}")
    ctx.check("domed_pointer_knob_centered", max(abs(value) for value in center) <= 0.005, f"center={center!r}")
    return ctx.report()


object_model = build_object_model()
