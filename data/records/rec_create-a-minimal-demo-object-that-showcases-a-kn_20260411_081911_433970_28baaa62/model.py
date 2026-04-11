from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobTopFeature,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

DIAMETER = 0.032
HEIGHT = 0.018


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knurled_encoder_knob_demo")
    finish = model.material("encoder_silver", rgba=(0.76, 0.78, 0.80, 1.0))

    knob = model.part("knurled_encoder_knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                DIAMETER,
                HEIGHT,
                body_style="cylindrical",
                grip=KnobGrip(style="knurled", count=24, depth=0.0010, helix_angle_deg=24.0),
                top_feature=KnobTopFeature(style="recessed_disk", diameter=0.016, depth=0.0016),
                bore=KnobBore(style="round", diameter=0.006),
            ),
            "knurled_encoder_knob",
        ),
        material=finish,
        name="knurled_encoder_knob",
    )
    knob.inertial = Inertial.from_geometry(Box((0.034, 0.034, HEIGHT)), mass=0.06)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    knob = object_model.get_part("knurled_encoder_knob")
    ctx.check("knurled_encoder_knob_part_present", knob is not None, "Expected a knurled_encoder_knob part.")
    if knob is None:
        return ctx.report()

    ctx.check(
        "knurled_encoder_knob_visual_present",
        knob.get_visual("knurled_encoder_knob") is not None,
        "Expected a mesh-backed knurled_encoder_knob visual.",
    )
    aabb = ctx.part_world_aabb(knob)
    ctx.check("knurled_encoder_knob_aabb_present", aabb is not None, "Expected a world AABB for the knob.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    ctx.check("knurled_encoder_knob_diameter", 0.030 <= max(size[0], size[1]) <= 0.036, f"size={size!r}")
    ctx.check("knurled_encoder_knob_height", 0.016 <= size[2] <= 0.022, f"size={size!r}")
    ctx.check("knurled_encoder_knob_centered", max(abs(value) for value in center) <= 0.004, f"center={center!r}")
    return ctx.report()


object_model = build_object_model()
