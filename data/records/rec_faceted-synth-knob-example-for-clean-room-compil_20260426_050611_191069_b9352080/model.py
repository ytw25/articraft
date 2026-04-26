from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobTopFeature,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

KNOB_DIAMETER = 0.026
KNOB_HEIGHT = 0.018
BASE_DIAMETER = 0.028


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="faceted_synth_knob")
    finish = model.material("knob_dark", rgba=(0.14, 0.15, 0.16, 1.0))

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                KNOB_DIAMETER,
                KNOB_HEIGHT,
                body_style="faceted",
                base_diameter=BASE_DIAMETER,
                top_diameter=0.020,
                edge_radius=0.0007,
                grip=KnobGrip(style="ribbed", count=12, depth=0.0007, width=0.0016),
                indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
                top_feature=KnobTopFeature(
                    style="top_insert",
                    diameter=0.010,
                    height=0.0012,
                ),
                center=False,
            ),
            "faceted_synth_knob",
        ),
        material=finish,
        name="knob_shell",
    )
    knob.inertial = Inertial.from_geometry(Box((BASE_DIAMETER, BASE_DIAMETER, KNOB_HEIGHT)), mass=0.03)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    knob = object_model.get_part("knob")
    ctx.check("knob_part_present", knob is not None, "Expected a knob part.")
    if knob is None:
        return ctx.report()

    aabb = ctx.part_world_aabb(knob)
    ctx.check("knob_aabb_present", aabb is not None, "Expected a world AABB for the knob.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("knob_diameter", 0.024 <= max(size[0], size[1]) <= 0.030, f"size={size!r}")
    ctx.check("knob_height", 0.017 <= size[2] <= 0.0205, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
