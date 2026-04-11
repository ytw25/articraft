from __future__ import annotations

from sdk import (
    ArticulatedObject,
    BarrelHingeGeometry,
    Box,
    HingeHolePattern,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

LENGTH = 0.090


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barrel_hinge_demo")
    finish = model.material("hinge_silver", rgba=(0.74, 0.76, 0.78, 1.0))

    hinge = model.part("barrel_hinge")
    hinge.visual(
        mesh_from_geometry(
            BarrelHingeGeometry(
                LENGTH,
                leaf_width_a=0.024,
                leaf_width_b=0.020,
                leaf_thickness=0.0024,
                pin_diameter=0.003,
                knuckle_count=5,
                holes_a=HingeHolePattern(style="round", count=3, diameter=0.0032, edge_margin=0.010),
                holes_b=HingeHolePattern(style="slotted", count=2, slot_size=(0.007, 0.003), edge_margin=0.012),
            ),
            "barrel_hinge",
        ),
        material=finish,
        name="barrel_hinge",
    )
    hinge.inertial = Inertial.from_geometry(Box((0.052, 0.008, LENGTH)), mass=0.09)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hinge = object_model.get_part("barrel_hinge")
    ctx.check("barrel_hinge_part_present", hinge is not None, "Expected a barrel_hinge part.")
    if hinge is None:
        return ctx.report()

    ctx.check(
        "barrel_hinge_visual_present",
        hinge.get_visual("barrel_hinge") is not None,
        "Expected a mesh-backed barrel_hinge visual.",
    )
    aabb = ctx.part_world_aabb(hinge)
    ctx.check("barrel_hinge_aabb_present", aabb is not None, "Expected a world AABB for the hinge.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("barrel_hinge_length", 0.088 <= size[2] <= 0.092, f"size={size!r}")
    ctx.check("barrel_hinge_span", 0.045 <= size[0] <= 0.052, f"size={size!r}")
    ctx.check("barrel_hinge_thickness", 0.004 <= size[1] <= 0.008, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
