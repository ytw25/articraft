from __future__ import annotations

from sdk import ArticulatedObject, Box, Inertial, PianoHingeGeometry, TestContext, TestReport, mesh_from_geometry

LENGTH = 0.180


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="piano_hinge_demo")
    finish = model.material("piano_hinge_silver", rgba=(0.76, 0.78, 0.80, 1.0))

    hinge = model.part("piano_hinge")
    hinge.visual(
        mesh_from_geometry(
            PianoHingeGeometry(
                LENGTH,
                leaf_width_a=0.016,
                leaf_width_b=0.014,
                leaf_thickness=0.0018,
                pin_diameter=0.0025,
                knuckle_pitch=0.012,
            ),
            "piano_hinge",
        ),
        material=finish,
        name="piano_hinge",
    )
    hinge.inertial = Inertial.from_geometry(Box((0.036, 0.006, LENGTH)), mass=0.11)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hinge = object_model.get_part("piano_hinge")
    ctx.check("piano_hinge_part_present", hinge is not None, "Expected a piano_hinge part.")
    if hinge is None:
        return ctx.report()

    ctx.check(
        "piano_hinge_visual_present",
        hinge.get_visual("piano_hinge") is not None,
        "Expected a mesh-backed piano_hinge visual.",
    )
    aabb = ctx.part_world_aabb(hinge)
    ctx.check("piano_hinge_aabb_present", aabb is not None, "Expected a world AABB for the hinge.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("piano_hinge_length", 0.178 <= size[2] <= 0.182, f"size={size!r}")
    ctx.check("piano_hinge_span", 0.030 <= size[0] <= 0.036, f"size={size!r}")
    ctx.check("piano_hinge_thickness", 0.003 <= size[1] <= 0.006, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
