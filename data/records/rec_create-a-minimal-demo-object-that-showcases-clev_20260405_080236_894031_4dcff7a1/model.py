from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    ClevisBracketGeometry,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

OVERALL_SIZE = (0.08, 0.04, 0.06)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clevis_bracket_demo")
    finish = model.material("clevis_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    clevis_bracket = model.part("clevis_bracket")
    clevis_bracket.visual(
        mesh_from_geometry(
            ClevisBracketGeometry(
                OVERALL_SIZE,
                gap_width=0.032,
                bore_diameter=0.012,
                bore_center_z=0.038,
                base_thickness=0.012,
                corner_radius=0.003,
            ),
            "clevis_bracket",
        ),
        material=finish,
        name="clevis_bracket",
    )
    clevis_bracket.inertial = Inertial.from_geometry(Box(OVERALL_SIZE), mass=0.28)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clevis_bracket = object_model.get_part("clevis_bracket")
    ctx.check("clevis_bracket_part_present", clevis_bracket is not None, "Expected a single clevis_bracket part.")
    if clevis_bracket is None:
        return ctx.report()

    ctx.check(
        "clevis_bracket_visual_present",
        clevis_bracket.get_visual("clevis_bracket") is not None,
        "Expected a mesh-backed clevis_bracket visual.",
    )
    aabb = ctx.part_world_aabb(clevis_bracket)
    ctx.check("clevis_bracket_aabb_present", aabb is not None, "Expected a world AABB for the clevis bracket.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    ctx.check(
        "clevis_bracket_overall_size",
        abs(size[0] - OVERALL_SIZE[0]) <= 0.004
        and abs(size[1] - OVERALL_SIZE[1]) <= 0.004
        and abs(size[2] - OVERALL_SIZE[2]) <= 0.004,
        f"size={size!r}",
    )
    ctx.check(
        "clevis_bracket_centered",
        max(abs(value) for value in center) <= 0.002,
        f"center={center!r}",
    )
    return ctx.report()


object_model = build_object_model()
