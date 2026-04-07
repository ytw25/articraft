from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)

OVERALL_SIZE = (0.12, 0.05, 0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trunnion_yoke_demo")
    finish = model.material("yoke_blue_gray", rgba=(0.39, 0.49, 0.60, 1.0))

    trunnion_yoke = model.part("trunnion_yoke")
    trunnion_yoke.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                OVERALL_SIZE,
                span_width=0.060,
                trunnion_diameter=0.016,
                trunnion_center_z=0.050,
                base_thickness=0.014,
                corner_radius=0.003,
            ),
            "trunnion_yoke",
        ),
        material=finish,
        name="trunnion_yoke",
    )
    trunnion_yoke.inertial = Inertial.from_geometry(Box(OVERALL_SIZE), mass=0.34)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    trunnion_yoke = object_model.get_part("trunnion_yoke")
    ctx.check("trunnion_yoke_part_present", trunnion_yoke is not None, "Expected a single trunnion_yoke part.")
    if trunnion_yoke is None:
        return ctx.report()

    ctx.check(
        "trunnion_yoke_visual_present",
        trunnion_yoke.get_visual("trunnion_yoke") is not None,
        "Expected a mesh-backed trunnion_yoke visual.",
    )
    aabb = ctx.part_world_aabb(trunnion_yoke)
    ctx.check("trunnion_yoke_aabb_present", aabb is not None, "Expected a world AABB for the trunnion yoke.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    ctx.check(
        "trunnion_yoke_overall_size",
        abs(size[0] - OVERALL_SIZE[0]) <= 0.004
        and abs(size[1] - OVERALL_SIZE[1]) <= 0.004
        and abs(size[2] - OVERALL_SIZE[2]) <= 0.004,
        f"size={size!r}",
    )
    ctx.check(
        "trunnion_yoke_centered",
        max(abs(value) for value in center) <= 0.002,
        f"center={center!r}",
    )
    return ctx.report()


object_model = build_object_model()
