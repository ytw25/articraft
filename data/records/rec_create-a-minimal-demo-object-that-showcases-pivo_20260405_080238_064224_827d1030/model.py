from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    PivotForkGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

OVERALL_SIZE = (0.08, 0.05, 0.05)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pivot_fork_demo")
    finish = model.material("pivot_fork_orange", rgba=(0.82, 0.44, 0.18, 1.0))

    pivot_fork = model.part("pivot_fork")
    pivot_fork.visual(
        mesh_from_geometry(
            PivotForkGeometry(
                OVERALL_SIZE,
                gap_width=0.034,
                bore_diameter=0.010,
                bore_center_z=0.028,
                bridge_thickness=0.012,
                corner_radius=0.002,
            ),
            "pivot_fork",
        ),
        material=finish,
        name="pivot_fork",
    )
    pivot_fork.inertial = Inertial.from_geometry(Box(OVERALL_SIZE), mass=0.24)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pivot_fork = object_model.get_part("pivot_fork")
    ctx.check("pivot_fork_part_present", pivot_fork is not None, "Expected a single pivot_fork part.")
    if pivot_fork is None:
        return ctx.report()

    ctx.check(
        "pivot_fork_visual_present",
        pivot_fork.get_visual("pivot_fork") is not None,
        "Expected a mesh-backed pivot_fork visual.",
    )
    aabb = ctx.part_world_aabb(pivot_fork)
    ctx.check("pivot_fork_aabb_present", aabb is not None, "Expected a world AABB for the pivot fork.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    ctx.check(
        "pivot_fork_overall_size",
        abs(size[0] - OVERALL_SIZE[0]) <= 0.004
        and abs(size[1] - OVERALL_SIZE[1]) <= 0.004
        and abs(size[2] - OVERALL_SIZE[2]) <= 0.004,
        f"size={size!r}",
    )
    ctx.check(
        "pivot_fork_centered",
        max(abs(value) for value in center) <= 0.002,
        f"center={center!r}",
    )
    return ctx.report()


object_model = build_object_model()
