from __future__ import annotations

from sdk import ArticulatedObject, Box, Origin, TestContext, TestReport

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_cube")

    cube = model.part("cube")
    cube.visual(
        Box((0.05, 0.05, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        name="cube_body",
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.
    cube = object_model.get_part("cube")
    aabb = ctx.part_world_aabb(cube)
    ok = False
    details = f"aabb={aabb}"
    if aabb is not None:
        mins, maxs = aabb
        dx = maxs[0] - mins[0]
        dy = maxs[1] - mins[1]
        dz = maxs[2] - mins[2]
        ok = (
            abs(dx - 0.05) < 1e-6
            and abs(dy - 0.05) < 1e-6
            and abs(dz - 0.05) < 1e-6
            and abs(mins[2]) < 1e-6
        )
        details = f"mins={mins}, maxs={maxs}, dims=({dx}, {dy}, {dz})"
    ctx.check("cube has 5 cm edges and sits on z=0", ok, details=details)

    return ctx.report()


object_model = build_object_model()
