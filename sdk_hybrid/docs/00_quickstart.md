# SDK Hybrid Quickstart

`sdk_hybrid` is the CadQuery-capable SDK package. Use it when you want CadQuery to author visual meshes while the SDK continues to own parts, articulations, compile-time collision generation, QC, and URDF export.

## Script contract

Always define:

- `def build_object_model() -> ArticulatedObject:`
- `def run_tests() -> TestReport:`
- `object_model = build_object_model()`

Do not emit URDF XML yourself. The harness compiles `object_model`, generates collisions, runs QC, and exports URDF.

## Golden path

1. Build parts with `model.part(...)`.
2. Author visuals with `part.visual(...)`. For CadQuery, export a mesh with `mesh_from_cadquery(...)` and attach it with `part.visual(...)`.
3. Use `Inertial.from_geometry(...)` explicitly for inertials.
4. Add motion with `model.articulation(...)`.
5. In `run_tests()`, always include:
   - `ctx.check_model_valid()`
   - `ctx.check_mesh_files_exist()`
   - `ctx.check_articulation_origin_near_geometry(tol=0.01)`
   - `ctx.check_part_geometry_connected(use="visual")`
   - `ctx.check_no_overlaps(..., ignore_adjacent=True, ignore_fixed=True)`

Collisions are generated automatically at compile time:

- primitive visuals become matching primitive collisions
- mesh visuals are decomposed into convex hulls with CoACD
- physical overlap checks run on generated collisions, not authored ones
- overlap QC is conservative; use attachment checks as the primary realism checks
- compile also emits non-blocking isolated-part warnings when a part in a multi-part object is not contacting any other part in the checked pose

## Recommended imports

```python
from sdk_hybrid import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    mesh_from_cadquery,
    Origin,
    TestContext,
    TestReport,
)
```

## Minimal example

```python
import cadquery as cq

from sdk_hybrid import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    mesh_from_cadquery,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="example", assets=ASSETS)

    base = model.part("base")
    base.visual(Box((0.20, 0.20, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)))
    base.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, 0.05)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    lid_shape = cq.Workplane("XY").box(0.18, 0.18, 0.02).edges("|Z").fillet(0.005)

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(lid_shape, "lid.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.02)),
        mass=0.3,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lid",
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )
    ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.05)
    ctx.expect_origin_distance("lid", "base", axes="xy", max_dist=0.02)
    ctx.expect_aabb_gap("lid", "base", axis="z", max_gap=0.001, max_penetration=0.0)
    return ctx.report()


object_model = build_object_model()
```

## Asset paths

Use a first-class asset context:

```python
from sdk_hybrid import AssetContext

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()
```

Important:

- If a model writes or references meshes, prefer `ASSETS = AssetContext.from_script(__file__)`.
- Construct the model with `ArticulatedObject(..., assets=ASSETS)` so generated collision hulls can also be written under the same asset root.
- In tests, either use `TestContext(object_model)` when the model already has `assets=ASSETS`, or pass `asset_root=HERE` explicitly.

## Rules of thumb

- Author the visible shape you actually want to see.
- Do not hand-author collisions in `sdk_hybrid`; compile-time generation owns that now.
- Use `mesh_from_cadquery(...)` as the CadQuery-to-visual bridge, then assign inertials explicitly.
- Treat `check_articulation_origin_near_geometry(tol=0.01)` as necessary but not sufficient for attachment quality.
- Only relax articulation-origin tolerance when the geometry genuinely needs it, and keep it tight.
- Make attachment checks primary: use near-zero `expect_aabb_gap(...)`, footprint overlap, `expect_aabb_contact(...)` where appropriate, and pose-specific mounting checks to prove that parts look attached.
- Slight intended interpenetration can be acceptable when it makes a mounted or nested assembly look seated instead of floating.
- Use `ctx.allow_overlap(...)` narrowly for legitimate nested mechanisms or conservative false-positives, and still call `ctx.check_no_overlaps(...)` so the allowance is tracked.
- Use `ctx.check_no_overlaps(..., ignore_adjacent=True, ignore_fixed=True)` as a conservative backstop, not as proof that attachment quality is good.
- The harness also runs an automatic isolated-part warning pass at compile time; if it reports a part that is not contacting any other part, treat that as a likely floating-geometry bug unless the separation is truly intentional.
- Add selective separation checks only for pairs that truly must remain clear across motion.
