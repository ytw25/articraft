# Testing and Intent Checks

Import:

```python
from sdk_hybrid import TestContext, TestReport, TestFailure
```

`TestContext` records structural checks, overlap checks, and pose-aware intent assertions for an articulated object.

## Construction

```python
ctx = TestContext(object_model, geometry_source="collision", seed=0)
```

If the model was built with `assets=AssetContext.from_script(__file__)`, `TestContext(object_model)` will infer `asset_root` automatically.
If not, pass `asset_root=HERE` explicitly whenever the model uses mesh files.

Mesh-backed models:

- Relative mesh references like `meshes/part.obj` are expected.
- QC and overlap checks still need an asset root to open those files.
- Without model assets or `asset_root=...`, mesh-aware checks can fail with `ValidationError: Mesh path is relative but no asset_root provided`.

## Built-in checks

- `ctx.check_model_valid()`
- `ctx.check_mesh_files_exist()`
- `ctx.check_articulation_origin_near_geometry(...)`
- `ctx.check_part_geometry_connected(use="visual")`
- `ctx.check_no_overlaps(..., ignore_adjacent=True, ignore_fixed=True)`

The harness also runs an automatic isolated-part warning pass at compile time:

- it checks multi-part objects for parts that are not contacting any other part in the checked pose
- it emits both `visual` and `collision` warnings when it finds isolated parts
- it is non-blocking, but should usually be treated as evidence of floating geometry or a bad mount

## Connectivity check signatures

Preferred calls:

```python
ctx.check_articulation_origin_near_geometry(tol=0.01)
ctx.check_part_geometry_connected(use="visual")
```

Important:

- `check_articulation_origin_near_geometry` is keyword-only after `self`
- pass `tol=` by keyword, not as a positional argument

Invalid example:

```python
ctx.check_articulation_origin_near_geometry(0.01)
```

## Overlap allowances

```python
ctx.allow_overlap("door", "body", reason="hinge adjacency")
ctx.allow_overlap(door_part, body_part, reason="hinge adjacency")
```

Use allowances narrowly. Slight intended interpenetration can be acceptable when it makes a mounted or nested assembly look attached instead of floating. Still call `ctx.check_no_overlaps(...)` so the allowance is tracked.

## Pose-aware intent checks

- `ctx.expect_origin_distance(...)`
- `ctx.expect_origin_gap(...)`
- `ctx.expect_aabb_within(...)`
- `ctx.expect_aabb_gap(...)`
- `ctx.expect_aabb_overlap(...)`
- `ctx.expect_aabb_contact(...)`
- `ctx.expect_joint_motion_axis(...)`

Use these to encode the intended behavior of the object, not just structural validity.
Treat them as the primary realism checks for mounted parts and articulated clearances.

### `expect_aabb_gap`

Preferred signature:

```python
ctx.expect_aabb_gap(
    "upper_link",
    "lower_link",
    axis="z",
    max_gap=0.001,
    max_penetration=0.0,
)
```

- `gap_z = upper_min_z - lower_max_z`
- Passes when `-max_penetration <= gap_z <= max_gap`
- Legacy `min_gap=...` is still accepted, but new examples should prefer `max_penetration=...`
- For mounted or nested assemblies, default to near-zero gap checks unless the real object visibly has more clearance.
- Pair gap checks with footprint/overlap checks such as `expect_aabb_overlap(..., axes="xy", ...)` or `expect_aabb_within(..., axes="xy", ...)`.
- Use `expect_aabb_contact(...)` when you want to assert direct touch/overlap rather than a directional gap window.
- Add pose-specific checks at important limits so the assembly still looks attached when articulated.

### `expect_joint_motion_axis`

Preferred signature:

```python
ctx.expect_joint_motion_axis(
    "lid_hinge",
    "lid",
    world_axis="z",
    direction="positive",
    min_delta=0.01,
)
```

- `world_axis` should be `"x"`, `"y"`, or `"z"`
- `direction` should be `"positive"` or `"negative"`
- Legacy tuple axes like `(0, 0, 1)` and numeric directions like `1.0` are accepted for compatibility, but new code should use the string form above

Use `expect_joint_motion_axis(...)` only when changing the joint should move the link's AABB center along the tested world axis.

Do not use it for centered continuous rotors, wheels, knobs, propellers, or fans spinning in place around their own centerline. For those, use `with ctx.pose(...):` plus pose-specific containment, overlap, or clearance checks at multiple angles instead.

For pairs that truly must not intersect across motion, add selective separation checks in the relevant poses rather than relying only on global overlap QC.

## `ctx.pose(...)`

Use one of these forms:

```python
with ctx.pose({"lid_hinge": 0.5}):
    ...

with ctx.pose(lid_hinge=1.0):
    ...
```

`ctx.pose(...)` takes either:

- a mapping from joint name to joint position
- keyword arguments mapping joint name to joint position

Do not pass positional `(joint_name, value)` arguments.

## Example

```python
ctx.check_model_valid()
ctx.check_mesh_files_exist()
ctx.check_articulation_origin_near_geometry(tol=0.01)
ctx.check_part_geometry_connected(use="visual")
ctx.check_no_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)
ctx.expect_aabb_overlap("lid", "base", axes="xy", min_overlap=0.05)
ctx.expect_origin_distance("lid", "base", axes="xy", max_dist=0.02)
ctx.expect_origin_gap("lid", "base", axis="z", min_gap=0.0)
ctx.expect_aabb_gap("lid", "base", axis="z", max_gap=0.001, max_penetration=0.0)
ctx.expect_aabb_contact("hinge_leaf", "frame")
ctx.expect_joint_motion_axis("lid_hinge", "lid", world_axis="z", direction="positive", min_delta=0.01)
with ctx.pose({"lid_hinge": 1.0}):
    ctx.expect_aabb_gap("lid", "base", axis="z", max_gap=0.08, max_penetration=0.0)
```
