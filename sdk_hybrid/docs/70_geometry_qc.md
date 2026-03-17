# Geometry QC

Import:

```python
from sdk_hybrid import (
    GeometryOverlap,
    UnsupportedPartFinding,
    default_contact_tol_from_env,
    default_overlap_tol_from_env,
    default_overlap_volume_tol_from_env,
    find_geometry_overlaps,
    find_unsupported_parts,
    validate_no_geometry_overlaps,
)
```

These checks sample articulation poses and look for intersections between parts.
Treat them as a safety backstop, not as the primary proof that parts look attached.

## `find_unsupported_parts(...)`

```python
find_unsupported_parts(
    object_model,
    asset_root=HERE,
    geometry_source="collision",
    max_pose_samples=1,
    contact_tol=1e-6,
    seed=0,
) -> list[UnsupportedPartFinding]
```

Find parts in a multi-part object that are not contacting any other part in the checked pose.

- Use `geometry_source="visual"` for visible-envelope support checks based on world AABBs.
- Use `geometry_source="collision"` for physical support checks on generated collision geometry.
- Collision-mode support queries require FCL. Missing or broken FCL support is an error; the SDK does not silently fall back.
- This function is used by the harness automatically at compile time to emit non-blocking isolated-part warnings.

## Main APIs

```python
find_geometry_overlaps(
    object_model,
    asset_root=HERE,
    geometry_source="collision",
    max_pose_samples=256,
    overlap_tol=1e-3,
    overlap_volume_tol=0.0,
    allowed_pairs=None,
    seed=0,
) -> list[GeometryOverlap]
```

```python
validate_no_geometry_overlaps(...)
```

Raises `ValidationError` when any disallowed overlap is found.

## Geometry sources

- Use `geometry_source="collision"` for physical overlap checks. In `sdk_hybrid`, collision geometry is generated from visuals during compile/test setup.
- Use `geometry_source="visual"` when you explicitly want visible-envelope overlap checks.

## Pose sampling

QC samples articulation positions automatically from fixed defaults, explicit metadata samples, or motion limits.

## Mesh support

Mesh-based QC currently supports `.obj` files for mesh-bound calculations.

## Important limitations

This QC is conservative. It relies on generated collision geometry and bounding-volume tests, not exact visual-surface contact.

- It can report false positives for nested mounts, forks, axle supports, and other tightly packed assemblies.
- It should not be used to force visible air gaps between parts that are supposed to look attached.
- Use intent checks such as `expect_aabb_gap(...)`, `expect_aabb_overlap(..., axes="xy", ...)`, `expect_aabb_contact(...)`, and pose-specific checks to prove attachment quality.
- When slight intended interpenetration is acceptable, allow the specific pair narrowly rather than disabling QC entirely.
