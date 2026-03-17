# Core Types (`sdk_hybrid.types`)

Most scripts should import these names from top-level `sdk_hybrid`:

```python
from sdk_hybrid import (
    Origin,
    Box, Cylinder, Sphere, Mesh,
    Material, Visual,
    Inertia, Inertial,
    ArticulationType, MotionLimits, MotionProperties,
    Part, Articulation,
)
```

## Coordinate conventions

- Distances are meters.
- Rotations are roll/pitch/yaw in radians (`rpy`).
- `Origin.xyz` and `Origin.rpy` are always 3-tuples of floats.

## Geometry

- `Box(size=(sx, sy, sz))`
- `Cylinder(radius=r, length=l)`
- `Sphere(radius=r)`
- `Mesh(filename="meshes/part.obj", scale=(sx, sy, sz) | None)`

Cylinders are aligned along local `+Z`.

Primitive geometry constructors describe shape only. They do not accept transforms:

- `Box(...)`, `Cylinder(...)`, and `Sphere(...)` do not take `origin=...`
- attach placement on the containing element instead:
  - `part.visual(Box(...), origin=Origin(...))`
  - `Inertial.from_geometry(Box(...), mass=..., origin=Origin(...))`

Example:

```python
panel = Box((0.2, 0.02, 0.1))
body.visual(panel, origin=Origin(xyz=(0.0, 0.03, 0.05)), material="steel")
body.inertial = Inertial.from_geometry(panel, mass=0.4, origin=Origin(xyz=(0.0, 0.03, 0.05)))
```

The `sdk_hybrid` package no longer exposes collision-authoring helpers. Compile/export generates collisions automatically from visuals.

## Visuals and materials

- `Material(name=..., rgba=... | None, texture=... | None)`
- `Visual(geometry=..., origin=Origin(...), material=Material | "name" | None, name=str | None)`

Typical color usage:

```python
steel = model.material("steel", rgba=(0.5, 0.5, 0.55, 1.0))
glass = model.material("glass", rgba=(0.72, 0.84, 0.92, 0.34))

housing.visual(Box((0.2, 0.2, 0.1)), material="steel")
window.visual(Box((0.1, 0.02, 0.08)), material=glass)
```

## Inertia

- `Inertia(ixx=..., ixy=..., ixz=..., iyy=..., iyz=..., izz=...)`
- `Inertial(mass=..., inertia=Inertia(...), origin=Origin(...))`
- `Inertial.from_geometry(...)` supports `Box`, `Cylinder`, and `Sphere`

## Parts

```python
Part(
    name="part_name",
    visuals=[...],
    inertial=Inertial(...) | None,
    meta=dict(...),
)
```

Convenience methods:

```python
part.visual(...)
```

The compiled model still carries an internal `collisions` field, but user-authored `sdk_hybrid` scripts should not set it directly.

## Articulations

```python
Articulation(
    name="hinge",
    articulation_type=ArticulationType.REVOLUTE,
    parent="base",
    child="lid",
    origin=Origin(...),
    axis=(0.0, 0.0, 1.0),
    motion_limits=MotionLimits(...) | None,
    motion_properties=MotionProperties(...) | None,
    meta=dict(...),
)
```

Articulation types:

- `ArticulationType.REVOLUTE`
- `ArticulationType.CONTINUOUS`
- `ArticulationType.PRISMATIC`
- `ArticulationType.FIXED`
- `ArticulationType.FLOATING`
- `ArticulationType.PLANAR`
