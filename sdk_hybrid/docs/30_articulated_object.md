# `ArticulatedObject`

Import:

```python
from sdk_hybrid import ArticulatedObject, AssetContext
```

`ArticulatedObject` is the main container for authored assemblies.

## Construction

```python
model = ArticulatedObject(name="desk_lamp")
```

With a script-local asset context:

```python
assets = AssetContext.from_script(__file__)
model = ArticulatedObject(name="desk_lamp", assets=assets)
```

Fields:

- `name: str`
- `parts: list[Part]`
- `articulations: list[Articulation]`
- `materials: list[Material]`
- `meta: dict[str, object]`
- `assets: AssetContext | None`

## Adding materials

```python
steel = model.material("steel", rgba=(0.5, 0.5, 0.55, 1.0))
accent = model.material("accent", rgba=(0.8, 0.1, 0.1, 1.0))

body = model.part("body")
body.visual(Box((0.2, 0.1, 0.05)), material="steel")

button = model.part("button")
button.visual(Box((0.02, 0.02, 0.01)), material=accent)
```

Robot-level materials are reusable across many visuals. Named material references are validated during `model.validate(...)` and are rendered by the viewer as URDF colors.

## Adding parts

```python
base = model.part("base")
# or
base = model.link("base")
```

`part(...)` and `link(...)` are equivalent entry points.

## Adding articulations

```python
hinge = model.articulation(
    "base_to_lid",
    ArticulationType.REVOLUTE,
    parent="base",   # or parent=base
    child="lid",     # or child=lid
    origin=Origin(...),
    axis=(0.0, 1.0, 0.0),
    motion_limits=MotionLimits(...) | None,
    motion_properties=MotionProperties(...) | None,
)
# or
hinge = model.joint(
    "base_to_lid",
    ArticulationType.REVOLUTE,
    parent=base,
    child=lid,
    origin=Origin(...),
)
```

`articulation(...)` and `joint(...)` are equivalent entry points. `parent=` and `child=` accept either string names or `Part` objects.

## Lookup helpers

```python
model.get_part("base")
model.get_articulation("hinge")
model.root_parts()
```

## Validation

```python
model.validate(strict=True, strict_mesh_paths=False)
```

Validation checks:

- at least one part exists
- part names, articulation names, and material names are unique
- every articulation references existing parent/child parts
- motion axes and limits are valid in strict mode
- geometry dimensions and mesh settings are valid
- the articulation graph is connected from at least one root part

## Geometry overlap QC

```python
model.validate_geometry(
    asset_root=HERE,
    geometry_source="collision",
    max_pose_samples=256,
    overlap_tol=None,
    overlap_volume_tol=None,
    ignore_adjacent=True,
    ignore_fixed=True,
    allowed_pairs=None,
    seed=0,
)
```

Use `geometry_source="collision"` for physical overlap checks and `geometry_source="visual"` for visible-envelope checks.
