---
title: 'Spur, Helical, and Herringbone Gears'
description: 'Use the vendored gear classes with the preserved `cadquery.Workplane.gear()` plugin workflow.'
tags:
  - cadquery
  - examples
  - gear
  - spur
  - helical
  - herringbone
---
# Spur, Helical, and Herringbone Gears

This example uses the vendored `sdk_hybrid` gear classes while keeping the familiar `cq_gears` plugin pattern on `cadquery.Workplane`.

```python
import cadquery as cq

from sdk_hybrid import HerringboneGear, SpurGear

spur = SpurGear(module=1.0, teeth_number=19, width=5.0, bore_d=5.0)
helical = SpurGear(
    module=1.0,
    teeth_number=17,
    width=6.0,
    helix_angle=25.0,
    bore_d=4.0,
)
herringbone = HerringboneGear(
    module=1.0,
    teeth_number=24,
    width=10.0,
    helix_angle=20.0,
    bore_d=5.0,
)

spur_body = cq.Workplane("XY").gear(spur).val()
helical_body = (
    cq.Workplane("XY")
    .moveTo(spur.r0 + helical.r0 + 4.0, 0.0)
    .gear(helical)
    .val()
)
herringbone_body = (
    cq.Workplane("XY")
    .moveTo(spur.r0 + helical.r0 + herringbone.r0 + 10.0, 0.0)
    .gear(herringbone)
    .val()
)

result = cq.Assembly(name="spur-family")
result.add(spur_body, name="spur")
result.add(helical_body, name="helical")
result.add(herringbone_body, name="herringbone")
```
