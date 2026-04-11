from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    WheelBore,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_geometry,
)

TIRE_OUTER_RADIUS = 0.068
TIRE_WIDTH = 0.028


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="caster_wheel_demo")
    wheel_finish = model.material("caster_wheel_gray", rgba=(0.70, 0.72, 0.74, 1.0))
    tire_finish = model.material("caster_tire_black", rgba=(0.12, 0.12, 0.13, 1.0))

    assembly = model.part("caster_wheel")
    assembly.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.055,
                0.024,
                rim=WheelRim(inner_radius=0.038, flange_height=0.006, flange_thickness=0.003),
                hub=WheelHub(radius=0.015, width=0.020),
                bore=WheelBore(style="round", diameter=0.008),
            ),
            "caster_wheel",
        ),
        material=wheel_finish,
        name="wheel",
    )
    assembly.visual(
        mesh_from_geometry(
            TireGeometry(
                TIRE_OUTER_RADIUS,
                TIRE_WIDTH,
                inner_radius=0.056,
                sidewall=TireSidewall(style="rounded", bulge=0.04),
            ),
            "caster_tire",
        ),
        material=tire_finish,
        name="tire",
    )
    assembly.inertial = Inertial.from_geometry(Box((TIRE_WIDTH, 0.136, 0.136)), mass=0.18)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    assembly = object_model.get_part("caster_wheel")
    ctx.check("caster_wheel_part_present", assembly is not None, "Expected a caster_wheel part.")
    if assembly is None:
        return ctx.report()

    ctx.check("caster_wheel_visual_present", assembly.get_visual("wheel") is not None, "Expected a wheel visual.")
    ctx.check("caster_tire_visual_present", assembly.get_visual("tire") is not None, "Expected a tire visual.")
    aabb = ctx.part_world_aabb(assembly)
    ctx.check("caster_wheel_aabb_present", aabb is not None, "Expected a world AABB for the wheel assembly.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    diameter = max(size[1], size[2])
    ctx.check("caster_wheel_diameter", 0.130 <= diameter <= 0.140, f"size={size!r}")
    ctx.check("caster_wheel_width", 0.024 <= size[0] <= 0.032, f"size={size!r}")
    ctx.check("caster_wheel_centered", max(abs(value) for value in center) <= 0.004, f"center={center!r}")
    return ctx.report()


object_model = build_object_model()
