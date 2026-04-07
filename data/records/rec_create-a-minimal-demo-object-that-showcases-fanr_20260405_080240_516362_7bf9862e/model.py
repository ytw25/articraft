from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    FanRotorGeometry,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

OUTER_RADIUS = 0.070
THICKNESS = 0.010


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fan_rotor_demo")
    finish = model.material("fan_black", rgba=(0.08, 0.08, 0.09, 1.0))

    fan_rotor = model.part("fan_rotor")
    fan_rotor.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                OUTER_RADIUS,
                0.020,
                5,
                thickness=THICKNESS,
                blade_pitch_deg=24.0,
                blade_sweep_deg=14.0,
            ),
            "fan_rotor",
        ),
        material=finish,
        name="fan_rotor",
    )
    fan_rotor.inertial = Inertial.from_geometry(Box((0.14, 0.14, 0.012)), mass=0.12)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fan_rotor = object_model.get_part("fan_rotor")
    ctx.check("fan_rotor_part_present", fan_rotor is not None, "Expected a single fan_rotor part.")
    if fan_rotor is None:
        return ctx.report()

    ctx.check(
        "fan_rotor_visual_present",
        fan_rotor.get_visual("fan_rotor") is not None,
        "Expected a mesh-backed fan_rotor visual.",
    )
    aabb = ctx.part_world_aabb(fan_rotor)
    ctx.check("fan_rotor_aabb_present", aabb is not None, "Expected a world AABB for the fan rotor.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    diameter = max(size[0], size[1])
    ctx.check(
        "fan_rotor_overall_diameter",
        abs(diameter - OUTER_RADIUS * 2.0) <= 0.012,
        f"size={size!r}",
    )
    ctx.check(
        "fan_rotor_thickness_reasonable",
        0.008 <= size[2] <= 0.014,
        f"size={size!r}",
    )
    ctx.check(
        "fan_rotor_centered",
        max(abs(value) for value in center) <= 0.005,
        f"center={center!r}",
    )
    return ctx.report()


object_model = build_object_model()
