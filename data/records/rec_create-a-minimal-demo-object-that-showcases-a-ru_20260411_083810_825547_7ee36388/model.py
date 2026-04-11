from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_geometry,
)

TIRE_OUTER_RADIUS = 0.118
TIRE_WIDTH = 0.050


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_utility_wheel_demo")
    wheel_finish = model.material("utility_wheel_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    tire_finish = model.material("utility_tire_black", rgba=(0.09, 0.09, 0.10, 1.0))

    assembly = model.part("utility_wheel")
    assembly.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.085,
                0.036,
                rim=WheelRim(inner_radius=0.060, flange_height=0.007, flange_thickness=0.003),
                hub=WheelHub(radius=0.022, width=0.024),
                face=WheelFace(dish_depth=0.002),
                bore=WheelBore(style="round", diameter=0.010),
            ),
            "utility_wheel",
        ),
        material=wheel_finish,
        name="wheel",
    )
    assembly.visual(
        mesh_from_geometry(
            TireGeometry(
                TIRE_OUTER_RADIUS,
                TIRE_WIDTH,
                inner_radius=0.086,
                tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.56),
                sidewall=TireSidewall(style="square", bulge=0.0),
                shoulder=TireShoulder(width=0.005, radius=0.003),
            ),
            "utility_tire",
        ),
        material=tire_finish,
        name="tire",
    )
    assembly.inertial = Inertial.from_geometry(Box((TIRE_WIDTH, 0.236, 0.236)), mass=0.36)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    assembly = object_model.get_part("utility_wheel")
    ctx.check("utility_wheel_part_present", assembly is not None, "Expected a utility_wheel part.")
    if assembly is None:
        return ctx.report()

    ctx.check("utility_wheel_visual_present", assembly.get_visual("wheel") is not None, "Expected a wheel visual.")
    ctx.check("utility_tire_visual_present", assembly.get_visual("tire") is not None, "Expected a tire visual.")
    aabb = ctx.part_world_aabb(assembly)
    ctx.check("utility_wheel_aabb_present", aabb is not None, "Expected a world AABB for the wheel assembly.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    diameter = max(size[1], size[2])
    ctx.check("utility_wheel_diameter", 0.232 <= diameter <= 0.240, f"size={size!r}")
    ctx.check("utility_wheel_width", 0.046 <= size[0] <= 0.054, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
