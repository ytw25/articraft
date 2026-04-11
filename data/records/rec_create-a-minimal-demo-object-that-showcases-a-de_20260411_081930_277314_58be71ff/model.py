from __future__ import annotations

from sdk import (
    ArticulatedObject,
    BoltPattern,
    Box,
    Inertial,
    TestContext,
    TestReport,
    TireGeometry,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)

TIRE_OUTER_RADIUS = 0.136
TIRE_WIDTH = 0.032


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deep_dish_wheel_demo")
    wheel_finish = model.material("deep_dish_wheel_silver", rgba=(0.80, 0.82, 0.84, 1.0))
    tire_finish = model.material("low_profile_tire_black", rgba=(0.09, 0.09, 0.10, 1.0))

    assembly = model.part("deep_dish_wheel")
    assembly.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.120,
                0.045,
                rim=WheelRim(inner_radius=0.082, flange_height=0.010, flange_thickness=0.004),
                hub=WheelHub(
                    radius=0.028,
                    width=0.028,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=5, circle_diameter=0.034, hole_diameter=0.004),
                ),
                face=WheelFace(dish_depth=0.010, front_inset=0.003, rear_inset=0.002),
                spokes=WheelSpokes(style="split_y", count=5, thickness=0.003, window_radius=0.010),
                bore=WheelBore(style="round", diameter=0.012),
            ),
            "deep_dish_wheel",
        ),
        material=wheel_finish,
        name="wheel",
    )
    assembly.visual(
        mesh_from_geometry(
            TireGeometry(
                TIRE_OUTER_RADIUS,
                TIRE_WIDTH,
                inner_radius=0.121,
                tread=TireTread(style="circumferential", depth=0.003, count=2),
            ),
            "low_profile_tire",
        ),
        material=tire_finish,
        name="tire",
    )
    assembly.inertial = Inertial.from_geometry(Box((0.048, 0.272, 0.272)), mass=0.42)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    assembly = object_model.get_part("deep_dish_wheel")
    ctx.check("deep_dish_wheel_part_present", assembly is not None, "Expected a deep_dish_wheel part.")
    if assembly is None:
        return ctx.report()

    ctx.check("deep_dish_wheel_visual_present", assembly.get_visual("wheel") is not None, "Expected a wheel visual.")
    ctx.check("low_profile_tire_visual_present", assembly.get_visual("tire") is not None, "Expected a tire visual.")
    aabb = ctx.part_world_aabb(assembly)
    ctx.check("deep_dish_wheel_aabb_present", aabb is not None, "Expected a world AABB for the wheel assembly.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    diameter = max(size[1], size[2])
    ctx.check("deep_dish_wheel_diameter", 0.268 <= diameter <= 0.276, f"size={size!r}")
    ctx.check("deep_dish_wheel_width", 0.030 <= size[0] <= 0.050, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
