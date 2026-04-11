from __future__ import annotations

from sdk import (
    ArticulatedObject,
    BoltPattern,
    Box,
    Inertial,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireSidewall,
    WheelBore,
    WheelFace,
    WheelFlange,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)

TIRE_OUTER_RADIUS = 0.140
TIRE_WIDTH = 0.030


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_rim_wheel_demo")
    wheel_finish = model.material("push_rim_silver", rgba=(0.80, 0.81, 0.83, 1.0))
    tire_finish = model.material("narrow_tire_black", rgba=(0.10, 0.10, 0.11, 1.0))

    assembly = model.part("push_rim_wheel")
    assembly.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.125,
                0.034,
                rim=WheelRim(inner_radius=0.086, flange_height=0.010, flange_thickness=0.004),
                hub=WheelHub(
                    radius=0.026,
                    width=0.024,
                    bolt_pattern=BoltPattern(count=5, circle_diameter=0.032, hole_diameter=0.0038),
                ),
                face=WheelFace(dish_depth=0.004, front_inset=0.002),
                spokes=WheelSpokes(style="straight", count=8, thickness=0.0028, window_radius=0.008),
                bore=WheelBore(style="round", diameter=0.012),
                flange=WheelFlange(radius=0.145, thickness=0.004, offset=0.009, side="both"),
            ),
            "push_rim_wheel",
        ),
        material=wheel_finish,
        name="wheel",
    )
    assembly.visual(
        mesh_from_geometry(
            TireGeometry(
                TIRE_OUTER_RADIUS,
                TIRE_WIDTH,
                inner_radius=0.126,
                carcass=TireCarcass(belt_width_ratio=0.62, sidewall_bulge=0.03),
                sidewall=TireSidewall(style="rounded", bulge=0.03),
            ),
            "narrow_tire",
        ),
        material=tire_finish,
        name="tire",
    )
    assembly.inertial = Inertial.from_geometry(Box((0.062, 0.290, 0.290)), mass=0.44)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    assembly = object_model.get_part("push_rim_wheel")
    ctx.check("push_rim_wheel_part_present", assembly is not None, "Expected a push_rim_wheel part.")
    if assembly is None:
        return ctx.report()

    ctx.check("push_rim_wheel_visual_present", assembly.get_visual("wheel") is not None, "Expected a wheel visual.")
    ctx.check("narrow_tire_visual_present", assembly.get_visual("tire") is not None, "Expected a tire visual.")
    aabb = ctx.part_world_aabb(assembly)
    ctx.check("push_rim_wheel_aabb_present", aabb is not None, "Expected a world AABB for the wheel assembly.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    diameter = max(size[1], size[2])
    ctx.check("push_rim_wheel_diameter", 0.278 <= diameter <= 0.292, f"size={size!r}")
    ctx.check("push_rim_wheel_width", 0.054 <= size[0] <= 0.066, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
