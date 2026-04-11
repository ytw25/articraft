from __future__ import annotations

from sdk import (
    ArticulatedObject,
    BoltPattern,
    Box,
    Inertial,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)

TIRE_OUTER_RADIUS = 0.108
TIRE_WIDTH = 0.044


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scooter_wheel_demo")
    wheel_finish = model.material("scooter_wheel_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    tire_finish = model.material("scooter_tire_black", rgba=(0.10, 0.10, 0.11, 1.0))

    assembly = model.part("scooter_wheel")
    assembly.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.090,
                0.036,
                rim=WheelRim(inner_radius=0.060, flange_height=0.008, flange_thickness=0.003),
                hub=WheelHub(
                    radius=0.022,
                    width=0.026,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(count=5, circle_diameter=0.028, hole_diameter=0.0035),
                ),
                face=WheelFace(dish_depth=0.004, front_inset=0.002),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.008),
                bore=WheelBore(style="round", diameter=0.010),
            ),
            "scooter_wheel",
        ),
        material=wheel_finish,
        name="wheel",
    )
    assembly.visual(
        mesh_from_geometry(
            TireGeometry(
                TIRE_OUTER_RADIUS,
                TIRE_WIDTH,
                inner_radius=0.091,
                tread=TireTread(style="circumferential", depth=0.0035, count=3),
                grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
                sidewall=TireSidewall(style="rounded", bulge=0.05),
            ),
            "scooter_tire",
        ),
        material=tire_finish,
        name="tire",
    )
    assembly.inertial = Inertial.from_geometry(Box((TIRE_WIDTH, 0.216, 0.216)), mass=0.34)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    assembly = object_model.get_part("scooter_wheel")
    ctx.check("scooter_wheel_part_present", assembly is not None, "Expected a scooter_wheel part.")
    if assembly is None:
        return ctx.report()

    ctx.check("scooter_wheel_visual_present", assembly.get_visual("wheel") is not None, "Expected a wheel visual.")
    ctx.check("scooter_tire_visual_present", assembly.get_visual("tire") is not None, "Expected a tire visual.")
    aabb = ctx.part_world_aabb(assembly)
    ctx.check("scooter_wheel_aabb_present", aabb is not None, "Expected a world AABB for the wheel assembly.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    diameter = max(size[1], size[2])
    ctx.check("scooter_wheel_diameter", 0.210 <= diameter <= 0.220, f"size={size!r}")
    ctx.check("scooter_wheel_width", 0.040 <= size[0] <= 0.050, f"size={size!r}")
    ctx.check("scooter_wheel_centered", max(abs(value) for value in center) <= 0.006, f"center={center!r}")
    return ctx.report()


object_model = build_object_model()
