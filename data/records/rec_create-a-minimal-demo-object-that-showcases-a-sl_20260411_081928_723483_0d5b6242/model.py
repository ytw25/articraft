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

TIRE_OUTER_RADIUS = 0.122
TIRE_WIDTH = 0.040


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slotted_steel_wheel_demo")
    wheel_finish = model.material("steel_wheel_gray", rgba=(0.66, 0.68, 0.70, 1.0))
    tire_finish = model.material("ribbed_tire_black", rgba=(0.11, 0.11, 0.12, 1.0))

    assembly = model.part("slotted_steel_wheel")
    assembly.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.100,
                0.032,
                rim=WheelRim(inner_radius=0.072, flange_height=0.008, flange_thickness=0.003),
                hub=WheelHub(
                    radius=0.024,
                    width=0.024,
                    bolt_pattern=BoltPattern(count=5, circle_diameter=0.030, hole_diameter=0.0036),
                ),
                face=WheelFace(dish_depth=0.003, front_inset=0.002),
                spokes=WheelSpokes(style="solid_slots", count=6, thickness=0.003, window_radius=0.009),
                bore=WheelBore(style="round", diameter=0.010),
            ),
            "slotted_steel_wheel",
        ),
        material=wheel_finish,
        name="wheel",
    )
    assembly.visual(
        mesh_from_geometry(
            TireGeometry(
                TIRE_OUTER_RADIUS,
                TIRE_WIDTH,
                inner_radius=0.101,
                tread=TireTread(style="rib", depth=0.004, count=3),
            ),
            "ribbed_tire",
        ),
        material=tire_finish,
        name="tire",
    )
    assembly.inertial = Inertial.from_geometry(Box((TIRE_WIDTH, 0.244, 0.244)), mass=0.38)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    assembly = object_model.get_part("slotted_steel_wheel")
    ctx.check("slotted_steel_wheel_part_present", assembly is not None, "Expected a slotted_steel_wheel part.")
    if assembly is None:
        return ctx.report()

    ctx.check("slotted_steel_wheel_visual_present", assembly.get_visual("wheel") is not None, "Expected a wheel visual.")
    ctx.check("ribbed_tire_visual_present", assembly.get_visual("tire") is not None, "Expected a tire visual.")
    aabb = ctx.part_world_aabb(assembly)
    ctx.check("slotted_steel_wheel_aabb_present", aabb is not None, "Expected a world AABB for the wheel assembly.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    diameter = max(size[1], size[2])
    ctx.check("slotted_steel_wheel_diameter", 0.238 <= diameter <= 0.248, f"size={size!r}")
    ctx.check("slotted_steel_wheel_width", 0.036 <= size[0] <= 0.044, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
