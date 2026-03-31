from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    *,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_trigger_spray_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.87, 0.82, 0.72, 1.0))
    head_paint = model.material("head_paint", rgba=(0.22, 0.26, 0.24, 1.0))
    adapter_paint = model.material("adapter_paint", rgba=(0.28, 0.31, 0.30, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.11, 0.12, 0.13, 1.0))

    bottle = model.part("bottle")
    bottle_body = section_loft(
        [
            _xy_section(0.082, 0.060, 0.012, 0.000),
            _xy_section(0.090, 0.066, 0.014, 0.048),
            _xy_section(0.088, 0.064, 0.013, 0.135),
            _xy_section(0.074, 0.054, 0.012, 0.188),
            _xy_section(0.040, 0.036, 0.009, 0.204),
        ]
    )
    bottle.visual(
        _mesh("spray_bottle_body", bottle_body),
        material=bottle_plastic,
        name="body_shell",
    )
    bottle.visual(
        Box((0.074, 0.046, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=bottle_plastic,
        name="base_band",
    )
    bottle.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.203)),
        material=bottle_plastic,
        name="neck_shoulder_ring",
    )
    bottle.visual(
        Cylinder(radius=0.016, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.221)),
        material=bottle_plastic,
        name="neck_finish",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.090, 0.066, 0.238)),
        mass=0.11,
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
    )

    head = model.part("head")
    head_shell = section_loft(
        [
            _yz_section(0.052, 0.030, 0.010, -0.004, z_center=0.026),
            _yz_section(0.064, 0.048, 0.015, 0.018, z_center=0.044),
            _yz_section(0.056, 0.054, 0.014, 0.058, z_center=0.052),
            _yz_section(0.032, 0.028, 0.009, 0.094, z_center=0.050),
        ]
    )
    head.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=adapter_paint,
        name="collar_clamp",
    )
    head.visual(
        Box((0.026, 0.052, 0.018)),
        origin=Origin(xyz=(0.006, 0.0, 0.015)),
        material=adapter_paint,
        name="adapter_bridge",
    )
    head.visual(
        _mesh("spray_head_shell", head_shell),
        material=head_paint,
        name="head_shell",
    )
    for side, sign in (("left", -1.0), ("right", 1.0)):
        head.visual(
            Box((0.012, 0.010, 0.018)),
            origin=Origin(xyz=(-0.002, sign * 0.029, 0.010)),
            material=adapter_paint,
            name=f"{side}_adapter_ear",
        )
        head.visual(
            Cylinder(radius=0.003, length=0.014),
            origin=Origin(
                xyz=(-0.002, sign * 0.031, 0.010),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"{side}_adapter_bolt",
        )
        head.visual(
            Box((0.018, 0.010, 0.020)),
            origin=Origin(xyz=(0.014, sign * 0.015, 0.012)),
            material=adapter_paint,
            name=f"{side}_gusset",
        )
        head.visual(
            Box((0.032, 0.006, 0.026)),
            origin=Origin(xyz=(0.032, sign * 0.033, 0.040)),
            material=adapter_paint,
            name=f"{side}_service_hatch",
        )
        for ix, iz, bolt_index in (
            (0.021, 0.030, 0),
            (0.043, 0.030, 1),
            (0.021, 0.050, 2),
            (0.043, 0.050, 3),
        ):
            head.visual(
                Cylinder(radius=0.0024, length=0.008),
                origin=Origin(
                    xyz=(ix, sign * 0.036, iz),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=steel,
                name=f"{side}_hatch_bolt_{bolt_index}",
            )

    head.visual(
        Box((0.008, 0.024, 0.020)),
        origin=Origin(xyz=(-0.014, 0.0, 0.015)),
        material=adapter_paint,
        name="rear_guide_block",
    )
    head.visual(
        Box((0.060, 0.004, 0.010)),
        origin=Origin(xyz=(0.020, -0.013, 0.015)),
        material=head_paint,
        name="left_guide_rail",
    )
    head.visual(
        Box((0.060, 0.004, 0.010)),
        origin=Origin(xyz=(0.020, 0.013, 0.015)),
        material=head_paint,
        name="right_guide_rail",
    )
    head.visual(
        Box((0.060, 0.022, 0.004)),
        origin=Origin(xyz=(0.020, 0.0, 0.006)),
        material=head_paint,
        name="guide_top_bridge",
    )
    head.visual(
        Box((0.030, 0.048, 0.006)),
        origin=Origin(xyz=(0.045, 0.0, 0.020)),
        material=adapter_paint,
        name="pivot_bridge",
    )
    head.visual(
        Box((0.012, 0.008, 0.018)),
        origin=Origin(xyz=(0.054, -0.020, -0.006)),
        material=adapter_paint,
        name="left_pivot_lug",
    )
    head.visual(
        Box((0.012, 0.008, 0.018)),
        origin=Origin(xyz=(0.054, 0.020, -0.006)),
        material=adapter_paint,
        name="right_pivot_lug",
    )
    head.visual(
        Box((0.026, 0.006, 0.040)),
        origin=Origin(xyz=(0.042, -0.020, 0.008)),
        material=adapter_paint,
        name="left_pivot_rib",
    )
    head.visual(
        Box((0.026, 0.006, 0.040)),
        origin=Origin(xyz=(0.042, 0.020, 0.008)),
        material=adapter_paint,
        name="right_pivot_rib",
    )
    head.visual(
        Box((0.018, 0.018, 0.020)),
        origin=Origin(xyz=(0.094, 0.0, 0.036)),
        material=adapter_paint,
        name="nozzle_brace",
    )
    head.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.106, 0.0, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=head_paint,
        name="nozzle_barrel",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.127, 0.0, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="nozzle_tip",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.140, 0.070, 0.080)),
        mass=0.14,
        origin=Origin(xyz=(0.040, 0.0, 0.030)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.040, 0.024, 0.048)),
        origin=Origin(xyz=(0.024, 0.0, -0.024)),
        material=steel,
        name="trigger_strap",
    )
    trigger.visual(
        Cylinder(radius=0.0065, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    trigger.visual(
        Box((0.016, 0.018, 0.008)),
        origin=Origin(xyz=(0.006, 0.0, 0.014)),
        material=steel,
        name="actuator_shoe",
    )
    trigger.visual(
        Box((0.012, 0.018, 0.024)),
        origin=Origin(xyz=(0.002, 0.0, 0.002)),
        material=steel,
        name="actuator_web",
    )
    trigger.visual(
        Box((0.020, 0.022, 0.012)),
        origin=Origin(xyz=(0.052, 0.0, -0.040)),
        material=dark_rubber,
        name="finger_pad",
    )
    trigger.visual(
        Box((0.022, 0.004, 0.036)),
        origin=Origin(xyz=(0.014, -0.012, -0.010)),
        material=steel,
        name="left_link_plate",
    )
    trigger.visual(
        Box((0.022, 0.004, 0.036)),
        origin=Origin(xyz=(0.014, 0.012, -0.010)),
        material=steel,
        name="right_link_plate",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.060, 0.030, 0.075)),
        mass=0.05,
        origin=Origin(xyz=(0.028, 0.0, -0.016)),
    )

    pump = model.part("pump_plunger")
    pump.visual(
        Box((0.010, 0.016, 0.010)),
        material=steel,
        name="pump_pad",
    )
    pump.visual(
        Box((0.022, 0.010, 0.010)),
        origin=Origin(xyz=(0.016, 0.0, 0.014)),
        material=steel,
        name="pump_rod",
    )
    pump.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.030, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="pump_rear_seal",
    )
    pump.visual(
        Box((0.010, 0.010, 0.024)),
        origin=Origin(xyz=(0.010, 0.0, 0.007)),
        material=steel,
        name="pump_link_web",
    )
    pump.inertial = Inertial.from_geometry(
        Box((0.040, 0.018, 0.028)),
        mass=0.025,
        origin=Origin(xyz=(0.015, 0.0, 0.009)),
    )

    model.articulation(
        "bottle_to_head",
        ArticulationType.FIXED,
        parent=bottle,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
    )
    model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head,
        child=trigger,
        origin=Origin(xyz=(0.054, 0.0, -0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=0.62,
        ),
    )
    model.articulation(
        "head_to_pump",
        ArticulationType.PRISMATIC,
        parent=head,
        child=pump,
        origin=Origin(xyz=(0.068, 0.0, 0.002)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.10,
            lower=0.0,
            upper=0.010,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    head = object_model.get_part("head")
    trigger = object_model.get_part("trigger")
    pump = object_model.get_part("pump_plunger")
    trigger_joint = object_model.get_articulation("head_to_trigger")
    pump_joint = object_model.get_articulation("head_to_pump")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        head,
        bottle,
        elem_a="collar_clamp",
        elem_b="neck_finish",
        name="head_clamp_seats_on_bottle_neck",
    )
    ctx.expect_overlap(
        head,
        bottle,
        axes="xy",
        elem_a="collar_clamp",
        elem_b="neck_finish",
        min_overlap=0.025,
        name="head_clamp_covers_neck_finish",
    )
    ctx.expect_contact(
        trigger,
        head,
        elem_a="pivot_barrel",
        elem_b="left_pivot_lug",
        name="trigger_barrel_supported_by_left_lug",
    )
    ctx.expect_contact(
        trigger,
        head,
        elem_a="pivot_barrel",
        elem_b="right_pivot_lug",
        name="trigger_barrel_supported_by_right_lug",
    )

    with ctx.pose({trigger_joint: 0.58}):
        ctx.expect_contact(
            trigger,
            pump,
            elem_a="actuator_shoe",
            elem_b="pump_pad",
            contact_tol=0.0015,
            name="trigger_actuator_reaches_pump_pad_when_squeezed",
        )
        ctx.expect_overlap(
            pump,
            head,
            axes="yz",
            min_overlap=0.012,
            name="pump_plunger_stays_inside_guide_envelope",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_finger = _aabb_center(ctx.part_element_world_aabb(trigger, elem="finger_pad"))
    rest_pump = _aabb_center(ctx.part_element_world_aabb(pump, elem="pump_pad"))
    with ctx.pose({trigger_joint: 0.58}):
        squeezed_finger = _aabb_center(ctx.part_element_world_aabb(trigger, elem="finger_pad"))
    with ctx.pose({pump_joint: 0.010}):
        retracted_pump = _aabb_center(ctx.part_element_world_aabb(pump, elem="pump_pad"))

    ctx.check(
        "positive_trigger_motion_pulls_finger_rearward",
        (
            rest_finger is not None
            and squeezed_finger is not None
            and squeezed_finger[0] < rest_finger[0] - 0.010
        ),
        details=f"rest={rest_finger}, squeezed={squeezed_finger}",
    )
    ctx.check(
        "positive_pump_motion_retracts_plunger_backward",
        (
            rest_pump is not None
            and retracted_pump is not None
            and retracted_pump[0] < rest_pump[0] - 0.008
        ),
        details=f"rest={rest_pump}, retracted={retracted_pump}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
