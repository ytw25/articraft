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
    superellipse_profile,
)


def _bottle_section(width: float, depth: float, z: float, *, exponent: float = 3.0):
    return [(x, y, z) for x, y in superellipse_profile(width, depth, exponent=exponent, segments=40)]


def _head_section(width: float, height: float, x: float, *, z_center: float, radius: float):
    return [
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_trigger_spray_bottle")

    bottle_plastic = model.material("bottle_plastic", rgba=(0.80, 0.88, 0.96, 0.35))
    head_plastic = model.material("head_plastic", rgba=(0.87, 0.89, 0.90, 1.0))
    trigger_plastic = model.material("trigger_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    mechanism_metal = model.material("mechanism_metal", rgba=(0.70, 0.73, 0.76, 1.0))
    accent = model.material("accent", rgba=(0.86, 0.30, 0.12, 1.0))

    bottle = model.part("bottle")
    bottle_shell = section_loft(
        [
            _bottle_section(0.084, 0.058, 0.000, exponent=3.2),
            _bottle_section(0.102, 0.070, 0.090, exponent=3.0),
            _bottle_section(0.112, 0.078, 0.190, exponent=2.9),
            _bottle_section(0.104, 0.072, 0.275, exponent=2.9),
            _bottle_section(0.076, 0.052, 0.315, exponent=2.7),
        ]
    )
    bottle.visual(
        mesh_from_geometry(bottle_shell, "bottle_shell"),
        material=bottle_plastic,
        name="bottle_shell",
    )
    bottle.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.309)),
        material=bottle_plastic,
        name="shoulder_ring",
    )
    bottle.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=bottle_plastic,
        name="neck_stem",
    )
    bottle.visual(
        Box((0.032, 0.006, 0.090)),
        origin=Origin(xyz=(0.048, 0.0, 0.165)),
        material=bottle_plastic,
        name="front_alignment_flat",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.112, 0.078, 0.375)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.1875)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.028, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=head_plastic,
        name="cap_pedestal",
    )
    head.visual(
        Box((0.040, 0.028, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, 0.014)),
        material=head_plastic,
        name="rear_bridge",
    )
    head.visual(
        Box((0.074, 0.008, 0.062)),
        origin=Origin(xyz=(0.060, 0.018, -0.020)),
        material=head_plastic,
        name="left_guard_plate",
    )
    head.visual(
        Box((0.074, 0.008, 0.062)),
        origin=Origin(xyz=(0.060, -0.018, -0.020)),
        material=head_plastic,
        name="right_guard_plate",
    )
    head.visual(
        Box((0.022, 0.036, 0.010)),
        origin=Origin(xyz=(0.108, 0.0, -0.046)),
        material=head_plastic,
        name="guard_bar",
    )
    head.visual(
        Box((0.098, 0.010, 0.022)),
        origin=Origin(xyz=(0.070, 0.019, 0.020)),
        material=head_plastic,
        name="left_spine",
    )
    head.visual(
        Box((0.098, 0.010, 0.022)),
        origin=Origin(xyz=(0.070, -0.019, 0.020)),
        material=head_plastic,
        name="right_spine",
    )
    head.visual(
        Box((0.048, 0.044, 0.014)),
        origin=Origin(xyz=(0.106, 0.0, 0.049)),
        material=head_plastic,
        name="head_shell",
    )
    head.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(0.074, 0.013, 0.002)),
        material=head_plastic,
        name="left_pivot_cheek",
    )
    head.visual(
        Box((0.010, 0.008, 0.020)),
        origin=Origin(xyz=(0.074, -0.013, 0.002)),
        material=head_plastic,
        name="right_pivot_cheek",
    )
    head.visual(
        Box((0.028, 0.010, 0.018)),
        origin=Origin(xyz=(0.056, 0.011, 0.040)),
        material=head_plastic,
        name="left_guide_rail",
    )
    head.visual(
        Box((0.028, 0.010, 0.018)),
        origin=Origin(xyz=(0.056, -0.011, 0.040)),
        material=head_plastic,
        name="right_guide_rail",
    )
    head.visual(
        Box((0.032, 0.036, 0.022)),
        origin=Origin(xyz=(0.118, 0.0, 0.030)),
        material=head_plastic,
        name="front_block",
    )
    head.visual(
        Box((0.010, 0.024, 0.018)),
        origin=Origin(xyz=(0.129, 0.0, 0.030)),
        material=head_plastic,
        name="nozzle_stop",
    )
    head.visual(
        Box((0.026, 0.010, 0.010)),
        origin=Origin(xyz=(0.090, 0.027, 0.044)),
        material=head_plastic,
        name="left_datum_pad",
    )
    head.visual(
        Box((0.026, 0.010, 0.010)),
        origin=Origin(xyz=(0.090, -0.027, 0.044)),
        material=head_plastic,
        name="right_datum_pad",
    )
    head.visual(
        Box((0.040, 0.022, 0.006)),
        origin=Origin(xyz=(0.088, 0.0, 0.059)),
        material=head_plastic,
        name="top_datum_pad",
    )
    head.visual(
        Box((0.003, 0.006, 0.010)),
        origin=Origin(xyz=(0.129, 0.0, 0.041)),
        material=accent,
        name="head_index_mark",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.150, 0.070, 0.135)),
        mass=0.11,
        origin=Origin(xyz=(0.050, 0.0, 0.000)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trigger_plastic,
        name="pivot_barrel",
    )
    trigger.visual(
        Box((0.014, 0.020, 0.018)),
        origin=Origin(xyz=(0.004, 0.0, -0.008)),
        material=trigger_plastic,
        name="rear_link_web",
    )
    trigger.visual(
        Box((0.020, 0.024, 0.048)),
        origin=Origin(xyz=(0.012, 0.0, -0.036)),
        material=trigger_plastic,
        name="finger_blade",
    )
    trigger.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.016, 0.0, -0.062), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trigger_plastic,
        name="finger_toe",
    )
    trigger.visual(
        Box((0.014, 0.018, 0.014)),
        origin=Origin(xyz=(0.010, 0.0, 0.006)),
        material=trigger_plastic,
        name="pivot_web",
    )
    trigger.visual(
        Box((0.032, 0.012, 0.008)),
        origin=Origin(xyz=(0.024, 0.0, 0.017)),
        material=trigger_plastic,
        name="upper_link",
    )
    trigger.visual(
        Box((0.012, 0.016, 0.006)),
        origin=Origin(xyz=(0.038, 0.0, 0.020)),
        material=accent,
        name="cam_pad",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.070, 0.024, 0.105)),
        mass=0.05,
        origin=Origin(xyz=(0.000, 0.0, -0.012)),
    )

    pump_plunger = model.part("pump_plunger")
    pump_plunger.visual(
        Box((0.014, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=mechanism_metal,
        name="guide_block",
    )
    pump_plunger.visual(
        Box((0.010, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=mechanism_metal,
        name="stem",
    )
    pump_plunger.visual(
        Cylinder(radius=0.004, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=mechanism_metal,
        name="plunger_rod",
    )
    pump_plunger.visual(
        Box((0.014, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=mechanism_metal,
        name="yoke",
    )
    pump_plunger.visual(
        Box((0.012, 0.010, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, -0.026)),
        material=mechanism_metal,
        name="actuator_arm",
    )
    pump_plunger.visual(
        Box((0.016, 0.012, 0.006)),
        origin=Origin(xyz=(0.020, 0.0, -0.023)),
        material=mechanism_metal,
        name="actuator_foot",
    )
    pump_plunger.inertial = Inertial.from_geometry(
        Box((0.016, 0.016, 0.032)),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    nozzle_collar = model.part("nozzle_collar")
    nozzle_collar.visual(
        Box((0.004, 0.022, 0.016)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=head_plastic,
        name="collar_flange",
    )
    nozzle_collar.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=head_plastic,
        name="collar_body",
    )
    nozzle_collar.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=head_plastic,
        name="tip_nose",
    )
    nozzle_collar.visual(
        Box((0.006, 0.004, 0.004)),
        origin=Origin(xyz=(0.002, 0.0, 0.010)),
        material=accent,
        name="collar_index_rib",
    )
    nozzle_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=0.026),
        mass=0.01,
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "bottle_to_head",
        ArticulationType.FIXED,
        parent=bottle,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.375)),
    )
    model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head,
        child=trigger,
        origin=Origin(xyz=(0.074, 0.0, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=0.20,
        ),
    )
    model.articulation(
        "head_to_pump_plunger",
        ArticulationType.PRISMATIC,
        parent=head,
        child=pump_plunger,
        origin=Origin(xyz=(0.056, 0.0, 0.046)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.08,
            lower=0.0,
            upper=0.006,
        ),
    )
    model.articulation(
        "head_to_nozzle_collar",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=nozzle_collar,
        origin=Origin(xyz=(0.134, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    head = object_model.get_part("head")
    trigger = object_model.get_part("trigger")
    pump_plunger = object_model.get_part("pump_plunger")
    nozzle_collar = object_model.get_part("nozzle_collar")

    trigger_hinge = object_model.get_articulation("head_to_trigger")
    plunger_slide = object_model.get_articulation("head_to_pump_plunger")
    nozzle_adjust = object_model.get_articulation("head_to_nozzle_collar")

    ctx.allow_overlap(
        bottle,
        head,
        elem_a="neck_stem",
        elem_b="cap_pedestal",
        reason="The trigger head intentionally nests over the bottle neck as a seated threaded press-fit interface.",
    )

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

    with ctx.pose({trigger_hinge: 0.0, plunger_slide: 0.0, nozzle_adjust: 0.0}):
        ctx.expect_contact(
            bottle,
            head,
            elem_a="neck_stem",
            elem_b="cap_pedestal",
            name="head_seats_on_bottle_neck",
        )
        ctx.expect_contact(
            head,
            trigger,
            elem_a="left_pivot_cheek",
            elem_b="pivot_barrel",
            name="trigger_left_pivot_support_contact",
        )
        ctx.expect_contact(
            head,
            trigger,
            elem_a="right_pivot_cheek",
            elem_b="pivot_barrel",
            name="trigger_right_pivot_support_contact",
        )
        ctx.expect_contact(
            head,
            pump_plunger,
            elem_a="left_guide_rail",
            elem_b="guide_block",
            contact_tol=0.0011,
            name="plunger_left_guide_guidance",
        )
        ctx.expect_contact(
            head,
            pump_plunger,
            elem_a="right_guide_rail",
            elem_b="guide_block",
            contact_tol=0.0011,
            name="plunger_right_guide_guidance",
        )
        ctx.expect_contact(
            head,
            nozzle_collar,
            elem_a="nozzle_stop",
            elem_b="collar_flange",
            name="nozzle_collar_mount_contact",
        )
        ctx.expect_contact(
            pump_plunger,
            trigger,
            elem_a="actuator_foot",
            elem_b="upper_link",
            contact_tol=0.0011,
            name="rest_linkage_contact",
        )
        ctx.expect_gap(
            head,
            trigger,
            axis="z",
            positive_elem="guard_bar",
            negative_elem="finger_toe",
            min_gap=0.001,
            name="rest_trigger_guard_clearance",
        )

    with ctx.pose({trigger_hinge: 0.20, plunger_slide: 0.006}):
        ctx.expect_gap(
            head,
            pump_plunger,
            axis="z",
            positive_elem="top_datum_pad",
            negative_elem="guide_block",
            min_gap=0.001,
            name="squeezed_plunger_below_top_datum",
        )
        ctx.expect_contact(
            pump_plunger,
            trigger,
            elem_a="actuator_foot",
            elem_b="upper_link",
            contact_tol=0.0011,
            name="squeezed_linkage_contact",
        )
        ctx.expect_gap(
            head,
            trigger,
            axis="z",
            positive_elem="guard_bar",
            negative_elem="finger_toe",
            min_gap=0.0008,
            name="squeezed_trigger_guard_clearance",
        )
        ctx.expect_overlap(
            trigger,
            bottle,
            axes="yz",
            min_overlap=0.010,
            name="trigger_stays_in_front_of_bottle_envelope",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
