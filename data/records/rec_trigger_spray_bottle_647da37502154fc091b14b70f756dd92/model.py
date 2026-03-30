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
    tube_from_spline_points,
)


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
    ]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_trigger_spray_bottle")

    bottle_white = model.material("bottle_white", rgba=(0.92, 0.94, 0.96, 1.0))
    plastic_charcoal = model.material("plastic_charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.11, 0.12, 1.0))
    nozzle_grey = model.material("nozzle_grey", rgba=(0.62, 0.66, 0.70, 1.0))
    linkage_grey = model.material("linkage_grey", rgba=(0.52, 0.55, 0.58, 1.0))

    bottle_body = model.part("bottle_body")
    bottle_shell = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.086, 0.056, 0.010, 0.000),
                _xy_section(0.092, 0.060, 0.013, 0.016),
                _xy_section(0.100, 0.068, 0.016, 0.070),
                _xy_section(0.096, 0.066, 0.016, 0.112),
                _xy_section(0.082, 0.056, 0.014, 0.136),
                _xy_section(0.056, 0.040, 0.010, 0.148),
            ]
        ),
        "compact_bottle_shell",
    )
    bottle_body.visual(bottle_shell, material=bottle_white, name="bottle_shell")
    bottle_body.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
        material=bottle_white,
        name="shoulder_ring",
    )
    bottle_body.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.147)),
        material=bottle_white,
        name="neck_mount",
    )
    bottle_body.inertial = Inertial.from_geometry(
        Box((0.100, 0.068, 0.154)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
    )

    head_housing = model.part("head_housing")
    head_shell = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.022, 0.026, 0.007, -0.006, 0.060),
                _yz_section(0.040, 0.042, 0.010, 0.020, 0.061),
                _yz_section(0.038, 0.036, 0.009, 0.048, 0.060),
                _yz_section(0.024, 0.020, 0.006, 0.076, 0.058),
            ]
        ),
        "trigger_head_shell",
    )
    trigger_guard_right = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.044, 0.0105, 0.018),
                (0.038, 0.0105, 0.010),
                (0.022, 0.0105, 0.002),
                (0.004, 0.0105, 0.004),
                (-0.012, 0.0105, 0.018),
            ],
            radius=0.0024,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
        "trigger_guard_right",
    )
    trigger_guard_left = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.044, -0.0105, 0.018),
                (0.038, -0.0105, 0.010),
                (0.022, -0.0105, 0.002),
                (0.004, -0.0105, 0.004),
                (-0.012, -0.0105, 0.018),
            ],
            radius=0.0024,
            samples_per_segment=18,
            radial_segments=16,
            cap_ends=True,
        ),
        "trigger_guard_left",
    )
    head_housing.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=plastic_charcoal,
        name="collar",
    )
    head_housing.visual(head_shell, material=plastic_charcoal, name="head_shell")
    head_housing.visual(
        Box((0.008, 0.024, 0.038)),
        origin=Origin(xyz=(-0.014, 0.0, 0.019)),
        material=plastic_charcoal,
        name="rear_bridge",
    )
    head_housing.visual(
        Box((0.020, 0.020, 0.022)),
        origin=Origin(xyz=(-0.002, 0.0, 0.048)),
        material=plastic_charcoal,
        name="rear_cap",
    )
    head_housing.visual(
        Box((0.024, 0.018, 0.010)),
        origin=Origin(xyz=(0.068, 0.0, 0.045)),
        material=plastic_charcoal,
        name="pump_roof",
    )
    head_housing.visual(
        Box((0.014, 0.006, 0.022)),
        origin=Origin(xyz=(0.052, 0.019, 0.033)),
        material=plastic_charcoal,
        name="pivot_cheek_right",
    )
    head_housing.visual(
        Box((0.014, 0.006, 0.022)),
        origin=Origin(xyz=(0.052, -0.019, 0.033)),
        material=plastic_charcoal,
        name="pivot_cheek_left",
    )
    head_housing.visual(
        Box((0.018, 0.018, 0.012)),
        origin=Origin(xyz=(0.072, 0.0, 0.042)),
        material=plastic_charcoal,
        name="nose_block",
    )
    head_housing.visual(
        trigger_guard_right,
        material=plastic_charcoal,
        name="trigger_guard_right",
    )
    head_housing.visual(
        trigger_guard_left,
        material=plastic_charcoal,
        name="trigger_guard_left",
    )
    head_housing.visual(
        Cylinder(radius=0.0028, length=0.026),
        origin=Origin(xyz=(0.004, 0.0, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic_charcoal,
        name="trigger_guard_front",
    )
    head_housing.inertial = Inertial.from_geometry(
        Box((0.092, 0.050, 0.070)),
        mass=0.11,
        origin=Origin(xyz=(0.030, 0.0, 0.035)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.0048, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="pivot_barrel",
    )
    trigger.visual(
        Box((0.012, 0.010, 0.030)),
        origin=Origin(xyz=(-0.002, 0.0, -0.015)),
        material=soft_black,
        name="lever_web",
    )
    trigger.visual(
        Box((0.018, 0.010, 0.010)),
        origin=Origin(xyz=(-0.016, 0.0, -0.028), rpy=(0.0, 0.10, 0.0)),
        material=soft_black,
        name="finger_pad",
    )
    trigger.visual(
        Box((0.010, 0.004, 0.006)),
        origin=Origin(xyz=(0.005, 0.0, -0.010)),
        material=linkage_grey,
        name="linkage_stem",
    )
    trigger.visual(
        Box((0.012, 0.004, 0.004)),
        origin=Origin(xyz=(0.012, 0.0, -0.009)),
        material=linkage_grey,
        name="linkage_bar",
    )
    trigger.visual(
        Box((0.004, 0.006, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, -0.009)),
        material=linkage_grey,
        name="linkage_tip",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.042, 0.020, 0.044)),
        mass=0.024,
        origin=Origin(xyz=(-0.004, 0.0, -0.016)),
    )

    pump_plunger = model.part("pump_plunger")
    pump_plunger.visual(
        Box((0.004, 0.006, 0.016)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=nozzle_grey,
        name="plunger_pad",
    )
    pump_plunger.visual(
        Cylinder(radius=0.0025, length=0.018),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_grey,
        name="plunger_rod",
    )
    pump_plunger.inertial = Inertial.from_geometry(
        Box((0.022, 0.008, 0.018)),
        mass=0.012,
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
    )

    nozzle_spout = model.part("nozzle_spout")
    nozzle_spout.visual(
        Box((0.026, 0.010, 0.008)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material=nozzle_grey,
        name="spout_body",
    )
    nozzle_spout.visual(
        Cylinder(radius=0.0035, length=0.010),
        origin=Origin(xyz=(0.029, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_grey,
        name="nozzle_tip",
    )
    nozzle_spout.inertial = Inertial.from_geometry(
        Box((0.036, 0.010, 0.010)),
        mass=0.010,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    model.articulation(
        "bottle_to_head",
        ArticulationType.FIXED,
        parent=bottle_body,
        child=head_housing,
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
    )
    model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head_housing,
        child=trigger,
        origin=Origin(xyz=(0.050, 0.0, 0.032)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=0.46,
        ),
    )
    model.articulation(
        "head_to_pump",
        ArticulationType.PRISMATIC,
        parent=head_housing,
        child=pump_plunger,
        origin=Origin(xyz=(0.072, 0.0, 0.021)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.10,
            lower=0.0,
            upper=0.0045,
        ),
    )
    model.articulation(
        "head_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=head_housing,
        child=nozzle_spout,
        origin=Origin(xyz=(0.081, 0.0, 0.046)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle_body")
    head = object_model.get_part("head_housing")
    trigger = object_model.get_part("trigger")
    plunger = object_model.get_part("pump_plunger")
    nozzle = object_model.get_part("nozzle_spout")

    trigger_joint = object_model.get_articulation("head_to_trigger")
    pump_joint = object_model.get_articulation("head_to_pump")
    nozzle_joint = object_model.get_articulation("head_to_nozzle")

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

    with ctx.pose({trigger_joint: 0.0, pump_joint: 0.0, nozzle_joint: 0.0}):
        ctx.expect_contact(
            head,
            bottle,
            elem_a="collar",
            elem_b="neck_mount",
            contact_tol=0.0012,
            name="head_seats_on_bottle",
        )
        ctx.expect_overlap(
            head,
            bottle,
            axes="xy",
            min_overlap=0.020,
            name="head_centered_over_bottle",
        )
        ctx.expect_overlap(
            trigger,
            plunger,
            axes="yz",
            min_overlap=0.004,
            elem_a="linkage_tip",
            elem_b="plunger_pad",
            name="rest_linkage_aligned_with_plunger",
        )
        ctx.expect_gap(
            plunger,
            trigger,
            axis="x",
            max_gap=0.0015,
            max_penetration=0.0002,
            positive_elem="plunger_pad",
            negative_elem="linkage_tip",
            name="rest_linkage_touches_plunger",
        )

    open_pad_aabb = ctx.part_element_world_aabb(trigger, elem="finger_pad")
    open_plunger_aabb = ctx.part_element_world_aabb(plunger, elem="plunger_pad")
    open_nozzle_aabb = ctx.part_world_aabb(nozzle)

    with ctx.pose(
        {
            trigger_joint: math.radians(24.0),
            pump_joint: 0.0045,
            nozzle_joint: 0.0,
        }
    ):
        squeezed_pad_aabb = ctx.part_element_world_aabb(trigger, elem="finger_pad")
        squeezed_plunger_aabb = ctx.part_element_world_aabb(plunger, elem="plunger_pad")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_when_trigger_squeezed")
        ctx.expect_overlap(
            trigger,
            plunger,
            axes="yz",
            min_overlap=0.004,
            elem_a="linkage_tip",
            elem_b="plunger_pad",
            name="squeezed_linkage_aligned_with_plunger",
        )
        ctx.expect_gap(
            plunger,
            trigger,
            axis="x",
            max_gap=0.0015,
            max_penetration=0.0010,
            positive_elem="plunger_pad",
            negative_elem="linkage_tip",
            name="trigger_stroke_drives_pump",
        )

    ctx.check(
        "trigger_squeezes_backward",
        (
            open_pad_aabb is not None
            and squeezed_pad_aabb is not None
            and squeezed_pad_aabb[1][0] < open_pad_aabb[1][0] - 0.008
        ),
        details="Finger pad should move rearward when the trigger is squeezed.",
    )
    ctx.check(
        "pump_retracts_under_trigger_stroke",
        (
            open_plunger_aabb is not None
            and squeezed_plunger_aabb is not None
            and squeezed_plunger_aabb[1][0] < open_plunger_aabb[1][0] - 0.003
        ),
        details="Pump plunger pad should retract rearward during the trigger stroke.",
    )

    with ctx.pose({nozzle_joint: math.radians(66.0)}):
        stowed_nozzle_aabb = ctx.part_world_aabb(nozzle)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_when_nozzle_stowed")

    ctx.check(
        "nozzle_folds_to_compact_envelope",
        (
            open_nozzle_aabb is not None
            and stowed_nozzle_aabb is not None
            and stowed_nozzle_aabb[1][0] < open_nozzle_aabb[1][0] - 0.010
        ),
        details="Folded nozzle should materially reduce the forward storage envelope.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
