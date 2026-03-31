from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = geometries[0].copy()
    for geometry in geometries[1:]:
        merged.merge(geometry)
    return merged


def _annular_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    axis: str = "z",
    segments: int = 56,
) -> MeshGeometry:
    shell = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -0.5 * length),
            (outer_radius, 0.5 * length),
        ],
        [
            (inner_radius, -0.5 * length),
            (inner_radius, 0.5 * length),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    if axis == "x":
        shell.rotate_y(math.pi / 2.0)
    elif axis == "y":
        shell.rotate_x(math.pi / 2.0)
    return shell


def _build_bottle_shell() -> MeshGeometry:
    outer_profile = [
        (0.014, 0.000),
        (0.034, 0.002),
        (0.038, 0.014),
        (0.039, 0.060),
        (0.039, 0.150),
        (0.036, 0.205),
        (0.028, 0.224),
        (0.021, 0.236),
        (0.0205, 0.242),
        (0.0205, 0.245),
    ]
    inner_profile = [
        (0.000, 0.000),
        (0.029, 0.006),
        (0.033, 0.016),
        (0.034, 0.060),
        (0.034, 0.148),
        (0.031, 0.202),
        (0.023, 0.222),
        (0.017, 0.234),
        (0.0165, 0.242),
        (0.0165, 0.245),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trigger_spray_bottle")

    bottle_resin = model.material("bottle_resin", rgba=(0.87, 0.93, 0.98, 0.42))
    head_plastic = model.material("head_plastic", rgba=(0.93, 0.93, 0.91, 1.0))
    trigger_plastic = model.material("trigger_plastic", rgba=(0.81, 0.83, 0.84, 1.0))
    nozzle_dark = model.material("nozzle_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    tube_natural = model.material("tube_natural", rgba=(0.93, 0.94, 0.90, 1.0))
    plunger_gray = model.material("plunger_gray", rgba=(0.72, 0.74, 0.76, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        mesh_from_geometry(_build_bottle_shell(), "bottle_shell"),
        material=bottle_resin,
        name="bottle_shell",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.262),
        mass=0.055,
        origin=Origin(xyz=(0.0, 0.0, 0.131)),
    )

    head = model.part("head_body")
    head.visual(
        mesh_from_geometry(
            _annular_shell(
                outer_radius=0.0240,
                inner_radius=0.0205,
                length=0.020,
                axis="z",
                segments=64,
            ).translate(0.0, 0.0, 0.010),
            "closure_collar",
        ),
        material=head_plastic,
        name="closure_collar",
    )
    head.visual(
        Box((0.052, 0.026, 0.008)),
        origin=Origin(xyz=(0.022, 0.0, 0.024)),
        material=head_plastic,
        name="deck_plate",
    )
    head.visual(
        Box((0.022, 0.024, 0.022)),
        origin=Origin(xyz=(-0.010, 0.0, 0.017)),
        material=head_plastic,
        name="rear_web",
    )
    head.visual(
        Box((0.014, 0.020, 0.022)),
        origin=Origin(xyz=(0.038, 0.0, 0.039)),
        material=head_plastic,
        name="spine_riser",
    )
    head.visual(
        Box((0.030, 0.024, 0.008)),
        origin=Origin(xyz=(0.067, 0.0, 0.074)),
        material=head_plastic,
        name="bridge_block",
    )
    head.visual(
        Box((0.026, 0.006, 0.032)),
        origin=Origin(xyz=(0.064, 0.015, 0.056)),
        material=head_plastic,
        name="left_cheek",
    )
    head.visual(
        Box((0.026, 0.006, 0.032)),
        origin=Origin(xyz=(0.064, -0.015, 0.056)),
        material=head_plastic,
        name="right_cheek",
    )
    head.visual(
        Box((0.016, 0.004, 0.018)),
        origin=Origin(xyz=(0.048, 0.012, 0.050)),
        material=head_plastic,
        name="left_gusset",
    )
    head.visual(
        Box((0.016, 0.004, 0.018)),
        origin=Origin(xyz=(0.048, -0.012, 0.050)),
        material=head_plastic,
        name="right_gusset",
    )
    head.visual(
        mesh_from_geometry(
            _annular_shell(
                outer_radius=0.0120,
                inner_radius=0.0072,
                length=0.040,
                axis="x",
                segments=48,
            ).translate(0.088, 0.0, 0.058),
            "pump_barrel_shell",
        ),
        material=head_plastic,
        name="pump_barrel_shell",
    )
    head.visual(
        Box((0.018, 0.018, 0.018)),
        origin=Origin(xyz=(0.100, 0.0, 0.058)),
        material=head_plastic,
        name="nozzle_housing",
    )
    head.visual(
        Cylinder(radius=0.0060, length=0.012),
        origin=Origin(xyz=(0.110, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nozzle_dark,
        name="nozzle_tip",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.120, 0.050, 0.085)),
        mass=0.070,
        origin=Origin(xyz=(0.050, 0.0, 0.040)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.0045, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trigger_plastic,
        name="pivot_barrel",
    )
    trigger.visual(
        Box((0.012, 0.010, 0.018)),
        origin=Origin(xyz=(-0.002, 0.0, -0.010)),
        material=trigger_plastic,
        name="link_web",
    )
    trigger.visual(
        Box((0.010, 0.010, 0.038)),
        origin=Origin(xyz=(-0.014, 0.0, -0.037)),
        material=trigger_plastic,
        name="trigger_arm",
    )
    trigger.visual(
        Box((0.010, 0.010, 0.008)),
        origin=Origin(xyz=(-0.009, 0.0, -0.015)),
        material=trigger_plastic,
        name="actuation_pawl",
    )
    trigger.visual(
        Box((0.016, 0.022, 0.010)),
        origin=Origin(xyz=(-0.019, 0.0, -0.060)),
        material=trigger_plastic,
        name="finger_pad",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.050, 0.024, 0.064)),
        mass=0.018,
        origin=Origin(xyz=(-0.010, 0.0, -0.026)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0048, length=0.030),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_gray,
        name="plunger_rod",
    )
    plunger.visual(
        Cylinder(radius=0.0062, length=0.006),
        origin=Origin(xyz=(0.030, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plunger_gray,
        name="piston_head",
    )
    plunger.visual(
        Box((0.006, 0.010, 0.018)),
        origin=Origin(xyz=(0.002, 0.0, -0.013)),
        material=plunger_gray,
        name="stem",
    )
    plunger.visual(
        Box((0.010, 0.012, 0.006)),
        origin=Origin(xyz=(-0.003, 0.0, -0.022)),
        material=plunger_gray,
        name="tail_pad",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.036, 0.018, 0.012)),
        mass=0.008,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    dip_tube = model.part("dip_tube")
    dip_tube.visual(
        Cylinder(radius=0.0022, length=0.204),
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
        material=tube_natural,
        name="dip_tube",
    )
    dip_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0022, length=0.204),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, -0.102)),
    )

    model.articulation(
        "bottle_to_head",
        ArticulationType.FIXED,
        parent=bottle,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
    )
    model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head,
        child=trigger,
        origin=Origin(xyz=(0.068, 0.0, 0.048)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=0.0,
            upper=0.60,
        ),
    )
    model.articulation(
        "head_to_plunger",
        ArticulationType.PRISMATIC,
        parent=head,
        child=plunger,
        origin=Origin(xyz=(0.060, 0.0, 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.010,
        ),
    )
    model.articulation(
        "head_to_dip_tube",
        ArticulationType.FIXED,
        parent=head,
        child=dip_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    head = object_model.get_part("head_body")
    trigger = object_model.get_part("trigger")
    plunger = object_model.get_part("plunger")
    dip_tube = object_model.get_part("dip_tube")
    trigger_joint = object_model.get_articulation("head_to_trigger")
    plunger_joint = object_model.get_articulation("head_to_plunger")

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

    ctx.expect_overlap(head, bottle, axes="xy", min_overlap=0.035, name="head_centered_over_bottle")
    ctx.expect_gap(head, bottle, axis="z", max_gap=0.0015, max_penetration=0.0, name="head_seats_on_bottle")
    ctx.expect_contact(dip_tube, head, name="dip_tube_seated_in_head")
    ctx.expect_within(dip_tube, bottle, axes="xy", margin=0.003, name="dip_tube_stays_inside_bottle")
    ctx.expect_contact(trigger, head, name="trigger_pivot_captured")

    with ctx.pose({trigger_joint: 0.58, plunger_joint: 0.010}):
        ctx.expect_contact(
            trigger,
            plunger,
            name="trigger_actuates_plunger",
        )

    def _aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    with ctx.pose({trigger_joint: 0.0}):
        finger_rest = _aabb_center(ctx.part_element_world_aabb(trigger, elem="finger_pad"))
    with ctx.pose({trigger_joint: 0.58}):
        finger_squeezed = _aabb_center(ctx.part_element_world_aabb(trigger, elem="finger_pad"))
    ctx.check(
        "trigger_moves_back_and_up",
        (finger_squeezed[0] < finger_rest[0] - 0.012) and (finger_squeezed[2] > finger_rest[2] + 0.010),
        details=(
            f"finger pad rest={finger_rest}, squeezed={finger_squeezed}; "
            "expected rearward and upward squeeze arc"
        ),
    )

    with ctx.pose({plunger_joint: 0.0}):
        pad_rest = _aabb_center(ctx.part_element_world_aabb(plunger, elem="tail_pad"))
    with ctx.pose({plunger_joint: 0.010}):
        pad_driven = _aabb_center(ctx.part_element_world_aabb(plunger, elem="tail_pad"))
    ctx.check(
        "plunger_moves_forward",
        pad_driven[0] > pad_rest[0] + 0.0085,
        details=f"tail pad rest={pad_rest}, driven={pad_driven}; expected forward pump stroke",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
