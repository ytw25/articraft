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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xy_rounded_section(
    z: float,
    width: float,
    depth: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, corner_radius)]


def _yz_superellipse_section(
    x: float,
    width: float,
    height: float,
    z_center: float,
    *,
    exponent: float = 2.7,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for y, z in superellipse_profile(width, height, exponent=exponent, segments=segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_tilt_head_mixer")

    body_enamel = model.material("body_enamel", rgba=(0.86, 0.88, 0.84, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.88, 0.89, 0.91, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.16, 0.17, 1.0))

    base = model.part("base")

    base_plate = ExtrudeGeometry(rounded_rect_profile(0.36, 0.24, 0.05), 0.042)
    base.visual(
        _mesh("mixer_base_plate", base_plate),
        origin=Origin(xyz=(0.035, 0.0, 0.021)),
        material=body_enamel,
        name="base_plate",
    )

    lower_body = Box((0.124, 0.130, 0.110))
    base.visual(
        lower_body,
        origin=Origin(xyz=(-0.097, 0.0, 0.076)),
        material=body_enamel,
        name="lower_body",
    )
    upper_body = Box((0.095, 0.100, 0.070))
    base.visual(
        upper_body,
        origin=Origin(xyz=(-0.092, 0.0, 0.166)),
        material=body_enamel,
        name="upper_body",
    )
    base.visual(
        Box((0.060, 0.086, 0.020)),
        origin=Origin(xyz=(-0.090, 0.0, 0.198)),
        material=body_enamel,
        name="shoulder_bridge",
    )
    base.visual(
        Box((0.050, 0.028, 0.046)),
        origin=Origin(xyz=(-0.090, -0.045, 0.231)),
        material=body_enamel,
        name="left_hinge_cheek",
    )
    base.visual(
        Box((0.050, 0.028, 0.046)),
        origin=Origin(xyz=(-0.090, 0.045, 0.231)),
        material=body_enamel,
        name="right_hinge_cheek",
    )
    base.visual(
        Box((0.180, 0.092, 0.016)),
        origin=Origin(xyz=(0.055, 0.0, 0.034)),
        material=body_enamel,
        name="front_support",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.012),
        origin=Origin(xyz=(0.125, 0.0, 0.048)),
        material=body_enamel,
        name="bowl_pad",
    )
    base.visual(
        Box((0.110, 0.020, 0.034)),
        origin=Origin(xyz=(0.065, -0.122, 0.059)),
        material=body_enamel,
        name="left_cradle_arm",
    )
    base.visual(
        Box((0.110, 0.020, 0.034)),
        origin=Origin(xyz=(0.065, 0.122, 0.059)),
        material=body_enamel,
        name="right_cradle_arm",
    )
    base.visual(
        Box((0.100, 0.018, 0.028)),
        origin=Origin(xyz=(0.100, -0.132, 0.090)),
        material=body_enamel,
        name="left_side_rail",
    )
    base.visual(
        Box((0.100, 0.018, 0.028)),
        origin=Origin(xyz=(0.100, 0.132, 0.090)),
        material=body_enamel,
        name="right_side_rail",
    )
    base.visual(
        Box((0.030, 0.026, 0.056)),
        origin=Origin(xyz=(0.150, -0.134, 0.122)),
        material=body_enamel,
        name="left_ear",
    )
    base.visual(
        Box((0.030, 0.026, 0.056)),
        origin=Origin(xyz=(0.150, 0.134, 0.122)),
        material=body_enamel,
        name="right_ear",
    )
    base.visual(
        Box((0.018, 0.008, 0.020)),
        origin=Origin(xyz=(0.144, -0.127, 0.124)),
        material=dark_trim,
        name="left_ear_pad",
    )
    base.visual(
        Box((0.018, 0.008, 0.020)),
        origin=Origin(xyz=(0.144, 0.127, 0.124)),
        material=dark_trim,
        name="right_ear_pad",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.036),
        origin=Origin(xyz=(-0.090, -0.045, 0.267), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="left_hinge_barrel",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.036),
        origin=Origin(xyz=(-0.090, 0.045, 0.267), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="right_hinge_barrel",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.40, 0.26, 0.28)),
        mass=8.5,
        origin=Origin(xyz=(0.015, 0.0, 0.14)),
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        [
            (0.034, 0.0),
            (0.074, 0.010),
            (0.097, 0.050),
            (0.108, 0.104),
            (0.112, 0.135),
            (0.110, 0.146),
        ],
        [
            (0.0, 0.005),
            (0.068, 0.014),
            (0.091, 0.050),
            (0.102, 0.104),
            (0.104, 0.140),
        ],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=10,
    )
    bowl.visual(
        _mesh("mixer_bowl_shell", bowl_shell),
        material=bright_steel,
        name="bowl_shell",
    )
    bowl.visual(
        Cylinder(radius=0.010, length=0.038),
        origin=Origin(xyz=(0.0, -0.108, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="left_trunnion",
    )
    bowl.visual(
        Cylinder(radius=0.010, length=0.038),
        origin=Origin(xyz=(0.0, 0.108, 0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="right_trunnion",
    )
    bowl.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.150),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.125, 0.0, 0.054)),
    )

    head = model.part("head")
    head.visual(
        Box((0.050, 0.030, 0.040)),
        origin=Origin(xyz=(0.025, 0.0, 0.024)),
        material=body_enamel,
        name="rear_yoke",
    )
    head.visual(
        Box((0.024, 0.040, 0.032)),
        origin=Origin(xyz=(0.050, 0.0, 0.028)),
        material=body_enamel,
        name="hinge_bridge",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="center_hinge_barrel",
    )
    head_shell = section_loft(
        [
            _yz_superellipse_section(0.060, 0.102, 0.088, 0.060),
            _yz_superellipse_section(0.120, 0.150, 0.128, 0.070),
            _yz_superellipse_section(0.210, 0.180, 0.152, 0.074),
            _yz_superellipse_section(0.300, 0.164, 0.142, 0.068),
            _yz_superellipse_section(0.355, 0.122, 0.112, 0.056),
            _yz_superellipse_section(0.392, 0.076, 0.082, 0.038),
        ]
    )
    head.visual(
        _mesh("mixer_head_shell", head_shell),
        material=body_enamel,
        name="head_shell",
    )
    head.visual(
        Box((0.100, 0.076, 0.032)),
        origin=Origin(xyz=(0.167, 0.0, 0.012)),
        material=body_enamel,
        name="nose_bridge",
    )
    head.visual(
        Cylinder(radius=0.030, length=0.030),
        origin=Origin(xyz=(0.215, 0.0, -0.006)),
        material=dark_trim,
        name="planetary_nose",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.39, 0.18, 0.17)),
        mass=4.8,
        origin=Origin(xyz=(0.180, 0.0, 0.045)),
    )

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.090, 0.0, 0.267)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(60.0),
        ),
    )

    tool_assembly = model.part("tool_assembly")
    tool_assembly.visual(
        Cylinder(radius=0.014, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=bright_steel,
        name="drive_hub",
    )
    tool_assembly.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=bright_steel,
        name="hub_collar",
    )
    dough_hook = tube_from_spline_points(
        [
            (0.0, 0.0, -0.020),
            (0.0, 0.004, -0.040),
            (0.007, 0.010, -0.064),
            (0.014, 0.008, -0.086),
            (0.017, -0.002, -0.104),
            (0.011, -0.012, -0.118),
            (0.001, -0.014, -0.124),
            (-0.007, -0.008, -0.114),
        ],
        radius=0.0038,
        samples_per_segment=18,
        radial_segments=20,
    )
    tool_assembly.visual(
        _mesh("mixer_dough_hook", dough_hook),
        material=bright_steel,
        name="hook_shell",
    )
    tool_assembly.inertial = Inertial.from_geometry(
        Box((0.08, 0.07, 0.20)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    model.articulation(
        "head_to_tool",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=tool_assembly,
        origin=Origin(xyz=(0.215, 0.0, -0.021)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    bowl_latch = model.part("bowl_latch")
    bowl_latch.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="pivot_barrel",
    )
    bowl_latch.visual(
        Box((0.044, 0.020, 0.010)),
        origin=Origin(xyz=(-0.022, 0.010, 0.0)),
        material=dark_trim,
        name="latch_arm",
    )
    bowl_latch.visual(
        Box((0.010, 0.012, 0.022)),
        origin=Origin(xyz=(-0.046, 0.034, -0.011)),
        material=dark_trim,
        name="latch_tip",
    )
    bowl_latch.visual(
        Box((0.006, 0.008, 0.010)),
        origin=Origin(xyz=(-0.0425, 0.024, -0.003)),
        material=dark_trim,
        name="tip_stem",
    )
    bowl_latch.inertial = Inertial.from_geometry(
        Box((0.030, 0.060, 0.025)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
    )

    model.articulation(
        "base_to_latch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=bowl_latch,
        origin=Origin(xyz=(0.173, -0.151, 0.156)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    tool_assembly = object_model.get_part("tool_assembly")
    bowl_latch = object_model.get_part("bowl_latch")
    head_hinge = object_model.get_articulation("base_to_head")
    tool_spin = object_model.get_articulation("head_to_tool")
    latch_hinge = object_model.get_articulation("base_to_latch")

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
        bowl,
        base,
        elem_a="bowl_shell",
        elem_b="bowl_pad",
        name="bowl rests on the wide base pad",
    )
    ctx.expect_contact(
        bowl,
        base,
        elem_a="left_trunnion",
        elem_b="left_ear_pad",
        name="left bowl trunnion seats in the left ear",
    )
    ctx.expect_contact(
        bowl,
        base,
        elem_a="right_trunnion",
        elem_b="right_ear_pad",
        name="right bowl trunnion seats in the right ear",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="center_hinge_barrel",
        elem_b="left_hinge_barrel",
        name="head hinge nests against the left base barrel",
    )
    ctx.expect_contact(
        head,
        base,
        elem_a="center_hinge_barrel",
        elem_b="right_hinge_barrel",
        name="head hinge nests against the right base barrel",
    )
    ctx.expect_contact(
        tool_assembly,
        head,
        elem_a="drive_hub",
        elem_b="planetary_nose",
        name="tool hub seats against the head nose",
    )
    ctx.expect_within(
        tool_assembly,
        bowl,
        axes="xy",
        inner_elem="hook_shell",
        outer_elem="bowl_shell",
        margin=0.012,
        name="dough hook stays centered over the bowl",
    )

    head_limits = head_hinge.motion_limits
    tool_limits = tool_spin.motion_limits
    latch_limits = latch_hinge.motion_limits
    ctx.check(
        "tilt head uses an upward rear hinge",
        head_hinge.axis == (0.0, -1.0, 0.0)
        and head_limits is not None
        and head_limits.lower == 0.0
        and head_limits.upper is not None
        and head_limits.upper > 0.9,
        details=f"axis={head_hinge.axis}, limits={head_limits}",
    )
    ctx.check(
        "tool shaft is a continuous vertical spin axis",
        tool_spin.articulation_type == ArticulationType.CONTINUOUS
        and tool_spin.axis == (0.0, 0.0, 1.0)
        and tool_limits is not None
        and tool_limits.lower is None
        and tool_limits.upper is None,
        details=f"type={tool_spin.articulation_type}, axis={tool_spin.axis}, limits={tool_limits}",
    )
    ctx.check(
        "bowl latch pivots on the side ear",
        latch_hinge.axis == (1.0, 0.0, 0.0)
        and latch_limits is not None
        and latch_limits.upper is not None
        and latch_limits.upper >= 0.75,
        details=f"axis={latch_hinge.axis}, limits={latch_limits}",
    )

    closed_tool_pos = ctx.part_world_position(tool_assembly)
    open_head_angle = head_limits.upper if head_limits is not None and head_limits.upper is not None else 0.0
    with ctx.pose({head_hinge: open_head_angle}):
        opened_tool_pos = ctx.part_world_position(tool_assembly)
    ctx.check(
        "tilted head lifts the dough hook away from the bowl",
        closed_tool_pos is not None
        and opened_tool_pos is not None
        and opened_tool_pos[2] > closed_tool_pos[2] + 0.08
        and opened_tool_pos[0] < closed_tool_pos[0] - 0.03,
        details=f"closed={closed_tool_pos}, opened={opened_tool_pos}",
    )

    with ctx.pose({latch_hinge: 0.0}):
        ctx.expect_contact(
            bowl_latch,
            bowl,
            elem_a="latch_tip",
            elem_b="left_trunnion",
            name="closed latch captures the left trunnion",
        )

    open_latch_angle = latch_limits.upper if latch_limits is not None and latch_limits.upper is not None else 0.0
    with ctx.pose({latch_hinge: open_latch_angle}):
        ctx.expect_gap(
            bowl_latch,
            bowl,
            axis="z",
            positive_elem="latch_tip",
            negative_elem="left_trunnion",
            min_gap=0.012,
            name="opened latch lifts clear of the trunnion",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
