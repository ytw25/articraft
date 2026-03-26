from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _handle_loop(width: float, height: float) -> list[tuple[float, float]]:
    top = 0.42 * height
    upper = 0.28 * height
    mid = -0.10 * height
    lower = -0.34 * height
    bottom = -0.58 * height
    half = width * 0.5
    shoulder = 0.46 * width
    waist = 0.30 * width

    return [
        (0.0, top),
        (waist, 0.98 * top),
        (shoulder, upper),
        (half, 0.06 * height),
        (shoulder, mid),
        (waist, lower),
        (0.0, bottom),
        (-waist, lower),
        (-shoulder, mid),
        (-half, 0.06 * height),
        (-shoulder, upper),
        (-waist, 0.98 * top),
    ]


def _handle_section(x_pos: float, width: float, height: float) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z) for y, z in _handle_loop(width, height)]


def _build_handle_shell():
    outer_sections = [
        _handle_section(0.000, 0.0290, 0.0190),
        _handle_section(0.040, 0.0315, 0.0220),
        _handle_section(0.092, 0.0295, 0.0210),
        _handle_section(0.136, 0.0220, 0.0185),
        _handle_section(0.168, 0.0110, 0.0100),
    ]
    inner_sections = [
        _handle_section(0.006, 0.0246, 0.0146),
        _handle_section(0.040, 0.0271, 0.0176),
        _handle_section(0.092, 0.0251, 0.0166),
        _handle_section(0.136, 0.0176, 0.0141),
        _handle_section(0.154, 0.0066, 0.0056),
    ]

    outer = repair_loft(section_loft(outer_sections))
    inner = repair_loft(section_loft(inner_sections))
    shell = boolean_difference(outer, inner)

    top_slot = BoxGeometry((0.086, 0.0100, 0.0140)).translate(0.083, 0.0, 0.0100)
    blade_mouth = BoxGeometry((0.038, 0.0032, 0.0120)).translate(0.150, 0.0, -0.0005)
    shell = boolean_difference(shell, top_slot)
    shell = boolean_difference(shell, blade_mouth)
    return mesh_from_geometry(shell, ASSETS.mesh_path("utility_knife_handle_shell.obj"))


def _build_blade():
    blade_profile = [
        (0.000, -0.0045),
        (0.014, -0.0045),
        (0.026, -0.0042),
        (0.040, -0.0032),
        (0.052, -0.0012),
        (0.062, 0.0020),
        (0.054, 0.0065),
        (0.010, 0.0065),
        (0.000, 0.0035),
    ]
    blade_geom = ExtrudeGeometry(
        blade_profile,
        0.0006,
        cap=True,
        center=True,
        closed=True,
    ).rotate_x(math.pi / 2.0)
    return mesh_from_geometry(blade_geom, ASSETS.mesh_path("utility_knife_blade.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_utility_knife", assets=ASSETS)

    handle_yellow = model.material("handle_yellow", rgba=(0.94, 0.78, 0.14, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))

    handle = model.part("handle")
    handle.visual(_build_handle_shell(), material=handle_yellow, name="handle_shell")
    handle.visual(
        Box((0.078, 0.008, 0.003)),
        origin=Origin(xyz=(0.051, 0.0, -0.0065)),
        material=charcoal,
        name="track_floor",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.168, 0.032, 0.024)),
        mass=0.18,
        origin=Origin(xyz=(0.084, 0.0, -0.001)),
    )

    slider = model.part("blade_slider")
    slider.visual(
        Box((0.048, 0.010, 0.006)),
        origin=Origin(xyz=(0.034, 0.0, -0.002)),
        material=charcoal,
        name="carrier_body",
    )
    slider.visual(
        Box((0.020, 0.006, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, -0.002)),
        material=charcoal,
        name="blade_clamp",
    )
    slider.visual(
        Box((0.008, 0.004, 0.0094)),
        origin=Origin(xyz=(0.056, 0.0, 0.0045)),
        material=charcoal,
        name="button_stem",
    )
    slider.visual(
        Box((0.014, 0.008, 0.0040)),
        origin=Origin(xyz=(0.056, 0.0, 0.0112)),
        material=charcoal,
        name="button_head",
    )
    slider.visual(
        _build_blade(),
        origin=Origin(xyz=(0.052, 0.0, -0.0013)),
        material=steel,
        name="blade",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.116, 0.014, 0.018)),
        mass=0.05,
        origin=Origin(xyz=(0.058, 0.0, 0.001)),
    )

    model.articulation(
        "handle_to_blade_slider",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=slider,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.18,
            lower=0.0,
            upper=0.040,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    handle = object_model.get_part("handle")
    slider = object_model.get_part("blade_slider")
    slider_joint = object_model.get_articulation("handle_to_blade_slider")
    handle_shell = handle.get_visual("handle_shell")
    track_floor = handle.get_visual("track_floor")
    carrier_body = slider.get_visual("carrier_body")
    button_stem = slider.get_visual("button_stem")
    button_head = slider.get_visual("button_head")
    blade = slider.get_visual("blade")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=12)

    with ctx.pose({slider_joint: 0.0}):
        ctx.expect_contact(
            slider,
            handle,
            elem_a=carrier_body,
            elem_b=track_floor,
            name="carrier_retracted_contacts_track",
        )
        ctx.expect_within(
            slider,
            handle,
            axes="yz",
            inner_elem=carrier_body,
            margin=0.0,
            name="carrier_retracted_within_handle_yz",
        )
        ctx.expect_within(
            slider,
            handle,
            axes="y",
            inner_elem=button_stem,
            margin=0.001,
            name="button_stem_retracted_centered_in_slot",
        )
        ctx.expect_within(
            slider,
            handle,
            axes="yz",
            inner_elem=blade,
            outer_elem=handle_shell,
            margin=0.0,
            name="blade_retracted_within_nose_channel",
        )
        ctx.expect_origin_distance(
            slider,
            handle,
            axes="yz",
            max_dist=0.001,
            name="slider_retracted_centerline_alignment",
        )
        rest_slider_pos = ctx.part_world_position(slider)
        rest_blade_aabb = ctx.part_element_world_aabb(slider, elem="blade")
        rest_button_aabb = ctx.part_element_world_aabb(slider, elem="button_head")
        handle_aabb = ctx.part_world_aabb(handle)

    with ctx.pose({slider_joint: 0.040}):
        ctx.expect_contact(
            slider,
            handle,
            elem_a=carrier_body,
            elem_b=track_floor,
            name="carrier_extended_contacts_track",
        )
        ctx.expect_within(
            slider,
            handle,
            axes="yz",
            inner_elem=carrier_body,
            margin=0.0,
            name="carrier_extended_within_handle_yz",
        )
        ctx.expect_within(
            slider,
            handle,
            axes="y",
            inner_elem=button_stem,
            margin=0.001,
            name="button_stem_extended_centered_in_slot",
        )
        ctx.expect_within(
            slider,
            handle,
            axes="yz",
            inner_elem=blade,
            outer_elem=handle_shell,
            margin=0.0,
            name="blade_extended_within_nose_channel",
        )
        ctx.expect_origin_distance(
            slider,
            handle,
            axes="yz",
            max_dist=0.001,
            name="slider_extended_centerline_alignment",
        )
        extended_slider_pos = ctx.part_world_position(slider)
        extended_blade_aabb = ctx.part_element_world_aabb(slider, elem="blade")
        extended_button_aabb = ctx.part_element_world_aabb(slider, elem="button_head")

    assert rest_slider_pos is not None
    assert extended_slider_pos is not None
    assert rest_blade_aabb is not None
    assert extended_blade_aabb is not None
    assert rest_button_aabb is not None
    assert extended_button_aabb is not None
    assert handle_aabb is not None

    handle_tip_x = handle_aabb[1][0]
    handle_top_z = handle_aabb[1][2]
    rest_tip_x = rest_blade_aabb[1][0]
    extended_tip_x = extended_blade_aabb[1][0]
    rest_button_x = rest_button_aabb[1][0]
    extended_button_x = extended_button_aabb[1][0]

    assert abs((extended_slider_pos[0] - rest_slider_pos[0]) - 0.040) < 0.001
    assert abs((extended_tip_x - rest_tip_x) - 0.040) < 0.001
    assert abs((extended_button_x - rest_button_x) - 0.040) < 0.001
    assert rest_tip_x < handle_tip_x - 0.020
    assert extended_tip_x > handle_tip_x + 0.002
    assert rest_button_aabb[0][2] > handle_top_z - 0.001
    assert extended_button_aabb[0][2] > handle_top_z - 0.001

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
