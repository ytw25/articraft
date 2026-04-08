from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xy_section(width: float, depth: float, radius: float, z: float, *, y_shift: float = 0.0):
    return [(x, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _body_shell_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.244, 0.258, 0.034, 0.018),
                _xy_section(0.268, 0.286, 0.040, 0.090),
                _xy_section(0.258, 0.278, 0.040, 0.168),
                _xy_section(0.248, 0.268, 0.036, 0.188),
            ]
        ),
        "rice_cooker_body_shell",
    )


def _base_ring_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.286, 0.302, 0.032), 0.018, center=True),
        "rice_cooker_base_ring",
    )


def _lid_shell_mesh():
    half_depth = 0.252 * 0.5
    return mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.254, 0.252, 0.042, 0.000, y_shift=half_depth),
                _xy_section(0.248, 0.246, 0.040, 0.018, y_shift=half_depth),
                _xy_section(0.228, 0.220, 0.038, 0.038, y_shift=half_depth + 0.003),
                _xy_section(0.180, 0.172, 0.034, 0.054, y_shift=half_depth + 0.006),
                _xy_section(0.112, 0.104, 0.024, 0.064, y_shift=half_depth + 0.010),
            ]
        ),
        "rice_cooker_lid_shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rice_cooker")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.92, 1.0))
    dark_base = model.material("dark_base", rgba=(0.18, 0.18, 0.19, 1.0))
    lid_white = model.material("lid_white", rgba=(0.95, 0.95, 0.93, 1.0))
    handle_grey = model.material("handle_grey", rgba=(0.32, 0.33, 0.35, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.66, 0.69, 0.72, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=body_white, name="body_shell")
    body.visual(
        _base_ring_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_base,
        name="base_ring",
    )
    body.visual(
        Box((0.234, 0.250, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.191)),
        material=trim_silver,
        name="top_seat",
    )
    body.visual(
        Box((0.170, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, -0.145, 0.184)),
        material=dark_base,
        name="rear_hinge_mount",
    )
    body.visual(
        Box((0.040, 0.014, 0.018)),
        origin=Origin(xyz=(-0.058, -0.145, 0.189)),
        material=dark_base,
        name="left_hinge_ear",
    )
    body.visual(
        Box((0.040, 0.014, 0.018)),
        origin=Origin(xyz=(0.058, -0.145, 0.189)),
        material=dark_base,
        name="right_hinge_ear",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.286, 0.302, 0.206)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
    )

    lid = model.part("lid")
    lid.visual(_lid_shell_mesh(), material=lid_white, name="lid_shell")
    lid.visual(
        Box((0.168, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.250, 0.036)),
        material=handle_grey,
        name="handle_shell",
    )
    lid.visual(
        Box((0.034, 0.026, 0.032)),
        origin=Origin(xyz=(-0.054, 0.229, 0.014)),
        material=handle_grey,
        name="handle_left_support",
    )
    lid.visual(
        Box((0.034, 0.026, 0.032)),
        origin=Origin(xyz=(0.054, 0.229, 0.014)),
        material=handle_grey,
        name="handle_right_support",
    )
    lid.visual(
        Box((0.140, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.224, 0.008)),
        material=trim_silver,
        name="front_trim",
    )
    lid.visual(
        Box((0.008, 0.016, 0.032)),
        origin=Origin(xyz=(-0.026, 0.279, 0.024)),
        material=trim_silver,
        name="button_left_bezel",
    )
    lid.visual(
        Box((0.008, 0.016, 0.032)),
        origin=Origin(xyz=(0.026, 0.279, 0.024)),
        material=trim_silver,
        name="button_right_bezel",
    )
    lid.visual(
        Box((0.044, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.279, 0.038)),
        material=trim_silver,
        name="button_top_bezel",
    )
    lid.visual(
        Box((0.044, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.279, 0.010)),
        material=trim_silver,
        name="button_bottom_bezel",
    )
    lid.visual(
        Box((0.026, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, 0.266, 0.024)),
        material=handle_grey,
        name="button_guide",
    )
    lid.visual(
        Box((0.150, 0.022, 0.018)),
        origin=Origin(xyz=(0.0, 0.008, 0.006)),
        material=dark_base,
        name="lid_hinge_leaf",
    )
    lid.visual(
        Box((0.074, 0.052, 0.010)),
        origin=Origin(xyz=(0.0, 0.106, 0.057)),
        material=trim_silver,
        name="vent_seat",
    )
    lid.visual(
        Box((0.056, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.084, 0.059)),
        material=dark_base,
        name="vent_hinge_base",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.260, 0.280, 0.070)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.136, 0.032)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.130, 0.198)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )

    latch_button = model.part("latch_button")
    latch_button.visual(
        Box((0.042, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=trim_silver,
        name="button_cap",
    )
    latch_button.visual(
        Box((0.018, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, -0.018, 0.0)),
        material=dark_base,
        name="button_stem",
    )
    latch_button.inertial = Inertial.from_geometry(
        Box((0.042, 0.030, 0.022)),
        mass=0.03,
        origin=Origin(xyz=(0.0, -0.004, 0.0)),
    )

    model.articulation(
        "lid_to_latch_button",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=latch_button,
        origin=Origin(xyz=(0.0, 0.294, 0.024)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.008,
        ),
    )

    vent_cover = model.part("vent_cover")
    vent_cover.visual(
        Box((0.062, 0.044, 0.012)),
        origin=Origin(xyz=(0.0, 0.023, 0.006)),
        material=dark_base,
        name="vent_cover_shell",
    )
    vent_cover.visual(
        Box((0.040, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.040, 0.003)),
        material=trim_silver,
        name="vent_cover_tab",
    )
    vent_cover.visual(
        Box((0.052, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.004, 0.006)),
        material=dark_base,
        name="vent_cover_hinge",
    )
    vent_cover.inertial = Inertial.from_geometry(
        Box((0.062, 0.044, 0.012)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.023, 0.006)),
    )

    model.articulation(
        "lid_to_vent_cover",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_cover,
        origin=Origin(xyz=(0.0, 0.084, 0.062)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    latch_button = object_model.get_part("latch_button")
    vent_cover = object_model.get_part("vent_cover")
    lid_joint = object_model.get_articulation("body_to_lid")
    button_joint = object_model.get_articulation("lid_to_latch_button")
    vent_joint = object_model.get_articulation("lid_to_vent_cover")

    ctx.allow_overlap(
        latch_button,
        lid,
        elem_a="button_stem",
        elem_b="button_guide",
        reason="The latch stem intentionally slides inside the front guide sleeve in the lid handle.",
    )

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.010,
            max_penetration=0.0,
            positive_elem="lid_shell",
            negative_elem="top_seat",
            name="closed lid sits just above the body seat",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.180,
            name="closed lid covers the cooker body plan",
        )

    closed_handle = ctx.part_element_world_aabb(lid, elem="handle_shell")
    with ctx.pose({lid_joint: math.radians(95.0)}):
        open_handle = ctx.part_element_world_aabb(lid, elem="handle_shell")

    closed_z = closed_handle[1][2] if closed_handle is not None else None
    open_z = open_handle[1][2] if open_handle is not None else None
    ctx.check(
        "lid opens upward at the front",
        closed_z is not None and open_z is not None and open_z > closed_z + 0.10,
        details=f"closed_z={closed_z}, open_z={open_z}",
    )

    rest_button_pos = ctx.part_world_position(latch_button)
    with ctx.pose({button_joint: 0.008}):
        pressed_button_pos = ctx.part_world_position(latch_button)
    rest_y = rest_button_pos[1] if rest_button_pos is not None else None
    pressed_y = pressed_button_pos[1] if pressed_button_pos is not None else None
    ctx.check(
        "front latch button presses into the handle",
        rest_y is not None and pressed_y is not None and pressed_y < rest_y - 0.006,
        details=f"rest_y={rest_y}, pressed_y={pressed_y}",
    )

    with ctx.pose({vent_joint: 0.0}):
        ctx.expect_gap(
            vent_cover,
            lid,
            axis="z",
            max_gap=0.003,
            max_penetration=0.0,
            positive_elem="vent_cover_shell",
            negative_elem="vent_seat",
            name="vent cover sits on the vent seat when closed",
        )

    closed_vent = ctx.part_element_world_aabb(vent_cover, elem="vent_cover_shell")
    with ctx.pose({vent_joint: math.radians(70.0)}):
        open_vent = ctx.part_element_world_aabb(vent_cover, elem="vent_cover_shell")

    closed_vent_z = closed_vent[1][2] if closed_vent is not None else None
    open_vent_z = open_vent[1][2] if open_vent is not None else None
    ctx.check(
        "vent cover flips upward",
        closed_vent_z is not None and open_vent_z is not None and open_vent_z > closed_vent_z + 0.015,
        details=f"closed_vent_z={closed_vent_z}, open_vent_z={open_vent_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
