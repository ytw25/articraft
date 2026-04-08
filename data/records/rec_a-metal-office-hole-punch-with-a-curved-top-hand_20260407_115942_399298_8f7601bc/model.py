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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_hole_punch")

    steel_dark = model.material("steel_dark", rgba=(0.26, 0.28, 0.31, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.56, 0.58, 0.61, 1.0))
    tray_black = model.material("tray_black", rgba=(0.14, 0.14, 0.15, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base_tray = model.part("base_tray")
    base_tray.visual(
        Box((0.128, 0.096, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=tray_black,
        name="tray_drawer",
    )
    base_tray.visual(
        Box((0.120, 0.088, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0135)),
        material=steel_mid,
        name="top_cover",
    )
    base_tray.visual(
        Box((0.014, 0.098, 0.005)),
        origin=Origin(xyz=(-0.051, 0.0, 0.0155)),
        material=steel_mid,
        name="front_guide_beam",
    )
    base_tray.visual(
        Box((0.006, 0.094, 0.004)),
        origin=Origin(xyz=(-0.048, 0.0, 0.020)),
        material=steel_dark,
        name="front_slider_rail",
    )
    base_tray.visual(
        Box((0.040, 0.066, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, 0.015)),
        material=steel_dark,
        name="rear_mount_pad",
    )
    base_tray.inertial = Inertial.from_geometry(
        Box((0.128, 0.096, 0.024)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    punch_body = model.part("punch_body")
    punch_body.visual(
        Box((0.058, 0.006, 0.026)),
        origin=Origin(xyz=(0.004, 0.027, 0.013)),
        material=steel_dark,
        name="left_cheek",
    )
    punch_body.visual(
        Box((0.058, 0.006, 0.026)),
        origin=Origin(xyz=(0.004, -0.027, 0.013)),
        material=steel_dark,
        name="right_cheek",
    )
    punch_body.visual(
        Box((0.044, 0.052, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, 0.008)),
        material=steel_dark,
        name="punch_head",
    )
    punch_body.visual(
        Box((0.032, 0.058, 0.012)),
        origin=Origin(xyz=(0.014, 0.0, 0.028)),
        material=steel_mid,
        name="top_bridge",
    )
    punch_body.visual(
        Box((0.018, 0.010, 0.016)),
        origin=Origin(xyz=(0.030, 0.024, 0.034)),
        material=steel_mid,
        name="left_hinge_block",
    )
    punch_body.visual(
        Box((0.018, 0.010, 0.016)),
        origin=Origin(xyz=(0.030, -0.024, 0.034)),
        material=steel_mid,
        name="right_hinge_block",
    )
    punch_body.visual(
        Box((0.016, 0.038, 0.008)),
        origin=Origin(xyz=(0.024, 0.0, 0.024)),
        material=steel_mid,
        name="rear_bridge",
    )
    punch_body.visual(
        Cylinder(radius=0.0042, length=0.010),
        origin=Origin(xyz=(-0.020, 0.011, 0.013)),
        material=steel_mid,
        name="left_die_ring",
    )
    punch_body.visual(
        Cylinder(radius=0.0042, length=0.010),
        origin=Origin(xyz=(-0.020, -0.011, 0.013)),
        material=steel_mid,
        name="right_die_ring",
    )
    punch_body.inertial = Inertial.from_geometry(
        Box((0.070, 0.060, 0.044)),
        mass=0.80,
        origin=Origin(xyz=(0.004, 0.0, 0.022)),
    )

    def handle_section(
        x_pos: float,
        *,
        width: float,
        height: float,
        z_center: float,
        radius: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y_val, z_center + z_val)
            for z_val, y_val in rounded_rect_profile(height, width, radius)
        ]

    handle_sections = [
        handle_section(-0.018, width=0.036, height=0.014, z_center=0.004, radius=0.004),
        handle_section(-0.032, width=0.050, height=0.016, z_center=0.015, radius=0.005),
        handle_section(-0.066, width=0.064, height=0.015, z_center=0.022, radius=0.005),
        handle_section(-0.100, width=0.052, height=0.010, z_center=0.012, radius=0.004),
    ]
    handle_mesh = mesh_from_geometry(section_loft(handle_sections), "hole_punch_handle_shell")

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.005, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mid,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.020, 0.010, 0.008)),
        origin=Origin(xyz=(-0.010, 0.013, 0.004)),
        material=steel_mid,
        name="left_hinge_arm",
    )
    handle.visual(
        Box((0.020, 0.010, 0.008)),
        origin=Origin(xyz=(-0.010, -0.013, 0.004)),
        material=steel_mid,
        name="right_hinge_arm",
    )
    handle.visual(
        handle_mesh,
        material=steel_mid,
        name="handle_shell",
    )
    handle.visual(
        Box((0.018, 0.038, 0.004)),
        origin=Origin(xyz=(-0.095, 0.0, 0.009)),
        material=steel_mid,
        name="front_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.102, 0.064, 0.034)),
        mass=0.32,
        origin=Origin(xyz=(-0.048, 0.0, 0.015)),
    )

    paper_stop = model.part("paper_stop")
    paper_stop.visual(
        Box((0.014, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=steel_mid,
        name="carriage_pad",
    )
    paper_stop.visual(
        Box((0.008, 0.014, 0.006)),
        origin=Origin(xyz=(0.006, 0.0, 0.003)),
        material=steel_dark,
        name="guide_shoe",
    )
    paper_stop.visual(
        Box((0.004, 0.060, 0.016)),
        origin=Origin(xyz=(-0.009, 0.0, 0.008)),
        material=steel_mid,
        name="stop_fence",
    )
    paper_stop.visual(
        Box((0.010, 0.010, 0.010)),
        origin=Origin(xyz=(-0.005, 0.032, 0.007)),
        material=steel_dark,
        name="knob_boss",
    )
    paper_stop.visual(
        Box((0.008, 0.010, 0.010)),
        origin=Origin(xyz=(-0.006, -0.031, 0.007)),
        material=steel_dark,
        name="left_end_cap",
    )
    paper_stop.inertial = Inertial.from_geometry(
        Box((0.022, 0.066, 0.024)),
        mass=0.12,
        origin=Origin(xyz=(-0.004, 0.0, 0.010)),
    )

    guide_knob = model.part("guide_knob")
    guide_knob.visual(
        Cylinder(radius=0.003, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_dark,
        name="shaft",
    )
    guide_knob.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_wheel",
    )
    guide_knob.visual(
        Box((0.004, 0.010, 0.002)),
        origin=Origin(xyz=(0.0, 0.013, 0.008)),
        material=steel_mid,
        name="knob_tab",
    )
    guide_knob.inertial = Inertial.from_geometry(
        Box((0.020, 0.024, 0.020)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
    )

    model.articulation(
        "base_to_body",
        ArticulationType.FIXED,
        parent=base_tray,
        child=punch_body,
        origin=Origin(xyz=(0.016, 0.0, 0.015)),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=punch_body,
        child=handle,
        origin=Origin(xyz=(0.030, 0.0, 0.036)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(58.0),
        ),
    )
    model.articulation(
        "base_to_paper_stop",
        ArticulationType.PRISMATIC,
        parent=base_tray,
        child=paper_stop,
        origin=Origin(xyz=(-0.048, 0.0, 0.022)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=-0.018,
            upper=0.018,
        ),
    )
    model.articulation(
        "paper_stop_to_guide_knob",
        ArticulationType.CONTINUOUS,
        parent=paper_stop,
        child=guide_knob,
        origin=Origin(xyz=(-0.005, 0.037, 0.007)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
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

    base_tray = object_model.get_part("base_tray")
    punch_body = object_model.get_part("punch_body")
    handle = object_model.get_part("handle")
    paper_stop = object_model.get_part("paper_stop")
    guide_knob = object_model.get_part("guide_knob")
    handle_joint = object_model.get_articulation("body_to_handle")
    stop_joint = object_model.get_articulation("base_to_paper_stop")

    ctx.expect_contact(
        punch_body,
        base_tray,
        elem_a="punch_head",
        elem_b="top_cover",
        name="punch body seats on the base tray",
    )
    with ctx.pose({handle_joint: 0.0}):
        ctx.expect_gap(
            handle,
            punch_body,
            axis="z",
            positive_elem="front_grip",
            negative_elem="top_bridge",
            min_gap=0.004,
            max_gap=0.020,
            name="closed handle sits just above the punch bridge",
        )
    rest_tip = ctx.part_element_world_aabb(handle, elem="front_grip")
    with ctx.pose({handle_joint: math.radians(55.0)}):
        raised_tip = ctx.part_element_world_aabb(handle, elem="front_grip")
        ctx.check(
            "handle front rises when opened",
            rest_tip is not None
            and raised_tip is not None
            and raised_tip[0][2] > rest_tip[0][2] + 0.030,
            details=f"rest={rest_tip}, raised={raised_tip}",
        )
    with ctx.pose({stop_joint: 0.0}):
        ctx.expect_contact(
            paper_stop,
            base_tray,
            elem_a="carriage_pad",
            elem_b="front_slider_rail",
            name="paper stop carriage rides on the front rail",
        )
        ctx.expect_within(
            paper_stop,
            base_tray,
            axes="y",
            inner_elem="carriage_pad",
            outer_elem="front_slider_rail",
            margin=0.0,
            name="paper stop carriage stays within the rail at center",
        )
        ctx.expect_overlap(
            paper_stop,
            base_tray,
            axes="xy",
            elem_a="carriage_pad",
            elem_b="front_slider_rail",
            min_overlap=0.005,
            name="paper stop carriage retains rail overlap at center",
        )
    rest_stop = ctx.part_world_position(paper_stop)
    with ctx.pose({stop_joint: 0.018}):
        shifted_stop = ctx.part_world_position(paper_stop)
        ctx.expect_contact(
            paper_stop,
            base_tray,
            elem_a="carriage_pad",
            elem_b="front_slider_rail",
            name="paper stop carriage stays seated at travel limit",
        )
        ctx.expect_within(
            paper_stop,
            base_tray,
            axes="y",
            inner_elem="carriage_pad",
            outer_elem="front_slider_rail",
            margin=0.0,
            name="paper stop carriage stays within the rail at travel limit",
        )
        ctx.check(
            "paper stop slides sideways along the front guide",
            rest_stop is not None
            and shifted_stop is not None
            and shifted_stop[1] > rest_stop[1] + 0.015,
            details=f"rest={rest_stop}, shifted={shifted_stop}",
        )
    ctx.expect_contact(
        guide_knob,
        paper_stop,
        elem_a="shaft",
        elem_b="knob_boss",
        name="guide knob mounts directly to the stop side boss",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
