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
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_body_shell():
    outer_profile = [
        (0.090, 0.000),
        (0.114, 0.012),
        (0.128, 0.042),
        (0.132, 0.176),
        (0.128, 0.214),
        (0.122, 0.228),
    ]
    inner_profile = [
        (0.000, 0.004),
        (0.098, 0.014),
        (0.119, 0.044),
        (0.120, 0.204),
        (0.114, 0.220),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_lid_shell():
    outer_profile = [
        (0.000, 0.026),
        (0.032, 0.030),
        (0.078, 0.028),
        (0.108, 0.021),
        (0.122, 0.010),
        (0.126, 0.002),
    ]
    inner_profile = [
        (0.000, 0.019),
        (0.030, 0.022),
        (0.076, 0.020),
        (0.104, 0.014),
        (0.116, 0.007),
        (0.120, 0.001),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_panel_face():
    profile = rounded_rect_profile(0.060, 0.090, 0.012, corner_segments=8)
    return ExtrudeGeometry(profile, 0.004, center=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_rice_cooker")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.92, 1.0))
    lid_white = model.material("lid_white", rgba=(0.95, 0.95, 0.94, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.80, 0.81, 0.79, 1.0))
    charcoal = model.material("charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.74, 0.76, 1.0))

    body = model.part("body")
    body.visual(
        _mesh("rice_cooker_body_shell", _build_body_shell()),
        material=body_white,
        name="body_shell",
    )
    body.visual(
        Box((0.020, 0.060, 0.004)),
        origin=Origin(xyz=(-0.122, 0.0, 0.226)),
        material=warm_gray,
        name="rear_hinge_seat",
    )
    body.visual(
        Box((0.006, 0.036, 0.102)),
        origin=Origin(xyz=(0.1274, 0.0, 0.079)),
        material=warm_gray,
        name="front_panel_land",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.132, length=0.228),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.008, 0.028, 0.094)),
        origin=Origin(xyz=(0.004, 0.0, 0.047)),
        material=warm_gray,
        name="panel_mount_boss",
    )
    control_panel.visual(
        Box((0.016, 0.084, 0.132)),
        origin=Origin(xyz=(0.016, 0.0, 0.066)),
        material=charcoal,
        name="panel_bezel",
    )
    control_panel.visual(
        _mesh("rice_cooker_panel_face", _build_panel_face()),
        origin=Origin(xyz=(0.026, 0.0, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="panel_face",
    )
    control_panel.visual(
        Box((0.014, 0.040, 0.016)),
        origin=Origin(xyz=(0.031, 0.0, 0.046)),
        material=charcoal,
        name="switch_mount",
    )
    control_panel.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.036, 0.019, 0.046)),
        material=charcoal,
        name="switch_ear_left",
    )
    control_panel.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.036, -0.019, 0.046)),
        material=charcoal,
        name="switch_ear_right",
    )
    control_panel.inertial = Inertial.from_geometry(
        Box((0.040, 0.084, 0.132)),
        mass=0.22,
        origin=Origin(xyz=(0.020, 0.0, 0.066)),
    )

    selector_switch = model.part("selector_switch")
    selector_switch.visual(
        Cylinder(radius=0.0032, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="switch_pivot_rod",
    )
    selector_switch.visual(
        Box((0.010, 0.020, 0.008)),
        origin=Origin(xyz=(0.005, 0.0, 0.000)),
        material=knob_black,
        name="switch_bridge",
    )
    selector_switch.visual(
        Box((0.020, 0.024, 0.016)),
        origin=Origin(xyz=(0.016, 0.0, -0.008)),
        material=knob_black,
        name="switch_paddle",
    )
    selector_switch.visual(
        Box((0.010, 0.020, 0.008)),
        origin=Origin(xyz=(0.026, 0.0, -0.014)),
        material=knob_black,
        name="switch_grip",
    )
    selector_switch.inertial = Inertial.from_geometry(
        Box((0.032, 0.032, 0.024)),
        mass=0.04,
        origin=Origin(xyz=(0.016, 0.0, -0.008)),
    )

    lid = model.part("lid")
    lid.visual(
        _mesh("rice_cooker_lid_shell", _build_lid_shell()),
        origin=Origin(xyz=(0.126, 0.0, 0.0)),
        material=lid_white,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.164, -0.026, 0.034)),
        material=charcoal,
        name="handle_post_left",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.014),
        origin=Origin(xyz=(0.164, 0.026, 0.034)),
        material=charcoal,
        name="handle_post_right",
    )
    lid.visual(
        Box((0.052, 0.070, 0.012)),
        origin=Origin(xyz=(0.164, 0.0, 0.046)),
        material=charcoal,
        name="lid_handle",
    )
    lid.visual(
        Box((0.018, 0.026, 0.004)),
        origin=Origin(xyz=(0.050, 0.0, 0.026)),
        material=warm_gray,
        name="vent_seat",
    )
    lid.visual(
        Box((0.020, 0.040, 0.004)),
        origin=Origin(xyz=(0.004, 0.0, 0.003)),
        material=warm_gray,
        name="rear_hinge_pad",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.252, 0.252, 0.060)),
        mass=0.65,
        origin=Origin(xyz=(0.126, 0.0, 0.024)),
    )

    vent_cap = model.part("vent_cap")
    vent_cap.visual(
        Cylinder(radius=0.003, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="vent_cap_hinge",
    )
    vent_cap.visual(
        Box((0.010, 0.018, 0.006)),
        origin=Origin(xyz=(0.005, 0.0, 0.001)),
        material=charcoal,
        name="vent_cap_bridge",
    )
    vent_cap.visual(
        Box((0.024, 0.020, 0.006)),
        origin=Origin(xyz=(0.022, 0.0, 0.004)),
        material=charcoal,
        name="vent_cap_shell",
    )
    vent_cap.inertial = Inertial.from_geometry(
        Box((0.034, 0.022, 0.010)),
        mass=0.02,
        origin=Origin(xyz=(0.017, 0.0, 0.003)),
    )

    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(0.1304, 0.0, 0.028)),
    )
    model.articulation(
        "panel_to_selector_switch",
        ArticulationType.REVOLUTE,
        parent=control_panel,
        child=selector_switch,
        origin=Origin(xyz=(0.041, 0.0, 0.046)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.0,
            lower=-0.40,
            upper=0.35,
        ),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.126, 0.0, 0.227)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(102.0),
        ),
    )
    model.articulation(
        "lid_to_vent_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(0.050, 0.0, 0.031)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    control_panel = object_model.get_part("control_panel")
    selector_switch = object_model.get_part("selector_switch")
    lid = object_model.get_part("lid")
    vent_cap = object_model.get_part("vent_cap")

    lid_hinge = object_model.get_articulation("body_to_lid")
    vent_hinge = object_model.get_articulation("lid_to_vent_cap")
    switch_pivot = object_model.get_articulation("panel_to_selector_switch")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            min_gap=0.0,
            max_gap=0.012,
            name="lid sits just above cooker rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="body_shell",
            min_overlap=0.18,
            name="lid covers body opening",
        )

    lid_rest_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: math.radians(80.0)}):
        lid_open_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward at rear hinge",
        lid_rest_aabb is not None
        and lid_open_aabb is not None
        and lid_open_aabb[1][2] > lid_rest_aabb[1][2] + 0.10
        and lid_open_aabb[0][0] > lid_rest_aabb[0][0] - 0.03,
        details=f"rest={lid_rest_aabb}, open={lid_open_aabb}",
    )

    with ctx.pose({vent_hinge: 0.0}):
        ctx.expect_gap(
            vent_cap,
            lid,
            axis="z",
            positive_elem="vent_cap_shell",
            negative_elem="vent_seat",
            min_gap=0.0,
            max_gap=0.010,
            name="vent cap rests close to vent seat",
        )

    vent_rest_aabb = ctx.part_element_world_aabb(vent_cap, elem="vent_cap_shell")
    with ctx.pose({vent_hinge: math.radians(55.0)}):
        vent_open_aabb = ctx.part_element_world_aabb(vent_cap, elem="vent_cap_shell")
    ctx.check(
        "vent cap swings upward",
        vent_rest_aabb is not None
        and vent_open_aabb is not None
        and vent_open_aabb[1][2] > vent_rest_aabb[1][2] + 0.012,
        details=f"rest={vent_rest_aabb}, open={vent_open_aabb}",
    )

    ctx.expect_gap(
        selector_switch,
        control_panel,
        axis="x",
        positive_elem="switch_paddle",
        negative_elem="panel_face",
        min_gap=0.0,
        max_gap=0.040,
        name="selector switch projects from panel face",
    )

    switch_rest_aabb = ctx.part_element_world_aabb(selector_switch, elem="switch_paddle")
    with ctx.pose({switch_pivot: 0.30}):
        switch_tilt_aabb = ctx.part_element_world_aabb(selector_switch, elem="switch_paddle")
    ctx.check(
        "selector switch tilts on transverse pivot",
        switch_rest_aabb is not None
        and switch_tilt_aabb is not None
        and switch_tilt_aabb[0][2] < switch_rest_aabb[0][2] - 0.004,
        details=f"rest={switch_rest_aabb}, tilted={switch_tilt_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
