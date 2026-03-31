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
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _connect_rings(geom: MeshGeometry, ring_a: list[int], ring_b: list[int]) -> None:
    count = len(ring_a)
    for index in range(count):
        a0 = ring_a[index]
        a1 = ring_a[(index + 1) % count]
        b0 = ring_b[index]
        b1 = ring_b[(index + 1) % count]
        _add_quad(geom, a0, a1, b1, b0)


def _cap_ring(
    geom: MeshGeometry,
    ring: list[int],
    *,
    center: tuple[float, float, float],
    reverse: bool,
) -> None:
    center_id = geom.add_vertex(*center)
    count = len(ring)
    for index in range(count):
        a = ring[index]
        b = ring[(index + 1) % count]
        if reverse:
            geom.add_face(center_id, b, a)
        else:
            geom.add_face(center_id, a, b)


def _rounded_loop(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _build_bottle_shell() -> MeshGeometry:
    wall = 0.003
    bottom_thickness = 0.004
    sections = (
        (0.000, 0.074, 0.052, 0.013),
        (0.010, 0.084, 0.060, 0.015),
        (0.082, 0.090, 0.062, 0.016),
        (0.146, 0.086, 0.058, 0.015),
        (0.170, 0.068, 0.048, 0.013),
        (0.184, 0.050, 0.038, 0.010),
    )

    geom = MeshGeometry()
    outer_rings: list[list[int]] = []
    inner_rings: list[list[int]] = []

    for z, width, depth, radius in sections:
        outer_rings.append(
            [geom.add_vertex(*point) for point in _rounded_loop(width, depth, radius, z)]
        )

    for z, width, depth, radius in sections:
        inner_z = min(z + bottom_thickness, 0.181) if z == sections[0][0] else z - wall
        inner_width = max(width - 2.0 * wall, 0.008)
        inner_depth = max(depth - 2.0 * wall, 0.008)
        inner_radius = max(radius - wall, 0.002)
        inner_rings.append(
            [
                geom.add_vertex(*point)
                for point in _rounded_loop(inner_width, inner_depth, inner_radius, inner_z)
            ]
        )

    for index in range(len(outer_rings) - 1):
        _connect_rings(geom, outer_rings[index], outer_rings[index + 1])
    for index in range(len(inner_rings) - 1):
        _connect_rings(geom, inner_rings[index + 1], inner_rings[index])

    _connect_rings(geom, outer_rings[0], inner_rings[0])
    _connect_rings(geom, inner_rings[-1], outer_rings[-1])

    _cap_ring(geom, outer_rings[0], center=(0.0, 0.0, sections[0][0]), reverse=True)
    _cap_ring(
        geom,
        inner_rings[0],
        center=(0.0, 0.0, bottom_thickness),
        reverse=False,
    )
    return geom


def _yz_section(
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
    *,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z)
        for z, y in rounded_rect_profile(
            height_z,
            width_y,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _build_head_shell() -> MeshGeometry:
    return section_loft(
        [
            _yz_section(-0.014, 0.050, 0.028, 0.008),
            _yz_section(0.020, 0.048, 0.034, 0.009),
            _yz_section(0.050, 0.038, 0.028, 0.007),
            _yz_section(0.070, 0.026, 0.020, 0.005),
        ]
    )


def _build_trigger_lever() -> MeshGeometry:
    return sweep_profile_along_spline(
        [
            (0.000, 0.000, 0.000),
            (0.004, 0.000, -0.018),
            (0.010, 0.000, -0.042),
            (0.020, 0.000, -0.070),
            (0.028, 0.000, -0.088),
        ],
        profile=rounded_rect_profile(0.010, 0.014, 0.0032),
        samples_per_segment=14,
        up_hint=(0.0, 1.0, 0.0),
        cap_profile=True,
    )


def _build_trigger_arm() -> MeshGeometry:
    return sweep_profile_along_spline(
        [
            (0.000, 0.000, 0.000),
            (0.004, 0.000, 0.006),
            (0.010, 0.000, 0.011),
            (0.017, 0.000, 0.013),
        ],
        profile=rounded_rect_profile(0.007, 0.008, 0.0022),
        samples_per_segment=10,
        up_hint=(0.0, 1.0, 0.0),
        cap_profile=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_trigger_spray_bottle", assets=ASSETS)

    bottle_shell = model.material("bottle_shell", rgba=(0.93, 0.94, 0.93, 1.0))
    collar_satin = model.material("collar_satin", rgba=(0.74, 0.76, 0.78, 1.0))
    head_matte = model.material("head_matte", rgba=(0.17, 0.18, 0.19, 1.0))
    trigger_satin = model.material("trigger_satin", rgba=(0.23, 0.24, 0.25, 1.0))
    hardware = model.material("hardware", rgba=(0.58, 0.60, 0.63, 1.0))
    linkage_dark = model.material("linkage_dark", rgba=(0.29, 0.30, 0.31, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        _save_mesh("trigger_bottle_shell.obj", _build_bottle_shell()),
        material=bottle_shell,
        name="bottle_shell",
    )
    bottle.visual(
        _save_mesh(
            "trigger_bottle_neck_finish.obj",
            LatheGeometry.from_shell_profiles(
                [(0.0205, 0.184), (0.0205, 0.198)],
                [(0.0165, 0.184), (0.0165, 0.198)],
                segments=48,
            ),
        ),
        material=bottle_shell,
        name="neck_finish",
    )
    bottle.visual(
        _save_mesh(
            "trigger_bottle_base_stand.obj",
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(0.076, 0.054, 0.012, corner_segments=8),
                0.004,
            ),
        ),
        material=bottle_shell,
        name="base_stand",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.090, 0.062, 0.198)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
    )

    collar = model.part("collar")
    collar.visual(
        Cylinder(radius=0.0235, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=collar_satin,
        name="closure_body",
    )
    for index, z in enumerate((0.004, 0.0085, 0.013)):
        collar.visual(
            Cylinder(radius=0.0247, length=0.0016),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=collar_satin,
            name=f"grip_ridge_{index}",
        )
    collar.visual(
        Cylinder(radius=0.0190, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=collar_satin,
        name="head_mount_land",
    )
    collar.visual(
        Cylinder(radius=0.0248, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.00075)),
        material=hardware,
        name="lower_interface_break",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0248, length=0.020),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    head_body = model.part("head_body")
    head_body.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=head_matte,
        name="rear_mount_core",
    )
    head_body.visual(
        Box((0.034, 0.028, 0.014)),
        origin=Origin(xyz=(0.017, 0.0, 0.013)),
        material=head_matte,
        name="rear_transition",
    )
    head_body.visual(
        Box((0.024, 0.022, 0.026)),
        origin=Origin(xyz=(0.042, 0.0, 0.026)),
        material=head_matte,
        name="neck_riser",
    )
    head_body.visual(
        Box((0.070, 0.020, 0.010)),
        origin=Origin(xyz=(0.076, 0.0, 0.039)),
        material=head_matte,
        name="rear_bridge",
    )
    head_body.visual(
        Box((0.110, 0.012, 0.018)),
        origin=Origin(xyz=(0.120, 0.012, 0.029)),
        material=head_matte,
        name="left_frame_arm",
    )
    head_body.visual(
        Box((0.110, 0.012, 0.018)),
        origin=Origin(xyz=(0.120, -0.012, 0.029)),
        material=head_matte,
        name="right_frame_arm",
    )
    head_body.visual(
        Box((0.070, 0.036, 0.010)),
        origin=Origin(xyz=(0.125, 0.0, 0.044)),
        material=head_matte,
        name="top_cover",
    )
    head_body.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.094, 0.012, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_trigger_boss",
    )
    head_body.visual(
        Cylinder(radius=0.0045, length=0.012),
        origin=Origin(xyz=(0.094, -0.012, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_trigger_boss",
    )
    head_body.visual(
        Box((0.070, 0.016, 0.008)),
        origin=Origin(xyz=(0.160, 0.0, 0.028)),
        material=head_matte,
        name="pump_connector",
    )
    head_body.visual(
        Box((0.028, 0.018, 0.016)),
        origin=Origin(xyz=(0.190, 0.0, 0.028)),
        material=head_matte,
        name="front_anchor_block",
    )
    head_body.visual(
        Cylinder(radius=0.0068, length=0.020),
        origin=Origin(xyz=(0.204, 0.0, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=head_matte,
        name="nozzle_barrel",
    )
    head_body.inertial = Inertial.from_geometry(
        Box((0.220, 0.040, 0.060)),
        mass=0.09,
        origin=Origin(xyz=(0.110, 0.0, 0.030)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Cylinder(radius=0.0032, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="trigger_crossbar",
    )
    trigger.visual(
        Box((0.030, 0.010, 0.010)),
        origin=Origin(xyz=(0.013, 0.0, 0.005)),
        material=trigger_satin,
        name="trigger_arm",
    )
    trigger.visual(
        Box((0.012, 0.010, 0.030)),
        origin=Origin(xyz=(-0.004, 0.0, -0.015)),
        material=trigger_satin,
        name="trigger_lever",
    )
    trigger.visual(
        Box((0.030, 0.022, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, -0.036)),
        material=trigger_satin,
        name="finger_pad",
    )
    trigger.visual(
        Box((0.008, 0.008, 0.008)),
        origin=Origin(xyz=(0.024, 0.0, 0.014)),
        material=trigger_satin,
        name="actuator_pad",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.050, 0.022, 0.062)),
        mass=0.03,
        origin=Origin(xyz=(0.002, 0.0, -0.011)),
    )

    pump_link = model.part("pump_link")
    pump_link.visual(
        Box((0.010, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=linkage_dark,
        name="plunger_pad",
    )
    pump_link.visual(
        Box((0.014, 0.010, 0.008)),
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
        material=linkage_dark,
        name="guide_block",
    )
    pump_link.visual(
        Cylinder(radius=0.0032, length=0.014),
        origin=Origin(xyz=(0.022, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="plunger_rod",
    )
    pump_link.visual(
        Cylinder(radius=0.0048, length=0.008),
        origin=Origin(xyz=(0.031, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="pump_piston",
    )
    pump_link.inertial = Inertial.from_geometry(
        Box((0.040, 0.012, 0.012)),
        mass=0.01,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
    )

    nozzle_tip = model.part("nozzle_tip")
    nozzle_tip.visual(
        Cylinder(radius=0.0082, length=0.004),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=collar_satin,
        name="rear_skirt",
    )
    nozzle_tip.visual(
        Cylinder(radius=0.0072, length=0.012),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=collar_satin,
        name="selector_body",
    )
    nozzle_tip.visual(
        Box((0.004, 0.012, 0.006)),
        origin=Origin(xyz=(0.008, 0.012, 0.000)),
        material=collar_satin,
        name="selector_tab",
    )
    nozzle_tip.inertial = Inertial.from_geometry(
        Box((0.018, 0.012, 0.020)),
        mass=0.008,
        origin=Origin(xyz=(0.007, 0.0, 0.004)),
    )

    model.articulation(
        "bottle_to_collar",
        ArticulationType.CONTINUOUS,
        parent=bottle,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "collar_to_head_body",
        ArticulationType.FIXED,
        parent=collar,
        child=head_body,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )
    model.articulation(
        "head_body_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head_body,
        child=trigger,
        origin=Origin(xyz=(0.094, 0.0, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(33.0),
        ),
    )
    model.articulation(
        "head_body_to_pump_link",
        ArticulationType.PRISMATIC,
        parent=head_body,
        child=pump_link,
        origin=Origin(xyz=(0.128, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=0.008,
        ),
    )
    model.articulation(
        "head_body_to_nozzle_tip",
        ArticulationType.REVOLUTE,
        parent=head_body,
        child=nozzle_tip,
        origin=Origin(xyz=(0.214, 0.0, 0.028)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=-1.05,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottle = object_model.get_part("bottle")
    collar = object_model.get_part("collar")
    head_body = object_model.get_part("head_body")
    trigger = object_model.get_part("trigger")
    pump_link = object_model.get_part("pump_link")
    nozzle_tip = object_model.get_part("nozzle_tip")

    trigger_joint = object_model.get_articulation("head_body_to_trigger")
    pump_joint = object_model.get_articulation("head_body_to_pump_link")
    nozzle_joint = object_model.get_articulation("head_body_to_nozzle_tip")

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

    bottle_aabb = ctx.part_world_aabb(bottle)
    head_aabb = ctx.part_world_aabb(head_body)
    nozzle_aabb = ctx.part_world_aabb(nozzle_tip)
    assert bottle_aabb is not None
    assert head_aabb is not None
    assert nozzle_aabb is not None

    overall_min_z = min(bottle_aabb[0][2], head_aabb[0][2], nozzle_aabb[0][2])
    overall_max_z = max(bottle_aabb[1][2], head_aabb[1][2], nozzle_aabb[1][2])
    overall_height = overall_max_z - overall_min_z
    bottle_width = bottle_aabb[1][0] - bottle_aabb[0][0]
    bottle_depth = bottle_aabb[1][1] - bottle_aabb[0][1]

    ctx.check(
        "realistic_overall_height",
        0.24 <= overall_height <= 0.31,
        details=f"overall height {overall_height:.3f} m outside realistic spray-bottle range",
    )
    ctx.check(
        "realistic_bottle_footprint",
        0.07 <= bottle_width <= 0.10 and 0.05 <= bottle_depth <= 0.07,
        details=(
            f"bottle footprint {bottle_width:.3f} x {bottle_depth:.3f} m "
            "outside premium trigger-bottle range"
        ),
    )

    ctx.expect_contact(collar, bottle, name="collar_contacts_bottle")
    ctx.expect_contact(head_body, collar, name="head_mount_contacts_collar")
    ctx.expect_contact(trigger, head_body, name="trigger_pivot_contacts_head")
    ctx.expect_contact(pump_link, head_body, name="pump_link_guided_by_head")
    ctx.expect_contact(nozzle_tip, head_body, name="nozzle_tip_contacts_head")

    ctx.expect_overlap(collar, bottle, axes="xy", min_overlap=0.020, name="collar_over_neck")
    ctx.expect_overlap(head_body, collar, axes="xy", min_overlap=0.015, name="head_over_collar")
    ctx.expect_within(
        pump_link,
        head_body,
        axes="yz",
        margin=0.001,
        name="pump_link_within_head_window",
    )

    ctx.check(
        "trigger_axis_is_lateral",
        tuple(trigger_joint.axis) == (0.0, 1.0, 0.0),
        details=f"trigger axis was {trigger_joint.axis}",
    )
    ctx.check(
        "pump_axis_is_longitudinal",
        tuple(pump_joint.axis) == (1.0, 0.0, 0.0),
        details=f"pump axis was {pump_joint.axis}",
    )

    finger_rest = ctx.part_element_world_aabb(trigger, elem="finger_pad")
    assert finger_rest is not None
    finger_rest_center = tuple((lo + hi) * 0.5 for lo, hi in zip(finger_rest[0], finger_rest[1]))
    with ctx.pose({trigger_joint: math.radians(33.0)}):
        finger_squeezed = ctx.part_element_world_aabb(trigger, elem="finger_pad")
        assert finger_squeezed is not None
        finger_squeezed_center = tuple(
            (lo + hi) * 0.5 for lo, hi in zip(finger_squeezed[0], finger_squeezed[1])
        )
        ctx.check(
            "trigger_retracts_when_squeezed",
            finger_squeezed_center[0] < finger_rest_center[0] - 0.010,
            details=(
                f"finger pad center x moved from {finger_rest_center[0]:.4f} "
                f"to {finger_squeezed_center[0]:.4f}"
            ),
        )
        ctx.check(
            "trigger_rises_when_squeezed",
            finger_squeezed_center[2] > finger_rest_center[2] + 0.010,
            details=(
                f"finger pad center z moved from {finger_rest_center[2]:.4f} "
                f"to {finger_squeezed_center[2]:.4f}"
            ),
        )
        ctx.expect_contact(trigger, head_body, name="trigger_contact_squeezed")
        ctx.fail_if_parts_overlap_in_current_pose(name="trigger_upper_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="trigger_upper_pose_no_floating")

    pump_rest = ctx.part_world_position(pump_link)
    assert pump_rest is not None
    with ctx.pose({pump_joint: 0.008}):
        pump_advanced = ctx.part_world_position(pump_link)
        assert pump_advanced is not None
        ctx.check(
            "pump_link_advances_forward",
            pump_advanced[0] > pump_rest[0] + 0.007,
            details=f"pump link x advanced from {pump_rest[0]:.4f} to {pump_advanced[0]:.4f}",
        )
        ctx.expect_contact(pump_link, head_body, name="pump_link_contact_extended")
        ctx.fail_if_parts_overlap_in_current_pose(name="pump_upper_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="pump_upper_pose_no_floating")

    with ctx.pose({trigger_joint: math.radians(33.0), pump_joint: 0.008}):
        ctx.expect_gap(
            pump_link,
            trigger,
            axis="x",
            positive_elem="plunger_pad",
            negative_elem="actuator_pad",
            max_gap=0.004,
            max_penetration=0.0,
            name="trigger_nearly_drives_plunger",
        )
        ctx.expect_overlap(
            pump_link,
            trigger,
            axes="yz",
            elem_a="plunger_pad",
            elem_b="actuator_pad",
            min_overlap=0.004,
            name="trigger_plunger_alignment",
        )

    selector_rest = ctx.part_element_world_aabb(nozzle_tip, elem="selector_tab")
    assert selector_rest is not None
    selector_rest_center = tuple((lo + hi) * 0.5 for lo, hi in zip(selector_rest[0], selector_rest[1]))
    with ctx.pose({nozzle_joint: 0.95}):
        selector_open = ctx.part_element_world_aabb(nozzle_tip, elem="selector_tab")
        assert selector_open is not None
        selector_open_center = tuple((lo + hi) * 0.5 for lo, hi in zip(selector_open[0], selector_open[1]))
        ctx.check(
            "nozzle_selector_rotation_is_visible",
            abs(selector_open_center[2] - selector_rest_center[2]) > 0.005,
            details=(
                f"selector tab z moved from {selector_rest_center[2]:.4f} "
                f"to {selector_open_center[2]:.4f}"
            ),
        )
        ctx.expect_contact(nozzle_tip, head_body, name="nozzle_tip_contact_rotated")
        ctx.fail_if_parts_overlap_in_current_pose(name="nozzle_rotated_no_overlap")
        ctx.fail_if_isolated_parts(name="nozzle_rotated_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
