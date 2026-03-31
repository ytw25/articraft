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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trigger_spray_bottle_utility", assets=ASSETS)

    bottle_hdpe = model.material("bottle_hdpe", rgba=(0.70, 0.75, 0.63, 0.78))
    bottle_trim = model.material("bottle_trim", rgba=(0.52, 0.57, 0.47, 1.0))
    body_poly = model.material("body_poly", rgba=(0.16, 0.17, 0.18, 1.0))
    trigger_accent = model.material("trigger_accent", rgba=(0.82, 0.38, 0.16, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.36, 0.38, 0.41, 1.0))
    zinc = model.material("zinc", rgba=(0.63, 0.66, 0.69, 1.0))
    tube_clear = model.material("tube_clear", rgba=(0.86, 0.88, 0.90, 0.45))

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def xz_section(width: float, height: float, radius: float, y_pos: float):
        return [(x, y_pos, z) for x, z in rounded_rect_profile(width, height, radius)]

    def yz_section(points_2d: list[tuple[float, float]], x_pos: float):
        return [(x_pos, y_pos, z_pos) for y_pos, z_pos in points_2d]

    trigger_side = [
        (0.006, 0.006),
        (-0.006, 0.022),
        (-0.018, 0.028),
        (-0.028, 0.022),
        (-0.034, 0.010),
        (-0.036, -0.008),
        (-0.032, -0.032),
        (-0.026, -0.054),
        (-0.018, -0.076),
        (-0.008, -0.094),
        (0.006, -0.098),
        (0.016, -0.090),
        (0.020, -0.072),
        (0.016, -0.048),
        (0.010, -0.022),
        (0.006, -0.004),
    ]
    trigger_mesh = save_mesh(
        "utility_trigger.obj",
        section_loft(
            [
                yz_section(trigger_side, -0.021),
                yz_section(trigger_side, 0.021),
            ]
        ),
    )
    head_cover_mesh = save_mesh(
        "utility_head_cover.obj",
        section_loft(
            [
                xz_section(0.042, 0.020, 0.006, 0.004),
                xz_section(0.060, 0.028, 0.010, -0.026),
                xz_section(0.050, 0.024, 0.008, -0.060),
            ]
        ),
    )

    bottle = model.part("bottle_shell")
    bottle.visual(
        Box((0.084, 0.060, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=bottle_hdpe,
        name="bottom_plate",
    )
    bottle.visual(
        Box((0.084, 0.0036, 0.148)),
        origin=Origin(xyz=(0.0, -0.0282, 0.078)),
        material=bottle_hdpe,
        name="front_wall",
    )
    bottle.visual(
        Box((0.084, 0.0036, 0.148)),
        origin=Origin(xyz=(0.0, 0.0282, 0.078)),
        material=bottle_hdpe,
        name="rear_wall",
    )
    bottle.visual(
        Box((0.0036, 0.0528, 0.148)),
        origin=Origin(xyz=(-0.0402, 0.0, 0.078)),
        material=bottle_hdpe,
        name="left_wall",
    )
    bottle.visual(
        Box((0.0036, 0.0528, 0.148)),
        origin=Origin(xyz=(0.0402, 0.0, 0.078)),
        material=bottle_hdpe,
        name="right_wall",
    )
    bottle.visual(
        Box((0.074, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, -0.023, 0.166), rpy=(math.radians(-34.0), 0.0, 0.0)),
        material=bottle_hdpe,
        name="front_shoulder",
    )
    bottle.visual(
        Box((0.074, 0.004, 0.040)),
        origin=Origin(xyz=(0.0, 0.023, 0.166), rpy=(math.radians(34.0), 0.0, 0.0)),
        material=bottle_hdpe,
        name="rear_shoulder",
    )
    bottle.visual(
        Box((0.040, 0.004, 0.046)),
        origin=Origin(xyz=(-0.025, 0.0, 0.166), rpy=(0.0, math.radians(34.0), 0.0)),
        material=bottle_hdpe,
        name="left_shoulder",
    )
    bottle.visual(
        Box((0.040, 0.004, 0.046)),
        origin=Origin(xyz=(0.025, 0.0, 0.166), rpy=(0.0, math.radians(-34.0), 0.0)),
        material=bottle_hdpe,
        name="right_shoulder",
    )
    bottle.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.178)),
        material=bottle_trim,
        name="neck_land",
    )
    bottle.visual(
        Cylinder(radius=0.0155, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.191)),
        material=bottle_hdpe,
        name="neck_finish",
    )
    bottle.visual(
        Cylinder(radius=0.0170, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.1825)),
        material=bottle_trim,
        name="thread_ridge_lower",
    )
    bottle.visual(
        Cylinder(radius=0.0170, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.1895)),
        material=bottle_trim,
        name="thread_ridge_upper",
    )
    for rib_index, rib_x in enumerate((-0.026, 0.026)):
        bottle.visual(
            Box((0.010, 0.005, 0.110)),
            origin=Origin(xyz=(rib_x, -0.0305, 0.073)),
            material=bottle_trim,
            name=f"front_rib_{rib_index}",
        )
        bottle.visual(
            Box((0.010, 0.005, 0.110)),
            origin=Origin(xyz=(rib_x, 0.0305, 0.073)),
            material=bottle_trim,
            name=f"rear_rib_{rib_index}",
        )
    bottle.visual(
        Box((0.052, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, -0.030, 0.026)),
        material=bottle_trim,
        name="front_bumper",
    )
    bottle.visual(
        Box((0.052, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.030, 0.026)),
        material=bottle_trim,
        name="rear_bumper",
    )
    bottle.inertial = Inertial.from_geometry(
        Box((0.090, 0.066, 0.205)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.1025)),
    )

    closure = model.part("neck_closure")
    closure.visual(
        Box((0.050, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, -0.020, 0.007)),
        material=body_poly,
        name="closure_front_band",
    )
    closure.visual(
        Box((0.050, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.020, 0.007)),
        material=body_poly,
        name="closure_rear_band",
    )
    closure.visual(
        Box((0.006, 0.034, 0.014)),
        origin=Origin(xyz=(-0.022, 0.0, 0.007)),
        material=body_poly,
        name="closure_left_band",
    )
    closure.visual(
        Box((0.006, 0.034, 0.014)),
        origin=Origin(xyz=(0.022, 0.0, 0.007)),
        material=body_poly,
        name="closure_right_band",
    )
    closure.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=body_poly,
        name="neck_socket",
    )
    closure.visual(
        Box((0.044, 0.036, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=body_poly,
        name="head_seat",
    )
    for rib_index, (rib_x, rib_y, yaw) in enumerate(
        (
            (-0.015, -0.025, 0.0),
            (0.015, -0.025, 0.0),
            (-0.015, 0.025, 0.0),
            (0.015, 0.025, 0.0),
        )
    ):
        closure.visual(
            Box((0.008, 0.004, 0.014)),
            origin=Origin(xyz=(rib_x, rib_y, 0.007), rpy=(0.0, 0.0, yaw)),
            material=dark_steel,
            name=f"closure_grip_{rib_index}",
        )
    closure.inertial = Inertial.from_geometry(
        Box((0.054, 0.054, 0.026)),
        mass=0.06,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    head = model.part("sprayer_head")
    head.visual(
        Box((0.030, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.010, 0.010)),
        material=body_poly,
        name="rear_mount_block",
    )
    head.visual(
        Box((0.056, 0.050, 0.018)),
        origin=Origin(xyz=(0.0, -0.012, 0.024)),
        material=body_poly,
        name="top_cover",
    )
    head.visual(
        Box((0.006, 0.048, 0.022)),
        origin=Origin(xyz=(-0.025, -0.014, 0.014)),
        material=body_poly,
        name="left_side_plate",
    )
    head.visual(
        Box((0.006, 0.048, 0.022)),
        origin=Origin(xyz=(0.025, -0.014, 0.014)),
        material=body_poly,
        name="right_side_plate",
    )
    head.visual(
        Box((0.004, 0.016, 0.008)),
        origin=Origin(xyz=(-0.010, -0.042, 0.013)),
        material=dark_steel,
        name="left_guide_rail",
    )
    head.visual(
        Box((0.004, 0.016, 0.008)),
        origin=Origin(xyz=(0.010, -0.042, 0.013)),
        material=dark_steel,
        name="right_guide_rail",
    )
    head.visual(
        Box((0.022, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.048, 0.016)),
        material=body_poly,
        name="front_bridge",
    )
    head.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, -0.058, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_poly,
        name="nozzle_collar",
    )
    head.visual(
        Box((0.010, 0.018, 0.014)),
        origin=Origin(xyz=(-0.014, 0.000, 0.010), rpy=(0.0, math.radians(10.0), 0.0)),
        material=body_poly,
        name="left_neck_gusset",
    )
    head.visual(
        Box((0.010, 0.018, 0.014)),
        origin=Origin(xyz=(0.014, 0.000, 0.010), rpy=(0.0, math.radians(-10.0), 0.0)),
        material=body_poly,
        name="right_neck_gusset",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(-0.028, -0.042, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_trigger_boss",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.028, -0.042, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_trigger_boss",
    )
    head.visual(
        Cylinder(radius=0.0042, length=0.002),
        origin=Origin(xyz=(-0.032, -0.042, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="left_trigger_fastener",
    )
    head.visual(
        Cylinder(radius=0.0042, length=0.002),
        origin=Origin(xyz=(0.032, -0.042, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=zinc,
        name="right_trigger_fastener",
    )
    head.visual(
        Cylinder(radius=0.0038, length=0.002),
        origin=Origin(xyz=(0.0, -0.058, 0.020), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=zinc,
        name="nozzle_fastener",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.070, 0.080, 0.050)),
        mass=0.15,
        origin=Origin(xyz=(0.0, -0.016, 0.018)),
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.028, 0.012, 0.048)),
        origin=Origin(xyz=(0.0, -0.016, -0.030)),
        material=trigger_accent,
        name="trigger_shell",
    )
    trigger.visual(
        Box((0.038, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.014, -0.006)),
        material=trigger_accent,
        name="pivot_web",
    )
    trigger.visual(
        Box((0.034, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.020, -0.058)),
        material=trigger_accent,
        name="finger_pad",
    )
    trigger.visual(
        Box((0.018, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, -0.010, -0.006)),
        material=trigger_accent,
        name="actuator_horn",
    )
    trigger.visual(
        Box((0.008, 0.024, 0.024)),
        origin=Origin(xyz=(-0.018, 0.002, -0.012)),
        material=trigger_accent,
        name="left_cheek",
    )
    trigger.visual(
        Box((0.008, 0.024, 0.024)),
        origin=Origin(xyz=(0.018, 0.002, -0.012)),
        material=trigger_accent,
        name="right_cheek",
    )
    trigger.visual(
        Cylinder(radius=0.0036, length=0.006),
        origin=Origin(xyz=(-0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_pivot_stub",
    )
    trigger.visual(
        Cylinder(radius=0.0036, length=0.006),
        origin=Origin(xyz=(0.024, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_pivot_stub",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.046, 0.038, 0.074)),
        mass=0.05,
        origin=Origin(xyz=(0.0, -0.018, -0.030)),
    )

    pump_plunger = model.part("pump_plunger")
    pump_plunger.visual(
        Box((0.010, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="plunger_rod",
    )
    pump_plunger.visual(
        Box((0.016, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
        material=zinc,
        name="plunger_pad",
    )
    pump_plunger.inertial = Inertial.from_geometry(
        Box((0.018, 0.018, 0.010)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.003, 0.0)),
    )

    nozzle_tip = model.part("nozzle_tip")
    nozzle_tip.visual(
        Box((0.014, 0.007, 0.014)),
        origin=Origin(xyz=(0.0, -0.0035, 0.0)),
        material=body_poly,
        name="selector_base",
    )
    nozzle_tip.visual(
        Box((0.012, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.011, 0.0)),
        material=body_poly,
        name="selector_block",
    )
    nozzle_tip.visual(
        Box((0.006, 0.008, 0.018)),
        origin=Origin(xyz=(0.007, -0.012, 0.008)),
        material=trigger_accent,
        name="selector_fin",
    )
    nozzle_tip.inertial = Inertial.from_geometry(
        Box((0.016, 0.022, 0.020)),
        mass=0.015,
        origin=Origin(xyz=(0.0035, -0.010, 0.004)),
    )

    pickup_tube = model.part("pickup_tube")
    pickup_tube.visual(
        Cylinder(radius=0.003, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, -0.0765)),
        material=tube_clear,
        name="dip_tube",
    )
    pickup_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.003, length=0.145),
        mass=0.01,
        origin=Origin(xyz=(0.0, 0.0, -0.0765)),
    )

    model.articulation(
        "bottle_to_neck_closure",
        ArticulationType.FIXED,
        parent=bottle,
        child=closure,
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
    )
    model.articulation(
        "closure_to_head",
        ArticulationType.FIXED,
        parent=closure,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )
    model.articulation(
        "closure_to_pickup_tube",
        ArticulationType.FIXED,
        parent=closure,
        child=pickup_tube,
        origin=Origin(),
    )
    model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head,
        child=trigger,
        origin=Origin(xyz=(0.0, -0.042, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(18.0),
        ),
    )
    model.articulation(
        "head_to_pump_plunger",
        ArticulationType.PRISMATIC,
        parent=head,
        child=pump_plunger,
        origin=Origin(xyz=(0.0, -0.040, 0.014)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.08,
            lower=0.0,
            upper=0.008,
        ),
    )
    model.articulation(
        "head_to_nozzle_tip",
        ArticulationType.REVOLUTE,
        parent=head,
        child=nozzle_tip,
        origin=Origin(xyz=(0.0, -0.058, 0.020)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(80.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bottle = object_model.get_part("bottle_shell")
    closure = object_model.get_part("neck_closure")
    head = object_model.get_part("sprayer_head")
    trigger = object_model.get_part("trigger")
    pump_plunger = object_model.get_part("pump_plunger")
    nozzle_tip = object_model.get_part("nozzle_tip")
    pickup_tube = object_model.get_part("pickup_tube")

    trigger_hinge = object_model.get_articulation("head_to_trigger")
    pump_slide = object_model.get_articulation("head_to_pump_plunger")
    nozzle_adjust = object_model.get_articulation("head_to_nozzle_tip")

    def aabb_center(aabb):
        return tuple((aabb[0][index] + aabb[1][index]) * 0.5 for index in range(3))

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
    ctx.allow_overlap(
        bottle,
        closure,
        elem_a="neck_finish",
        elem_b="neck_socket",
        reason="Threaded closure intentionally wraps around the bottle neck finish.",
    )
    ctx.allow_overlap(
        bottle,
        closure,
        elem_a="neck_finish",
        elem_b="head_seat",
        reason="The overcap seat nests tightly around the neck finish as part of the rugged threaded closure molding.",
    )
    ctx.allow_overlap(
        pump_plunger,
        head,
        reason="The pump plunger is intentionally captured within the head guide and bridge structure.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_origin_gap(
        closure,
        bottle,
        axis="z",
        min_gap=0.17,
        max_gap=0.19,
        name="closure_above_bottle_origin",
    )
    ctx.expect_overlap(closure, bottle, axes="xy", min_overlap=0.032, name="closure_over_bottle_neck")
    ctx.expect_contact(head, closure, name="head_seats_on_closure")
    ctx.expect_overlap(head, closure, axes="xy", min_overlap=0.020, name="head_over_closure")
    ctx.expect_contact(trigger, head, name="trigger_supported_by_pivots")
    ctx.expect_contact(pump_plunger, head, name="pump_plunger_guided_in_head")
    ctx.expect_contact(nozzle_tip, head, name="nozzle_tip_supported_in_collar")
    ctx.expect_within(
        pickup_tube,
        bottle,
        axes="xy",
        margin=0.001,
        name="pickup_tube_inside_bottle_plan",
    )
    ctx.expect_overlap(
        trigger,
        pump_plunger,
        axes="xz",
        min_overlap=0.004,
        elem_a="actuator_horn",
        elem_b="plunger_pad",
        name="trigger_linkage_path_visible",
    )

    trigger_rest_aabb = ctx.part_element_world_aabb(trigger, elem="finger_pad")
    pump_rest_pos = ctx.part_world_position(pump_plunger)
    nozzle_rest_aabb = ctx.part_element_world_aabb(nozzle_tip, elem="selector_fin")
    if trigger_rest_aabb is not None and pump_rest_pos is not None and nozzle_rest_aabb is not None:
        trigger_rest_center = aabb_center(trigger_rest_aabb)
        nozzle_rest_center = aabb_center(nozzle_rest_aabb)

        with ctx.pose({trigger_hinge: trigger_hinge.motion_limits.upper, pump_slide: pump_slide.motion_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="actuated_trigger_pump_no_overlap")
            ctx.fail_if_isolated_parts(name="actuated_trigger_pump_no_floating")
            ctx.expect_contact(trigger, head, name="trigger_supported_when_pulled")
            ctx.expect_contact(pump_plunger, head, name="plunger_supported_when_advanced")
            trigger_pulled_aabb = ctx.part_element_world_aabb(trigger, elem="finger_pad")
            pump_pulled_pos = ctx.part_world_position(pump_plunger)
            if trigger_pulled_aabb is not None and pump_pulled_pos is not None:
                trigger_pulled_center = aabb_center(trigger_pulled_aabb)
                ctx.check(
                    "trigger_finger_pad_moves_back",
                    trigger_pulled_center[1] > trigger_rest_center[1] + 0.018,
                    details=f"rest_y={trigger_rest_center[1]:.4f}, pulled_y={trigger_pulled_center[1]:.4f}",
                )
                ctx.check(
                    "pump_plunger_advances_forward",
                    pump_pulled_pos[1] < pump_rest_pos[1] - 0.006,
                    details=f"rest_y={pump_rest_pos[1]:.4f}, pulled_y={pump_pulled_pos[1]:.4f}",
                )

        with ctx.pose({nozzle_adjust: nozzle_adjust.motion_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="nozzle_adjust_no_overlap")
            ctx.fail_if_isolated_parts(name="nozzle_adjust_no_floating")
            ctx.expect_contact(nozzle_tip, head, name="nozzle_adjust_stays_supported")
            ctx.expect_contact(nozzle_tip, head, name="nozzle_adjust_keeps_face_contact")
            nozzle_open_aabb = ctx.part_element_world_aabb(nozzle_tip, elem="selector_fin")
            if nozzle_open_aabb is not None:
                nozzle_open_center = aabb_center(nozzle_open_aabb)
                ctx.check(
                    "nozzle_selector_rotates_visibly",
                    abs(nozzle_open_center[0] - nozzle_rest_center[0]) > 0.006
                    or abs(nozzle_open_center[2] - nozzle_rest_center[2]) > 0.006,
                    details=(
                        f"rest_center={nozzle_rest_center}, "
                        f"open_center={nozzle_open_center}"
                    ),
                )

        with ctx.pose(
            {
                trigger_hinge: trigger_hinge.motion_limits.upper,
                pump_slide: pump_slide.motion_limits.upper,
                nozzle_adjust: nozzle_adjust.motion_limits.upper,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="full_operating_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="full_operating_pose_no_floating")
            ctx.expect_contact(trigger, head, name="trigger_supported_full_pose")
            ctx.expect_contact(pump_plunger, head, name="plunger_supported_full_pose")
            ctx.expect_contact(nozzle_tip, head, name="nozzle_supported_full_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
