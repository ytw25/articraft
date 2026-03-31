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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _ring_shell_mesh(*, outer_radius: float, inner_radius: float, height: float, name: str):
    ring = LatheGeometry.from_shell_profiles(
        [(outer_radius, -0.5 * height), (outer_radius, 0.5 * height)],
        [(inner_radius, -0.5 * height), (inner_radius, 0.5 * height)],
        segments=48,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(ring, name)


def _bottle_shell_mesh():
    bottle_shell = LatheGeometry.from_shell_profiles(
        [
            (0.028, 0.000),
            (0.045, 0.008),
            (0.051, 0.058),
            (0.050, 0.138),
            (0.044, 0.192),
            (0.032, 0.214),
            (0.0185, 0.225),
            (0.0185, 0.239),
        ],
        [
            (0.021, 0.004),
            (0.042, 0.011),
            (0.046, 0.058),
            (0.045, 0.136),
            (0.039, 0.189),
            (0.028, 0.212),
            (0.0145, 0.225),
            (0.0145, 0.239),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(bottle_shell, "service_bottle_shell")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trigger_spray_bottle")

    bottle_hdpe = model.material("bottle_hdpe", rgba=(0.80, 0.84, 0.74, 0.86))
    housing_gray = model.material("housing_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    trigger_black = model.material("trigger_black", rgba=(0.17, 0.18, 0.19, 1.0))
    cartridge_black = model.material("cartridge_black", rgba=(0.22, 0.23, 0.24, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    service_orange = model.material("service_orange", rgba=(0.89, 0.48, 0.13, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        _bottle_shell_mesh(),
        material=bottle_hdpe,
        name="bottle_shell",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.239),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.1195)),
    )

    head_frame = model.part("head_frame")
    head_frame.visual(
        _ring_shell_mesh(
            outer_radius=0.026,
            inner_radius=0.0185,
            height=0.010,
            name="head_collar_band",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.244)),
        material=housing_gray,
        name="collar_band",
    )
    head_frame.visual(
        Box((0.068, 0.008, 0.044)),
        origin=Origin(xyz=(0.021, 0.018, 0.262)),
        material=housing_gray,
        name="left_cheek",
    )
    head_frame.visual(
        Box((0.068, 0.008, 0.044)),
        origin=Origin(xyz=(0.021, -0.018, 0.262)),
        material=housing_gray,
        name="right_cheek",
    )
    head_frame.visual(
        Box((0.052, 0.032, 0.012)),
        origin=Origin(xyz=(0.014, 0.0, 0.282)),
        material=housing_gray,
        name="top_cap",
    )
    head_frame.visual(
        Box((0.024, 0.032, 0.050)),
        origin=Origin(xyz=(-0.024, 0.0, 0.264)),
        material=housing_gray,
        name="back_spine",
    )
    head_frame.visual(
        Box((0.018, 0.006, 0.030)),
        origin=Origin(xyz=(0.058, 0.020, 0.269)),
        material=housing_gray,
        name="left_nose_rib",
    )
    head_frame.visual(
        Box((0.018, 0.006, 0.030)),
        origin=Origin(xyz=(0.058, -0.020, 0.269)),
        material=housing_gray,
        name="right_nose_rib",
    )
    head_frame.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.048, 0.018, 0.248), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_gray,
        name="pivot_boss_left",
    )
    head_frame.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.048, -0.018, 0.248), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=housing_gray,
        name="pivot_boss_right",
    )
    head_frame.inertial = Inertial.from_geometry(
        Box((0.09, 0.05, 0.08)),
        mass=0.20,
        origin=Origin(xyz=(0.012, 0.0, 0.255)),
    )

    pump_cartridge = model.part("pump_cartridge")
    pump_cartridge.visual(
        Box((0.036, 0.026, 0.010)),
        origin=Origin(xyz=(0.067, 0.0, 0.279)),
        material=cartridge_black,
        name="top_spine",
    )
    pump_cartridge.visual(
        Box((0.024, 0.006, 0.018)),
        origin=Origin(xyz=(0.057, 0.011, 0.266)),
        material=cartridge_black,
        name="left_side_rail",
    )
    pump_cartridge.visual(
        Box((0.024, 0.006, 0.018)),
        origin=Origin(xyz=(0.057, -0.011, 0.266)),
        material=cartridge_black,
        name="right_side_rail",
    )
    pump_cartridge.visual(
        Box((0.014, 0.026, 0.010)),
        origin=Origin(xyz=(0.092, 0.0, 0.279)),
        material=cartridge_black,
        name="front_top_block",
    )
    pump_cartridge.visual(
        Box((0.014, 0.022, 0.006)),
        origin=Origin(xyz=(0.092, 0.0, 0.255)),
        material=cartridge_black,
        name="front_bottom_crossbar",
    )
    pump_cartridge.visual(
        Box((0.014, 0.006, 0.020)),
        origin=Origin(xyz=(0.092, 0.011, 0.267)),
        material=cartridge_black,
        name="front_left_keeper",
    )
    pump_cartridge.visual(
        Box((0.014, 0.006, 0.020)),
        origin=Origin(xyz=(0.092, -0.011, 0.267)),
        material=cartridge_black,
        name="front_right_keeper",
    )
    pump_cartridge.visual(
        Box((0.024, 0.006, 0.006)),
        origin=Origin(xyz=(0.026, 0.011, 0.255)),
        material=cartridge_black,
        name="left_lower_brace",
    )
    pump_cartridge.visual(
        Box((0.024, 0.006, 0.006)),
        origin=Origin(xyz=(0.026, -0.011, 0.255)),
        material=cartridge_black,
        name="right_lower_brace",
    )
    pump_cartridge.visual(
        Box((0.014, 0.006, 0.012)),
        origin=Origin(xyz=(0.042, 0.011, 0.261)),
        material=cartridge_black,
        name="left_mid_strut",
    )
    pump_cartridge.visual(
        Box((0.014, 0.006, 0.012)),
        origin=Origin(xyz=(0.042, -0.011, 0.261)),
        material=cartridge_black,
        name="right_mid_strut",
    )
    pump_cartridge.visual(
        Box((0.018, 0.018, 0.012)),
        origin=Origin(xyz=(0.005, 0.0, 0.247)),
        material=cartridge_black,
        name="inlet_socket",
    )
    pump_cartridge.inertial = Inertial.from_geometry(
        Box((0.10, 0.04, 0.05)),
        mass=0.10,
        origin=Origin(xyz=(0.058, 0.0, 0.267)),
    )

    nozzle_tip = model.part("nozzle_tip")
    nozzle_tip.visual(
        Box((0.010, 0.022, 0.018)),
        origin=Origin(xyz=(0.104, 0.0, 0.267)),
        material=service_orange,
        name="nozzle_collar",
    )
    nozzle_tip.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.117, 0.0, 0.267), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=service_orange,
        name="nozzle_barrel",
    )
    nozzle_tip.visual(
        Box((0.004, 0.012, 0.012)),
        origin=Origin(xyz=(0.127, 0.0, 0.267)),
        material=service_orange,
        name="spray_face",
    )

    dip_tube = model.part("dip_tube")
    dip_tube.visual(
        Box((0.006, 0.006, 0.010)),
        origin=Origin(xyz=(0.005, 0.0, 0.236)),
        material=steel,
        name="tube_socket",
    )
    dip_tube.visual(
        Cylinder(radius=0.0025, length=0.170),
        origin=Origin(xyz=(0.005, 0.0, 0.146)),
        material=steel,
        name="tube_run",
    )

    trigger = model.part("trigger")
    trigger.visual(
        Box((0.018, 0.020, 0.014)),
        origin=Origin(xyz=(0.007, 0.0, 0.001)),
        material=trigger_black,
        name="upper_yoke",
    )
    trigger.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.000, 0.011, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trigger_black,
        name="pivot_collar_left",
    )
    trigger.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.000, -0.011, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trigger_black,
        name="pivot_collar_right",
    )
    trigger.visual(
        Box((0.020, 0.010, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, 0.010)),
        material=trigger_black,
        name="actuator_link",
    )
    trigger.visual(
        Box((0.008, 0.016, 0.008)),
        origin=Origin(xyz=(0.012, 0.0, 0.015)),
        material=trigger_black,
        name="actuator_pad",
    )
    trigger.visual(
        Box((0.015, 0.020, 0.022)),
        origin=Origin(xyz=(0.018, 0.0, -0.012), rpy=(0.0, 0.35, 0.0)),
        material=trigger_black,
        name="diagonal_web",
    )
    trigger.visual(
        Box((0.014, 0.020, 0.042)),
        origin=Origin(xyz=(0.028, 0.0, -0.032), rpy=(0.0, 0.08, 0.0)),
        material=trigger_black,
        name="finger_blade",
    )
    trigger.visual(
        Box((0.018, 0.020, 0.012)),
        origin=Origin(xyz=(0.035, 0.0, -0.055)),
        material=trigger_black,
        name="finger_pad",
    )
    trigger.inertial = Inertial.from_geometry(
        Box((0.06, 0.03, 0.11)),
        mass=0.06,
        origin=Origin(xyz=(0.022, 0.0, -0.028)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Box((0.008, 0.016, 0.012)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=steel,
        name="actuator_shoe",
    )
    plunger.visual(
        Box((0.008, 0.014, 0.012)),
        origin=Origin(xyz=(0.011, 0.0, 0.006)),
        material=steel,
        name="seal_block",
    )
    plunger.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.018, 0.0, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="stem",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.05, 0.02, 0.02)),
        mass=0.03,
        origin=Origin(xyz=(0.017, 0.0, 0.006)),
    )

    model.articulation(
        "bottle_to_head",
        ArticulationType.FIXED,
        parent=bottle,
        child=head_frame,
        origin=Origin(),
    )
    model.articulation(
        "head_to_pump",
        ArticulationType.FIXED,
        parent=head_frame,
        child=pump_cartridge,
        origin=Origin(),
    )
    model.articulation(
        "pump_to_nozzle",
        ArticulationType.FIXED,
        parent=pump_cartridge,
        child=nozzle_tip,
        origin=Origin(),
    )
    model.articulation(
        "pump_to_dip_tube",
        ArticulationType.FIXED,
        parent=pump_cartridge,
        child=dip_tube,
        origin=Origin(),
    )
    model.articulation(
        "head_to_trigger",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=trigger,
        origin=Origin(xyz=(0.048, 0.0, 0.248)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=0.0,
            upper=0.30,
        ),
    )
    model.articulation(
        "pump_to_plunger",
        ArticulationType.PRISMATIC,
        parent=pump_cartridge,
        child=plunger,
        origin=Origin(xyz=(0.066, 0.0, 0.261)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bottle = object_model.get_part("bottle")
    head_frame = object_model.get_part("head_frame")
    pump_cartridge = object_model.get_part("pump_cartridge")
    nozzle_tip = object_model.get_part("nozzle_tip")
    dip_tube = object_model.get_part("dip_tube")
    trigger = object_model.get_part("trigger")
    plunger = object_model.get_part("plunger")

    trigger_hinge = object_model.get_articulation("head_to_trigger")
    plunger_slide = object_model.get_articulation("pump_to_plunger")

    bottle_shell = bottle.get_visual("bottle_shell")
    collar_band = head_frame.get_visual("collar_band")
    left_cheek = head_frame.get_visual("left_cheek")
    pivot_boss_left = head_frame.get_visual("pivot_boss_left")
    pivot_boss_right = head_frame.get_visual("pivot_boss_right")
    left_side_rail = pump_cartridge.get_visual("left_side_rail")
    inlet_socket = pump_cartridge.get_visual("inlet_socket")
    nozzle_collar = nozzle_tip.get_visual("nozzle_collar")
    tube_socket = dip_tube.get_visual("tube_socket")
    trigger_pad = trigger.get_visual("actuator_pad")
    trigger_collar_left = trigger.get_visual("pivot_collar_left")
    trigger_collar_right = trigger.get_visual("pivot_collar_right")
    plunger_shoe = plunger.get_visual("actuator_shoe")

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

    ctx.expect_gap(
        head_frame,
        bottle,
        axis="z",
        positive_elem=collar_band,
        negative_elem=bottle_shell,
        max_gap=0.001,
        max_penetration=0.0,
        name="head collar seats on the bottle neck",
    )
    ctx.expect_overlap(
        head_frame,
        bottle,
        axes="xy",
        elem_a=collar_band,
        elem_b=bottle_shell,
        min_overlap=0.035,
        name="head collar wraps the bottle neck footprint",
    )
    ctx.expect_contact(
        pump_cartridge,
        head_frame,
        elem_a=left_side_rail,
        elem_b=left_cheek,
        name="pump cartridge is mounted to the service frame",
    )
    ctx.expect_contact(
        nozzle_tip,
        pump_cartridge,
        elem_a=nozzle_collar,
        elem_b=pump_cartridge.get_visual("front_top_block"),
        name="replaceable nozzle seats on the pump frame",
    )
    ctx.expect_contact(
        dip_tube,
        pump_cartridge,
        elem_a=tube_socket,
        elem_b=inlet_socket,
        name="dip tube plugs into the pump inlet",
    )
    ctx.expect_contact(
        trigger,
        head_frame,
        elem_a=trigger_collar_left,
        elem_b=pivot_boss_left,
        name="left trigger collar is carried by a real boss",
    )
    ctx.expect_contact(
        trigger,
        head_frame,
        elem_a=trigger_collar_right,
        elem_b=pivot_boss_right,
        name="right trigger collar is carried by a real boss",
    )

    with ctx.pose({trigger_hinge: 0.0, plunger_slide: 0.0}):
        ctx.expect_gap(
            plunger,
            trigger,
            axis="x",
            positive_elem=plunger_shoe,
            negative_elem=trigger_pad,
            min_gap=0.002,
            max_gap=0.005,
            name="rest pose keeps a realistic trigger service gap",
        )

    with ctx.pose({trigger_hinge: 0.30, plunger_slide: 0.003}):
        ctx.fail_if_parts_overlap_in_current_pose(name="full-pull pose stays clear")
        ctx.expect_gap(
            plunger,
            trigger,
            axis="x",
            positive_elem=plunger_shoe,
            negative_elem=trigger_pad,
            max_gap=0.001,
            max_penetration=0.0,
            name="full pull closes the visible trigger-to-pump path",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
