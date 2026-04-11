from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LEG_LENGTH = 0.68
BASE_LEG_WIDTH = 0.075
BASE_LEG_HEIGHT = 0.040
BASE_LEG_BOTTOM_Z = 0.074
LEG_CENTER_X = 0.060
LEG_CENTER_Y = 0.230

CROSSBAR_LENGTH_X = 0.100
CROSSBAR_WIDTH_Y = 2.0 * LEG_CENTER_Y
CROSSBAR_CENTER_X = -0.220

UPRIGHT_OUTER_X = 0.050
UPRIGHT_OUTER_Y = 0.038
UPRIGHT_WALL = 0.003
UPRIGHT_HEIGHT = 0.410
UPRIGHT_CENTER_X = -0.180

POST_X = 0.040
POST_Y = 0.028
POST_LENGTH = 0.580
POST_BOTTOM_BELOW_FRAME = 0.360
BRIDGE_LIFT = 0.270
HINGE_AXIS_Z = 0.218

TABLETOP_LENGTH = 0.780
TABLETOP_WIDTH = 0.420
TABLETOP_THICKNESS = 0.022

CASTER_FRONT_X = 0.340
CASTER_REAR_X = -0.240
CASTER_SWIVEL_Z = BASE_LEG_BOTTOM_Z
WHEEL_RADIUS = 0.032
WHEEL_WIDTH = 0.016
WHEEL_AXLE_DROP = 0.042
WHEEL_AXLE_X = 0.050


def _box_at(size_x: float, size_y: float, size_z: float, x: float, y: float, z0: float) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z, centered=(True, True, False)).translate((x, y, z0))


def _rect_tube_at(
    outer_x: float,
    outer_y: float,
    height: float,
    wall: float,
    x: float,
    y: float,
    z0: float,
) -> cq.Workplane:
    outer = _box_at(outer_x, outer_y, height, x, y, z0)
    inner = _box_at(outer_x - 2.0 * wall, outer_y - 2.0 * wall, height + 0.002, x, y, z0 - 0.001)
    return outer.cut(inner)


def _rect_ring_at(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    height: float,
    x: float,
    y: float,
    z0: float,
) -> cq.Workplane:
    outer = _box_at(outer_x, outer_y, height, x, y, z0)
    inner = _box_at(inner_x, inner_y, height + 0.002, x, y, z0 - 0.001)
    return outer.cut(inner)


def _base_frame_shape() -> cq.Workplane:
    left_leg = _box_at(BASE_LEG_LENGTH, BASE_LEG_WIDTH, BASE_LEG_HEIGHT, LEG_CENTER_X, LEG_CENTER_Y, BASE_LEG_BOTTOM_Z)
    right_leg = _box_at(BASE_LEG_LENGTH, BASE_LEG_WIDTH, BASE_LEG_HEIGHT, LEG_CENTER_X, -LEG_CENTER_Y, BASE_LEG_BOTTOM_Z)
    crossbar = _box_at(
        CROSSBAR_LENGTH_X,
        CROSSBAR_WIDTH_Y + BASE_LEG_WIDTH,
        BASE_LEG_HEIGHT,
        CROSSBAR_CENTER_X,
        0.0,
        BASE_LEG_BOTTOM_Z,
    )
    left_collar = _box_at(0.085, 0.080, 0.052, UPRIGHT_CENTER_X, LEG_CENTER_Y, BASE_LEG_BOTTOM_Z + BASE_LEG_HEIGHT - 0.002)
    right_collar = _box_at(0.085, 0.080, 0.052, UPRIGHT_CENTER_X, -LEG_CENTER_Y, BASE_LEG_BOTTOM_Z + BASE_LEG_HEIGHT - 0.002)
    return left_leg.union(right_leg).union(crossbar).union(left_collar).union(right_collar)


def _upright_shape(y_pos: float) -> cq.Workplane:
    return _rect_tube_at(
        UPRIGHT_OUTER_X,
        UPRIGHT_OUTER_Y,
        UPRIGHT_HEIGHT,
        UPRIGHT_WALL,
        UPRIGHT_CENTER_X,
        y_pos,
        BASE_LEG_BOTTOM_Z + BASE_LEG_HEIGHT,
    )


def _bridge_post_shape(y_pos: float) -> cq.Workplane:
    post_top_above_frame = POST_LENGTH - POST_BOTTOM_BELOW_FRAME
    post_center_z = (post_top_above_frame - POST_BOTTOM_BELOW_FRAME) / 2.0
    return _box_at(POST_X, POST_Y, POST_LENGTH, 0.0, y_pos, post_center_z - POST_LENGTH / 2.0)


def _bridge_carriage_shape(y_pos: float) -> cq.Workplane:
    return _box_at(0.060, 0.046, 0.016, 0.0, y_pos, -0.001)


def _bridge_head_shape() -> cq.Workplane:
    beam = _box_at(0.060, 0.500, 0.028, -0.050, 0.0, 0.148)
    rear_web = _box_at(0.012, 0.500, 0.060, -0.014, 0.0, 0.148)
    left_cheek = _box_at(0.012, 0.040, 0.060, -0.006, -0.145, 0.158)
    right_cheek = _box_at(0.012, 0.040, 0.060, -0.006, 0.145, 0.158)
    bridge = beam.union(rear_web).union(left_cheek).union(right_cheek)
    for y_pos in (-0.145, 0.145):
        barrel = (
            cq.Workplane("XY")
            .cylinder(0.045, 0.007, centered=(True, True, True))
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
            .translate((0.0, y_pos, HINGE_AXIS_Z))
        )
        bridge = bridge.union(barrel)
    return bridge


def _bridge_support_pads_shape() -> cq.Workplane:
    shelf = _box_at(0.120, 0.280, 0.008, 0.070, 0.0, 0.203002)
    web = _box_at(0.020, 0.250, 0.044, 0.010, 0.0, 0.160)
    left_knee = _box_at(0.028, 0.055, 0.025, 0.056, -0.112, 0.179)
    right_knee = _box_at(0.028, 0.055, 0.025, 0.056, 0.112, 0.179)
    return shelf.union(web).union(left_knee).union(right_knee)


def _tabletop_shape() -> cq.Workplane:
    board = _box_at(TABLETOP_LENGTH - 0.014, TABLETOP_WIDTH, TABLETOP_THICKNESS, (TABLETOP_LENGTH + 0.014) / 2.0, 0.0, 0.006)
    top = board
    hinge_leaf = _box_at(0.020, 0.180, 0.006, 0.010, 0.0, 0.002)
    center_barrel = (
        cq.Workplane("XY")
        .cylinder(0.140, 0.007, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, 0.0, 0.0))
    )
    top = top.union(hinge_leaf).union(center_barrel)
    return top


def _caster_body_shape() -> cq.Workplane:
    top_plate = _box_at(0.056, 0.040, 0.006, 0.0, 0.0, -0.006)
    swivel_barrel = cq.Workplane("XY").cylinder(0.022, 0.012, centered=(True, True, False)).translate((0.0, 0.0, -0.028))
    stem = _box_at(0.014, 0.036, 0.022, 0.004, 0.0, -0.050)
    left_arm = _box_at(0.060, 0.004, 0.034, 0.031, 0.017, -0.076)
    right_arm = _box_at(0.060, 0.004, 0.034, 0.031, -0.017, -0.076)
    left_stub = (
        cq.Workplane("XY")
        .cylinder(0.008, 0.004, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((WHEEL_AXLE_X, 0.013, -WHEEL_AXLE_DROP))
    )
    right_stub = (
        cq.Workplane("XY")
        .cylinder(0.008, 0.004, centered=(True, True, True))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((WHEEL_AXLE_X, -0.013, -WHEEL_AXLE_DROP))
    )
    return top_plate.union(swivel_barrel).union(stem).union(left_arm).union(right_arm).union(left_stub).union(right_stub)


def _add_caster(model: ArticulatedObject, base, prefix: str, x_pos: float, y_pos: float) -> None:
    caster = model.part(f"{prefix}_caster")
    caster.visual(
        Box((0.056, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material="caster_black",
        name="top_plate",
    )
    caster.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material="caster_black",
        name="swivel_barrel",
    )
    caster.visual(
        Box((0.014, 0.030, 0.024)),
        origin=Origin(xyz=(0.004, 0.0, -0.040)),
        material="caster_black",
        name="stem",
    )
    caster.visual(
        Box((0.036, 0.004, 0.024)),
        origin=Origin(xyz=(0.028, 0.014, -0.054)),
        material="caster_black",
        name="left_arm",
    )
    caster.visual(
        Box((0.036, 0.004, 0.024)),
        origin=Origin(xyz=(0.028, -0.014, -0.054)),
        material="caster_black",
        name="right_arm",
    )
    caster.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(WHEEL_AXLE_X, 0.014, -WHEEL_AXLE_DROP), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="wheel_hub",
        name="left_stub",
    )
    caster.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(WHEEL_AXLE_X, -0.014, -WHEEL_AXLE_DROP), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="wheel_hub",
        name="right_stub",
    )

    wheel = model.part(f"{prefix}_wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="wheel_rubber",
        name="tread",
    )
    wheel.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="wheel_hub",
        name="hub",
    )

    model.articulation(
        f"{prefix}_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=caster,
        origin=Origin(xyz=(x_pos, y_pos, CASTER_SWIVEL_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=12.0),
    )
    model.articulation(
        f"{prefix}_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=caster,
        child=wheel,
        origin=Origin(xyz=(WHEEL_AXLE_X, 0.0, -WHEEL_AXLE_DROP)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=28.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rehab_overbed_table")

    model.material("steel_gray", rgba=(0.74, 0.76, 0.78, 1.0))
    model.material("powder_charcoal", rgba=(0.23, 0.24, 0.26, 1.0))
    model.material("satin_chrome", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("laminate_top", rgba=(0.84, 0.82, 0.76, 1.0))
    model.material("caster_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("wheel_rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    model.material("wheel_hub", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("pad_black", rgba=(0.11, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_frame_shape(), "base_frame"), material="powder_charcoal", name="base_frame")
    base.visual(mesh_from_cadquery(_upright_shape(LEG_CENTER_Y), "left_upright"), material="steel_gray", name="left_upright")
    base.visual(mesh_from_cadquery(_upright_shape(-LEG_CENTER_Y), "right_upright"), material="steel_gray", name="right_upright")

    bridge = model.part("bridge")
    bridge.visual(mesh_from_cadquery(_bridge_post_shape(LEG_CENTER_Y), "left_bridge_post"), material="satin_chrome", name="left_post")
    bridge.visual(mesh_from_cadquery(_bridge_post_shape(-LEG_CENTER_Y), "right_bridge_post"), material="satin_chrome", name="right_post")
    bridge.visual(mesh_from_cadquery(_bridge_carriage_shape(LEG_CENTER_Y), "left_bridge_carriage"), material="steel_gray", name="left_carriage")
    bridge.visual(mesh_from_cadquery(_bridge_carriage_shape(-LEG_CENTER_Y), "right_bridge_carriage"), material="steel_gray", name="right_carriage")
    bridge.visual(mesh_from_cadquery(_bridge_head_shape(), "bridge_head"), material="powder_charcoal", name="bridge_head")
    bridge.visual(
        Box((0.120, 0.280, 0.006)),
        origin=Origin(xyz=(0.070, 0.0, HINGE_AXIS_Z - 0.003)),
        material="pad_black",
        name="support_pads",
    )
    bridge.visual(
        Box((0.016, 0.250, 0.030)),
        origin=Origin(xyz=(0.008, 0.0, 0.197)),
        material="powder_charcoal",
        name="support_web",
    )
    bridge.visual(
        Box((0.030, 0.055, 0.042)),
        origin=Origin(xyz=(0.050, -0.112, 0.197)),
        material="powder_charcoal",
        name="left_support_knee",
    )
    bridge.visual(
        Box((0.030, 0.055, 0.042)),
        origin=Origin(xyz=(0.050, 0.112, 0.197)),
        material="powder_charcoal",
        name="right_support_knee",
    )

    tabletop = model.part("tabletop")
    tabletop.visual(
        Box((TABLETOP_LENGTH - 0.014, TABLETOP_WIDTH, TABLETOP_THICKNESS)),
        origin=Origin(xyz=((TABLETOP_LENGTH + 0.014) / 2.0, 0.0, TABLETOP_THICKNESS / 2.0)),
        material="laminate_top",
        name="panel",
    )
    tabletop.visual(
        Box((0.020, 0.180, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.003)),
        material="powder_charcoal",
        name="hinge_leaf",
    )
    tabletop.visual(
        Cylinder(radius=0.007, length=0.140),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="powder_charcoal",
        name="hinge_barrel",
    )

    model.articulation(
        "base_to_bridge_lift",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(UPRIGHT_CENTER_X, 0.0, BASE_LEG_BOTTOM_Z + BASE_LEG_HEIGHT + UPRIGHT_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=BRIDGE_LIFT, effort=80.0, velocity=0.18),
    )
    model.articulation(
        "bridge_to_tabletop_tilt",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=tabletop,
        origin=Origin(xyz=(0.0, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.95, effort=18.0, velocity=1.2),
    )

    _add_caster(model, base, "front_left", CASTER_FRONT_X, LEG_CENTER_Y)
    _add_caster(model, base, "front_right", CASTER_FRONT_X, -LEG_CENTER_Y)
    _add_caster(model, base, "rear_left", CASTER_REAR_X, LEG_CENTER_Y)
    _add_caster(model, base, "rear_right", CASTER_REAR_X, -LEG_CENTER_Y)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    required_parts = {
        "base",
        "bridge",
        "tabletop",
        "front_left_caster",
        "front_right_caster",
        "rear_left_caster",
        "rear_right_caster",
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    }
    required_joints = {
        "base_to_bridge_lift",
        "bridge_to_tabletop_tilt",
        "front_left_caster_swivel",
        "front_right_caster_swivel",
        "rear_left_caster_swivel",
        "rear_right_caster_swivel",
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    }

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

    part_names = {part.name for part in object_model.parts}
    joint_names = {articulation.name for articulation in object_model.articulations}
    for part_name in sorted(required_parts):
        ctx.check(f"part {part_name} exists", part_name in part_names, details=f"parts={sorted(part_names)}")
    for joint_name in sorted(required_joints):
        ctx.check(f"joint {joint_name} exists", joint_name in joint_names, details=f"joints={sorted(joint_names)}")

    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    tabletop = object_model.get_part("tabletop")
    lift_joint = object_model.get_articulation("base_to_bridge_lift")
    tilt_joint = object_model.get_articulation("bridge_to_tabletop_tilt")

    ctx.check("bridge lift axis is vertical", tuple(lift_joint.axis) == (0.0, 0.0, 1.0), details=f"axis={lift_joint.axis}")
    ctx.check("tabletop hinge axis opens upward", tuple(tilt_joint.axis) == (0.0, -1.0, 0.0), details=f"axis={tilt_joint.axis}")

    ctx.expect_contact(bridge, base, elem_a="left_carriage", elem_b="left_upright", name="left carriage is seated on the left upright")
    ctx.expect_contact(bridge, base, elem_a="right_carriage", elem_b="right_upright", name="right carriage is seated on the right upright")
    ctx.expect_within(
        bridge,
        base,
        axes="xy",
        inner_elem="left_post",
        outer_elem="left_upright",
        margin=0.0,
        name="left bridge post stays aligned inside the left upright footprint",
    )
    ctx.expect_within(
        bridge,
        base,
        axes="xy",
        inner_elem="right_post",
        outer_elem="right_upright",
        margin=0.0,
        name="right bridge post stays aligned inside the right upright footprint",
    )
    ctx.expect_overlap(
        bridge,
        base,
        axes="z",
        elem_a="left_post",
        elem_b="left_upright",
        min_overlap=0.30,
        name="left post retains deep insertion at the lowest height",
    )
    ctx.expect_overlap(
        bridge,
        base,
        axes="z",
        elem_a="right_post",
        elem_b="right_upright",
        min_overlap=0.30,
        name="right post retains deep insertion at the lowest height",
    )
    ctx.expect_contact(tabletop, bridge, elem_a="panel", elem_b="support_pads", name="closed tabletop rests on the bridge support pads")

    rest_bridge_pos = ctx.part_world_position(bridge)
    rest_panel_aabb = ctx.part_element_world_aabb(tabletop, elem="panel")
    lift_upper = 0.0 if lift_joint.motion_limits is None or lift_joint.motion_limits.upper is None else lift_joint.motion_limits.upper
    tilt_upper = 0.0 if tilt_joint.motion_limits is None or tilt_joint.motion_limits.upper is None else tilt_joint.motion_limits.upper
    raised_bridge_pos = None

    with ctx.pose({lift_joint: lift_upper}):
        ctx.expect_within(
            bridge,
            base,
            axes="xy",
            inner_elem="left_post",
            outer_elem="left_upright",
            margin=0.0,
            name="left bridge post stays laterally guided when raised",
        )
        ctx.expect_within(
            bridge,
            base,
            axes="xy",
            inner_elem="right_post",
            outer_elem="right_upright",
            margin=0.0,
            name="right bridge post stays laterally guided when raised",
        )
        ctx.expect_overlap(
            bridge,
            base,
            axes="z",
            elem_a="left_post",
            elem_b="left_upright",
            min_overlap=0.085,
            name="left post keeps retained insertion at full height",
        )
        ctx.expect_overlap(
            bridge,
            base,
            axes="z",
            elem_a="right_post",
            elem_b="right_upright",
            min_overlap=0.085,
            name="right post keeps retained insertion at full height",
        )
        raised_bridge_pos = ctx.part_world_position(bridge)

    ctx.check(
        "bridge rises upward through the lift stroke",
        rest_bridge_pos is not None and raised_bridge_pos is not None and raised_bridge_pos[2] > rest_bridge_pos[2] + 0.20,
        details=f"rest={rest_bridge_pos}, raised={raised_bridge_pos}",
    )

    open_panel_aabb = None
    with ctx.pose({tilt_joint: tilt_upper}):
        open_panel_aabb = ctx.part_element_world_aabb(tabletop, elem="panel")

    ctx.check(
        "tabletop front edge lifts when tilted",
        rest_panel_aabb is not None and open_panel_aabb is not None and open_panel_aabb[1][2] > rest_panel_aabb[1][2] + 0.22,
        details=f"closed={rest_panel_aabb}, open={open_panel_aabb}",
    )

    caster_specs = (
        ("front_left", "front_left_caster", "front_left_wheel"),
        ("front_right", "front_right_caster", "front_right_wheel"),
        ("rear_left", "rear_left_caster", "rear_left_wheel"),
        ("rear_right", "rear_right_caster", "rear_right_wheel"),
    )
    for prefix, caster_name, wheel_name in caster_specs:
        caster = object_model.get_part(caster_name)
        wheel = object_model.get_part(wheel_name)
        swivel = object_model.get_articulation(f"{prefix}_caster_swivel")
        spin = object_model.get_articulation(f"{prefix}_wheel_spin")
        ctx.check(f"{prefix} caster swivel axis is vertical", tuple(swivel.axis) == (0.0, 0.0, 1.0), details=f"axis={swivel.axis}")
        ctx.check(f"{prefix} wheel spin axis is lateral", tuple(spin.axis) == (0.0, 1.0, 0.0), details=f"axis={spin.axis}")
        ctx.expect_contact(base, caster, elem_a="base_frame", elem_b="top_plate", name=f"{prefix} caster body is mounted to the H-base")
        ctx.expect_contact(caster, wheel, elem_a="left_stub", elem_b="hub", name=f"{prefix} left axle stub supports the wheel hub")
        ctx.expect_contact(caster, wheel, elem_a="right_stub", elem_b="hub", name=f"{prefix} right axle stub supports the wheel hub")
        ctx.expect_origin_gap(
            caster,
            wheel,
            axis="z",
            min_gap=0.035,
            max_gap=0.055,
            name=f"{prefix} wheel hangs below its swivel fork",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
