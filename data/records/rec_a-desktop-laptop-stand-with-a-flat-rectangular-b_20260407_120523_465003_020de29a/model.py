from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_DEPTH = 0.26
BASE_WIDTH = 0.29
BASE_THICKNESS = 0.008
BASE_REAR_OVERHANG = 0.035
BASE_TOP_Z = -0.018
BASE_PLATE_CENTER_Z = BASE_TOP_Z - BASE_THICKNESS / 2.0

PIVOT_SPACING = 0.18
LINK_DX = 0.10
LINK_DZ = 0.125
LINK_LENGTH = math.hypot(LINK_DX, LINK_DZ)

TRAY_REAR_OVERHANG = 0.025
TRAY_FRONT_OVERHANG = 0.05
TRAY_DEPTH = PIVOT_SPACING + TRAY_REAR_OVERHANG + TRAY_FRONT_OVERHANG
TRAY_WIDTH = 0.285
TRAY_THICKNESS = 0.004
TRAY_BOTTOM_Z = 0.022
TRAY_PLATE_CENTER_Z = TRAY_BOTTOM_Z + TRAY_THICKNESS / 2.0

BRACKET_Y = 0.132
BRACKET_THICKNESS = 0.006
LINK_Y = 0.126
LINK_THICKNESS = 0.006
LINK_BAR_HEIGHT = 0.018
ANCHOR_Y = 0.121
ANCHOR_THICKNESS = 0.004

BASE_BOSS_RADIUS = 0.0145
TRAY_BOSS_RADIUS = 0.014
LINK_BOSS_RADIUS = 0.0135
ANCHOR_BOSS_RADIUS = 0.01
BRACE_RADIUS = 0.006

LIP_SIZE = (0.015, 0.04, 0.012)
RAISED_ANGLE = 0.42


def _fuse_all(solids: list[cq.Workplane]) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _base_or_tray_bracket(
    *,
    pivot_x: float,
    y_center: float,
    web_height: float,
    web_center_z: float,
    web_length: float,
    boss_radius: float,
) -> cq.Workplane:
    web = cq.Workplane("XY").box(web_length, BRACKET_THICKNESS, web_height).translate(
        (pivot_x, y_center, web_center_z)
    )
    boss = (
        cq.Workplane("XZ")
        .circle(boss_radius)
        .extrude(BRACKET_THICKNESS / 2.0, both=True)
        .translate((pivot_x, y_center, 0.0))
    )
    return web.union(boss)


def _link_side_plate(dx: float, dz: float, y_center: float) -> cq.Workplane:
    angle_deg = math.degrees(math.atan2(dz, dx))
    bar = cq.Workplane("XY").box(LINK_LENGTH, LINK_THICKNESS, LINK_BAR_HEIGHT).translate(
        (LINK_LENGTH / 2.0, y_center, 0.0)
    )
    bar = bar.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -angle_deg)
    proximal = (
        cq.Workplane("XZ")
        .circle(LINK_BOSS_RADIUS)
        .extrude(LINK_THICKNESS / 2.0, both=True)
        .translate((0.0, y_center, 0.0))
    )
    distal = (
        cq.Workplane("XZ")
        .circle(LINK_BOSS_RADIUS)
        .extrude(LINK_THICKNESS / 2.0, both=True)
        .translate((dx, y_center, dz))
    )
    return bar.union(proximal).union(distal)


def _build_base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_DEPTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((BASE_DEPTH / 2.0 - BASE_REAR_OVERHANG, 0.0, BASE_PLATE_CENTER_Z))
        .edges("|Z")
        .fillet(0.01)
    )

    brackets: list[cq.Workplane] = []
    for y_center in (-BRACKET_Y, BRACKET_Y):
        brackets.append(
            _base_or_tray_bracket(
                pivot_x=0.0,
                y_center=y_center,
                web_height=0.034,
                web_center_z=-0.005,
                web_length=0.022,
                boss_radius=BASE_BOSS_RADIUS,
            )
        )

    front_anchor_pedestal = (
        cq.Workplane("XY")
        .box(0.032, 0.065, 0.008)
        .translate((PIVOT_SPACING, 0.0, -0.014))
        .edges("|Z")
        .fillet(0.004)
    )

    return _fuse_all([plate, front_anchor_pedestal, *brackets])


def _build_tray_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(TRAY_DEPTH, TRAY_WIDTH, TRAY_THICKNESS)
        .translate((TRAY_DEPTH / 2.0 - TRAY_REAR_OVERHANG, 0.0, TRAY_PLATE_CENTER_Z))
        .edges("|Z")
        .fillet(0.008)
    )

    slot_points = [
        (x, y)
        for x in (-0.055, 0.0, 0.055)
        for y in (-0.075, 0.0, 0.075)
    ]
    plate = (
        plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(slot_points)
        .slot2D(0.045, 0.011, 0.0)
        .cutThruAll()
    )

    rails = [
        cq.Workplane("XY").box(TRAY_DEPTH - 0.02, 0.012, 0.012).translate(
            (TRAY_DEPTH / 2.0 - TRAY_REAR_OVERHANG, side_y, 0.028)
        )
        for side_y in (-0.122, 0.122)
    ]

    brackets: list[cq.Workplane] = []
    for pivot_x in (0.0, PIVOT_SPACING):
        for y_center in (-BRACKET_Y, BRACKET_Y):
            brackets.append(
                _base_or_tray_bracket(
                    pivot_x=pivot_x,
                    y_center=y_center,
                    web_height=0.03,
                    web_center_z=0.010,
                    web_length=0.02,
                    boss_radius=TRAY_BOSS_RADIUS,
                )
            )

    rear_bridge = cq.Workplane("XY").box(0.018, 0.16, 0.008).translate((0.0, 0.0, 0.019))
    front_bridge = cq.Workplane("XY").box(0.018, 0.16, 0.008).translate(
        (PIVOT_SPACING, 0.0, 0.019)
    )

    return _fuse_all([plate, rear_bridge, front_bridge, *rails, *brackets])


def _build_link_frame(dx: float, dz: float) -> cq.Workplane:
    left = _link_side_plate(dx, dz, LINK_Y)
    right = _link_side_plate(dx, dz, -LINK_Y)
    brace_mid = (
        cq.Workplane("XZ")
        .circle(BRACE_RADIUS)
        .extrude(LINK_Y, both=True)
        .translate((dx * 0.52, 0.0, dz * 0.52 - 0.004))
    )
    brace_near = (
        cq.Workplane("XZ")
        .circle(BRACE_RADIUS * 0.9)
        .extrude(LINK_Y, both=True)
        .translate((dx * 0.24, 0.0, dz * 0.24 - 0.003))
    )
    return _fuse_all([left, right, brace_mid, brace_near])


def _build_front_anchor_shape() -> cq.Workplane:
    shaft = (
        cq.Workplane("XZ")
        .circle(0.0045)
        .extrude(ANCHOR_Y, both=True)
        .translate((0.0, 0.0, 0.0))
    )
    caps = [
        cq.Workplane("XZ").circle(ANCHOR_BOSS_RADIUS).extrude(ANCHOR_THICKNESS / 2.0, both=True).translate(
            (0.0, side_y, 0.0)
        )
        for side_y in (-ANCHOR_Y, ANCHOR_Y)
    ]
    center_collar = cq.Workplane("XZ").circle(0.0075).extrude(0.018, both=True)
    return _fuse_all([shaft, center_collar, *caps])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_laptop_stand")

    model.material("base_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("tray_graphite", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("link_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("pivot_dark", rgba=(0.21, 0.22, 0.24, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_build_base_shape(), "base"), material="base_black", name="base_body")
    base.inertial = Inertial.from_geometry(
        Box((BASE_DEPTH, BASE_WIDTH, 0.05)),
        mass=1.45,
        origin=Origin(xyz=(BASE_DEPTH / 2.0 - BASE_REAR_OVERHANG, 0.0, -0.01)),
    )

    lower_frame = model.part("lower_link_frame")
    lower_frame.visual(
        mesh_from_cadquery(_build_link_frame(LINK_DX, LINK_DZ), "lower_link_frame"),
        material="link_aluminum",
        name="lower_frame_mesh",
    )
    lower_frame.inertial = Inertial.from_geometry(
        Box((LINK_DX + 0.03, 0.26, 0.03)),
        mass=0.32,
        origin=Origin(xyz=(LINK_DX / 2.0, 0.0, LINK_DZ / 2.0)),
    )

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_build_tray_shape(), "tray"), material="tray_graphite", name="tray_shell")
    tray.visual(
        Box(LIP_SIZE),
        origin=Origin(
            xyz=(PIVOT_SPACING + TRAY_FRONT_OVERHANG - LIP_SIZE[0] / 2.0, -0.07, TRAY_BOTTOM_Z + 0.0085)
        ),
        material="pivot_dark",
        name="left_retaining_lip",
    )
    tray.visual(
        Box(LIP_SIZE),
        origin=Origin(
            xyz=(PIVOT_SPACING + TRAY_FRONT_OVERHANG - LIP_SIZE[0] / 2.0, 0.07, TRAY_BOTTOM_Z + 0.0085)
        ),
        material="pivot_dark",
        name="right_retaining_lip",
    )
    tray.visual(
        Box((0.02, 0.03, 0.003)),
        origin=Origin(xyz=(0.02, 0.0, TRAY_BOTTOM_Z + 0.0005)),
        material="tray_graphite",
        name="rear_level_marker",
    )
    tray.visual(
        Box((0.02, 0.03, 0.003)),
        origin=Origin(xyz=(PIVOT_SPACING + 0.02, 0.0, TRAY_BOTTOM_Z + 0.0005)),
        material="tray_graphite",
        name="front_level_marker",
    )
    tray.inertial = Inertial.from_geometry(
        Box((TRAY_DEPTH, TRAY_WIDTH, 0.04)),
        mass=0.85,
        origin=Origin(xyz=(TRAY_DEPTH / 2.0 - TRAY_REAR_OVERHANG, 0.0, 0.02)),
    )

    upper_frame = model.part("upper_link_frame")
    upper_frame.visual(
        mesh_from_cadquery(_build_link_frame(-LINK_DX, -LINK_DZ), "upper_link_frame"),
        material="link_aluminum",
        name="upper_frame_mesh",
    )
    upper_frame.inertial = Inertial.from_geometry(
        Box((LINK_DX + 0.03, 0.26, 0.03)),
        mass=0.30,
        origin=Origin(xyz=(-LINK_DX / 2.0, 0.0, -LINK_DZ / 2.0)),
    )

    front_anchor = model.part("front_anchor")
    front_anchor.visual(
        mesh_from_cadquery(_build_front_anchor_shape(), "front_anchor"),
        material="pivot_dark",
        name="front_anchor_mesh",
    )
    front_anchor.inertial = Inertial.from_geometry(
        Box((0.03, 0.25, 0.03)),
        mass=0.14,
        origin=Origin(),
    )

    model.articulation(
        "base_to_lower_frame",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.3, upper=0.7, effort=25.0, velocity=1.5),
    )
    model.articulation(
        "lower_frame_to_tray",
        ArticulationType.REVOLUTE,
        parent=lower_frame,
        child=tray,
        origin=Origin(xyz=(LINK_DX, 0.0, LINK_DZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.7, upper=0.3, effort=20.0, velocity=1.5),
    )
    model.articulation(
        "tray_to_upper_frame",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=upper_frame,
        origin=Origin(xyz=(PIVOT_SPACING, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.3, upper=0.7, effort=20.0, velocity=1.5),
    )
    model.articulation(
        "upper_frame_to_front_anchor",
        ArticulationType.REVOLUTE,
        parent=upper_frame,
        child=front_anchor,
        origin=Origin(xyz=(-LINK_DX, 0.0, -LINK_DZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.0, upper=1.0, effort=10.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lower_frame = object_model.get_part("lower_link_frame")
    tray = object_model.get_part("tray")
    upper_frame = object_model.get_part("upper_link_frame")
    front_anchor = object_model.get_part("front_anchor")

    ctx.allow_overlap(
        base,
        lower_frame,
        reason="The rear pivot is represented with interleaved bracket bosses and link knuckles that intentionally share pivot volume.",
    )
    ctx.allow_overlap(
        lower_frame,
        tray,
        reason="The rear tray pivot is modeled as overlapping coaxial pivot bosses rather than a separate pin and washers.",
    )
    ctx.allow_overlap(
        tray,
        upper_frame,
        reason="The front tray pivot is represented with overlapping pivot knuckles to keep the four-bar linkage visually compact.",
    )
    ctx.allow_overlap(
        upper_frame,
        front_anchor,
        reason="The front ground-side pivot uses overlapping simplified pivot bosses between the upper link assembly and the seated anchor shaft.",
    )

    lower_joint = object_model.get_articulation("base_to_lower_frame")
    tray_joint = object_model.get_articulation("lower_frame_to_tray")
    upper_joint = object_model.get_articulation("tray_to_upper_frame")
    anchor_joint = object_model.get_articulation("upper_frame_to_front_anchor")

    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.095,
        name="tray sits clearly above the base at rest",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="xy",
        min_overlap=0.14,
        name="tray remains broadly over the base footprint",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    rest_anchor_pos = ctx.part_world_position(front_anchor)

    with ctx.pose(
        {
            lower_joint: RAISED_ANGLE,
            tray_joint: -RAISED_ANGLE,
            upper_joint: RAISED_ANGLE,
            anchor_joint: -RAISED_ANGLE,
        }
    ):
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.125,
            name="raised tray gains additional clearance above the base",
        )
        raised_tray_pos = ctx.part_world_position(tray)
        raised_anchor_pos = ctx.part_world_position(front_anchor)
        rear_marker = ctx.part_element_world_aabb(tray, elem="rear_level_marker")
        front_marker = ctx.part_element_world_aabb(tray, elem="front_level_marker")

        level_ok = False
        level_details = f"rear={rear_marker}, front={front_marker}"
        if rear_marker is not None and front_marker is not None:
            rear_z = (rear_marker[0][2] + rear_marker[1][2]) / 2.0
            front_z = (front_marker[0][2] + front_marker[1][2]) / 2.0
            level_ok = abs(front_z - rear_z) <= 0.004
            level_details = f"rear_z={rear_z:.4f}, front_z={front_z:.4f}"
        ctx.check("tray remains approximately level in the raised pose", level_ok, level_details)

        lift_ok = False
        lift_details = f"rest={rest_tray_pos}, raised={raised_tray_pos}"
        if rest_tray_pos is not None and raised_tray_pos is not None:
            lift_ok = raised_tray_pos[2] > rest_tray_pos[2] + 0.025
        ctx.check("parallel links lift the tray upward", lift_ok, lift_details)

        anchor_ok = False
        anchor_details = f"rest={rest_anchor_pos}, raised={raised_anchor_pos}"
        if rest_anchor_pos is not None and raised_anchor_pos is not None:
            anchor_dx = abs(rest_anchor_pos[0] - raised_anchor_pos[0])
            anchor_dz = abs(rest_anchor_pos[2] - raised_anchor_pos[2])
            anchor_ok = anchor_dx <= 0.003 and anchor_dz <= 0.003
            anchor_details = (
                f"rest={rest_anchor_pos}, raised={raised_anchor_pos}, "
                f"dx={anchor_dx:.4f}, dz={anchor_dz:.4f}"
            )
        ctx.check("front pivot anchor stays seated at the base pivot line", anchor_ok, anchor_details)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
