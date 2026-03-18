from __future__ import annotations

from math import pi

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import cadquery as cq

from sdk_hybrid import (
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
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

MAIN_LENGTH = 0.300
CARD_HEIGHT = 0.126
SHROUD_HEIGHT = 0.118
SHROUD_THICKNESS = 0.022
CORE_LENGTH = 0.282
CORE_HEIGHT = 0.102
CORE_THICKNESS = 0.014
PCB_LENGTH = 0.286
PCB_HEIGHT = 0.104
PCB_THICKNESS = 0.0022
BACKPLATE_LENGTH = 0.292
BACKPLATE_HEIGHT = 0.114
BACKPLATE_THICKNESS = 0.0022
BODY_FRONT_Y = 0.004

BRACKET_THICKNESS = 0.002
BRACKET_DEPTH = 0.026
BRACKET_HEIGHT = 0.120

FAN_CENTERS = (-0.092, 0.0, 0.092)
FAN_OPENING_RADIUS = 0.0415
FAN_BEZEL_RADIUS = 0.046
FAN_THICKNESS = 0.009
FAN_RING_RADIUS = 0.0405
FAN_RING_INNER_RADIUS = 0.0345
FAN_HUB_RADIUS = 0.011
FAN_BLADE_COUNT = 11


def _make_shroud_shape():
    shell = cq.Workplane("XY").box(MAIN_LENGTH, SHROUD_THICKNESS, SHROUD_HEIGHT)
    shell = shell.edges("|Y").fillet(0.004)

    opening_cutters = (
        cq.Workplane("XZ", origin=(0.0, -0.020, 0.0))
        .pushPoints([(x, 0.0) for x in FAN_CENTERS])
        .circle(FAN_OPENING_RADIUS)
        .extrude(0.040)
    )
    shell = shell.cut(opening_cutters)

    bezels = (
        cq.Workplane(
            "XZ",
            origin=(0.0, SHROUD_THICKNESS / 2.0 - 0.003, 0.0),
        )
        .pushPoints([(x, 0.0) for x in FAN_CENTERS])
        .circle(FAN_BEZEL_RADIUS)
        .circle(FAN_OPENING_RADIUS + 0.004)
        .extrude(0.003)
    )

    top_bar = cq.Workplane(
        "XY",
        origin=(0.0, SHROUD_THICKNESS / 2.0 - 0.0015, 0.036),
    ).box(0.212, 0.003, 0.016)

    nose_block = cq.Workplane(
        "XY",
        origin=(MAIN_LENGTH / 2.0 - 0.026, SHROUD_THICKNESS / 2.0 - 0.0015, -0.010),
    ).box(0.044, 0.003, 0.034)

    side_notch = cq.Workplane(
        "XY",
        origin=(MAIN_LENGTH / 2.0 - 0.012, 0.0, -0.038),
    ).box(0.022, SHROUD_THICKNESS + 0.006, 0.020)

    return shell.union(bezels).union(top_bar).union(nose_block).cut(side_notch)


def _make_bracket_shape():
    bracket = cq.Workplane("XY").box(BRACKET_THICKNESS, BRACKET_DEPTH, BRACKET_HEIGHT)

    front_lip = cq.Workplane(
        "XY",
        origin=(0.0, BRACKET_DEPTH / 2.0 - 0.0015, 0.0),
    ).box(BRACKET_THICKNESS * 1.6, 0.003, BRACKET_HEIGHT * 0.94)

    mount_tab = cq.Workplane(
        "XY",
        origin=(0.0, BRACKET_DEPTH / 2.0 - 0.003, BRACKET_HEIGHT / 2.0 - 0.010),
    ).box(BRACKET_THICKNESS * 1.6, 0.006, 0.020)

    bottom_foot = cq.Workplane(
        "XY",
        origin=(0.0, -BRACKET_DEPTH / 2.0 + 0.004, -BRACKET_HEIGHT / 2.0 + 0.013),
    ).box(BRACKET_THICKNESS * 1.6, 0.008, 0.018)

    bracket = bracket.union(front_lip).union(mount_tab).union(bottom_foot)

    port_cutters = (
        cq.Workplane("YZ", origin=(-BRACKET_THICKNESS, 0.0, 0.0))
        .pushPoints(
            [
                (0.004, 0.032),
                (0.004, 0.004),
                (0.004, -0.024),
            ]
        )
        .rect(0.015, 0.020)
        .extrude(BRACKET_THICKNESS * 3.0)
    )
    hdmi_cutter = (
        cq.Workplane("YZ", origin=(-BRACKET_THICKNESS, 0.0, 0.0))
        .center(0.003, -0.053)
        .rect(0.017, 0.010)
        .extrude(BRACKET_THICKNESS * 3.0)
    )
    vent_cutters = (
        cq.Workplane("YZ", origin=(-BRACKET_THICKNESS, 0.0, 0.0))
        .pushPoints(
            [
                (0.010, 0.054),
                (0.010, 0.040),
                (0.010, -0.074),
                (0.010, -0.088),
            ]
        )
        .rect(0.006, 0.016)
        .extrude(BRACKET_THICKNESS * 3.0)
    )
    screw_hole = (
        cq.Workplane(
            "YZ",
            origin=(-BRACKET_THICKNESS, BRACKET_DEPTH / 2.0 - 0.003, BRACKET_HEIGHT / 2.0 - 0.010),
        )
        .circle(0.0022)
        .extrude(BRACKET_THICKNESS * 3.0)
    )

    return bracket.cut(port_cutters).cut(hdmi_cutter).cut(vent_cutters).cut(screw_hole)


def _make_fan_shape():
    ring = (
        cq.Workplane("XZ", origin=(0.0, -FAN_THICKNESS / 2.0, 0.0))
        .circle(FAN_RING_RADIUS)
        .circle(FAN_RING_INNER_RADIUS)
        .extrude(FAN_THICKNESS)
    )
    hub = (
        cq.Workplane("XZ", origin=(0.0, -FAN_THICKNESS / 2.0, 0.0))
        .circle(FAN_HUB_RADIUS)
        .extrude(FAN_THICKNESS)
    )
    cap = cq.Workplane("XZ", origin=(0.0, -0.001, 0.0)).circle(FAN_HUB_RADIUS * 0.82).extrude(0.005)
    axle = cq.Workplane("XZ", origin=(0.0, -0.008, 0.0)).circle(0.005).extrude(0.012)
    blade = (
        cq.Workplane("XZ", origin=(0.0, -FAN_THICKNESS / 2.0, 0.0))
        .moveTo(FAN_HUB_RADIUS * 0.55, -0.0045)
        .lineTo(FAN_RING_INNER_RADIUS * 0.58, -0.012)
        .lineTo(FAN_RING_INNER_RADIUS * 0.98, -0.002)
        .lineTo(FAN_RING_INNER_RADIUS * 0.80, 0.0105)
        .lineTo(FAN_HUB_RADIUS * 0.95, 0.0105)
        .close()
        .extrude(FAN_THICKNESS)
    )

    rotor = ring.union(hub).union(cap).union(axle)
    for blade_index in range(FAN_BLADE_COUNT):
        rotor = rotor.union(
            blade.rotate(
                (0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                360.0 * blade_index / FAN_BLADE_COUNT,
            )
        )
    return rotor


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="graphics_card", assets=ASSETS)

    model.material("shroud_graphite", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("fan_black", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("heatsink_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("brushed_metal", rgba=(0.62, 0.64, 0.68, 1.0))
    model.material("backplate_metal", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("pcb_green", rgba=(0.08, 0.11, 0.08, 1.0))
    model.material("connector_gold", rgba=(0.80, 0.66, 0.24, 1.0))
    model.material("accent_silver", rgba=(0.55, 0.57, 0.60, 1.0))

    shroud_mesh = mesh_from_cadquery(
        _make_shroud_shape(), "graphics_card_shroud.obj", assets=ASSETS
    )
    bracket_mesh = mesh_from_cadquery(
        _make_bracket_shape(), "graphics_card_bracket.obj", assets=ASSETS
    )
    fan_mesh = mesh_from_cadquery(_make_fan_shape(), "graphics_card_fan.obj", assets=ASSETS)

    body = model.part("card_body")
    body.visual(
        shroud_mesh,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y, 0.0)),
        material="shroud_graphite",
    )
    body.visual(
        Box((CORE_LENGTH, CORE_THICKNESS, CORE_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material="heatsink_dark",
    )
    body.visual(
        Box((PCB_LENGTH, PCB_THICKNESS, PCB_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.0081, -0.004)),
        material="pcb_green",
    )
    body.visual(
        Box((BACKPLATE_LENGTH, BACKPLATE_THICKNESS, BACKPLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.0141, 0.002)),
        material="backplate_metal",
    )
    body.visual(
        Box((0.214, 0.003, 0.016)),
        origin=Origin(xyz=(0.0, -0.0152, 0.031)),
        material="accent_silver",
    )
    body.visual(
        Box((0.134, 0.003, 0.014)),
        origin=Origin(xyz=(0.058, -0.0152, -0.024)),
        material="accent_silver",
    )
    body.visual(
        Box((0.210, 0.008, 0.012)),
        origin=Origin(xyz=(0.018, 0.006, 0.055)),
        material="accent_silver",
    )
    body.visual(
        Box((0.022, 0.013, 0.012)),
        origin=Origin(xyz=(0.105, 0.0005, 0.056)),
        material="heatsink_dark",
    )
    body.visual(
        Box((0.024, 0.024, 0.084)),
        origin=Origin(xyz=(MAIN_LENGTH / 2.0 - 0.012, 0.0005, -0.002)),
        material="heatsink_dark",
    )
    body.visual(
        Box((0.092, 0.004, 0.010)),
        origin=Origin(xyz=(-0.010, -0.0081, -0.055)),
        material="connector_gold",
    )
    for fan_x in FAN_CENTERS:
        body.visual(
            Cylinder(radius=0.0055, length=0.014),
            origin=Origin(
                xyz=(fan_x, -0.001, 0.0),
                rpy=(-pi / 2.0, 0.0, 0.0),
            ),
            material="heatsink_dark",
        )
    body.inertial = Inertial.from_geometry(
        Box((MAIN_LENGTH, 0.030, CARD_HEIGHT)),
        mass=1.35,
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
    )

    bracket = model.part("io_bracket")
    bracket.visual(bracket_mesh, material="brushed_metal")
    bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_THICKNESS, BRACKET_DEPTH, BRACKET_HEIGHT)),
        mass=0.08,
    )

    fan_names = ("fan_left", "fan_center", "fan_right")
    fan_parts = []
    for fan_name in fan_names:
        fan_part = model.part(fan_name)
        fan_part.visual(fan_mesh, material="fan_black")
        fan_part.inertial = Inertial.from_geometry(
            Cylinder(radius=FAN_RING_RADIUS, length=FAN_THICKNESS),
            mass=0.05,
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        )
        fan_parts.append(fan_part)

    model.articulation(
        "body_to_io_bracket",
        ArticulationType.FIXED,
        parent=body,
        child=bracket,
        origin=Origin(xyz=(-(MAIN_LENGTH + BRACKET_THICKNESS) / 2.0, 0.0, 0.0)),
    )

    for fan_name, fan_part, fan_x in zip(fan_names, fan_parts, FAN_CENTERS):
        model.articulation(
            f"body_to_{fan_name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=fan_part,
            origin=Origin(xyz=(fan_x, BODY_FRONT_Y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=220.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.allow_overlap(
        "card_body",
        "io_bracket",
        reason="metal bracket is seated tightly into the card body root mount",
    )
    ctx.allow_overlap(
        "card_body",
        "fan_left",
        reason="fan rotor sits closely within the shroud opening and collision hulls are conservative",
    )
    ctx.allow_overlap(
        "card_body",
        "fan_center",
        reason="fan rotor sits closely within the shroud opening and collision hulls are conservative",
    )
    ctx.allow_overlap(
        "card_body",
        "fan_right",
        reason="fan rotor sits closely within the shroud opening and collision hulls are conservative",
    )
    ctx.allow_overlap(
        "fan_left",
        "fan_center",
        reason="adjacent large fan envelopes are nearly tangent and convex-hull overlap can be conservative",
    )
    ctx.allow_overlap(
        "fan_center",
        "fan_right",
        reason="adjacent large fan envelopes are nearly tangent and convex-hull overlap can be conservative",
    )
    ctx.check_no_overlaps(
        max_pose_samples=96,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.expect_aabb_contact("io_bracket", "card_body")
    ctx.expect_aabb_overlap("io_bracket", "card_body", axes="yz", min_overlap=0.02)
    ctx.expect_aabb_gap(
        "card_body",
        "io_bracket",
        axis="x",
        max_gap=0.0015,
        max_penetration=0.001,
    )

    ctx.expect_aabb_overlap("fan_left", "card_body", axes="xz", min_overlap=0.078)
    ctx.expect_aabb_overlap("fan_center", "card_body", axes="xz", min_overlap=0.078)
    ctx.expect_aabb_overlap("fan_right", "card_body", axes="xz", min_overlap=0.078)
    ctx.expect_aabb_contact("fan_left", "card_body")
    ctx.expect_aabb_contact("fan_center", "card_body")
    ctx.expect_aabb_contact("fan_right", "card_body")
    ctx.expect_origin_distance("fan_left", "fan_center", axes="y", max_dist=0.001)
    ctx.expect_origin_distance("fan_left", "fan_center", axes="z", max_dist=0.001)
    ctx.expect_origin_distance("fan_center", "fan_right", axes="y", max_dist=0.001)
    ctx.expect_origin_distance("fan_center", "fan_right", axes="z", max_dist=0.001)
    ctx.expect_aabb_gap(
        "fan_center",
        "fan_left",
        axis="x",
        max_gap=0.012,
        max_penetration=0.007,
    )
    ctx.expect_aabb_gap(
        "fan_right",
        "fan_center",
        axis="x",
        max_gap=0.012,
        max_penetration=0.007,
    )

    with ctx.pose(
        body_to_fan_left=1.1,
        body_to_fan_center=2.4,
        body_to_fan_right=0.6,
    ):
        ctx.expect_aabb_overlap("fan_left", "card_body", axes="xz", min_overlap=0.078)
        ctx.expect_aabb_overlap("fan_center", "card_body", axes="xz", min_overlap=0.078)
        ctx.expect_aabb_overlap("fan_right", "card_body", axes="xz", min_overlap=0.078)
        ctx.expect_aabb_gap(
            "fan_center",
            "fan_left",
            axis="x",
            max_gap=0.012,
            max_penetration=0.007,
        )
        ctx.expect_aabb_gap(
            "fan_right",
            "fan_center",
            axis="x",
            max_gap=0.012,
            max_penetration=0.007,
        )

    with ctx.pose(
        body_to_fan_left=3.2,
        body_to_fan_center=0.7,
        body_to_fan_right=5.1,
    ):
        ctx.expect_aabb_contact("fan_left", "card_body")
        ctx.expect_aabb_contact("fan_center", "card_body")
        ctx.expect_aabb_contact("fan_right", "card_body")
        ctx.expect_aabb_gap(
            "fan_center",
            "fan_left",
            axis="x",
            max_gap=0.012,
            max_penetration=0.007,
        )
        ctx.expect_aabb_gap(
            "fan_right",
            "fan_center",
            axis="x",
            max_gap=0.012,
            max_penetration=0.007,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
