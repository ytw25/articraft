from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
import cadquery as cq
from math import pi

from sdk_hybrid import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
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

CARD_L = 0.315
CARD_H = 0.128
CARD_T = 0.052

SHROUD_T = 0.024
FINPACK_T = 0.024
PCB_T = 0.0026
BACKPLATE_T = 0.003

FAN_R = 0.0395
FAN_OPEN_R = 0.043
FAN_T = 0.007
FAN_HUB_R = 0.013
FAN_BLADE_COUNT = 8
FAN_XS = (-0.098, 0.0, 0.098)
FAN_Z = 0.0105

BEARING_POST_R = 0.009
BEARING_POST_L = 0.014
BEARING_POST_Z = 0.0085

BRACKET_THK = 0.0035
BRACKET_H = 0.118
BRACKET_T = 0.060

CONN_L = 0.119
CONN_H = 0.015
CONN_T = 0.0036


def _cylinder_x(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )


def _make_shroud() -> cq.Workplane:
    shroud = cq.Workplane("XY").box(CARD_L, CARD_H, SHROUD_T)
    shroud = shroud.edges("|Z").fillet(0.006)

    for fan_x in FAN_XS:
        opening = (
            cq.Workplane("XY")
            .circle(FAN_OPEN_R)
            .extrude(SHROUD_T + 0.010)
            .translate((fan_x, 0.0, -SHROUD_T / 2.0 - 0.005))
        )
        bezel = (
            cq.Workplane("XY")
            .circle(FAN_OPEN_R + 0.006)
            .circle(FAN_OPEN_R + 0.002)
            .extrude(0.002)
            .translate((fan_x, 0.0, SHROUD_T / 2.0 - 0.002))
        )
        shroud = shroud.cut(opening).union(bezel)

    accent_bar = (
        cq.Workplane("XY")
        .box(0.112, 0.010, 0.003)
        .translate((0.0, -CARD_H * 0.33, SHROUD_T / 2.0 - 0.0015))
    )
    shroud = shroud.union(accent_bar)

    for x_pos, angle in ((-0.132, -20.0), (0.132, 20.0)):
        rib = (
            cq.Workplane("XY")
            .box(0.050, 0.014, 0.003)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
            .translate((x_pos, CARD_H * 0.30, SHROUD_T / 2.0 - 0.0015))
        )
        shroud = shroud.union(rib)

    for x_pos in (0.095, 0.122):
        vent_slot = (
            cq.Workplane("XY")
            .box(0.030, 0.008, SHROUD_T + 0.004)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), -18.0)
            .translate((x_pos, CARD_H * 0.26, 0.0))
        )
        shroud = shroud.cut(vent_slot)

    lower_groove = (
        cq.Workplane("XY")
        .box(0.135, 0.007, SHROUD_T + 0.004)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 12.0)
        .translate((0.024, -CARD_H * 0.26, 0.0))
    )
    shroud = shroud.cut(lower_groove)

    return shroud.translate((0.0, 0.0, CARD_T / 2.0 - SHROUD_T / 2.0))


def _make_finpack() -> cq.Workplane:
    fin_l = CARD_L - 0.036
    fin_h = CARD_H - 0.032
    finpack = cq.Workplane("XY").box(fin_l, fin_h, FINPACK_T)

    for idx in range(11):
        y_pos = -0.036 + idx * 0.0072
        slot = (
            cq.Workplane("XY")
            .box(fin_l + 0.004, 0.0034, FINPACK_T + 0.004)
            .translate((0.0, y_pos, 0.0))
        )
        finpack = finpack.cut(slot)

    io_clearance = (
        cq.Workplane("XY")
        .box(0.040, CARD_H, FINPACK_T + 0.004)
        .translate((-CARD_L / 2.0 + 0.020, 0.0, 0.0))
    )
    finpack = finpack.cut(io_clearance)

    return finpack.translate((0.0, 0.0, -0.0035))


def _make_pcb() -> cq.Workplane:
    pcb = cq.Workplane("XY").box(CARD_L - 0.018, CARD_H - 0.020, PCB_T)
    right_notch = (
        cq.Workplane("XY")
        .box(0.030, 0.026, PCB_T + 0.002)
        .translate((CARD_L / 2.0 - 0.022, CARD_H / 2.0 - 0.013, 0.0))
    )
    left_notch = (
        cq.Workplane("XY")
        .box(0.018, 0.022, PCB_T + 0.002)
        .translate((-CARD_L / 2.0 + 0.014, -CARD_H / 2.0 + 0.011, 0.0))
    )
    pcb = pcb.cut(right_notch).cut(left_notch)
    return pcb.translate((0.0, 0.0, -0.017))


def _make_backplate() -> cq.Workplane:
    backplate = cq.Workplane("XY").box(CARD_L - 0.006, CARD_H - 0.008, BACKPLATE_T)
    slot = (
        cq.Workplane("XY")
        .box(0.076, 0.006, BACKPLATE_T + 0.002)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 18.0)
        .translate((0.070, -0.018, 0.0))
    )
    backplate = backplate.cut(slot)
    return backplate.translate((0.0, 0.0, -0.0195))


def _make_power_housing() -> cq.Workplane:
    housing = cq.Workplane("XY").box(0.038, 0.012, 0.008)
    for x_pos in (-0.008, 0.008):
        notch = (
            cq.Workplane("XY")
            .box(0.004, 0.014, 0.005)
            .translate((x_pos, -0.001, 0.001))
        )
        housing = housing.cut(notch)
    return housing.translate((0.066, CARD_H / 2.0 - 0.008, 0.018))


def _make_fan_rotor() -> cq.Workplane:
    blade = (
        cq.Workplane("XY")
        .moveTo(FAN_HUB_R * 1.08, -0.0012)
        .lineTo(FAN_HUB_R * 1.20, -0.0068)
        .threePointArc((FAN_R * 0.56, -0.021), (FAN_R * 0.92, -0.0008))
        .threePointArc((FAN_R * 0.62, 0.016), (FAN_HUB_R * 1.12, 0.0020))
        .close()
        .extrude(FAN_T * 0.86)
        .translate((0.0, 0.0, -FAN_T * 0.43))
    )
    fan_ring = (
        cq.Workplane("XY")
        .circle(FAN_R * 0.965)
        .circle(FAN_R * 0.735)
        .extrude(0.0012)
        .translate((0.0, 0.0, -0.0010))
    )
    hub = (
        cq.Workplane("XY")
        .circle(FAN_HUB_R * 1.03)
        .circle(FAN_HUB_R * 0.62)
        .extrude(FAN_T)
        .translate((0.0, 0.0, -FAN_T / 2.0))
    )
    frame = fan_ring.union(hub)
    for blade_idx in range(FAN_BLADE_COUNT):
        frame = frame.union(
            blade.rotate(
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 1.0),
                blade_idx * (360.0 / FAN_BLADE_COUNT),
            )
        )
    intake_boss = (
        cq.Workplane("XY")
        .circle(FAN_HUB_R * 0.68)
        .extrude(FAN_T)
        .translate((0.0, 0.0, -FAN_T * 0.35))
    )
    frame = frame.union(intake_boss)
    center_nut = (
        cq.Workplane("XY")
        .circle(FAN_HUB_R * 0.44)
        .extrude(0.0007)
        .translate((0.0, 0.0, FAN_T / 2.0 - 0.0006))
    )
    return frame.union(center_nut)


def _make_bracket() -> cq.Workplane:
    bracket = cq.Workplane("XY").box(BRACKET_THK, BRACKET_H, BRACKET_T)
    upper_tab = (
        cq.Workplane("XY")
        .box(BRACKET_THK, 0.018, 0.016)
        .translate((0.0, BRACKET_H / 2.0 + 0.009, 0.006))
    )
    lower_ear = (
        cq.Workplane("XY")
        .box(BRACKET_THK, 0.010, 0.012)
        .translate((0.0, -BRACKET_H / 2.0 - 0.005, -0.010))
    )
    bracket = bracket.union(upper_tab).union(lower_ear)

    screw_hole = _cylinder_x(0.0032, BRACKET_THK + 0.004).translate(
        (0.0, BRACKET_H / 2.0 + 0.009, 0.006)
    )
    bracket = bracket.cut(screw_hole)

    for y_pos in (0.030, 0.006, -0.018):
        port = (
            cq.Workplane("XY")
            .box(BRACKET_THK + 0.004, 0.016, 0.011)
            .translate((0.0, y_pos, 0.008))
        )
        bracket = bracket.cut(port)

    for y_pos in (-0.042, -0.030, -0.018, -0.006):
        vent = (
            cq.Workplane("XY")
            .box(BRACKET_THK + 0.004, 0.005, 0.018)
            .translate((0.0, y_pos, -0.014))
        )
        bracket = bracket.cut(vent)

    return bracket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="graphics_card", assets=ASSETS)

    shroud_black = model.material("shroud_black", rgba=(0.08, 0.09, 0.10, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.63, 0.66, 0.69, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.10, 0.23, 0.14, 1.0))
    connector_gold = model.material("connector_gold", rgba=(0.76, 0.63, 0.22, 1.0))
    fan_black = model.material("fan_black", rgba=(0.11, 0.12, 0.13, 1.0))
    badge_silver = model.material("badge_silver", rgba=(0.78, 0.80, 0.83, 1.0))

    shroud_mesh = mesh_from_cadquery(_make_shroud(), "gpu_shroud.obj", assets=ASSETS)
    finpack_mesh = mesh_from_cadquery(_make_finpack(), "gpu_finpack.obj", assets=ASSETS)
    pcb_mesh = mesh_from_cadquery(_make_pcb(), "gpu_pcb.obj", assets=ASSETS)
    backplate_mesh = mesh_from_cadquery(
        _make_backplate(), "gpu_backplate.obj", assets=ASSETS
    )
    power_mesh = mesh_from_cadquery(
        _make_power_housing(), "gpu_power_housing.obj", assets=ASSETS
    )
    fan_mesh = mesh_from_cadquery(_make_fan_rotor(), "gpu_fan_rotor.obj", assets=ASSETS)
    bracket_mesh = mesh_from_cadquery(_make_bracket(), "gpu_io_bracket.obj", assets=ASSETS)

    body = model.part("card_body")
    body.visual(shroud_mesh, material=shroud_black)
    body.visual(finpack_mesh, material=dark_metal)
    body.visual(pcb_mesh, material=pcb_green)
    body.visual(backplate_mesh, material=dark_metal)
    body.visual(power_mesh, material=shroud_black)
    for fan_x in FAN_XS:
        body.visual(
            Cylinder(radius=BEARING_POST_R, length=BEARING_POST_L),
            origin=Origin(xyz=(fan_x, 0.0, BEARING_POST_Z)),
            material=dark_metal,
        )
    body.inertial = Inertial.from_geometry(
        Box((CARD_L, CARD_H, CARD_T)),
        mass=1.55,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    bracket = model.part("io_bracket")
    bracket.visual(bracket_mesh, material=brushed_metal)
    bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_THK, BRACKET_H + 0.020, BRACKET_T)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "bracket_mount",
        ArticulationType.FIXED,
        parent="card_body",
        child="io_bracket",
        origin=Origin(
            xyz=(-CARD_L / 2.0 - BRACKET_THK / 2.0 + 0.0008, 0.0, 0.0)
        ),
    )

    connector = model.part("pcie_connector")
    connector.visual(
        Box((CONN_L, CONN_H, CONN_T)),
        material=pcb_green,
    )
    connector.visual(
        Box((CONN_L - 0.012, 0.0064, CONN_T + 0.0002)),
        origin=Origin(xyz=(0.0, -0.0038, 0.0)),
        material=connector_gold,
    )
    connector.inertial = Inertial.from_geometry(
        Box((CONN_L, CONN_H, CONN_T)),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "connector_mount",
        ArticulationType.FIXED,
        parent="card_body",
        child="pcie_connector",
        origin=Origin(
            xyz=(-0.022, -(CARD_H / 2.0 + CONN_H / 2.0 - 0.0008), 0.0)
        ),
    )

    fan_specs = (
        ("fan_left", FAN_XS[0]),
        ("fan_center", FAN_XS[1]),
        ("fan_right", FAN_XS[2]),
    )
    for fan_name, fan_x in fan_specs:
        fan = model.part(fan_name)
        fan.visual(fan_mesh, material=fan_black)
        fan.visual(
            Cylinder(radius=FAN_HUB_R * 0.55, length=0.0012),
            origin=Origin(xyz=(0.0, 0.0, FAN_T / 2.0 - 0.0006)),
            material=badge_silver,
        )
        fan.inertial = Inertial.from_geometry(
            Cylinder(radius=FAN_R * 0.92, length=FAN_T),
            mass=0.045,
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
        )
        model.articulation(
            f"{fan_name}_spin",
            ArticulationType.CONTINUOUS,
            parent="card_body",
            child=fan_name,
            origin=Origin(xyz=(fan_x, 0.0, FAN_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.3, velocity=80.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    for fan_name in ("fan_left", "fan_center", "fan_right"):
        ctx.allow_overlap("card_body", fan_name, reason="fan hub seats on the bearing post")
    ctx.warn_if_overlaps(max_pose_samples=96, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_overlap("io_bracket", "card_body", axes="y", min_overlap=0.10)
    ctx.expect_aabb_overlap("io_bracket", "card_body", axes="z", min_overlap=0.045)
    ctx.expect_aabb_gap(
        "card_body",
        "io_bracket",
        axis="x",
        max_gap=0.001,
        max_penetration=0.002,
    )
    ctx.expect_origin_distance("io_bracket", "card_body", axes="z", max_dist=0.006)

    ctx.expect_aabb_overlap("pcie_connector", "card_body", axes="x", min_overlap=0.10)
    ctx.expect_aabb_overlap("pcie_connector", "card_body", axes="z", min_overlap=0.003)
    ctx.expect_aabb_gap(
        "card_body",
        "pcie_connector",
        axis="y",
        max_gap=0.001,
        max_penetration=0.001,
    )
    ctx.expect_origin_distance("pcie_connector", "card_body", axes="z", max_dist=0.005)

    ctx.expect_origin_distance("fan_left", "fan_center", axes="y", max_dist=0.003)
    ctx.expect_origin_distance("fan_left", "fan_center", axes="z", max_dist=0.003)
    ctx.expect_origin_distance("fan_center", "fan_right", axes="y", max_dist=0.003)
    ctx.expect_origin_distance("fan_center", "fan_right", axes="z", max_dist=0.003)
    ctx.expect_origin_distance("fan_center", "card_body", axes="y", max_dist=0.020)

    sampled_poses = (
        {"fan_left_spin": 0.0, "fan_center_spin": 0.0, "fan_right_spin": 0.0},
        {"fan_left_spin": pi / 2.0, "fan_center_spin": pi, "fan_right_spin": 3.0 * pi / 2.0},
        {"fan_left_spin": pi, "fan_center_spin": pi / 2.0, "fan_right_spin": pi / 2.0},
    )
    for pose in sampled_poses:
        with ctx.pose(pose):
            for fan_name in ("fan_left", "fan_center", "fan_right"):
                ctx.expect_aabb_overlap(fan_name, "card_body", axes="x", min_overlap=0.070)
                ctx.expect_aabb_overlap(fan_name, "card_body", axes="y", min_overlap=0.070)

    return ctx.report()

# >>> USER_CODE_END

object_model = build_object_model()
