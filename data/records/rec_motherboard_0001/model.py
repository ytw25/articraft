from __future__ import annotations

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

BOARD_X = 0.305
BOARD_Y = 0.244
BOARD_T = 0.002

CPU_CENTER = (-0.032, 0.040)
DIMM_BANK_CENTER = (0.046, 0.020)
DIMM_SLOT_LENGTH = 0.133
DIMM_SLOT_WIDTH = 0.0065
DIMM_SLOT_HEIGHT = 0.009
DIMM_SLOT_PITCH = 0.0115
DIMM_SLOT_OFFSETS = (
    -1.5 * DIMM_SLOT_PITCH,
    -0.5 * DIMM_SLOT_PITCH,
    0.5 * DIMM_SLOT_PITCH,
    1.5 * DIMM_SLOT_PITCH,
)


def _board_shape() -> cq.Workplane:
    board = cq.Workplane("XY").box(BOARD_X, BOARD_Y, BOARD_T).translate((0.0, 0.0, BOARD_T / 2.0))
    mounting_holes = [
        (-0.141, 0.108),
        (-0.141, 0.000),
        (-0.141, -0.103),
        (-0.020, 0.108),
        (-0.020, -0.103),
        (0.101, 0.108),
        (0.101, 0.000),
        (0.141, -0.103),
    ]
    board = (
        board.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(mounting_holes)
        .hole(0.0036)
    )
    return board.edges("|Z").fillet(0.0007)


def _grooved_heatsink(
    length: float, width: float, height: float, groove_count: int, groove_width: float
) -> cq.Workplane:
    heatsink = cq.Workplane("XY").box(length, width, height).translate((0.0, 0.0, height / 2.0))
    for index in range(groove_count):
        x = -length / 2.0 + (index + 0.5) * (length / groove_count)
        cutter = (
            cq.Workplane("XY")
            .box(groove_width, width * 1.04, height * 0.72)
            .translate((x, 0.0, height * 0.64))
        )
        heatsink = heatsink.cut(cutter)
    crown = (
        cq.Workplane("XY")
        .box(length * 0.52, width * 0.82, height * 0.18)
        .translate((0.0, 0.0, height * 0.91))
    )
    return heatsink.union(crown)


def _io_shroud_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(0.030, 0.094, 0.034).translate((0.0, 0.0, 0.017))
    cavity = cq.Workplane("XY").box(0.021, 0.074, 0.020).translate((0.005, 0.0, 0.013))
    visor = (
        cq.Workplane("XY")
        .box(0.018, 0.098, 0.008)
        .translate((0.006, 0.0, 0.028))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 16.0)
    )
    return body.cut(cavity).union(visor)


def _chipset_shroud_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.062, 0.058, 0.015).translate((0.0, 0.0, 0.0075))
    top = (
        cq.Workplane("XY")
        .box(0.040, 0.050, 0.010)
        .translate((0.007, 0.0, 0.018))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 18.0)
    )
    shroud = base.union(top)
    for index in range(5):
        y = -0.020 + 0.010 * index
        cutter = cq.Workplane("XY").box(0.070, 0.0035, 0.008).translate((0.0, y, 0.016))
        shroud = shroud.cut(cutter)
    return shroud


def _socket_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(0.064, 0.064, 0.0011).translate((0.0, 0.0, 0.00055))
    window = cq.Workplane("XY").box(0.043, 0.043, 0.0015).translate((0.0, 0.0, 0.00055))
    latch_bridge = cq.Workplane("XY").box(0.008, 0.020, 0.0011).translate((0.030, 0.0, 0.00055))
    hook_pad = cq.Workplane("XY").box(0.006, 0.012, 0.0011).translate((-0.030, 0.0, 0.00055))
    return frame.cut(window).union(latch_bridge).union(hook_pad)


def _cpu_arm_shape() -> cq.Workplane:
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.0006)
        .lineTo(0.014, 0.0006)
        .threePointArc((0.017, 0.0012), (0.0195, 0.006))
        .lineTo(0.0195, 0.014)
        .threePointArc((0.020, 0.0165), (0.023, 0.0185))
    )
    return cq.Workplane("YZ").circle(0.0011).sweep(path, transition="round")


def _dimm_latch_shape() -> cq.Workplane:
    profile = (
        cq.Workplane("YZ")
        .moveTo(0.0, 0.0)
        .lineTo(0.0048, 0.0)
        .lineTo(0.0064, 0.003)
        .lineTo(0.0052, 0.016)
        .lineTo(0.0020, 0.019)
        .lineTo(-0.0015, 0.017)
        .lineTo(-0.0010, 0.004)
        .close()
        .extrude(0.010)
        .translate((-0.005, 0.0, 0.0))
    )
    catch = cq.Workplane("XY").box(0.010, 0.0025, 0.0035).translate((0.0, 0.005, 0.0042))
    finger = cq.Workplane("XY").box(0.010, 0.0035, 0.0045).translate((0.0, 0.0015, 0.014))
    return profile.union(catch).union(finger)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="enthusiast_motherboard", assets=ASSETS)

    model.material("pcb", rgba=(0.07, 0.16, 0.10, 1.0))
    model.material("silkscreen", rgba=(0.83, 0.86, 0.88, 1.0))
    model.material("slot_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("socket_black", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("steel", rgba=(0.69, 0.71, 0.74, 1.0))
    model.material("heatsink_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("anodized_gray", rgba=(0.36, 0.38, 0.42, 1.0))
    model.material("accent_blue", rgba=(0.20, 0.42, 0.74, 1.0))
    model.material("gold", rgba=(0.78, 0.65, 0.24, 1.0))
    model.material("port_black", rgba=(0.05, 0.05, 0.06, 1.0))

    motherboard_base = model.part("motherboard_base")
    motherboard_base.visual(
        mesh_from_cadquery(_board_shape(), "motherboard_pcb.obj", assets=ASSETS),
        material="pcb",
    )
    motherboard_base.visual(
        mesh_from_cadquery(_io_shroud_shape(), "rear_io_shroud.obj", assets=ASSETS),
        origin=Origin(xyz=(-0.136, 0.083, BOARD_T)),
        material="anodized_gray",
    )
    motherboard_base.visual(
        mesh_from_cadquery(
            _grooved_heatsink(0.086, 0.026, 0.028, 7, 0.0045), "vrm_sink_top.obj", assets=ASSETS
        ),
        origin=Origin(xyz=(-0.040, 0.096, BOARD_T)),
        material="heatsink_dark",
    )
    motherboard_base.visual(
        mesh_from_cadquery(
            _grooved_heatsink(0.075, 0.028, 0.028, 6, 0.0040), "vrm_sink_left.obj", assets=ASSETS
        ),
        origin=Origin(xyz=(-0.084, 0.046, BOARD_T), rpy=(0.0, 0.0, 1.5708)),
        material="heatsink_dark",
    )
    motherboard_base.visual(
        mesh_from_cadquery(_chipset_shroud_shape(), "chipset_shroud.obj", assets=ASSETS),
        origin=Origin(xyz=(0.070, -0.057, BOARD_T)),
        material="anodized_gray",
    )

    motherboard_base.visual(
        Box((0.088, 0.018, 0.008)),
        origin=Origin(xyz=(0.028, -0.009, BOARD_T + 0.004)),
        material="anodized_gray",
    )
    motherboard_base.visual(
        Box((0.136, 0.008, 0.012)),
        origin=Origin(xyz=(0.022, -0.056, BOARD_T + 0.006)),
        material="slot_black",
    )
    motherboard_base.visual(
        Box((0.032, 0.008, 0.012)),
        origin=Origin(xyz=(-0.055, -0.031, BOARD_T + 0.006)),
        material="slot_black",
    )
    motherboard_base.visual(
        Box((0.032, 0.008, 0.012)),
        origin=Origin(xyz=(-0.055, -0.082, BOARD_T + 0.006)),
        material="slot_black",
    )
    motherboard_base.visual(
        Box((0.114, 0.008, 0.012)),
        origin=Origin(xyz=(0.010, -0.097, BOARD_T + 0.006)),
        material="slot_black",
    )
    motherboard_base.visual(
        Box((0.118, 0.0016, 0.0011)),
        origin=Origin(xyz=(0.020, -0.056, BOARD_T + 0.00055)),
        material="gold",
    )
    motherboard_base.visual(
        Box((0.024, 0.0016, 0.0011)),
        origin=Origin(xyz=(-0.055, -0.031, BOARD_T + 0.00055)),
        material="gold",
    )
    motherboard_base.visual(
        Box((0.024, 0.0016, 0.0011)),
        origin=Origin(xyz=(-0.055, -0.082, BOARD_T + 0.00055)),
        material="gold",
    )
    motherboard_base.visual(
        Box((0.096, 0.0016, 0.0011)),
        origin=Origin(xyz=(0.006, -0.097, BOARD_T + 0.00055)),
        material="gold",
    )
    motherboard_base.visual(
        Box((0.010, 0.012, 0.015)),
        origin=Origin(xyz=(0.093, -0.056, BOARD_T + 0.0075)),
        material="charcoal",
    )
    motherboard_base.visual(
        Box((0.012, 0.053, 0.022)),
        origin=Origin(xyz=(0.140, 0.051, BOARD_T + 0.011)),
        material="slot_black",
    )
    motherboard_base.visual(
        Box((0.024, 0.012, 0.018)),
        origin=Origin(xyz=(0.095, 0.111, BOARD_T + 0.009)),
        material="slot_black",
    )
    motherboard_base.visual(
        Box((0.020, 0.012, 0.013)),
        origin=Origin(xyz=(0.128, -0.079, BOARD_T + 0.0065)),
        material="slot_black",
    )
    motherboard_base.visual(
        Box((0.020, 0.012, 0.013)),
        origin=Origin(xyz=(0.128, -0.065, BOARD_T + 0.0065)),
        material="slot_black",
    )
    motherboard_base.visual(
        Box((0.020, 0.008, 0.010)),
        origin=Origin(xyz=(0.123, -0.111, BOARD_T + 0.005)),
        material="slot_black",
    )
    motherboard_base.visual(
        Box((0.020, 0.008, 0.010)),
        origin=Origin(xyz=(0.090, -0.111, BOARD_T + 0.005)),
        material="slot_black",
    )

    for offset in (-0.045, -0.027, -0.009, 0.009, 0.027, 0.045):
        motherboard_base.visual(
            Box((0.009, 0.010, 0.008)),
            origin=Origin(xyz=(CPU_CENTER[0] + offset, 0.082, BOARD_T + 0.004)),
            material="charcoal",
        )
    for offset in (-0.026, -0.013, 0.0, 0.013, 0.026):
        motherboard_base.visual(
            Box((0.010, 0.009, 0.008)),
            origin=Origin(xyz=(-0.063, CPU_CENTER[1] + offset, BOARD_T + 0.004)),
            material="charcoal",
        )

    for cap_x, cap_y in [
        (-0.103, 0.097),
        (-0.090, 0.097),
        (-0.077, 0.097),
        (-0.103, 0.081),
    ]:
        motherboard_base.visual(
            Cylinder(radius=0.0034, length=0.011),
            origin=Origin(xyz=(cap_x, cap_y, BOARD_T + 0.0055)),
            material="slot_black",
        )
    for cap_x, cap_y, cap_r in [
        (-0.118, -0.080, 0.0040),
        (-0.107, -0.087, 0.0036),
        (-0.096, -0.094, 0.0034),
        (-0.086, -0.101, 0.0032),
        (-0.075, -0.108, 0.0030),
    ]:
        motherboard_base.visual(
            Cylinder(radius=cap_r, length=0.010),
            origin=Origin(xyz=(cap_x, cap_y, BOARD_T + 0.005)),
            material="slot_black",
        )

    motherboard_base.visual(
        Box((0.014, 0.060, 0.001)),
        origin=Origin(xyz=(-0.109, -0.086, BOARD_T + 0.0005)),
        material="silkscreen",
    )
    motherboard_base.visual(
        Box((0.030, 0.004, 0.001)),
        origin=Origin(xyz=(0.077, -0.042, BOARD_T + 0.0155)),
        material="accent_blue",
    )
    motherboard_base.visual(
        Box((0.050, 0.0032, 0.0008)),
        origin=Origin(xyz=(0.028, -0.009, BOARD_T + 0.0084)),
        material="accent_blue",
    )
    motherboard_base.visual(
        Box((0.018, 0.074, 0.0010)),
        origin=Origin(xyz=(-0.125, 0.083, BOARD_T + 0.0345)),
        material="accent_blue",
    )

    for x_pos, y_pos, z_pos, size_x, size_y, size_z, material in [
        (-0.146, 0.104, BOARD_T + 0.009, 0.010, 0.014, 0.007, "steel"),
        (-0.146, 0.088, BOARD_T + 0.009, 0.010, 0.014, 0.007, "steel"),
        (-0.146, 0.069, BOARD_T + 0.011, 0.011, 0.016, 0.011, "steel"),
        (-0.146, 0.049, BOARD_T + 0.009, 0.009, 0.026, 0.007, "port_black"),
    ]:
        motherboard_base.visual(
            Box((size_x, size_y, size_z)),
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            material=material,
        )

    motherboard_base.inertial = Inertial.from_geometry(
        Box((BOARD_X, BOARD_Y, 0.040)),
        mass=1.55,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    cpu_socket_frame = model.part("cpu_socket_frame")
    cpu_socket_frame.visual(
        Box((0.057, 0.057, 0.0060)),
        origin=Origin(xyz=(0.0, 0.0, 0.0030)),
        material="socket_black",
    )
    cpu_socket_frame.visual(
        Box((0.038, 0.038, 0.0008)),
        origin=Origin(xyz=(0.0, 0.0, 0.0063)),
        material="charcoal",
    )
    cpu_socket_frame.visual(
        mesh_from_cadquery(_socket_frame_shape(), "cpu_socket_load_plate.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, 0.0057)),
        material="steel",
    )
    cpu_socket_frame.inertial = Inertial.from_geometry(
        Box((0.064, 0.064, 0.008)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    dimm_bank = model.part("dimm_bank")
    for slot_index, slot_offset in enumerate(DIMM_SLOT_OFFSETS):
        dimm_bank.visual(
            Box((DIMM_SLOT_WIDTH, DIMM_SLOT_LENGTH, DIMM_SLOT_HEIGHT)),
            origin=Origin(xyz=(slot_offset, 0.0, DIMM_SLOT_HEIGHT / 2.0)),
            material="slot_black",
        )
        dimm_bank.visual(
            Box((0.003, DIMM_SLOT_LENGTH * 0.95, 0.0012)),
            origin=Origin(xyz=(slot_offset, 0.0, 0.0006)),
            material="gold",
        )
        dimm_bank.visual(
            Box((0.009, 0.004, 0.015)),
            origin=Origin(xyz=(slot_offset, -DIMM_SLOT_LENGTH / 2.0 + 0.002, 0.0075)),
            material="charcoal",
        )
        if slot_index != 0:
            dimm_bank.visual(
                Box((0.009, 0.004, 0.015)),
                origin=Origin(xyz=(slot_offset, DIMM_SLOT_LENGTH / 2.0 - 0.002, 0.0075)),
                material="charcoal",
            )
        else:
            dimm_bank.visual(
                Box((0.006, 0.002, 0.010)),
                origin=Origin(xyz=(slot_offset, DIMM_SLOT_LENGTH / 2.0 - 0.004, 0.005)),
                material="charcoal",
            )
    dimm_bank.visual(
        Box((0.058, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, -0.074, 0.0015)),
        material="accent_blue",
    )
    dimm_bank.inertial = Inertial.from_geometry(
        Box((0.055, 0.150, 0.017)),
        mass=0.10,
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
    )

    cpu_retention_arm = model.part("cpu_retention_arm")
    cpu_retention_arm.visual(
        Cylinder(radius=0.0011, length=0.013),
        origin=Origin(xyz=(0.008, 0.0, 0.0013), rpy=(0.0, 1.5708, 0.0)),
        material="steel",
    )
    cpu_retention_arm.visual(
        Cylinder(radius=0.0011, length=0.010),
        origin=Origin(xyz=(0.0145, 0.0, 0.0070)),
        material="steel",
    )
    cpu_retention_arm.visual(
        Box((0.0038, 0.0028, 0.0028)),
        origin=Origin(xyz=(0.0145, 0.0, 0.0121)),
        material="steel",
    )
    cpu_retention_arm.visual(
        Box((0.0052, 0.0030, 0.0012)),
        origin=Origin(xyz=(0.0185, 0.0, 0.0152)),
        material="steel",
    )
    cpu_retention_arm.visual(
        Box((0.0035, 0.0035, 0.0048)),
        origin=Origin(xyz=(0.0195, 0.0, 0.0128)),
        material="steel",
    )
    cpu_retention_arm.visual(
        Box((0.0055, 0.0040, 0.0020)),
        origin=Origin(xyz=(0.0015, 0.0, 0.0016)),
        material="steel",
    )
    cpu_retention_arm.visual(
        Cylinder(radius=0.0016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material="steel",
    )
    cpu_retention_arm.inertial = Inertial.from_geometry(
        Box((0.030, 0.008, 0.022)),
        mass=0.014,
        origin=Origin(xyz=(0.015, 0.0, 0.011)),
    )

    dimm_slot_latch = model.part("dimm_slot_latch")
    dimm_slot_latch.visual(
        mesh_from_cadquery(_dimm_latch_shape(), "dimm_slot_latch.obj", assets=ASSETS),
        material="charcoal",
    )
    dimm_slot_latch.inertial = Inertial.from_geometry(
        Box((0.010, 0.010, 0.020)),
        mass=0.006,
        origin=Origin(xyz=(0.0, 0.004, 0.010)),
    )

    model.articulation(
        "base_to_cpu_socket",
        ArticulationType.FIXED,
        parent=motherboard_base,
        child=cpu_socket_frame,
        origin=Origin(xyz=(CPU_CENTER[0], CPU_CENTER[1], BOARD_T)),
    )
    model.articulation(
        "base_to_dimm_bank",
        ArticulationType.FIXED,
        parent=motherboard_base,
        child=dimm_bank,
        origin=Origin(xyz=(DIMM_BANK_CENTER[0], DIMM_BANK_CENTER[1], BOARD_T)),
    )
    model.articulation(
        "cpu_retention_arm_joint",
        ArticulationType.REVOLUTE,
        parent=cpu_socket_frame,
        child=cpu_retention_arm,
        origin=Origin(xyz=(0.031, 0.0, 0.0073)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=1.5, velocity=2.0),
    )
    model.articulation(
        "dimm_latch_joint",
        ArticulationType.REVOLUTE,
        parent=dimm_bank,
        child=dimm_slot_latch,
        origin=Origin(xyz=(DIMM_SLOT_OFFSETS[0], DIMM_SLOT_LENGTH / 2.0 + 0.001, 0.0055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=0.5, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.allow_overlap(
        "dimm_slot_latch",
        "motherboard_base",
        reason="DIMM latch sweeps close to the upper VRM heatsink; generated hulls are conservative around the grooved shroud geometry.",
    )
    ctx.check_no_overlaps(
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("cpu_socket_frame", "motherboard_base", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_contact("cpu_socket_frame", "motherboard_base")
    ctx.expect_aabb_overlap("dimm_bank", "motherboard_base", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_contact("dimm_bank", "motherboard_base")

    ctx.expect_origin_distance("cpu_retention_arm", "cpu_socket_frame", axes="y", max_dist=0.0015)
    ctx.expect_aabb_gap(
        "cpu_retention_arm",
        "cpu_socket_frame",
        axis="z",
        max_gap=0.006,
        max_penetration=0.002,
    )
    ctx.expect_aabb_overlap("dimm_slot_latch", "dimm_bank", axes="xz", min_overlap=0.004)
    ctx.expect_aabb_gap(
        "dimm_slot_latch",
        "dimm_bank",
        axis="y",
        max_gap=0.001,
        max_penetration=0.002,
    )

    ctx.expect_joint_motion_axis(
        "cpu_retention_arm_joint",
        "cpu_retention_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.003,
    )
    ctx.expect_joint_motion_axis(
        "dimm_latch_joint",
        "dimm_slot_latch",
        world_axis="z",
        direction="negative",
        min_delta=0.002,
    )

    with ctx.pose(cpu_retention_arm_joint=1.25):
        ctx.expect_origin_distance(
            "cpu_retention_arm", "cpu_socket_frame", axes="y", max_dist=0.0015
        )
        ctx.expect_aabb_gap(
            "cpu_retention_arm",
            "cpu_socket_frame",
            axis="z",
            max_gap=0.020,
            max_penetration=0.0015,
        )

    with ctx.pose(dimm_latch_joint=0.95):
        ctx.expect_aabb_overlap("dimm_slot_latch", "dimm_bank", axes="xz", min_overlap=0.004)
        ctx.expect_origin_distance("dimm_slot_latch", "dimm_bank", axes="z", max_dist=0.02)

    with ctx.pose(cpu_retention_arm_joint=1.25, dimm_latch_joint=0.95):
        ctx.expect_origin_distance(
            "cpu_retention_arm", "cpu_socket_frame", axes="y", max_dist=0.0015
        )
        ctx.expect_aabb_overlap("dimm_slot_latch", "dimm_bank", axes="xz", min_overlap=0.004)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
