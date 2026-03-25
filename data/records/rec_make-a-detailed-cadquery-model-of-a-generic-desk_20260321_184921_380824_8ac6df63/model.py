from __future__ import annotations

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
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BOARD_X = 0.305
BOARD_Y = 0.244
CORE_T = 0.00150
MASK_T = 0.00006
BOARD_T = CORE_T + 2.0 * MASK_T
BOARD_CORNER_R = 0.005
MASK_INSET = 0.00030
MOUNT_HOLE_D = 0.00350
MOUNT_HOLES = [
    (-0.140, 0.108),
    (-0.015, 0.108),
    (0.135, 0.102),
    (-0.140, 0.022),
    (-0.015, 0.022),
    (0.135, 0.022),
    (-0.140, -0.107),
    (0.018, -0.107),
    (0.135, -0.107),
]


def _box_inertial(part, size: tuple[float, float, float], mass: float) -> None:
    part.inertial = Inertial.from_geometry(
        Box(size),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, size[2] * 0.5)),
    )


def _mount_fixed(
    model: ArticulatedObject,
    child_name: str,
    xyz: tuple[float, float, float],
) -> None:
    model.articulation(
        f"board_to_{child_name}",
        ArticulationType.FIXED,
        parent="board",
        child=child_name,
        origin=Origin(xyz=xyz),
    )


def _pcb_plate(
    length: float,
    width: float,
    thickness: float,
    corner_radius: float,
    hole_diameter: float,
) -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(length, width, thickness, centered=(True, True, False))
        .edges("|Z")
        .fillet(corner_radius)
    )
    for x, y in MOUNT_HOLES:
        cutter = (
            cq.Workplane("XY", origin=(x, y, -thickness))
            .circle(hole_diameter * 0.5)
            .extrude(thickness * 3.0)
        )
        plate = plate.cut(cutter)
    return plate


def _finned_block(
    length: float,
    width: float,
    base_height: float,
    fin_height: float,
    fin_count: int,
    axis: str,
) -> cq.Workplane:
    block = cq.Workplane("XY").box(
        length,
        width,
        base_height,
        centered=(True, True, False),
    )
    if axis == "x":
        pitch = width / (fin_count + 1)
        fin_length = length * 0.92
        fin_width = min(width * 0.08, pitch * 0.55)
        for index in range(fin_count):
            y = -width * 0.5 + pitch * (index + 1)
            block = block.union(
                cq.Workplane("XY", origin=(0.0, y, base_height)).box(
                    fin_length,
                    fin_width,
                    fin_height,
                    centered=(True, True, False),
                )
            )
    else:
        pitch = length / (fin_count + 1)
        fin_width = min(length * 0.08, pitch * 0.55)
        fin_length = width * 0.92
        for index in range(fin_count):
            x = -length * 0.5 + pitch * (index + 1)
            block = block.union(
                cq.Workplane("XY", origin=(x, 0.0, base_height)).box(
                    fin_width,
                    fin_length,
                    fin_height,
                    centered=(True, True, False),
                )
            )
    return block


def _multi_cavity_connector(
    length: float,
    width: float,
    height: float,
    rows: int,
    cols: int,
    cavity_depth: float,
) -> cq.Workplane:
    body = cq.Workplane("XY").box(
        length,
        width,
        height,
        centered=(True, True, False),
    )
    pad_x = length * 0.18
    pad_y = width * 0.10
    xs = [0.0] if cols == 1 else [
        -length * 0.5 + pad_x + (length - 2.0 * pad_x) * i / (cols - 1)
        for i in range(cols)
    ]
    ys = [0.0] if rows == 1 else [
        -width * 0.5 + pad_y + (width - 2.0 * pad_y) * j / (rows - 1)
        for j in range(rows)
    ]
    points = [(x, y) for y in ys for x in xs]
    cavity_x = max(length / (cols + 2.2), 0.0014)
    cavity_y = max(width / (rows + 1.8), 0.0016)
    for x, y in points:
        cavity = cq.Workplane("XY", origin=(x, y, height - cavity_depth)).box(
            cavity_x,
            cavity_y,
            cavity_depth + 0.0002,
            centered=(True, True, False),
        )
        body = body.cut(cavity)
    return body


def _cpu_socket_body() -> cq.Workplane:
    socket = cq.Workplane("XY").box(
        0.050,
        0.052,
        0.0042,
        centered=(True, True, False),
    )
    center_cavity = cq.Workplane("XY", origin=(0.0, 0.0, 0.0020)).box(
        0.037,
        0.039,
        0.0024,
        centered=(True, True, False),
    )
    top_relief = cq.Workplane("XY", origin=(0.0, 0.0, 0.0034)).box(
        0.043,
        0.045,
        0.0009,
        centered=(True, True, False),
    )
    return socket.cut(center_cavity).cut(top_relief)


def _cpu_socket_clamp() -> cq.Workplane:
    outer = cq.Workplane("XY", origin=(0.0, 0.0, 0.0042)).box(
        0.048,
        0.050,
        0.0012,
        centered=(True, True, False),
    )
    inner = cq.Workplane("XY", origin=(0.0, 0.0, 0.0042)).box(
        0.037,
        0.039,
        0.0012,
        centered=(True, True, False),
    )
    clamp = outer.cut(inner)
    latch = cq.Workplane("XY", origin=(0.024, -0.010, 0.0042)).box(
        0.004,
        0.013,
        0.002,
        centered=(True, True, False),
    )
    return clamp.union(latch)


def _ram_bank() -> cq.Workplane:
    bank = cq.Workplane("XY").box(
        0.038,
        0.136,
        0.0030,
        centered=(True, True, False),
    )
    for x in (-0.012, -0.004, 0.004, 0.012):
        slot = cq.Workplane("XY", origin=(x, 0.0, 0.0030)).box(
            0.0055,
            0.133,
            0.0057,
            centered=(True, True, False),
        )
        groove = cq.Workplane("XY", origin=(x, 0.0, 0.0062)).box(
            0.0023,
            0.118,
            0.0018,
            centered=(True, True, False),
        )
        latch_top = cq.Workplane("XY", origin=(x, 0.063, 0.0030)).box(
            0.007,
            0.006,
            0.008,
            centered=(True, True, False),
        )
        latch_bottom = cq.Workplane("XY", origin=(x, -0.063, 0.0030)).box(
            0.007,
            0.006,
            0.008,
            centered=(True, True, False),
        )
        bank = bank.union(slot.cut(groove)).union(latch_top).union(latch_bottom)
    return bank


def _pcie_slots_cluster() -> cq.Workplane:
    cluster = cq.Workplane("XY").box(
        0.012,
        0.074,
        0.003,
        centered=(True, True, False),
    )

    def _slot(center_x: float, center_y: float, slot_length: float) -> cq.Workplane:
        body = cq.Workplane("XY", origin=(center_x, center_y, 0.003)).box(
            slot_length,
            0.0065,
            0.0076,
            centered=(True, True, False),
        )
        groove = cq.Workplane(
            "XY",
            origin=(center_x + slot_length * 0.12, center_y, 0.007),
        ).box(
            slot_length * 0.62,
            0.0019,
            0.0017,
            centered=(True, True, False),
        )
        return body.cut(groove)

    cluster = cluster.union(_slot(-0.006, -0.024, 0.090))
    cluster = cluster.union(_slot(-0.030, 0.002, 0.038))
    cluster = cluster.union(_slot(-0.030, 0.026, 0.038))
    latch = cq.Workplane("XY", origin=(0.038, -0.024, 0.003)).box(
        0.005,
        0.008,
        0.011,
        centered=(True, True, False),
    )
    return cluster.union(latch)


def _sata_cluster() -> cq.Workplane:
    cluster = cq.Workplane("XY").box(
        0.046,
        0.032,
        0.0015,
        centered=(True, True, False),
    )
    for x in (-0.014, 0.0, 0.014):
        for y in (-0.008, 0.008):
            port = cq.Workplane("XY", origin=(x, y, 0.0015)).box(
                0.013,
                0.013,
                0.010,
                centered=(True, True, False),
            )
            mouth = cq.Workplane("YZ", origin=(x + 0.0045, y, 0.007)).rect(0.009, 0.006).extrude(-0.006)
            cluster = cluster.union(port.cut(mouth))
    return cluster


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_motherboard", assets=ASSETS)

    pcb_core = model.material("pcb_core", rgba=(0.72, 0.66, 0.45, 1.0))
    solder_mask = model.material("solder_mask", rgba=(0.08, 0.34, 0.16, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    slot_beige = model.material("slot_beige", rgba=(0.78, 0.77, 0.70, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.70, 0.73, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    gold = model.material("gold", rgba=(0.79, 0.63, 0.18, 1.0))
    capacitor_black = model.material("capacitor_black", rgba=(0.18, 0.19, 0.21, 1.0))
    capacitor_top = model.material("capacitor_top", rgba=(0.81, 0.82, 0.84, 1.0))
    battery_silver = model.material("battery_silver", rgba=(0.80, 0.81, 0.84, 1.0))
    header_blue = model.material("header_blue", rgba=(0.20, 0.39, 0.76, 1.0))
    connector_white = model.material("connector_white", rgba=(0.92, 0.92, 0.90, 1.0))

    board = model.part("board")
    substrate = _pcb_plate(
        BOARD_X,
        BOARD_Y,
        CORE_T,
        BOARD_CORNER_R,
        MOUNT_HOLE_D,
    )
    mask_plate = _pcb_plate(
        BOARD_X - 2.0 * MASK_INSET,
        BOARD_Y - 2.0 * MASK_INSET,
        MASK_T,
        BOARD_CORNER_R - MASK_INSET,
        MOUNT_HOLE_D + 0.0012,
    )
    board.visual(
        mesh_from_cadquery(substrate, "board_substrate.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, MASK_T)),
        material=pcb_core,
    )
    board.visual(
        mesh_from_cadquery(mask_plate, "board_mask_top.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, MASK_T + CORE_T)),
        material=solder_mask,
    )
    board.visual(
        mesh_from_cadquery(mask_plate, "board_mask_bottom.obj", assets=ASSETS),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=solder_mask,
    )
    _box_inertial(board, (BOARD_X, BOARD_Y, BOARD_T), 0.52)

    cpu_socket = model.part("cpu_socket")
    cpu_socket.visual(
        mesh_from_cadquery(_cpu_socket_body(), "cpu_socket_body.obj", assets=ASSETS),
        material=black_plastic,
    )
    cpu_socket.visual(
        mesh_from_cadquery(_cpu_socket_clamp(), "cpu_socket_clamp.obj", assets=ASSETS),
        material=steel,
    )
    cpu_socket.visual(
        Box((0.0020, 0.030, 0.0010)),
        origin=Origin(xyz=(0.026, 0.0, 0.0061)),
        material=steel,
    )
    cpu_socket.visual(
        Box((0.0040, 0.0040, 0.0012)),
        origin=Origin(xyz=(0.0285, 0.015, 0.0063)),
        material=steel,
    )
    _box_inertial(cpu_socket, (0.052, 0.054, 0.0075), 0.08)

    ram_slots = model.part("ram_slots")
    ram_slots.visual(
        mesh_from_cadquery(_ram_bank(), "ram_slots.obj", assets=ASSETS),
        material=slot_beige,
    )
    _box_inertial(ram_slots, (0.038, 0.136, 0.011), 0.10)

    pcie_slots = model.part("pcie_slots")
    pcie_slots.visual(
        mesh_from_cadquery(_pcie_slots_cluster(), "pcie_slots.obj", assets=ASSETS),
        material=black_plastic,
    )
    _box_inertial(pcie_slots, (0.102, 0.074, 0.014), 0.08)

    chipset_heatsink = model.part("chipset_heatsink")
    chipset_heatsink.visual(
        mesh_from_cadquery(
            _finned_block(0.043, 0.043, 0.004, 0.014, 7, "x"),
            "chipset_heatsink.obj",
            assets=ASSETS,
        ),
        material=aluminum,
    )
    _box_inertial(chipset_heatsink, (0.043, 0.043, 0.018), 0.05)

    rear_io_block = model.part("rear_io_block")
    rear_io_block.visual(
        Box((0.016, 0.104, 0.029)),
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=steel,
    )
    rear_io_block.visual(
        Box((0.010, 0.028, 0.014)),
        origin=Origin(xyz=(0.003, 0.030, 0.010)),
        material=dark_plastic,
    )
    rear_io_block.visual(
        Box((0.010, 0.028, 0.014)),
        origin=Origin(xyz=(0.003, 0.000, 0.010)),
        material=dark_plastic,
    )
    rear_io_block.visual(
        Box((0.011, 0.018, 0.016)),
        origin=Origin(xyz=(0.0025, -0.027, 0.012)),
        material=steel,
    )
    for y in (-0.050, -0.040, -0.030):
        rear_io_block.visual(
            Cylinder(radius=0.0042, length=0.010),
            origin=Origin(xyz=(0.0025, y, 0.010)),
            material=dark_plastic,
        )
    _box_inertial(rear_io_block, (0.016, 0.104, 0.029), 0.09)

    atx_power_connector = model.part("atx_power_connector")
    atx_power_connector.visual(
        mesh_from_cadquery(
            _multi_cavity_connector(0.014, 0.053, 0.015, 12, 2, 0.010),
            "atx_24pin.obj",
            assets=ASSETS,
        ),
        material=connector_white,
    )
    _box_inertial(atx_power_connector, (0.014, 0.053, 0.015), 0.04)

    cpu_power_connector = model.part("cpu_power_connector")
    cpu_power_connector.visual(
        mesh_from_cadquery(
            _multi_cavity_connector(0.018, 0.018, 0.014, 4, 2, 0.009),
            "cpu_8pin.obj",
            assets=ASSETS,
        ),
        material=connector_white,
    )
    _box_inertial(cpu_power_connector, (0.018, 0.018, 0.014), 0.02)

    sata_ports = model.part("sata_ports")
    sata_ports.visual(
        mesh_from_cadquery(_sata_cluster(), "sata_ports.obj", assets=ASSETS),
        material=black_plastic,
    )
    _box_inertial(sata_ports, (0.046, 0.032, 0.012), 0.03)

    front_panel_headers = model.part("front_panel_headers")
    front_panel_headers.visual(
        Box((0.024, 0.008, 0.0004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0002)),
        material=solder_mask,
    )
    front_panel_headers.visual(
        Box((0.016, 0.006, 0.006)),
        origin=Origin(xyz=(-0.003, 0.0, 0.0034)),
        material=black_plastic,
    )
    front_panel_headers.visual(
        Box((0.007, 0.006, 0.005)),
        origin=Origin(xyz=(0.008, 0.0, 0.0029)),
        material=dark_plastic,
    )
    for x in (-0.008, -0.004, 0.0, 0.004, 0.008):
        front_panel_headers.visual(
            Box((0.0007, 0.0007, 0.003)),
            origin=Origin(xyz=(x, 0.0015, 0.0073)),
            material=gold,
        )
        front_panel_headers.visual(
            Box((0.0007, 0.0007, 0.003)),
            origin=Origin(xyz=(x, -0.0015, 0.0073)),
            material=gold,
        )
    _box_inertial(front_panel_headers, (0.024, 0.008, 0.009), 0.01)

    usb_headers = model.part("usb_headers")
    usb_headers.visual(
        Box((0.047, 0.016, 0.0004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0002)),
        material=solder_mask,
    )
    usb_headers.visual(
        Box((0.021, 0.010, 0.011)),
        origin=Origin(xyz=(-0.011, 0.0, 0.0059)),
        material=header_blue,
    )
    usb_headers.visual(
        Box((0.016, 0.006, 0.007)),
        origin=Origin(xyz=(0.012, 0.004, 0.0039)),
        material=black_plastic,
    )
    usb_headers.visual(
        Box((0.016, 0.006, 0.007)),
        origin=Origin(xyz=(0.012, -0.004, 0.0039)),
        material=black_plastic,
    )
    _box_inertial(usb_headers, (0.047, 0.016, 0.012), 0.02)

    fan_headers = model.part("fan_headers")
    fan_headers.visual(
        Box((0.036, 0.008, 0.0004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0002)),
        material=solder_mask,
    )
    for x in (-0.012, 0.0, 0.012):
        fan_headers.visual(
            Box((0.008, 0.006, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.0034)),
            material=black_plastic,
        )
    _box_inertial(fan_headers, (0.036, 0.008, 0.007), 0.01)

    cmos_battery = model.part("cmos_battery")
    cmos_battery.visual(
        Box((0.024, 0.024, 0.0006)),
        origin=Origin(xyz=(0.0, 0.0, 0.0003)),
        material=dark_plastic,
    )
    cmos_battery.visual(
        Box((0.024, 0.0025, 0.0032)),
        origin=Origin(xyz=(0.0, 0.011, 0.0016)),
        material=dark_plastic,
    )
    cmos_battery.visual(
        Box((0.024, 0.0025, 0.0032)),
        origin=Origin(xyz=(0.0, -0.011, 0.0016)),
        material=dark_plastic,
    )
    cmos_battery.visual(
        Cylinder(radius=0.010, length=0.0032),
        origin=Origin(xyz=(0.0, 0.0, 0.0022)),
        material=battery_silver,
    )
    _box_inertial(cmos_battery, (0.024, 0.024, 0.006), 0.03)

    vrm_heatsink_top = model.part("vrm_heatsink_top")
    vrm_heatsink_top.visual(
        mesh_from_cadquery(
            _finned_block(0.052, 0.018, 0.0035, 0.012, 6, "x"),
            "vrm_heatsink_top.obj",
            assets=ASSETS,
        ),
        material=aluminum,
    )
    _box_inertial(vrm_heatsink_top, (0.052, 0.018, 0.016), 0.03)

    vrm_heatsink_left = model.part("vrm_heatsink_left")
    vrm_heatsink_left.visual(
        mesh_from_cadquery(
            _finned_block(0.018, 0.050, 0.0035, 0.012, 6, "y"),
            "vrm_heatsink_left.obj",
            assets=ASSETS,
        ),
        material=aluminum,
    )
    _box_inertial(vrm_heatsink_left, (0.018, 0.050, 0.016), 0.03)

    capacitor_bank = model.part("capacitor_bank")
    capacitor_bank.visual(
        Box((0.028, 0.050, 0.0004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0002)),
        material=solder_mask,
    )
    for y in (-0.017, -0.007, 0.003, 0.013):
        capacitor_bank.visual(
            Cylinder(radius=0.0035, length=0.010),
            origin=Origin(xyz=(-0.006, y, 0.0054)),
            material=capacitor_black,
        )
        capacitor_bank.visual(
            Cylinder(radius=0.0032, length=0.0008),
            origin=Origin(xyz=(-0.006, y, 0.0108)),
            material=capacitor_top,
        )
    for y in (-0.010, 0.010):
        capacitor_bank.visual(
            Cylinder(radius=0.0028, length=0.0085),
            origin=Origin(xyz=(0.007, y, 0.0047)),
            material=capacitor_black,
        )
        capacitor_bank.visual(
            Cylinder(radius=0.0025, length=0.0007),
            origin=Origin(xyz=(0.007, y, 0.0093)),
            material=capacitor_top,
        )
    _box_inertial(capacitor_bank, (0.028, 0.050, 0.012), 0.03)

    chip_packages = model.part("chip_packages")
    chip_packages.visual(
        Box((0.086, 0.046, 0.0004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0002)),
        material=solder_mask,
    )
    chip_packages.visual(
        Box((0.024, 0.024, 0.0024)),
        origin=Origin(xyz=(-0.022, 0.006, 0.0016)),
        material=dark_plastic,
    )
    chip_packages.visual(
        Box((0.018, 0.018, 0.0020)),
        origin=Origin(xyz=(0.009, -0.006, 0.0014)),
        material=black_plastic,
    )
    chip_packages.visual(
        Box((0.014, 0.010, 0.0018)),
        origin=Origin(xyz=(0.025, 0.010, 0.0013)),
        material=dark_plastic,
    )
    chip_packages.visual(
        Box((0.010, 0.006, 0.0017)),
        origin=Origin(xyz=(0.030, -0.010, 0.00125)),
        material=steel,
    )
    chip_packages.visual(
        Box((0.012, 0.012, 0.0016)),
        origin=Origin(xyz=(-0.001, 0.014, 0.0012)),
        material=black_plastic,
    )
    _box_inertial(chip_packages, (0.086, 0.046, 0.003), 0.03)

    _mount_fixed(model, "cpu_socket", (-0.018, 0.056, BOARD_T))
    _mount_fixed(model, "ram_slots", (0.080, 0.034, BOARD_T))
    _mount_fixed(model, "pcie_slots", (-0.073, -0.060, BOARD_T))
    _mount_fixed(model, "chipset_heatsink", (0.034, -0.030, BOARD_T))
    _mount_fixed(model, "rear_io_block", (-0.147, 0.072, BOARD_T))
    _mount_fixed(model, "atx_power_connector", (0.138, 0.042, BOARD_T))
    _mount_fixed(model, "cpu_power_connector", (-0.108, 0.109, BOARD_T))
    _mount_fixed(model, "sata_ports", (0.121, -0.052, BOARD_T))
    _mount_fixed(model, "front_panel_headers", (0.111, -0.111, BOARD_T))
    _mount_fixed(model, "usb_headers", (0.048, -0.110, BOARD_T))
    _mount_fixed(model, "fan_headers", (0.124, 0.094, BOARD_T))
    _mount_fixed(model, "cmos_battery", (0.016, -0.083, BOARD_T))
    _mount_fixed(model, "vrm_heatsink_top", (-0.030, 0.102, BOARD_T))
    _mount_fixed(model, "vrm_heatsink_left", (-0.077, 0.058, BOARD_T))
    _mount_fixed(model, "capacitor_bank", (-0.094, 0.077, BOARD_T))
    _mount_fixed(model, "chip_packages", (0.020, -0.004, BOARD_T))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=32,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    mounted_parts = {
        "cpu_socket": 0.0020,
        "ram_slots": 0.0040,
        "pcie_slots": 0.0040,
        "chipset_heatsink": 0.0030,
        "rear_io_block": 0.0010,
        "atx_power_connector": 0.0010,
        "cpu_power_connector": 0.0010,
        "sata_ports": 0.0015,
        "front_panel_headers": 0.0010,
        "usb_headers": 0.0015,
        "fan_headers": 0.0010,
        "cmos_battery": 0.0010,
        "vrm_heatsink_top": 0.0015,
        "vrm_heatsink_left": 0.0015,
        "capacitor_bank": 0.0012,
        "chip_packages": 0.0020,
    }
    for part_name, min_overlap in mounted_parts.items():
        ctx.expect_aabb_gap(
            part_name,
            "board",
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
        )
        ctx.expect_aabb_overlap(
            part_name,
            "board",
            axes="xy",
            min_overlap=min_overlap,
        )

    ctx.expect_aabb_overlap(
        "ram_slots",
        "cpu_socket",
        axes="y",
        min_overlap=0.050,
    )
    ctx.expect_aabb_gap(
        "cpu_socket",
        "pcie_slots",
        axis="y",
        max_gap=0.060,
        max_penetration=0.0,
    )
    ctx.expect_aabb_gap(
        "cpu_power_connector",
        "rear_io_block",
        axis="x",
        max_gap=0.030,
        max_penetration=0.0,
    )
    ctx.expect_aabb_overlap(
        "sata_ports",
        "front_panel_headers",
        axes="x",
        min_overlap=0.020,
    )
    ctx.expect_aabb_gap(
        "cmos_battery",
        "front_panel_headers",
        axis="y",
        max_gap=0.020,
        max_penetration=0.0,
    )

    ctx.expect_aabb_gap(
        "ram_slots",
        "cpu_socket",
        axis="x",
        max_gap=0.055,
        max_penetration=0.0,
    )
    ctx.expect_aabb_gap(
        "cpu_power_connector",
        "cpu_socket",
        axis="y",
        max_gap=0.050,
        max_penetration=0.0,
    )
    ctx.expect_aabb_gap(
        "vrm_heatsink_top",
        "cpu_socket",
        axis="y",
        max_gap=0.028,
        max_penetration=0.0,
    )
    ctx.expect_aabb_gap(
        "cpu_socket",
        "vrm_heatsink_left",
        axis="x",
        max_gap=0.026,
        max_penetration=0.0,
    )
    ctx.expect_aabb_gap(
        "atx_power_connector",
        "ram_slots",
        axis="x",
        max_gap=0.034,
        max_penetration=0.0,
    )
    ctx.expect_aabb_gap(
        "chipset_heatsink",
        "pcie_slots",
        axis="x",
        max_gap=0.050,
        max_penetration=0.0,
    )
    ctx.expect_aabb_gap(
        "sata_ports",
        "front_panel_headers",
        axis="y",
        max_gap=0.050,
        max_penetration=0.0,
    )
    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
