from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
import cadquery as cq

from sdk_hybrid import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


ASSETS = AssetContext.from_script(__file__)

BOARD_LENGTH = 0.085
BOARD_WIDTH = 0.056
BOARD_THICKNESS = 0.0016
SOLDER_MASK_THICKNESS = 0.00006
CORNER_RADIUS = 0.003
MOUNT_HOLE_DIAMETER = 0.00275
MASK_HOLE_DIAMETER = 0.0058
HOLE_EDGE_OFFSET = 0.0035

HEADER_ROWS = 2
HEADER_COLUMNS = 20
HEADER_PITCH = 0.00254
HEADER_PIN_WIDTH = 0.00064
HEADER_BASE_HEIGHT = 0.0024
HEADER_PIN_ABOVE = 0.0068
HEADER_PIN_BELOW = 0.0032

RJ45_LENGTH = 0.021
RJ45_WIDTH = 0.0162
RJ45_HEIGHT = 0.0136

SOC_SIZE = (0.014, 0.014, 0.0014)
MEMORY_SIZE = (0.012, 0.009, 0.0012)
ETHERNET_CTRL_SIZE = (0.009, 0.009, 0.0010)
PMIC_SIZE = (0.007, 0.007, 0.0010)

MOUNT_HOLE_POINTS = [
    (BOARD_LENGTH / 2 - HOLE_EDGE_OFFSET, BOARD_WIDTH / 2 - HOLE_EDGE_OFFSET),
    (-(BOARD_LENGTH / 2 - HOLE_EDGE_OFFSET), BOARD_WIDTH / 2 - HOLE_EDGE_OFFSET),
    (-(BOARD_LENGTH / 2 - HOLE_EDGE_OFFSET), -(BOARD_WIDTH / 2 - HOLE_EDGE_OFFSET)),
    (BOARD_LENGTH / 2 - HOLE_EDGE_OFFSET, -(BOARD_WIDTH / 2 - HOLE_EDGE_OFFSET)),
]

HEADER_POSITION = (-0.006, 0.021, BOARD_THICKNESS + SOLDER_MASK_THICKNESS)
RJ45_POSITION = (0.034, 0.008, BOARD_THICKNESS + SOLDER_MASK_THICKNESS)
SOC_POSITION = (-0.002, 0.001, BOARD_THICKNESS + SOLDER_MASK_THICKNESS)
MEMORY_POSITION = (0.016, -0.014, BOARD_THICKNESS + SOLDER_MASK_THICKNESS)
ETHERNET_CTRL_POSITION = (-0.019, -0.012, BOARD_THICKNESS + SOLDER_MASK_THICKNESS)
PMIC_POSITION = (-0.030, 0.000, BOARD_THICKNESS + SOLDER_MASK_THICKNESS)


def _centered_grid(rows: int, columns: int, pitch: float) -> list[tuple[float, float]]:
    return [
        (
            (column - (columns - 1) / 2) * pitch,
            (row - (rows - 1) / 2) * pitch,
        )
        for row in range(rows)
        for column in range(columns)
    ]


def _make_board() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BOARD_LENGTH, BOARD_WIDTH, BOARD_THICKNESS, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(MOUNT_HOLE_POINTS)
        .hole(MOUNT_HOLE_DIAMETER)
        .edges("|Z")
        .fillet(CORNER_RADIUS)
    )


def _make_solder_mask(*, top: bool) -> cq.Workplane:
    mask = (
        cq.Workplane("XY")
        .box(
            BOARD_LENGTH,
            BOARD_WIDTH,
            SOLDER_MASK_THICKNESS,
            centered=(True, True, False),
        )
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(MOUNT_HOLE_POINTS)
        .hole(MASK_HOLE_DIAMETER)
        .edges("|Z")
        .fillet(CORNER_RADIUS)
    )
    if top:
        return mask
    return mask.translate((0.0, 0.0, -SOLDER_MASK_THICKNESS))


def _make_pin_header() -> tuple[cq.Workplane, cq.Workplane]:
    base_length = HEADER_COLUMNS * HEADER_PITCH
    base_width = HEADER_ROWS * HEADER_PITCH
    pin_length = HEADER_PIN_ABOVE + HEADER_BASE_HEIGHT + HEADER_PIN_BELOW
    pin_positions = _centered_grid(HEADER_ROWS, HEADER_COLUMNS, HEADER_PITCH)

    base = (
        cq.Workplane("XY")
        .box(base_length, base_width, HEADER_BASE_HEIGHT, centered=(True, True, False))
        .faces(">Z")
        .workplane()
        .pushPoints(pin_positions)
        .rect(HEADER_PIN_WIDTH * 1.15, HEADER_PIN_WIDTH * 1.15)
        .cutThruAll()
    )

    pins = (
        cq.Workplane("XY")
        .pushPoints(pin_positions)
        .box(
            HEADER_PIN_WIDTH,
            HEADER_PIN_WIDTH,
            pin_length,
            combine=False,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, -HEADER_PIN_BELOW))
        .edges("<Z")
        .chamfer(0.00018)
        .edges(">Z")
        .chamfer(0.00018)
    )

    return base, pins


def _make_rj45_shell() -> cq.Workplane:
    aperture_width = 0.01168
    aperture_height = 0.00775
    aperture_depth = -0.0145
    keyway_width = 0.006
    keyway_height = 0.0015
    retainer_width = 0.00325
    retainer_height = 0.0015
    retainer_depth = 0.002
    keyway_elevation = -(aperture_height / 2) - (keyway_height / 2)
    retainer_elevation = keyway_elevation - (retainer_height / 2)

    return (
        cq.Workplane("XY")
        .box(RJ45_LENGTH, RJ45_WIDTH, RJ45_HEIGHT, centered=(True, True, False))
        .faces(">X")
        .workplane()
        .tag("aperture")
        .rect(aperture_width, aperture_height)
        .cutBlind(aperture_depth)
        .workplaneFromTagged("aperture")
        .move(0.0, keyway_elevation)
        .rect(keyway_width, keyway_height)
        .cutBlind(aperture_depth)
        .workplaneFromTagged("aperture")
        .move(0.0, retainer_elevation)
        .rect(retainer_width, retainer_height)
        .cutBlind(aperture_depth)
        .faces(">X")
        .workplane(offset=-retainer_depth)
        .move(0.0, retainer_elevation)
        .rect(keyway_width, keyway_height)
        .cutBlind(aperture_depth + retainer_depth)
        .edges("|Z")
        .fillet(0.0004)
    )


def _make_rj45_insert() -> cq.Workplane:
    return cq.Workplane("XY").box(
        0.010,
        0.0115,
        0.0022,
        centered=(True, True, False),
    ).translate((0.0045, 0.0, 0.0030))


def _make_chip_package(
    length: float,
    width: float,
    height: float,
    *,
    pin1_radius: float,
) -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .edges("|Z")
        .fillet(min(height * 0.18, 0.00018))
    )

    pin1_mark = cq.Workplane(
        origin=(
            -(length / 2) + pin1_radius * 2.1,
            -(width / 2) + pin1_radius * 2.1,
            height + pin1_radius * 0.6,
        )
    ).sphere(pin1_radius)

    return body.cut(pin1_mark)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="raspberry_pi_style_assembly", assets=ASSETS)

    pcb_tan = model.material("pcb_tan", rgba=(0.84, 0.80, 0.58, 1.0))
    solder_mask_green = model.material("solder_mask_green", rgba=(0.05, 0.47, 0.14, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.08, 1.0))
    gold = model.material("gold", rgba=(0.93, 0.72, 0.18, 1.0))
    shell_metal = model.material("shell_metal", rgba=(0.72, 0.75, 0.77, 1.0))
    jack_insert_black = model.material("jack_insert_black", rgba=(0.10, 0.10, 0.11, 1.0))

    pcb_substrate = model.part("pcb_substrate")
    pcb_substrate.visual(
        mesh_from_cadquery(_make_board(), "pcb_substrate.obj", assets=ASSETS),
        material=pcb_tan,
    )
    pcb_substrate.inertial = Inertial.from_geometry(
        Box((BOARD_LENGTH, BOARD_WIDTH, BOARD_THICKNESS)),
        mass=0.020,
        origin=Origin(xyz=(0.0, 0.0, BOARD_THICKNESS / 2)),
    )

    pcb_solder_mask_top = model.part("pcb_solder_mask_top")
    pcb_solder_mask_top.visual(
        mesh_from_cadquery(_make_solder_mask(top=True), "pcb_solder_mask_top.obj", assets=ASSETS),
        material=solder_mask_green,
    )
    pcb_solder_mask_top.inertial = Inertial.from_geometry(
        Box((BOARD_LENGTH, BOARD_WIDTH, SOLDER_MASK_THICKNESS)),
        mass=0.001,
        origin=Origin(xyz=(0.0, 0.0, SOLDER_MASK_THICKNESS / 2)),
    )

    pcb_solder_mask_bottom = model.part("pcb_solder_mask_bottom")
    pcb_solder_mask_bottom.visual(
        mesh_from_cadquery(
            _make_solder_mask(top=False),
            "pcb_solder_mask_bottom.obj",
            assets=ASSETS,
        ),
        material=solder_mask_green,
    )
    pcb_solder_mask_bottom.inertial = Inertial.from_geometry(
        Box((BOARD_LENGTH, BOARD_WIDTH, SOLDER_MASK_THICKNESS)),
        mass=0.001,
        origin=Origin(xyz=(0.0, 0.0, -SOLDER_MASK_THICKNESS / 2)),
    )

    header_base_shape, header_pins_shape = _make_pin_header()
    gpio_header = model.part("gpio_header")
    gpio_header.visual(
        mesh_from_cadquery(header_base_shape, "gpio_header_base.obj", assets=ASSETS),
        material=matte_black,
    )
    gpio_header.visual(
        mesh_from_cadquery(header_pins_shape, "gpio_header_pins.obj", assets=ASSETS),
        material=gold,
    )
    gpio_header.inertial = Inertial.from_geometry(
        Box(
            (
                HEADER_COLUMNS * HEADER_PITCH,
                HEADER_ROWS * HEADER_PITCH,
                HEADER_PIN_ABOVE + HEADER_BASE_HEIGHT + HEADER_PIN_BELOW,
            )
        ),
        mass=0.006,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (HEADER_BASE_HEIGHT + HEADER_PIN_ABOVE - HEADER_PIN_BELOW) / 2,
            )
        ),
    )

    rj45_jack = model.part("rj45_jack")
    rj45_jack.visual(
        mesh_from_cadquery(_make_rj45_shell(), "rj45_shell.obj", assets=ASSETS),
        material=shell_metal,
    )
    rj45_jack.visual(
        mesh_from_cadquery(_make_rj45_insert(), "rj45_insert.obj", assets=ASSETS),
        material=jack_insert_black,
    )
    rj45_jack.inertial = Inertial.from_geometry(
        Box((RJ45_LENGTH, RJ45_WIDTH, RJ45_HEIGHT)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, RJ45_HEIGHT / 2)),
    )

    soc_package = model.part("soc_package")
    soc_package.visual(
        mesh_from_cadquery(
            _make_chip_package(*SOC_SIZE, pin1_radius=0.0011),
            "soc_package.obj",
            assets=ASSETS,
        ),
        material=matte_black,
    )
    soc_package.inertial = Inertial.from_geometry(
        Box(SOC_SIZE),
        mass=0.0025,
        origin=Origin(xyz=(0.0, 0.0, SOC_SIZE[2] / 2)),
    )

    memory_package = model.part("memory_package")
    memory_package.visual(
        mesh_from_cadquery(
            _make_chip_package(*MEMORY_SIZE, pin1_radius=0.0008),
            "memory_package.obj",
            assets=ASSETS,
        ),
        material=matte_black,
    )
    memory_package.inertial = Inertial.from_geometry(
        Box(MEMORY_SIZE),
        mass=0.0018,
        origin=Origin(xyz=(0.0, 0.0, MEMORY_SIZE[2] / 2)),
    )

    ethernet_controller = model.part("ethernet_controller")
    ethernet_controller.visual(
        mesh_from_cadquery(
            _make_chip_package(*ETHERNET_CTRL_SIZE, pin1_radius=0.0007),
            "ethernet_controller.obj",
            assets=ASSETS,
        ),
        material=matte_black,
    )
    ethernet_controller.inertial = Inertial.from_geometry(
        Box(ETHERNET_CTRL_SIZE),
        mass=0.0012,
        origin=Origin(xyz=(0.0, 0.0, ETHERNET_CTRL_SIZE[2] / 2)),
    )

    pmic_package = model.part("pmic_package")
    pmic_package.visual(
        mesh_from_cadquery(
            _make_chip_package(*PMIC_SIZE, pin1_radius=0.00055),
            "pmic_package.obj",
            assets=ASSETS,
        ),
        material=matte_black,
    )
    pmic_package.inertial = Inertial.from_geometry(
        Box(PMIC_SIZE),
        mass=0.0008,
        origin=Origin(xyz=(0.0, 0.0, PMIC_SIZE[2] / 2)),
    )

    model.articulation(
        "pcb_to_solder_mask_top",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="pcb_solder_mask_top",
        origin=Origin(xyz=(0.0, 0.0, BOARD_THICKNESS)),
    )
    model.articulation(
        "pcb_to_solder_mask_bottom",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="pcb_solder_mask_bottom",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "pcb_to_gpio_header",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="gpio_header",
        origin=Origin(xyz=HEADER_POSITION),
    )
    model.articulation(
        "pcb_to_rj45_jack",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="rj45_jack",
        origin=Origin(xyz=RJ45_POSITION),
    )
    model.articulation(
        "pcb_to_soc_package",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="soc_package",
        origin=Origin(xyz=SOC_POSITION),
    )
    model.articulation(
        "pcb_to_memory_package",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="memory_package",
        origin=Origin(xyz=MEMORY_POSITION),
    )
    model.articulation(
        "pcb_to_ethernet_controller",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="ethernet_controller",
        origin=Origin(xyz=ETHERNET_CTRL_POSITION),
    )
    model.articulation(
        "pcb_to_pmic_package",
        ArticulationType.FIXED,
        parent="pcb_substrate",
        child="pmic_package",
        origin=Origin(xyz=PMIC_POSITION),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")

    ctx.allow_overlap(
        "pcb_substrate",
        "pcb_solder_mask_top",
        reason="solder mask is a flush coating on the top face",
    )
    ctx.allow_overlap(
        "pcb_substrate",
        "pcb_solder_mask_bottom",
        reason="solder mask is a flush coating on the bottom face",
    )
    ctx.allow_overlap(
        "gpio_header",
        "pcb_substrate",
        reason="through-hole header pins pass through the plated board holes",
    )
    ctx.allow_overlap(
        "gpio_header",
        "pcb_solder_mask_top",
        reason="header shroud seats on the coated board surface",
    )
    ctx.allow_overlap(
        "gpio_header",
        "pcb_solder_mask_bottom",
        reason="header pin tails extend through the full PCB stackup",
    )

    ctx.warn_if_overlaps(
        max_pose_samples=32,
        ignore_adjacent=True,
        ignore_fixed=False,
    )

    ctx.expect_aabb_overlap("pcb_solder_mask_top", "pcb_substrate", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_gap(
        "pcb_solder_mask_top",
        "pcb_substrate",
        axis="z",
        max_gap=0.0002,
        max_penetration=0.0002,
    )
    ctx.expect_aabb_overlap(
        "pcb_solder_mask_bottom",
        "pcb_substrate",
        axes="xy",
        min_overlap=0.05,
    )
    ctx.expect_aabb_gap(
        "pcb_substrate",
        "pcb_solder_mask_bottom",
        axis="z",
        max_gap=0.0002,
        max_penetration=0.0002,
    )

    ctx.expect_aabb_contact("gpio_header", "pcb_substrate")
    ctx.expect_aabb_overlap("gpio_header", "pcb_substrate", axes="x", min_overlap=0.048)
    ctx.expect_aabb_overlap("gpio_header", "pcb_substrate", axes="y", min_overlap=0.005)

    ctx.expect_aabb_overlap("rj45_jack", "pcb_substrate", axes="x", min_overlap=0.017)
    ctx.expect_aabb_overlap("rj45_jack", "pcb_substrate", axes="y", min_overlap=0.014)
    ctx.expect_aabb_gap(
        "rj45_jack",
        "pcb_solder_mask_top",
        axis="z",
        max_gap=0.0008,
        max_penetration=0.0,
    )

    ctx.expect_origin_distance("soc_package", "pcb_substrate", axes="xy", max_dist=0.01)
    ctx.expect_aabb_overlap("soc_package", "pcb_substrate", axes="x", min_overlap=0.013)
    ctx.expect_aabb_overlap("soc_package", "pcb_substrate", axes="y", min_overlap=0.013)
    ctx.expect_aabb_gap(
        "soc_package",
        "pcb_solder_mask_top",
        axis="z",
        max_gap=0.0006,
        max_penetration=0.0,
    )

    ctx.expect_aabb_overlap("memory_package", "pcb_substrate", axes="x", min_overlap=0.010)
    ctx.expect_aabb_overlap("memory_package", "pcb_substrate", axes="y", min_overlap=0.008)
    ctx.expect_aabb_gap(
        "memory_package",
        "pcb_solder_mask_top",
        axis="z",
        max_gap=0.0006,
        max_penetration=0.0,
    )

    ctx.expect_aabb_overlap("ethernet_controller", "pcb_substrate", axes="x", min_overlap=0.008)
    ctx.expect_aabb_overlap("ethernet_controller", "pcb_substrate", axes="y", min_overlap=0.008)
    ctx.expect_aabb_gap(
        "ethernet_controller",
        "pcb_solder_mask_top",
        axis="z",
        max_gap=0.0006,
        max_penetration=0.0,
    )

    ctx.expect_aabb_overlap("pmic_package", "pcb_substrate", axes="x", min_overlap=0.006)
    ctx.expect_aabb_overlap("pmic_package", "pcb_substrate", axes="y", min_overlap=0.006)
    ctx.expect_aabb_gap(
        "pmic_package",
        "pcb_solder_mask_top",
        axis="z",
        max_gap=0.0006,
        max_penetration=0.0,
    )

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
