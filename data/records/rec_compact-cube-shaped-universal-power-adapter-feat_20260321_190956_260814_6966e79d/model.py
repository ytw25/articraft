from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BODY_W = 0.064
BODY_D = 0.064
BODY_H = 0.066


def _rect_profile(width: float, height: float, *, cx: float = 0.0, cy: float = 0.0) -> list[tuple[float, float]]:
    hw = width / 2.0
    hh = height / 2.0
    return [
        (cx - hw, cy - hh),
        (cx + hw, cy - hh),
        (cx + hw, cy + hh),
        (cx - hw, cy + hh),
    ]


def _shift_profile(profile: list[tuple[float, float]], dx: float, dy: float) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _lift_profile(profile: list[tuple[float, float]], z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in profile]


_GLYPHS: dict[str, tuple[str, ...]] = {
    "A": ("0110", "1001", "1111", "1001", "1001"),
    "C": ("0111", "1000", "1000", "1000", "0111"),
    "E": ("1111", "1000", "1110", "1000", "1111"),
    "K": ("1001", "1010", "1100", "1010", "1001"),
    "M": ("10001", "11011", "10101", "10001", "10001"),
    "O": ("0110", "1001", "1001", "1001", "0110"),
    "R": ("1110", "1001", "1110", "1010", "1001"),
    "S": ("0111", "1000", "0110", "0001", "1110"),
    "U": ("1001", "1001", "1001", "1001", "0110"),
    "V": ("10001", "10001", "10001", "01010", "00100"),
    "X": ("10001", "01010", "00100", "01010", "10001"),
    "/": ("0001", "0010", "0100", "1000", "0000"),
    ".": ("0", "0", "0", "0", "1"),
    "1": ("010", "110", "010", "010", "111"),
    "2": ("1110", "0001", "0110", "1000", "1111"),
    "3": ("1110", "0001", "0110", "0001", "1110"),
    "4": ("1001", "1001", "1111", "0001", "0001"),
    "5": ("1111", "1000", "1110", "0001", "1110"),
    " ": ("00", "00", "00", "00", "00"),
}


def _text_geometry(text: str, pixel: float, depth: float):
    geom = None
    cursor = 0.0
    max_height = 0
    for char in text:
        pattern = _GLYPHS[char]
        width = len(pattern[0])
        max_height = max(max_height, len(pattern))
        for row, row_bits in enumerate(pattern):
            for col, bit in enumerate(row_bits):
                if bit != "1":
                    continue
                px = cursor + (col + 0.5) * pixel
                py = (len(pattern) - row - 0.5) * pixel
                cell = BoxGeometry((pixel, pixel, depth)).translate(px, py, 0.0)
                geom = cell if geom is None else geom.merge(cell)
        cursor += (width + 1) * pixel
    if geom is None:
        geom = BoxGeometry((pixel, pixel, depth))
    total_width = max(cursor - pixel, pixel)
    total_height = max(max_height * pixel, pixel)
    geom.translate(-total_width / 2.0, -total_height / 2.0, 0.0)
    return geom


def _save_text_mesh(
    text: str,
    filename: str,
    *,
    pixel: float,
    depth: float,
    orientation: str,
):
    geom = _text_geometry(text, pixel, depth)
    if orientation == "front":
        geom.rotate_x(math.pi / 2.0)
    elif orientation == "side":
        geom.rotate_x(math.pi / 2.0).rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _body_shell_mesh():
    geom = BoxGeometry((0.060, 0.060, 0.018)).translate(0.0, 0.0, 0.009)
    geom.merge(BoxGeometry((0.064, 0.064, 0.030)).translate(0.0, 0.0, 0.033))
    geom.merge(BoxGeometry((0.0615, 0.0615, 0.018)).translate(0.0, 0.0, 0.057))
    geom.merge(BoxGeometry((0.056, 0.056, 0.004)).translate(0.0, 0.0, 0.064))
    return mesh_from_geometry(geom, ASSETS.mesh_path("body_shell.obj"))


def _top_socket_plate_mesh():
    geom = BoxGeometry((0.036, 0.0045, 0.0022)).translate(0.0, 0.0108, 0.0)
    geom.merge(BoxGeometry((0.036, 0.0045, 0.0022)).translate(0.0, -0.0108, 0.0))
    geom.merge(BoxGeometry((0.0045, 0.0170, 0.0022)).translate(-0.0158, 0.0, 0.0))
    geom.merge(BoxGeometry((0.0045, 0.0170, 0.0022)).translate(0.0158, 0.0, 0.0))
    geom.merge(BoxGeometry((0.0110, 0.0032, 0.0022)).translate(0.0, 0.0044, 0.0))
    geom.merge(BoxGeometry((0.0022, 0.0100, 0.0022)).translate(-0.0072, 0.0012, 0.0))
    geom.merge(BoxGeometry((0.0022, 0.0100, 0.0022)).translate(0.0072, 0.0012, 0.0))
    return mesh_from_geometry(geom, ASSETS.mesh_path("top_socket_plate.obj"))


def _front_panel_mesh():
    geom = BoxGeometry((0.050, 0.0024, 0.0040)).translate(0.0, 0.0, 0.0210)
    geom.merge(BoxGeometry((0.050, 0.0024, 0.0040)).translate(0.0, 0.0, -0.0210))
    geom.merge(BoxGeometry((0.0040, 0.0024, 0.046)).translate(-0.0230, 0.0, 0.0))
    geom.merge(BoxGeometry((0.0040, 0.0024, 0.046)).translate(0.0230, 0.0, 0.0))
    geom.merge(BoxGeometry((0.0040, 0.0024, 0.030)).translate(-0.0105, 0.0, -0.0040))
    geom.merge(BoxGeometry((0.0040, 0.0024, 0.030)).translate(0.0105, 0.0, -0.0040))
    geom.merge(BoxGeometry((0.0380, 0.0024, 0.0030)).translate(0.0, 0.0, 0.0110))
    return mesh_from_geometry(geom, ASSETS.mesh_path("front_panel.obj"))


def _side_panel_mesh():
    geom = BoxGeometry((0.0024, 0.036, 0.0040)).translate(0.0, 0.0, 0.0230)
    geom.merge(BoxGeometry((0.0024, 0.036, 0.0040)).translate(0.0, 0.0, -0.0230))
    geom.merge(BoxGeometry((0.0024, 0.0040, 0.050)).translate(0.0, -0.0160, 0.0))
    geom.merge(BoxGeometry((0.0024, 0.0040, 0.050)).translate(0.0, 0.0160, 0.0))
    geom.merge(BoxGeometry((0.0024, 0.0030, 0.032)).translate(0.0, 0.0, 0.0020))
    geom.merge(BoxGeometry((0.0024, 0.030, 0.0030)).translate(0.0, 0.0, 0.0005))
    return mesh_from_geometry(geom, ASSETS.mesh_path("side_panel.obj"))


def _linkage_rod_mesh():
    geom = tube_from_spline_points(
        [
            (0.0, 0.000, 0.000),
            (0.0, 0.003, -0.006),
            (0.0, 0.008, -0.016),
            (0.0, 0.013, -0.030),
        ],
        radius=0.0014,
        samples_per_segment=16,
        radial_segments=16,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("linkage_rod.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="universal_travel_adapter", assets=ASSETS)

    shell_white = model.material("shell_white", rgba=(0.95, 0.95, 0.93, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.11, 0.12, 0.14, 1.0))
    slider_black = model.material("slider_black", rgba=(0.18, 0.18, 0.20, 1.0))
    text_light = model.material("text_light", rgba=(0.86, 0.86, 0.84, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    shutter_gray = model.material("shutter_gray", rgba=(0.77, 0.78, 0.80, 1.0))
    phosphor_bronze = model.material("phosphor_bronze", rgba=(0.73, 0.50, 0.20, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.11, 0.40, 0.22, 1.0))
    chip_black = model.material("chip_black", rgba=(0.05, 0.05, 0.06, 1.0))

    body_mesh = _body_shell_mesh()
    top_panel_mesh = _top_socket_plate_mesh()
    front_panel_mesh = _front_panel_mesh()
    side_panel_mesh = _side_panel_mesh()
    rod_mesh = _linkage_rod_mesh()

    front_label_uk = _save_text_mesh("UK", "front_label_uk.obj", pixel=0.00075, depth=0.00045, orientation="front")
    front_label_usa = _save_text_mesh(
        "USA/AUS",
        "front_label_usa.obj",
        pixel=0.00050,
        depth=0.00045,
        orientation="front",
    )
    front_label_euro = _save_text_mesh(
        "EURO",
        "front_label_euro.obj",
        pixel=0.00060,
        depth=0.00045,
        orientation="front",
    )
    side_label_c1 = _save_text_mesh("C1 5V/3A", "side_label_c1.obj", pixel=0.00052, depth=0.00040, orientation="side")
    side_label_c2 = _save_text_mesh("C2 5V/3A", "side_label_c2.obj", pixel=0.00052, depth=0.00040, orientation="side")
    side_label_max = _save_text_mesh(
        "3.4A MAX",
        "side_label_max.obj",
        pixel=0.00052,
        depth=0.00040,
        orientation="side",
    )

    body = model.part("body")
    body.visual(body_mesh, material=shell_white)
    body.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_D, BODY_H)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
    )

    top_panel = model.part("top_panel")
    top_panel.visual(top_panel_mesh, material=shell_dark)
    top_panel.visual(
        Box((0.029, 0.019, 0.0055)),
        origin=Origin(xyz=(0.0, 0.0, -0.0024)),
        material=chip_black,
    )
    top_panel.visual(
        Box((0.012, 0.0016, 0.0010)),
        origin=Origin(xyz=(0.0, 0.0, -0.0010)),
        material=steel,
    )
    top_panel.inertial = Inertial.from_geometry(Box((0.036, 0.026, 0.006)), mass=0.012)

    front_panel = model.part("front_panel")
    front_panel.visual(front_panel_mesh, material=shell_dark)
    front_panel.visual(
        front_label_uk,
        origin=Origin(xyz=(-0.016, -0.0010, 0.0155)),
        material=text_light,
    )
    front_panel.visual(
        front_label_usa,
        origin=Origin(xyz=(0.0, -0.0010, 0.0155)),
        material=text_light,
    )
    front_panel.visual(
        front_label_euro,
        origin=Origin(xyz=(0.016, -0.0010, 0.0155)),
        material=text_light,
    )
    front_panel.visual(
        Box((0.046, 0.0022, 0.030)),
        origin=Origin(xyz=(0.0, 0.0010, -0.0010)),
        material=shell_dark,
    )
    front_panel.inertial = Inertial.from_geometry(Box((0.050, 0.004, 0.046)), mass=0.015)

    side_panel = model.part("side_panel")
    side_panel.visual(side_panel_mesh, material=shell_dark)
    side_panel.visual(
        Box((0.0018, 0.030, 0.032)),
        origin=Origin(xyz=(-0.0007, 0.0, -0.0020)),
        material=chip_black,
    )
    side_panel.visual(
        side_label_c1,
        origin=Origin(xyz=(0.0006, -0.0088, 0.0170)),
        material=text_light,
    )
    side_panel.visual(
        side_label_c2,
        origin=Origin(xyz=(0.0006, 0.0092, 0.0170)),
        material=text_light,
    )
    side_panel.visual(
        side_label_max,
        origin=Origin(xyz=(0.0006, 0.0000, -0.0200)),
        material=text_light,
    )
    side_panel.inertial = Inertial.from_geometry(Box((0.004, 0.036, 0.050)), mass=0.018)

    usb_c1_port = model.part("usb_c1_port")
    usb_c1_port.visual(Box((0.0032, 0.0100, 0.0036)), material=chip_black)
    usb_c1_port.visual(Box((0.0010, 0.0040, 0.0008)), origin=Origin(xyz=(-0.0007, 0.0, -0.0005)), material=steel)
    usb_c1_port.inertial = Inertial.from_geometry(Box((0.0032, 0.0100, 0.0036)), mass=0.0015)

    usb_c2_port = model.part("usb_c2_port")
    usb_c2_port.visual(Box((0.0032, 0.0100, 0.0036)), material=chip_black)
    usb_c2_port.visual(Box((0.0010, 0.0040, 0.0008)), origin=Origin(xyz=(-0.0007, 0.0, -0.0005)), material=steel)
    usb_c2_port.inertial = Inertial.from_geometry(Box((0.0032, 0.0100, 0.0036)), mass=0.0015)

    usb_a1_port = model.part("usb_a1_port")
    usb_a1_port.visual(Box((0.0038, 0.0135, 0.0052)), material=chip_black)
    usb_a1_port.visual(Box((0.0010, 0.0105, 0.0008)), origin=Origin(xyz=(-0.0008, 0.0, -0.0006)), material=steel)
    usb_a1_port.inertial = Inertial.from_geometry(Box((0.0038, 0.0135, 0.0052)), mass=0.0018)

    usb_a2_port = model.part("usb_a2_port")
    usb_a2_port.visual(Box((0.0038, 0.0135, 0.0052)), material=chip_black)
    usb_a2_port.visual(Box((0.0010, 0.0105, 0.0008)), origin=Origin(xyz=(-0.0008, 0.0, -0.0006)), material=steel)
    usb_a2_port.inertial = Inertial.from_geometry(Box((0.0038, 0.0135, 0.0052)), mass=0.0018)

    pcba = model.part("pcba")
    pcba.visual(Box((0.0022, 0.040, 0.045)), material=pcb_green)
    pcba.visual(Box((0.0032, 0.010, 0.007)), origin=Origin(xyz=(0.0006, -0.010, 0.008)), material=chip_black)
    pcba.visual(Box((0.0032, 0.012, 0.008)), origin=Origin(xyz=(0.0006, 0.010, -0.010)), material=chip_black)
    pcba.visual(Box((0.0012, 0.006, 0.006)), origin=Origin(xyz=(-0.0010, -0.015, -0.014)), material=steel)
    pcba.visual(Box((0.0012, 0.006, 0.006)), origin=Origin(xyz=(-0.0010, 0.015, 0.014)), material=steel)
    pcba.inertial = Inertial.from_geometry(Box((0.0032, 0.040, 0.045)), mass=0.015)

    busbars = model.part("busbars")
    busbars.visual(Box((0.030, 0.008, 0.0010)), origin=Origin(xyz=(0.0, 0.000, 0.0155)), material=phosphor_bronze)
    busbars.visual(Box((0.0026, 0.008, 0.030)), origin=Origin(xyz=(-0.016, 0.000, 0.0000)), material=phosphor_bronze)
    busbars.visual(Box((0.0026, 0.008, 0.030)), origin=Origin(xyz=(0.000, 0.000, 0.0000)), material=phosphor_bronze)
    busbars.visual(Box((0.0026, 0.008, 0.030)), origin=Origin(xyz=(0.016, 0.000, 0.0000)), material=phosphor_bronze)
    busbars.visual(Box((0.040, 0.008, 0.0010)), origin=Origin(xyz=(0.0, 0.000, -0.0150)), material=phosphor_bronze)
    busbars.visual(Box((0.020, 0.010, 0.0010)), origin=Origin(xyz=(0.010, 0.013, -0.0070)), material=phosphor_bronze)
    busbars.visual(Box((0.0010, 0.018, 0.0060)), origin=Origin(xyz=(0.0195, 0.013, -0.0040)), material=phosphor_bronze)
    busbars.inertial = Inertial.from_geometry(Box((0.040, 0.018, 0.032)), mass=0.010, origin=Origin(xyz=(0.0, 0.004, 0.000)))

    interlock_bar = model.part("interlock_bar")
    interlock_bar.visual(Box((0.042, 0.0025, 0.0045)), material=shell_dark)
    interlock_bar.visual(Box((0.004, 0.0030, 0.010)), origin=Origin(xyz=(-0.016, 0.0002, 0.0)), material=shell_dark)
    interlock_bar.visual(Box((0.004, 0.0030, 0.010)), origin=Origin(xyz=(0.000, 0.0002, 0.0)), material=shell_dark)
    interlock_bar.visual(Box((0.004, 0.0030, 0.010)), origin=Origin(xyz=(0.016, 0.0002, 0.0)), material=shell_dark)
    interlock_bar.inertial = Inertial.from_geometry(Box((0.042, 0.0030, 0.010)), mass=0.006)

    for name in ("uk_slider", "usa_slider", "euro_slider"):
        slider = model.part(name)
        slider.visual(Box((0.0094, 0.0048, 0.0090)), origin=Origin(xyz=(0.0, -0.0010, 0.0)), material=slider_black)
        slider.visual(Box((0.0036, 0.0060, 0.020)), origin=Origin(xyz=(0.0, 0.0018, -0.006)), material=slider_black)
        slider.visual(Box((0.0060, 0.0008, 0.0010)), origin=Origin(xyz=(0.0, -0.0031, 0.0024)), material=text_light)
        slider.inertial = Inertial.from_geometry(Box((0.0094, 0.0060, 0.020)), mass=0.008, origin=Origin(xyz=(0.0, 0.001, -0.005)))

    for name in ("uk_linkage", "usa_linkage", "euro_linkage"):
        linkage = model.part(name)
        linkage.visual(rod_mesh, material=steel)
        linkage.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0016, length=0.033),
            mass=0.004,
            origin=Origin(xyz=(0.0, 0.006, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

    shutter_left = model.part("socket_shutter_left")
    shutter_left.visual(Box((0.0115, 0.0090, 0.0012)), material=shutter_gray)
    shutter_left.visual(Box((0.0030, 0.0040, 0.0008)), origin=Origin(xyz=(0.0035, 0.0015, 0.0)), material=shutter_gray)
    shutter_left.inertial = Inertial.from_geometry(Box((0.0115, 0.0090, 0.0012)), mass=0.003)

    shutter_right = model.part("socket_shutter_right")
    shutter_right.visual(Box((0.0115, 0.0090, 0.0012)), material=shutter_gray)
    shutter_right.visual(Box((0.0030, 0.0040, 0.0008)), origin=Origin(xyz=(-0.0035, 0.0015, 0.0)), material=shutter_gray)
    shutter_right.inertial = Inertial.from_geometry(Box((0.0115, 0.0090, 0.0012)), mass=0.003)

    uk_plug = model.part("uk_plug")
    uk_plug.visual(Box((0.008, 0.016, 0.010)), origin=Origin(xyz=(0.0035, 0.0, 0.0)), material=shell_dark)
    uk_plug.visual(Box((0.014, 0.0022, 0.0052)), origin=Origin(xyz=(0.011, -0.0062, -0.0022)), material=steel)
    uk_plug.visual(Box((0.014, 0.0022, 0.0052)), origin=Origin(xyz=(0.011, 0.0062, -0.0022)), material=steel)
    uk_plug.visual(Box((0.013, 0.0026, 0.0062)), origin=Origin(xyz=(0.0105, 0.0, 0.0045)), material=steel)
    uk_plug.inertial = Inertial.from_geometry(Box((0.020, 0.016, 0.012)), mass=0.018, origin=Origin(xyz=(0.008, 0.0, 0.0)))

    usa_plug = model.part("usa_plug")
    usa_plug.visual(Box((0.010, 0.016, 0.0065)), origin=Origin(xyz=(0.0140, 0.0, 0.0)), material=shell_dark)
    usa_plug.visual(Box((0.005, 0.012, 0.004)), origin=Origin(xyz=(0.0060, 0.0, 0.0)), material=shell_dark)
    usa_plug.inertial = Inertial.from_geometry(Box((0.020, 0.016, 0.007)), mass=0.012, origin=Origin(xyz=(0.0110, 0.0, 0.0)))

    usa_blades = model.part("usa_blades")
    usa_blades.visual(Box((0.015, 0.0015, 0.0052)), origin=Origin(xyz=(0.0120, -0.0052, 0.0)), material=steel)
    usa_blades.visual(Box((0.015, 0.0015, 0.0052)), origin=Origin(xyz=(0.0120, 0.0052, 0.0)), material=steel)
    usa_blades.visual(Box((0.0040, 0.014, 0.0040)), origin=Origin(xyz=(0.0040, 0.0, 0.0)), material=steel)
    usa_blades.inertial = Inertial.from_geometry(Box((0.020, 0.014, 0.006)), mass=0.010, origin=Origin(xyz=(0.0110, 0.0, 0.0)))

    euro_plug = model.part("euro_plug")
    euro_plug.visual(Box((0.008, 0.014, 0.0065)), origin=Origin(xyz=(0.0060, 0.0, 0.0)), material=shell_dark)
    euro_plug.visual(
        Cylinder(radius=0.0019, length=0.014),
        origin=Origin(xyz=(0.0155, -0.0048, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
    )
    euro_plug.visual(
        Cylinder(radius=0.0019, length=0.014),
        origin=Origin(xyz=(0.0155, 0.0048, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
    )
    euro_plug.inertial = Inertial.from_geometry(Box((0.022, 0.014, 0.007)), mass=0.014, origin=Origin(xyz=(0.0110, 0.0, 0.0)))

    model.articulation("body_to_top_panel", ArticulationType.FIXED, parent="body", child="top_panel", origin=Origin(xyz=(0.0, 0.0, 0.0649)))
    model.articulation("body_to_front_panel", ArticulationType.FIXED, parent="body", child="front_panel", origin=Origin(xyz=(0.0, -0.0310, 0.0400)))
    model.articulation("body_to_side_panel", ArticulationType.FIXED, parent="body", child="side_panel", origin=Origin(xyz=(0.0310, 0.0, 0.0330)))
    model.articulation("body_to_pcba", ArticulationType.FIXED, parent="body", child="pcba", origin=Origin(xyz=(0.0190, 0.0, 0.0330)))
    model.articulation("body_to_busbars", ArticulationType.FIXED, parent="body", child="busbars", origin=Origin(xyz=(0.0, 0.0, 0.0390)))
    model.articulation("side_panel_to_usb_c1", ArticulationType.FIXED, parent="side_panel", child="usb_c1_port", origin=Origin(xyz=(-0.0015, -0.0090, 0.0105)))
    model.articulation("side_panel_to_usb_c2", ArticulationType.FIXED, parent="side_panel", child="usb_c2_port", origin=Origin(xyz=(-0.0015, 0.0090, 0.0105)))
    model.articulation("side_panel_to_usb_a1", ArticulationType.FIXED, parent="side_panel", child="usb_a1_port", origin=Origin(xyz=(-0.0018, -0.0095, -0.0090)))
    model.articulation("side_panel_to_usb_a2", ArticulationType.FIXED, parent="side_panel", child="usb_a2_port", origin=Origin(xyz=(-0.0018, 0.0095, -0.0090)))

    model.articulation(
        "socket_shutter_left_joint",
        ArticulationType.PRISMATIC,
        parent="top_panel",
        child="socket_shutter_left",
        origin=Origin(xyz=(-0.0045, 0.0, -0.0015)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.0042),
    )
    model.articulation(
        "socket_shutter_right_joint",
        ArticulationType.PRISMATIC,
        parent="top_panel",
        child="socket_shutter_right",
        origin=Origin(xyz=(0.0045, 0.0, -0.0015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.0042),
    )

    model.articulation(
        "interlock_joint",
        ArticulationType.PRISMATIC,
        parent="front_panel",
        child="interlock_bar",
        origin=Origin(xyz=(0.0, 0.0043, -0.0015)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.05, lower=-0.008, upper=0.008),
    )

    slider_specs = [
        ("uk_slider_joint", "uk_slider", -0.016),
        ("usa_slider_joint", "usa_slider", 0.000),
        ("euro_slider_joint", "euro_slider", 0.016),
    ]
    for joint_name, child_name, x_pos in slider_specs:
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent="front_panel",
            child=child_name,
            origin=Origin(xyz=(x_pos, 0.0005, 0.0060)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=6.0, velocity=0.06, lower=0.0, upper=0.012),
        )

    linkage_specs = [
        ("uk_linkage_joint", "uk_slider", "uk_linkage"),
        ("usa_linkage_joint", "usa_slider", "usa_linkage"),
        ("euro_linkage_joint", "euro_slider", "euro_linkage"),
    ]
    for joint_name, parent_name, child_name in linkage_specs:
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=parent_name,
            child=child_name,
            origin=Origin(xyz=(0.0, 0.0030, -0.0040)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=0.95),
        )

    model.articulation(
        "uk_plug_joint",
        ArticulationType.REVOLUTE,
        parent="body",
        child="uk_plug",
        origin=Origin(xyz=(-0.0180, 0.0000, 0.0070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "usa_plug_joint",
        ArticulationType.REVOLUTE,
        parent="body",
        child="usa_plug",
        origin=Origin(xyz=(0.0000, 0.0000, 0.0070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )
    model.articulation(
        "usa_swivel_joint",
        ArticulationType.REVOLUTE,
        parent="usa_plug",
        child="usa_blades",
        origin=Origin(xyz=(0.0055, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.5, lower=0.0, upper=0.42),
    )
    model.articulation(
        "euro_plug_joint",
        ArticulationType.REVOLUTE,
        parent="body",
        child="euro_plug",
        origin=Origin(xyz=(0.0180, 0.0000, 0.0070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=math.pi / 2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual", ignore_adjacent=True, ignore_fixed=True)
    ctx.allow_overlap("body", "usa_blades", reason="retracted and transitioning USA/AUS blades nest within the body cavity")
    ctx.allow_overlap("euro_plug", "usa_blades", reason="generic pose sampling can combine mutually exclusive plug states that the interlock would prevent")
    ctx.warn_if_overlaps(
        max_pose_samples=160,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("top_panel", "body")
    ctx.expect_aabb_contact("front_panel", "body")
    ctx.expect_aabb_contact("side_panel", "body")
    ctx.expect_aabb_contact("usb_c1_port", "side_panel")
    ctx.expect_aabb_contact("usb_c2_port", "side_panel")
    ctx.expect_aabb_contact("usb_a1_port", "side_panel")
    ctx.expect_aabb_contact("usb_a2_port", "side_panel")

    ctx.expect_aabb_overlap("pcba", "side_panel", axes="yz", min_overlap=0.022)
    ctx.expect_aabb_gap("pcba", "side_panel", axis="x", max_gap=0.013, max_penetration=0.016)
    ctx.expect_aabb_overlap("usb_c1_port", "pcba", axes="yz", min_overlap=0.003)
    ctx.expect_aabb_overlap("usb_c2_port", "pcba", axes="yz", min_overlap=0.003)
    ctx.expect_aabb_overlap("usb_a1_port", "pcba", axes="yz", min_overlap=0.004)
    ctx.expect_aabb_overlap("usb_a2_port", "pcba", axes="yz", min_overlap=0.004)
    ctx.expect_aabb_overlap("busbars", "top_panel", axes="xy", min_overlap=0.016)
    ctx.expect_aabb_overlap("busbars", "body", axes="xz", min_overlap=0.018)

    ctx.expect_aabb_overlap("interlock_bar", "front_panel", axes="xz", min_overlap=0.009)
    ctx.expect_aabb_gap("interlock_bar", "front_panel", axis="y", max_gap=0.008, max_penetration=0.002)
    ctx.expect_aabb_overlap("interlock_bar", "uk_slider", axes="xz", min_overlap=0.005)
    ctx.expect_aabb_overlap("interlock_bar", "usa_slider", axes="xz", min_overlap=0.005)
    ctx.expect_aabb_overlap("interlock_bar", "euro_slider", axes="xz", min_overlap=0.005)

    ctx.expect_joint_motion_axis("uk_slider_joint", "uk_slider", world_axis="z", direction="negative", min_delta=0.008)
    ctx.expect_joint_motion_axis("usa_slider_joint", "usa_slider", world_axis="z", direction="negative", min_delta=0.008)
    ctx.expect_joint_motion_axis("euro_slider_joint", "euro_slider", world_axis="z", direction="negative", min_delta=0.008)
    ctx.expect_joint_motion_axis(
        "socket_shutter_left_joint",
        "socket_shutter_left",
        world_axis="x",
        direction="negative",
        min_delta=0.0025,
    )
    ctx.expect_joint_motion_axis(
        "socket_shutter_right_joint",
        "socket_shutter_right",
        world_axis="x",
        direction="positive",
        min_delta=0.0025,
    )
    ctx.expect_joint_motion_axis("uk_plug_joint", "uk_plug", world_axis="z", direction="negative", min_delta=0.010)
    ctx.expect_joint_motion_axis("usa_swivel_joint", "usa_blades", world_axis="y", direction="positive", min_delta=0.002)

    ctx.expect_aabb_gap("socket_shutter_left", "top_panel", axis="z", max_gap=0.004, max_penetration=0.004)
    ctx.expect_aabb_gap("socket_shutter_right", "top_panel", axis="z", max_gap=0.004, max_penetration=0.004)
    ctx.expect_aabb_overlap("socket_shutter_left", "top_panel", axes="xy", min_overlap=0.006)
    ctx.expect_aabb_overlap("socket_shutter_right", "top_panel", axes="xy", min_overlap=0.006)

    with ctx.pose(uk_slider_joint=0.011, uk_linkage_joint=0.80, uk_plug_joint=math.pi / 2.0, interlock_joint=-0.006):
        ctx.expect_aabb_contact("uk_plug", "body")
        ctx.expect_aabb_overlap("uk_plug", "body", axes="xy", min_overlap=0.004)
        ctx.expect_aabb_overlap("uk_linkage", "uk_slider", axes="yz", min_overlap=0.003)
        ctx.expect_aabb_overlap("interlock_bar", "usa_slider", axes="xz", min_overlap=0.003)
        ctx.expect_aabb_overlap("interlock_bar", "euro_slider", axes="xz", min_overlap=0.003)

    with ctx.pose(usa_slider_joint=0.011, usa_linkage_joint=0.82, usa_plug_joint=math.pi / 2.0, usa_swivel_joint=0.38):
        ctx.expect_aabb_contact("usa_plug", "body")
        ctx.expect_aabb_contact("usa_blades", "usa_plug")
        ctx.expect_aabb_overlap("usa_blades", "body", axes="xy", min_overlap=0.003)

    with ctx.pose(euro_slider_joint=0.011, euro_linkage_joint=0.78, euro_plug_joint=math.pi / 2.0, interlock_joint=0.006):
        ctx.expect_aabb_contact("euro_plug", "body")
        ctx.expect_aabb_overlap("euro_plug", "body", axes="xy", min_overlap=0.004)
        ctx.expect_aabb_overlap("euro_linkage", "euro_slider", axes="yz", min_overlap=0.003)
        ctx.expect_aabb_overlap("interlock_bar", "uk_slider", axes="xz", min_overlap=0.003)
        ctx.expect_aabb_overlap("interlock_bar", "usa_slider", axes="xz", min_overlap=0.003)

    with ctx.pose(socket_shutter_left_joint=0.0038, socket_shutter_right_joint=0.0038):
        ctx.expect_aabb_overlap("socket_shutter_left", "top_panel", axes="xy", min_overlap=0.003)
        ctx.expect_aabb_overlap("socket_shutter_right", "top_panel", axes="xy", min_overlap=0.003)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
