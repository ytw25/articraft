from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
from pathlib import Path

HERE = Path(__file__).parent
_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        os.chdir(HERE)
        return _REAL_GETCWD()


os.getcwd = _safe_getcwd

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_FLANGE = 0.44
BASE_PLINTH = 0.31
COLUMN_OUTER = 0.24
WALL_THICKNESS = 0.022
COLUMN_BOTTOM_Z = 0.15
COLUMN_HEIGHT = 0.58
COLUMN_TOP_Z = COLUMN_BOTTOM_Z + COLUMN_HEIGHT
PANEL_RECESS_OFFSET = 0.111
PANEL_FRAME_OFFSET = 0.121

DIAL_RADIUS = 0.052
DIAL_CENTER_Z = 0.43
FACE_BOSS_OFFSET = 0.121
HAND_ORIGIN_OFFSET = 0.122
SHAFT_RADIUS = 0.006
SHAFT_LENGTH = COLUMN_OUTER + 0.004


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_strut(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _face_visual_rpy(face_name: str) -> tuple[float, float, float]:
    if face_name in {"front", "back"}:
        return (math.pi * 0.5, 0.0, 0.0)
    if face_name == "right":
        return (math.pi * 0.5, 0.0, math.pi * 0.5)
    return (math.pi * 0.5, 0.0, -math.pi * 0.5)


def _axis_cylinder_rpy(axis_name: str) -> tuple[float, float, float]:
    if axis_name == "y":
        return (-math.pi * 0.5, 0.0, 0.0)
    return (0.0, math.pi * 0.5, 0.0)


def _face_center(
    axis_name: str,
    sign: float,
    offset: float,
    z: float,
    lateral: float = 0.0,
) -> tuple[float, float, float]:
    if axis_name == "y":
        return (lateral, sign * offset, z)
    return (sign * offset, lateral, z)


def _face_box_size(axis_name: str, width: float, thickness: float, height: float) -> tuple[float, float, float]:
    if axis_name == "y":
        return (width, thickness, height)
    return (thickness, width, height)


def _add_face_panels(
    part,
    *,
    face_name: str,
    axis_name: str,
    sign: float,
    iron,
    recess,
    bezel,
    dial,
    tick,
) -> None:
    dial_mount = _face_center(axis_name, sign, 0.116, DIAL_CENTER_Z)
    boss_mount = _face_center(axis_name, sign, FACE_BOSS_OFFSET, DIAL_CENTER_Z)
    panel_rect_center = _face_center(axis_name, sign, PANEL_RECESS_OFFSET, 0.397)
    panel_arch_center = _face_center(axis_name, sign, PANEL_RECESS_OFFSET, 0.492)
    frame_arch_center = _face_center(axis_name, sign, PANEL_FRAME_OFFSET, 0.491)
    if axis_name == "y":
        tick_specs = (
            ((0.0, sign * 0.1185, DIAL_CENTER_Z + 0.038), (0.006, 0.002, 0.014)),
            ((0.0, sign * 0.1185, DIAL_CENTER_Z - 0.038), (0.006, 0.002, 0.014)),
            ((0.038, sign * 0.1185, DIAL_CENTER_Z), (0.014, 0.002, 0.006)),
            ((-0.038, sign * 0.1185, DIAL_CENTER_Z), (0.014, 0.002, 0.006)),
        )
        jamb_centers = ((-0.078, sign * 0.121, 0.388), (0.078, sign * 0.121, 0.388))
        jamb_size = (0.014, 0.012, 0.206)
        sill_center = (0.0, sign * 0.121, 0.289)
        sill_size = (0.170, 0.012, 0.014)
    else:
        tick_specs = (
            ((sign * 0.1185, 0.0, DIAL_CENTER_Z + 0.038), (0.002, 0.006, 0.014)),
            ((sign * 0.1185, 0.0, DIAL_CENTER_Z - 0.038), (0.002, 0.006, 0.014)),
            ((sign * 0.1185, 0.038, DIAL_CENTER_Z), (0.002, 0.014, 0.006)),
            ((sign * 0.1185, -0.038, DIAL_CENTER_Z), (0.002, 0.014, 0.006)),
        )
        jamb_centers = ((sign * 0.121, -0.078, 0.388), (sign * 0.121, 0.078, 0.388))
        jamb_size = (0.012, 0.014, 0.206)
        sill_center = (sign * 0.121, 0.0, 0.289)
        sill_size = (0.012, 0.170, 0.014)

    part.visual(
        Box(_face_box_size(axis_name, 0.154, 0.010, 0.190)),
        origin=Origin(xyz=panel_rect_center),
        material=recess,
        name=f"{face_name}_arch_panel",
    )
    part.visual(
        Cylinder(radius=0.077, length=0.010),
        origin=Origin(xyz=panel_arch_center, rpy=_axis_cylinder_rpy(axis_name)),
        material=recess,
        name=f"{face_name}_arch_cap",
    )
    part.visual(
        Cylinder(radius=0.085, length=0.012),
        origin=Origin(xyz=frame_arch_center, rpy=_axis_cylinder_rpy(axis_name)),
        material=iron,
        name=f"{face_name}_arch_frame",
    )
    for index, center in enumerate(jamb_centers):
        part.visual(
            Box(jamb_size),
            origin=Origin(xyz=center),
            material=iron,
            name=f"{face_name}_arch_jamb_{index}",
        )
    part.visual(
        Box(sill_size),
        origin=Origin(xyz=sill_center),
        material=iron,
        name=f"{face_name}_arch_sill",
    )
    part.visual(
        Cylinder(radius=0.060, length=0.010),
        origin=Origin(xyz=_face_center(axis_name, sign, 0.1175, DIAL_CENTER_Z), rpy=_axis_cylinder_rpy(axis_name)),
        material=bezel,
        name=f"{face_name}_bezel",
    )
    part.visual(
        Cylinder(radius=DIAL_RADIUS, length=0.006),
        origin=Origin(xyz=dial_mount, rpy=_axis_cylinder_rpy(axis_name)),
        material=dial,
        name=f"{face_name}_dial",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.002),
        origin=Origin(xyz=boss_mount, rpy=_axis_cylinder_rpy(axis_name)),
        material=bezel,
        name=f"{face_name}_boss",
    )
    for index, (xyz, size) in enumerate(tick_specs):
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=tick,
            name=f"{face_name}_tick_{index}",
        )


def _add_hand_geometry(
    part,
    *,
    axis_name: str,
    sign: float,
    hand_kind: str,
    material,
) -> None:
    if hand_kind == "hour":
        hub_offset = 0.001
        arm_offset = 0.0012
        blade_length = 0.031
        blade_width = 0.008
        tail_length = 0.014
        tail_width = 0.004
    else:
        hub_offset = 0.003
        arm_offset = 0.0032
        blade_length = 0.044
        blade_width = 0.005
        tail_length = 0.018
        tail_width = 0.003

    thickness = 0.002
    if axis_name == "y":
        part.visual(
            Cylinder(radius=0.008 if hand_kind == "hour" else 0.0065, length=0.002),
            origin=Origin(
                xyz=(0.0, sign * hub_offset, 0.0),
                rpy=_axis_cylinder_rpy(axis_name),
            ),
            material=material,
            name=f"{hand_kind}_hub",
        )
        part.visual(
            Box((blade_width, thickness, blade_length)),
            origin=Origin(xyz=(0.0, sign * arm_offset, blade_length * 0.5 - 0.004)),
            material=material,
            name=f"{hand_kind}_blade",
        )
        part.visual(
            Box((tail_width, thickness, tail_length)),
            origin=Origin(xyz=(0.0, sign * arm_offset, -tail_length * 0.5 - 0.004)),
            material=material,
            name=f"{hand_kind}_tail",
        )
    else:
        part.visual(
            Cylinder(radius=0.008 if hand_kind == "hour" else 0.0065, length=0.002),
            origin=Origin(
                xyz=(sign * hub_offset, 0.0, 0.0),
                rpy=_axis_cylinder_rpy(axis_name),
            ),
            material=material,
            name=f"{hand_kind}_hub",
        )
        part.visual(
            Box((thickness, blade_width, blade_length)),
            origin=Origin(xyz=(sign * arm_offset, 0.0, blade_length * 0.5 - 0.004)),
            material=material,
            name=f"{hand_kind}_blade",
        )
        part.visual(
            Box((thickness, tail_width, tail_length)),
            origin=Origin(xyz=(sign * arm_offset, 0.0, -tail_length * 0.5 - 0.004)),
            material=material,
            name=f"{hand_kind}_tail",
        )

    part.inertial = Inertial.from_geometry(
        Box((0.016, 0.016, 0.070)),
        mass=0.025 if hand_kind == "hour" else 0.018,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )


def _add_hand_part(
    model: ArticulatedObject,
    pillar,
    *,
    face_name: str,
    axis_name: str,
    sign: float,
    hand_kind: str,
    material,
) -> None:
    part = model.part(f"{face_name}_{hand_kind}_hand")
    _add_hand_geometry(part, axis_name=axis_name, sign=sign, hand_kind=hand_kind, material=material)
    axis = (0.0, 1.0, 0.0) if axis_name == "y" else (1.0, 0.0, 0.0)
    model.articulation(
        f"{face_name}_{hand_kind}_spin",
        ArticulationType.CONTINUOUS,
        parent=pillar,
        child=part,
        origin=Origin(xyz=_face_center(axis_name, sign, HAND_ORIGIN_OFFSET, DIAL_CENTER_Z)),
        axis=axis,
        motion_limits=MotionLimits(effort=0.2, velocity=1.5),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pillar_clock")

    cast_iron = model.material("cast_iron", rgba=(0.20, 0.20, 0.22, 1.0))
    recess_shadow = model.material("recess_shadow", rgba=(0.14, 0.13, 0.12, 1.0))
    dial_cream = model.material("dial_cream", rgba=(0.90, 0.86, 0.77, 1.0))
    bezel_metal = model.material("bezel_metal", rgba=(0.47, 0.39, 0.28, 1.0))
    tick_dark = model.material("tick_dark", rgba=(0.18, 0.15, 0.12, 1.0))
    hand_black = model.material("hand_black", rgba=(0.08, 0.07, 0.06, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.72, 0.80, 0.84, 0.35))
    lantern_roof = model.material("lantern_roof", rgba=(0.24, 0.23, 0.22, 1.0))

    pillar = model.part("pillar")
    pillar.visual(
        Box((BASE_FLANGE, BASE_FLANGE, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=cast_iron,
        name="base_flange",
    )
    pillar.visual(
        Box((0.360, 0.360, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=cast_iron,
        name="base_step",
    )
    pillar.visual(
        Box((BASE_PLINTH, BASE_PLINTH, 0.054)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=cast_iron,
        name="base_plinth",
    )
    pillar.visual(
        Box((0.200, 0.200, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_BOTTOM_Z - 0.014)),
        material=cast_iron,
        name="column_pedestal",
    )
    pillar.visual(
        Box((COLUMN_OUTER, WALL_THICKNESS, COLUMN_HEIGHT)),
        origin=Origin(xyz=(0.0, COLUMN_OUTER * 0.5 - WALL_THICKNESS * 0.5, COLUMN_BOTTOM_Z + COLUMN_HEIGHT * 0.5)),
        material=cast_iron,
        name="front_wall",
    )
    pillar.visual(
        Box((COLUMN_OUTER, WALL_THICKNESS, COLUMN_HEIGHT)),
        origin=Origin(xyz=(0.0, -COLUMN_OUTER * 0.5 + WALL_THICKNESS * 0.5, COLUMN_BOTTOM_Z + COLUMN_HEIGHT * 0.5)),
        material=cast_iron,
        name="back_wall",
    )
    pillar.visual(
        Box((WALL_THICKNESS, COLUMN_OUTER, COLUMN_HEIGHT)),
        origin=Origin(xyz=(COLUMN_OUTER * 0.5 - WALL_THICKNESS * 0.5, 0.0, COLUMN_BOTTOM_Z + COLUMN_HEIGHT * 0.5)),
        material=cast_iron,
        name="right_wall",
    )
    pillar.visual(
        Box((WALL_THICKNESS, COLUMN_OUTER, COLUMN_HEIGHT)),
        origin=Origin(xyz=(-COLUMN_OUTER * 0.5 + WALL_THICKNESS * 0.5, 0.0, COLUMN_BOTTOM_Z + COLUMN_HEIGHT * 0.5)),
        material=cast_iron,
        name="left_wall",
    )
    pillar.visual(
        Box((0.190, 0.190, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_BOTTOM_Z + 0.012)),
        material=cast_iron,
        name="column_lower_tie",
    )
    pillar.visual(
        Box((0.198, 0.198, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_TOP_Z - 0.015)),
        material=cast_iron,
        name="column_upper_tie",
    )

    face_specs = (
        ("front", "y", 1.0),
        ("back", "y", -1.0),
        ("right", "x", 1.0),
        ("left", "x", -1.0),
    )
    for face_name, axis_name, sign in face_specs:
        _add_face_panels(
            pillar,
            face_name=face_name,
            axis_name=axis_name,
            sign=sign,
            iron=cast_iron,
            recess=recess_shadow,
            bezel=bezel_metal,
            dial=dial_cream,
            tick=tick_dark,
        )

    pillar.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, DIAL_CENTER_Z),
            rpy=_axis_cylinder_rpy("y"),
        ),
        material=bezel_metal,
        name="front_back_shaft",
    )
    pillar.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, DIAL_CENTER_Z),
            rpy=_axis_cylinder_rpy("x"),
        ),
        material=bezel_metal,
        name="left_right_shaft",
    )
    pillar.visual(
        Box((0.054, 0.054, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, DIAL_CENTER_Z)),
        material=cast_iron,
        name="movement_core",
    )
    pillar.visual(
        Box((0.204, 0.204, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, 0.753)),
        material=cast_iron,
        name="capital_neck",
    )
    pillar.visual(
        Box((0.280, 0.280, 0.056)),
        origin=Origin(xyz=(0.0, 0.0, 0.805)),
        material=cast_iron,
        name="capital_block",
    )
    pillar.visual(
        Box((0.340, 0.340, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.849)),
        material=cast_iron,
        name="capital_cornice",
    )
    pillar.visual(
        Box((0.210, 0.210, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.875)),
        material=cast_iron,
        name="capital_top",
    )
    for sign in (-1.0, 1.0):
        pillar.visual(
            Box((0.080, 0.028, 0.060)),
            origin=Origin(xyz=(0.0, sign * 0.127, 0.792)),
            material=cast_iron,
            name=f"capital_corbels_y_{int(sign)}",
        )
        pillar.visual(
            Box((0.028, 0.080, 0.060)),
            origin=Origin(xyz=(sign * 0.127, 0.0, 0.792)),
            material=cast_iron,
            name=f"capital_corbels_x_{int(sign)}",
        )
    _add_strut(
        pillar,
        (0.0, 0.120, 0.850),
        (0.0, 0.052, 0.885),
        0.006,
        cast_iron,
        name="lantern_bracket_front",
    )
    _add_strut(
        pillar,
        (0.0, -0.120, 0.850),
        (0.0, -0.052, 0.885),
        0.006,
        cast_iron,
        name="lantern_bracket_back",
    )
    _add_strut(
        pillar,
        (0.120, 0.0, 0.850),
        (0.052, 0.0, 0.885),
        0.006,
        cast_iron,
        name="lantern_bracket_right",
    )
    _add_strut(
        pillar,
        (-0.120, 0.0, 0.850),
        (-0.052, 0.0, 0.885),
        0.006,
        cast_iron,
        name="lantern_bracket_left",
    )
    pillar.inertial = Inertial.from_geometry(
        Box((BASE_FLANGE, BASE_FLANGE, 0.90)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
    )

    lantern = model.part("lantern")
    lantern.visual(
        Box((0.120, 0.120, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=lantern_roof,
        name="lantern_base",
    )
    lantern.visual(
        Box((0.094, 0.094, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=lantern_roof,
        name="lantern_floor",
    )
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            lantern.visual(
                Box((0.010, 0.010, 0.108)),
                origin=Origin(xyz=(x_sign * 0.036, y_sign * 0.036, 0.078)),
                material=lantern_roof,
                name=f"lantern_post_{int(x_sign)}_{int(y_sign)}",
            )
    lantern.visual(
        Box((0.070, 0.004, 0.090)),
        origin=Origin(xyz=(0.0, 0.036, 0.079)),
        material=lantern_glass,
        name="lantern_glass_front",
    )
    lantern.visual(
        Box((0.070, 0.004, 0.090)),
        origin=Origin(xyz=(0.0, -0.036, 0.079)),
        material=lantern_glass,
        name="lantern_glass_back",
    )
    lantern.visual(
        Box((0.004, 0.070, 0.090)),
        origin=Origin(xyz=(0.036, 0.0, 0.079)),
        material=lantern_glass,
        name="lantern_glass_right",
    )
    lantern.visual(
        Box((0.004, 0.070, 0.090)),
        origin=Origin(xyz=(-0.036, 0.0, 0.079)),
        material=lantern_glass,
        name="lantern_glass_left",
    )
    lantern.visual(
        Box((0.140, 0.140, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=lantern_roof,
        name="lantern_eave",
    )
    lantern.visual(
        Box((0.096, 0.096, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.1475)),
        material=lantern_roof,
        name="lantern_roof_block",
    )
    lantern.visual(
        Box((0.060, 0.060, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.171)),
        material=lantern_roof,
        name="lantern_roof_cap",
    )
    lantern.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        material=bezel_metal,
        name="lantern_finial",
    )
    lantern.inertial = Inertial.from_geometry(
        Box((0.140, 0.140, 0.220)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )
    model.articulation(
        "pillar_to_lantern",
        ArticulationType.FIXED,
        parent=pillar,
        child=lantern,
        origin=Origin(xyz=(0.0, 0.0, 0.885)),
    )

    for face_name, axis_name, sign in face_specs:
        _add_hand_part(
            model,
            pillar,
            face_name=face_name,
            axis_name=axis_name,
            sign=sign,
            hand_kind="hour",
            material=hand_black,
        )
        _add_hand_part(
            model,
            pillar,
            face_name=face_name,
            axis_name=axis_name,
            sign=sign,
            hand_kind="minute",
            material=hand_black,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    pillar = object_model.get_part("pillar")
    lantern = object_model.get_part("lantern")

    front_hour = object_model.get_part("front_hour_hand")
    front_minute = object_model.get_part("front_minute_hand")
    back_hour = object_model.get_part("back_hour_hand")
    back_minute = object_model.get_part("back_minute_hand")
    right_hour = object_model.get_part("right_hour_hand")
    right_minute = object_model.get_part("right_minute_hand")
    left_hour = object_model.get_part("left_hour_hand")
    left_minute = object_model.get_part("left_minute_hand")

    front_hour_spin = object_model.get_articulation("front_hour_spin")
    front_minute_spin = object_model.get_articulation("front_minute_spin")
    back_hour_spin = object_model.get_articulation("back_hour_spin")
    back_minute_spin = object_model.get_articulation("back_minute_spin")
    right_hour_spin = object_model.get_articulation("right_hour_spin")
    right_minute_spin = object_model.get_articulation("right_minute_spin")
    left_hour_spin = object_model.get_articulation("left_hour_spin")
    left_minute_spin = object_model.get_articulation("left_minute_spin")

    base_flange = pillar.get_visual("base_flange")
    front_wall = pillar.get_visual("front_wall")
    capital_top = pillar.get_visual("capital_top")
    lantern_base = lantern.get_visual("lantern_base")
    front_back_shaft = pillar.get_visual("front_back_shaft")
    left_right_shaft = pillar.get_visual("left_right_shaft")

    front_panel = pillar.get_visual("front_arch_panel")
    back_panel = pillar.get_visual("back_arch_panel")
    left_panel = pillar.get_visual("left_arch_panel")
    right_panel = pillar.get_visual("right_arch_panel")
    front_dial = pillar.get_visual("front_dial")
    back_dial = pillar.get_visual("back_dial")
    left_dial = pillar.get_visual("left_dial")
    right_dial = pillar.get_visual("right_dial")
    front_boss = pillar.get_visual("front_boss")
    back_boss = pillar.get_visual("back_boss")
    left_boss = pillar.get_visual("left_boss")
    right_boss = pillar.get_visual("right_boss")

    front_hour_hub = front_hour.get_visual("hour_hub")
    front_hour_blade = front_hour.get_visual("hour_blade")
    front_minute_hub = front_minute.get_visual("minute_hub")
    front_minute_blade = front_minute.get_visual("minute_blade")
    back_hour_hub = back_hour.get_visual("hour_hub")
    back_hour_blade = back_hour.get_visual("hour_blade")
    back_minute_hub = back_minute.get_visual("minute_hub")
    back_minute_blade = back_minute.get_visual("minute_blade")
    right_hour_hub = right_hour.get_visual("hour_hub")
    right_hour_blade = right_hour.get_visual("hour_blade")
    right_minute_hub = right_minute.get_visual("minute_hub")
    right_minute_blade = right_minute.get_visual("minute_blade")
    left_hour_hub = left_hour.get_visual("hour_hub")
    left_hour_blade = left_hour.get_visual("hour_blade")
    left_minute_hub = left_minute.get_visual("minute_hub")
    left_minute_blade = left_minute.get_visual("minute_blade")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(pillar, pillar, axes="xy", inner_elem=front_wall, outer_elem=base_flange)
    ctx.expect_within(lantern, pillar, axes="xy", inner_elem=lantern_base, outer_elem=capital_top)
    ctx.expect_contact(lantern, pillar, elem_a=lantern_base, elem_b=capital_top)
    ctx.expect_origin_distance(lantern, pillar, axes="xy", max_dist=0.005)
    ctx.expect_contact(pillar, pillar, elem_a=front_back_shaft, elem_b=front_boss)
    ctx.expect_contact(pillar, pillar, elem_a=front_back_shaft, elem_b=back_boss)
    ctx.expect_contact(pillar, pillar, elem_a=left_right_shaft, elem_b=left_boss)
    ctx.expect_contact(pillar, pillar, elem_a=left_right_shaft, elem_b=right_boss)

    ctx.expect_within(pillar, pillar, axes="xz", inner_elem=front_dial, outer_elem=front_panel)
    ctx.expect_within(pillar, pillar, axes="xz", inner_elem=back_dial, outer_elem=back_panel)
    ctx.expect_within(pillar, pillar, axes="yz", inner_elem=left_dial, outer_elem=left_panel)
    ctx.expect_within(pillar, pillar, axes="yz", inner_elem=right_dial, outer_elem=right_panel)

    ctx.expect_contact(front_hour, pillar, elem_a=front_hour_hub, elem_b=front_boss)
    ctx.expect_contact(front_minute, front_hour, elem_a=front_minute_hub, elem_b=front_hour_hub)
    ctx.expect_within(front_hour, pillar, axes="xz", inner_elem=front_hour_blade, outer_elem=front_dial)
    ctx.expect_within(front_minute, pillar, axes="xz", inner_elem=front_minute_blade, outer_elem=front_dial)

    ctx.expect_contact(back_hour, pillar, elem_a=back_hour_hub, elem_b=back_boss)
    ctx.expect_contact(back_minute, back_hour, elem_a=back_minute_hub, elem_b=back_hour_hub)
    ctx.expect_within(back_hour, pillar, axes="xz", inner_elem=back_hour_blade, outer_elem=back_dial)
    ctx.expect_within(back_minute, pillar, axes="xz", inner_elem=back_minute_blade, outer_elem=back_dial)

    ctx.expect_contact(right_hour, pillar, elem_a=right_hour_hub, elem_b=right_boss)
    ctx.expect_contact(right_minute, right_hour, elem_a=right_minute_hub, elem_b=right_hour_hub)
    ctx.expect_within(right_hour, pillar, axes="yz", inner_elem=right_hour_blade, outer_elem=right_dial)
    ctx.expect_within(right_minute, pillar, axes="yz", inner_elem=right_minute_blade, outer_elem=right_dial)

    ctx.expect_contact(left_hour, pillar, elem_a=left_hour_hub, elem_b=left_boss)
    ctx.expect_contact(left_minute, left_hour, elem_a=left_minute_hub, elem_b=left_hour_hub)
    ctx.expect_within(left_hour, pillar, axes="yz", inner_elem=left_hour_blade, outer_elem=left_dial)
    ctx.expect_within(left_minute, pillar, axes="yz", inner_elem=left_minute_blade, outer_elem=left_dial)

    with ctx.pose({front_minute_spin: math.pi * 0.5, front_hour_spin: math.pi / 6.0}):
        ctx.expect_contact(front_hour, pillar, elem_a=front_hour_hub, elem_b=front_boss)
        ctx.expect_within(front_hour, pillar, axes="xz", inner_elem=front_hour_blade, outer_elem=front_dial)
        ctx.expect_within(front_minute, pillar, axes="xz", inner_elem=front_minute_blade, outer_elem=front_dial)

    with ctx.pose({back_minute_spin: -math.pi * 0.5, back_hour_spin: -math.pi / 6.0}):
        ctx.expect_contact(back_hour, pillar, elem_a=back_hour_hub, elem_b=back_boss)
        ctx.expect_within(back_hour, pillar, axes="xz", inner_elem=back_hour_blade, outer_elem=back_dial)
        ctx.expect_within(back_minute, pillar, axes="xz", inner_elem=back_minute_blade, outer_elem=back_dial)

    with ctx.pose({right_minute_spin: -math.pi * 0.5, right_hour_spin: -math.pi / 3.0}):
        ctx.expect_contact(right_hour, pillar, elem_a=right_hour_hub, elem_b=right_boss)
        ctx.expect_within(right_hour, pillar, axes="yz", inner_elem=right_hour_blade, outer_elem=right_dial)
        ctx.expect_within(right_minute, pillar, axes="yz", inner_elem=right_minute_blade, outer_elem=right_dial)

    with ctx.pose({left_minute_spin: math.pi * 0.5, left_hour_spin: math.pi / 3.0}):
        ctx.expect_contact(left_hour, pillar, elem_a=left_hour_hub, elem_b=left_boss)
        ctx.expect_within(left_hour, pillar, axes="yz", inner_elem=left_hour_blade, outer_elem=left_dial)
        ctx.expect_within(left_minute, pillar, axes="yz", inner_elem=left_minute_blade, outer_elem=left_dial)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
