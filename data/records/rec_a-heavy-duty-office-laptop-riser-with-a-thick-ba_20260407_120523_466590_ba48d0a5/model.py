from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, sin

import cadquery as cq

from sdk import (
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


BASE_LENGTH = 0.320
BASE_WIDTH = 0.240
BASE_THICKNESS = 0.028
BASE_PIVOT_Z = BASE_THICKNESS + 0.018

TRAY_LENGTH = 0.345
TRAY_WIDTH = 0.265
TRAY_DECK_WIDTH = 0.245
TRAY_CENTER_X = 0.0475
TRAY_REAR_PIVOT_X = 0.180
TRAY_DECK_BOTTOM_Z = 0.030
TRAY_DECK_THICKNESS = 0.008

PIVOT_BOSS_RADIUS = 0.016
PIVOT_BOSS_LENGTH = 0.012
ARM_PLATE_THICKNESS = 0.010
ARM_SIDE_Y = 0.149
LINK_PIVOT_BARREL_THICKNESS = 0.024

BASE_FRONT_PIVOT_X = -0.090
BASE_REAR_PIVOT_X = BASE_FRONT_PIVOT_X + TRAY_REAR_PIVOT_X

ARM_REST_ANGLE = 0.78
ARM_CENTER_DISTANCE = 0.172
ARM_DX = ARM_CENTER_DISTANCE * cos(ARM_REST_ANGLE)
ARM_DZ = ARM_CENTER_DISTANCE * sin(ARM_REST_ANGLE)

LOWER_OPEN = 0.26
LOWER_CLOSED = -0.22


def _translated(shape: cq.Workplane, xyz: tuple[float, float, float]) -> cq.Workplane:
    return shape.translate(xyz)


def _boss_y(sign: float) -> float:
    return sign * 0.138


def _extrude_centered_xz(profile: cq.Workplane, thickness: float) -> cq.Workplane:
    return profile.extrude(thickness / 2.0, both=True)


def _extrude_symmetric_xz(profile: cq.Workplane, thickness: float) -> cq.Workplane:
    return profile.extrude(thickness / 2.0, both=True)


def _make_base_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.012)
    )

    top_rib = (
        cq.Workplane("XY")
        .box(0.210, 0.110, 0.006)
        .translate((0.0, 0.0, BASE_THICKNESS - 0.001))
        .edges("|Z")
        .fillet(0.004)
    )
    base = base.union(top_rib)

    mount_height = BASE_PIVOT_Z - 0.006
    mount_length = BASE_REAR_PIVOT_X - BASE_FRONT_PIVOT_X + 0.060
    for sign in (-1.0, 1.0):
        side_mount = (
            cq.Workplane("XY")
            .box(mount_length, 0.020, mount_height)
            .translate((0.0, sign * 0.124, mount_height / 2.0))
            .edges("|Z")
            .fillet(0.004)
        )
        base = base.union(side_mount)

        for pivot_x in (BASE_FRONT_PIVOT_X, BASE_REAR_PIVOT_X):
            boss = _translated(
                _extrude_centered_xz(
                    cq.Workplane("XZ").center(pivot_x, BASE_PIVOT_Z).circle(PIVOT_BOSS_RADIUS),
                    PIVOT_BOSS_LENGTH,
                ),
                (0.0, _boss_y(sign), 0.0),
            )
            web = _translated(
                _extrude_centered_xz(
                    cq.Workplane("XZ")
                    .polyline(
                        [
                            (pivot_x - 0.024, BASE_THICKNESS - 0.002),
                            (pivot_x + 0.024, BASE_THICKNESS - 0.002),
                            (pivot_x + 0.015, BASE_PIVOT_Z + 0.010),
                            (pivot_x - 0.015, BASE_PIVOT_Z + 0.010),
                        ]
                    )
                    .close(),
                    0.016,
                ),
                (0.0, sign * 0.127, 0.0),
            )
            base = base.union(web).union(boss)

    return base


def _make_tray_shape() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(TRAY_LENGTH, TRAY_DECK_WIDTH, TRAY_DECK_THICKNESS)
        .translate(
            (
                TRAY_CENTER_X,
                0.0,
                TRAY_DECK_BOTTOM_Z + TRAY_DECK_THICKNESS / 2.0,
            )
        )
        .edges("|Z")
        .fillet(0.010)
    )

    rear_lip = (
        cq.Workplane("XY")
        .box(0.010, TRAY_DECK_WIDTH, 0.024)
        .translate(
            (
                TRAY_CENTER_X + TRAY_LENGTH / 2.0 - 0.005,
                0.0,
                TRAY_DECK_BOTTOM_Z + 0.012,
            )
        )
        .edges("|Z")
        .fillet(0.003)
    )

    hanger_length = TRAY_REAR_PIVOT_X + 0.070
    hanger_height = TRAY_DECK_BOTTOM_Z + 0.004
    tray = deck.union(rear_lip)

    for sign in (-1.0, 1.0):
        hanger = (
            cq.Workplane("XY")
            .box(hanger_length, 0.020, hanger_height)
            .translate((TRAY_REAR_PIVOT_X / 2.0, sign * 0.124, hanger_height / 2.0))
            .edges("|Z")
            .fillet(0.004)
        )
        tray = tray.union(hanger)

        for pivot_x in (0.0, TRAY_REAR_PIVOT_X):
            boss = _translated(
                _extrude_centered_xz(
                    cq.Workplane("XZ").center(pivot_x, 0.0).circle(PIVOT_BOSS_RADIUS),
                    PIVOT_BOSS_LENGTH,
                ),
                (0.0, _boss_y(sign), 0.0),
            )
            web = _translated(
                _extrude_centered_xz(
                    cq.Workplane("XZ")
                    .polyline(
                        [
                            (pivot_x - 0.026, 0.002),
                            (pivot_x + 0.026, 0.002),
                            (pivot_x + 0.018, TRAY_DECK_BOTTOM_Z + 0.006),
                            (pivot_x - 0.018, TRAY_DECK_BOTTOM_Z + 0.006),
                        ]
                    )
                    .close(),
                    0.016,
                ),
                (0.0, sign * 0.127, 0.0),
            )
            tray = tray.union(web).union(boss)

    return tray


def _make_link_plate(end_x: float, end_z: float, side_sign: float) -> cq.Workplane:
    length = (end_x**2 + end_z**2) ** 0.5
    ux = end_x / length
    uz = end_z / length
    px = -uz
    pz = ux

    outer_points = [
        (px * 0.022, pz * 0.022),
        (ux * (length * 0.28) + px * 0.054, uz * (length * 0.28) + pz * 0.054),
        (ux * (length * 0.76) + px * 0.037, uz * (length * 0.76) + pz * 0.037),
        (end_x + px * 0.022, end_z + pz * 0.022),
        (end_x - px * 0.022, end_z - pz * 0.022),
        (ux * (length * 0.18) - px * 0.038, uz * (length * 0.18) - pz * 0.038),
        (-px * 0.022, -pz * 0.022),
    ]
    outer_web = _extrude_symmetric_xz(cq.Workplane("XZ").polyline(outer_points).close(), ARM_PLATE_THICKNESS)
    start_lobe = _extrude_symmetric_xz(
        cq.Workplane("XZ").center(0.0, 0.0).circle(PIVOT_BOSS_RADIUS + 0.013),
        ARM_PLATE_THICKNESS,
    )
    end_lobe = _extrude_symmetric_xz(
        cq.Workplane("XZ").center(end_x, end_z).circle(PIVOT_BOSS_RADIUS + 0.010),
        ARM_PLATE_THICKNESS,
    )
    plate = outer_web.union(start_lobe).union(end_lobe)

    lightening_points = [
        (ux * (length * 0.28) + px * 0.012, uz * (length * 0.28) + pz * 0.012),
        (ux * (length * 0.57) + px * 0.013, uz * (length * 0.57) + pz * 0.013),
        (ux * (length * 0.50) - px * 0.012, uz * (length * 0.50) - pz * 0.012),
        (ux * (length * 0.24) - px * 0.011, uz * (length * 0.24) - pz * 0.011),
    ]
    lightening_cut = _extrude_symmetric_xz(
        cq.Workplane("XZ").polyline(lightening_points).close(),
        ARM_PLATE_THICKNESS + 0.004,
    )
    start_hole = _extrude_symmetric_xz(
        cq.Workplane("XZ").center(0.0, 0.0).circle(PIVOT_BOSS_RADIUS + 0.0055),
        ARM_PLATE_THICKNESS + 0.004,
    )
    end_hole = _extrude_symmetric_xz(
        cq.Workplane("XZ").center(end_x, end_z).circle(PIVOT_BOSS_RADIUS + 0.0055),
        ARM_PLATE_THICKNESS + 0.004,
    )
    collar_length = 0.004
    collar_offset = -side_sign * (ARM_PLATE_THICKNESS / 2.0 - collar_length / 2.0)
    start_collar_outer = _extrude_symmetric_xz(
        cq.Workplane("XZ").center(0.0, 0.0).circle(PIVOT_BOSS_RADIUS + 0.008),
        collar_length,
    ).translate((0.0, collar_offset, 0.0))
    start_collar_inner = _extrude_symmetric_xz(
        cq.Workplane("XZ").center(0.0, 0.0).circle(PIVOT_BOSS_RADIUS),
        collar_length + 0.001,
    ).translate((0.0, collar_offset, 0.0))
    end_collar_outer = _extrude_symmetric_xz(
        cq.Workplane("XZ").center(end_x, end_z).circle(PIVOT_BOSS_RADIUS + 0.008),
        collar_length,
    ).translate((0.0, collar_offset, 0.0))
    end_collar_inner = _extrude_symmetric_xz(
        cq.Workplane("XZ").center(end_x, end_z).circle(PIVOT_BOSS_RADIUS),
        collar_length + 0.001,
    ).translate((0.0, collar_offset, 0.0))
    collars = start_collar_outer.cut(start_collar_inner).union(end_collar_outer.cut(end_collar_inner))
    return plate.cut(lightening_cut).cut(start_hole).cut(end_hole).union(collars)


def _make_link_frame(end_x: float, end_z: float) -> cq.Workplane:
    length = (end_x**2 + end_z**2) ** 0.5
    ux = end_x / length
    uz = end_z / length
    px = -uz
    pz = ux

    left_plate = _make_link_plate(end_x, end_z, 1.0).translate((0.0, ARM_SIDE_Y, 0.0))
    right_plate = _make_link_plate(end_x, end_z, -1.0).translate((0.0, -ARM_SIDE_Y, 0.0))

    spacer_length = 2.0 * ARM_SIDE_Y + ARM_PLATE_THICKNESS
    upper_spacer = _extrude_symmetric_xz(
        cq.Workplane("XZ")
        .center(ux * (length * 0.34) + px * 0.029, uz * (length * 0.34) + pz * 0.029)
        .circle(0.007),
        spacer_length,
    )
    lower_spacer = _extrude_symmetric_xz(
        cq.Workplane("XZ")
        .center(ux * (length * 0.67) - px * 0.022, uz * (length * 0.67) - pz * 0.022)
        .circle(0.007),
        spacer_length,
    )

    return left_plate.union(right_plate).union(upper_spacer).union(lower_spacer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_laptop_riser")

    model.material("powder_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("tray_silver", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("graphite_link", rgba=(0.34, 0.36, 0.39, 1.0))
    model.material("pad_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shape(), "base_slab"), material="powder_black", name="base_shell")
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS + 0.030)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    lower_frame = model.part("lower_link_frame")
    lower_frame.visual(
        mesh_from_cadquery(_make_link_frame(ARM_DX, ARM_DZ), "lower_link_frame"),
        material="graphite_link",
        name="lower_frame_shell",
    )
    lower_frame.inertial = Inertial.from_geometry(
        Box((ARM_DX + 0.040, 2.0 * ARM_SIDE_Y + 0.020, ARM_DZ + 0.050)),
        mass=1.1,
        origin=Origin(xyz=(ARM_DX * 0.50, 0.0, ARM_DZ * 0.45)),
    )

    tray = model.part("tray")
    tray.visual(mesh_from_cadquery(_make_tray_shape(), "tray_platform"), material="tray_silver", name="tray_shell")
    tray.inertial = Inertial.from_geometry(
        Box((TRAY_LENGTH, TRAY_WIDTH, 0.040)),
        mass=1.5,
        origin=Origin(xyz=(TRAY_CENTER_X, 0.0, 0.020)),
    )

    pad_size = (0.042, 0.030, 0.0035)
    pad_z = TRAY_DECK_BOTTOM_Z + TRAY_DECK_THICKNESS + pad_size[2] / 2.0 - 0.0005
    pad_x_front = TRAY_CENTER_X - 0.105
    pad_x_rear = TRAY_CENTER_X + 0.095
    pad_y = 0.082
    for name, x_pos, y_pos in (
        ("front_left_pad", pad_x_front, pad_y),
        ("front_right_pad", pad_x_front, -pad_y),
        ("rear_left_pad", pad_x_rear, pad_y),
        ("rear_right_pad", pad_x_rear, -pad_y),
    ):
        tray.visual(
            Box(pad_size),
            origin=Origin(xyz=(x_pos, y_pos, pad_z)),
            material="pad_rubber",
            name=name,
        )

    upper_frame = model.part("upper_link_frame")
    upper_frame.visual(
        mesh_from_cadquery(_make_link_frame(-ARM_DX, -ARM_DZ), "upper_link_frame"),
        material="graphite_link",
        name="upper_frame_shell",
    )
    upper_frame.inertial = Inertial.from_geometry(
        Box((ARM_DX + 0.040, 2.0 * ARM_SIDE_Y + 0.020, ARM_DZ + 0.050)),
        mass=1.0,
        origin=Origin(xyz=(-ARM_DX * 0.50, 0.0, -ARM_DZ * 0.45)),
    )

    model.articulation(
        "base_to_lower_frame",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_frame,
        origin=Origin(xyz=(BASE_FRONT_PIVOT_X, 0.0, BASE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=LOWER_CLOSED,
            upper=LOWER_OPEN,
            effort=60.0,
            velocity=1.3,
        ),
    )
    model.articulation(
        "lower_frame_to_tray",
        ArticulationType.REVOLUTE,
        parent=lower_frame,
        child=tray,
        origin=Origin(xyz=(ARM_DX, 0.0, ARM_DZ)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-LOWER_OPEN,
            upper=-LOWER_CLOSED,
            effort=40.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "tray_to_upper_frame",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=upper_frame,
        origin=Origin(xyz=(TRAY_REAR_PIVOT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=LOWER_CLOSED,
            upper=LOWER_OPEN,
            effort=40.0,
            velocity=1.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    lower_frame = object_model.get_part("lower_link_frame")
    tray = object_model.get_part("tray")
    upper_frame = object_model.get_part("upper_link_frame")
    lower_joint = object_model.get_articulation("base_to_lower_frame")
    tray_joint = object_model.get_articulation("lower_frame_to_tray")
    upper_joint = object_model.get_articulation("tray_to_upper_frame")

    ctx.allow_overlap(
        base,
        lower_frame,
        reason="The lower side-link plates are modeled as captured around the exposed base hinge bosses with a simplified zero-clearance clevis fit.",
    )
    ctx.allow_overlap(
        lower_frame,
        tray,
        reason="The front tray pivots are represented as tightly nested hinge bosses inside the lower link frame's end lobes.",
    )
    ctx.allow_overlap(
        tray,
        upper_frame,
        reason="The rear tray pivots are represented as tightly nested hinge bosses inside the upper link frame's end lobes.",
    )
    ctx.allow_overlap(
        base,
        upper_frame,
        reason="The rear base pivots visually close the parallel-lift loop with an intentionally simplified captured hinge fit at the upper links.",
    )

    def _elem_center_z(elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(tray, elem=elem_name)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    with ctx.pose({lower_joint: LOWER_CLOSED, tray_joint: -LOWER_CLOSED, upper_joint: LOWER_CLOSED}):
        low_pos = ctx.part_world_position(tray)
        front_z = _elem_center_z("front_left_pad")
        rear_z = _elem_center_z("rear_left_pad")
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.045,
            name="tray stays above the base in the lowered pose",
        )
        ctx.check(
            "tray stays near level in the lowered pose",
            front_z is not None and rear_z is not None and abs(front_z - rear_z) <= 0.006,
            details=f"front_z={front_z}, rear_z={rear_z}",
        )

    with ctx.pose({lower_joint: LOWER_OPEN, tray_joint: -LOWER_OPEN, upper_joint: LOWER_OPEN}):
        high_pos = ctx.part_world_position(tray)
        front_z = _elem_center_z("front_left_pad")
        rear_z = _elem_center_z("rear_left_pad")
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.085,
            name="tray clears the base when raised",
        )
        ctx.check(
            "tray stays near level in the raised pose",
            front_z is not None and rear_z is not None and abs(front_z - rear_z) <= 0.006,
            details=f"front_z={front_z}, rear_z={rear_z}",
        )
        ctx.check(
            "parallel-link motion raises the tray",
            low_pos is not None and high_pos is not None and high_pos[2] > low_pos[2] + 0.040,
            details=f"lowered={low_pos}, raised={high_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
