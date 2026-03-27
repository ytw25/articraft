from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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

POLAR_TILT = math.radians(34.0)
RA_ORIGIN_Z = 1.06


def _segment_origin(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.hypot(dx, dy), dz)
    return (
        Origin(
            xyz=(
                (start[0] + end[0]) * 0.5,
                (start[1] + end[1]) * 0.5,
                (start[2] + end[2]) * 0.5,
            ),
            rpy=(0.0, pitch, yaw),
        ),
        length,
    )


def _polar_offset(distance: float) -> tuple[float, float, float]:
    return (
        -math.sin(POLAR_TILT) * distance,
        0.0,
        math.cos(POLAR_TILT) * distance,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_tube_eq_mount")

    tube_white = model.material("tube_white", rgba=(0.92, 0.93, 0.95, 1.0))
    mount_gray = model.material("mount_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.11, 0.12, 0.14, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))
    leg_gray = model.material("leg_gray", rgba=(0.35, 0.37, 0.40, 1.0))

    tripod = model.part("tripod_base")
    tripod.visual(
        Cylinder(radius=0.046, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 0.53)),
        material=mount_gray,
        name="center_pier",
    )
    tripod.visual(
        Cylinder(radius=0.075, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=mount_gray,
        name="tripod_head",
    )
    tripod.visual(
        Box((0.10, 0.10, 0.15)),
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
        material=mount_gray,
        name="polar_support_block",
    )

    head_attach_radius = 0.032
    foot_radius = 0.34
    leg_top_z = 0.79
    foot_z = 0.04
    spreader_center = (0.0, 0.0, 0.44)
    for index, angle_deg in enumerate((0.0, 120.0, 240.0)):
        angle = math.radians(angle_deg)
        leg_top = (
            head_attach_radius * math.cos(angle),
            head_attach_radius * math.sin(angle),
            leg_top_z,
        )
        foot = (
            foot_radius * math.cos(angle),
            foot_radius * math.sin(angle),
            foot_z,
        )
        leg_origin, leg_length = _segment_origin(leg_top, foot)
        tripod.visual(
            Cylinder(radius=0.014, length=leg_length),
            origin=leg_origin,
            material=leg_gray,
            name=f"leg_{index + 1}",
        )

        spreader_end = (
            0.19 * math.cos(angle),
            0.19 * math.sin(angle),
            0.36,
        )
        spreader_origin, spreader_length = _segment_origin(spreader_center, spreader_end)
        tripod.visual(
            Cylinder(radius=0.008, length=spreader_length),
            origin=spreader_origin,
            material=steel,
            name=f"spreader_{index + 1}",
        )

    bearing_offset = _polar_offset(-0.03)
    tripod.visual(
        Cylinder(radius=0.056, length=0.06),
        origin=Origin(
            xyz=(bearing_offset[0], bearing_offset[1], RA_ORIGIN_Z + bearing_offset[2]),
            rpy=(0.0, -POLAR_TILT, 0.0),
        ),
        material=mount_gray,
        name="polar_bearing",
    )
    polar_spine_origin, polar_spine_length = _segment_origin(
        (0.0, 0.0, 0.935),
        (
            bearing_offset[0] * 0.7,
            0.0,
            (RA_ORIGIN_Z + bearing_offset[2]) - 0.025,
        ),
    )
    tripod.visual(
        Cylinder(radius=0.022, length=polar_spine_length),
        origin=polar_spine_origin,
        material=mount_gray,
        name="polar_spine",
    )
    tripod.inertial = Inertial.from_geometry(
        Box((0.76, 0.76, 1.02)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )

    ra_head = model.part("ra_head")
    ra_head.visual(
        Cylinder(radius=0.048, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=mount_gray,
        name="ra_housing",
    )
    ra_head.visual(
        Box((0.085, 0.09, 0.06)),
        origin=Origin(xyz=(0.0375, 0.0, 0.165)),
        material=mount_gray,
        name="axis_shoulder",
    )
    ra_head.visual(
        Cylinder(radius=0.018, length=0.036),
        origin=Origin(xyz=(0.0, 0.059, 0.10), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="ra_clutch_boss",
    )
    ra_head.visual(
        Cylinder(radius=0.022, length=0.084),
        origin=Origin(xyz=(0.08, 0.0, 0.175), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mount_gray,
        name="dec_axis_trunnion",
    )
    ra_head.inertial = Inertial.from_geometry(
        Box((0.14, 0.14, 0.24)),
        mass=2.0,
        origin=Origin(xyz=(0.01, 0.0, 0.12)),
    )

    ota = model.part("ota_dec_assembly")
    ota.visual(
        Cylinder(radius=0.03, length=0.106),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mount_gray,
        name="dec_housing",
    )
    ota.visual(
        Cylinder(radius=0.014, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=dark_trim,
        name="dec_clutch_boss",
    )
    ota.visual(
        Box((0.13, 0.06, 0.08)),
        origin=Origin(xyz=(0.065, 0.0, 0.04)),
        material=mount_gray,
        name="tube_support",
    )
    ota.visual(
        Cylinder(radius=0.068, length=0.42),
        origin=Origin(xyz=(0.255, 0.0, 0.088), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_white,
        name="tube_shell",
    )
    ota.visual(
        Cylinder(radius=0.080, length=0.08),
        origin=Origin(xyz=(0.505, 0.0, 0.088), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_trim,
        name="dew_shield_ring",
    )
    ota.visual(
        Cylinder(radius=0.032, length=0.05),
        origin=Origin(xyz=(0.02, 0.0, 0.088), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_gray,
        name="focuser_body",
    )
    ota.visual(
        Cylinder(radius=0.022, length=0.05),
        origin=Origin(xyz=(-0.03, 0.0, 0.088), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="focuser_barrel",
    )
    ota.inertial = Inertial.from_geometry(
        Box((0.70, 0.18, 0.24)),
        mass=2.6,
        origin=Origin(xyz=(0.23, 0.0, 0.06)),
    )

    model.articulation(
        "right_ascension",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=ra_head,
        origin=Origin(xyz=(0.0, 0.0, RA_ORIGIN_Z), rpy=(0.0, -POLAR_TILT, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5),
    )
    model.articulation(
        "declination",
        ArticulationType.REVOLUTE,
        parent=ra_head,
        child=ota,
        origin=Origin(xyz=(0.08, 0.0, 0.175)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-0.7,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod_base")
    ra_head = object_model.get_part("ra_head")
    ota = object_model.get_part("ota_dec_assembly")
    ra_axis = object_model.get_articulation("right_ascension")
    dec_axis = object_model.get_articulation("declination")

    polar_bearing = tripod.get_visual("polar_bearing")
    tripod_head = tripod.get_visual("tripod_head")
    ra_housing = ra_head.get_visual("ra_housing")
    axis_shoulder = ra_head.get_visual("axis_shoulder")
    dec_axis_trunnion = ra_head.get_visual("dec_axis_trunnion")
    ra_clutch_boss = ra_head.get_visual("ra_clutch_boss")
    dec_housing = ota.get_visual("dec_housing")
    dec_clutch_boss = ota.get_visual("dec_clutch_boss")
    tube_support = ota.get_visual("tube_support")
    tube_shell = ota.get_visual("tube_shell")
    dew_shield_ring = ota.get_visual("dew_shield_ring")
    focuser_body = ota.get_visual("focuser_body")
    focuser_barrel = ota.get_visual("focuser_barrel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        ota,
        ra_head,
        reason="declination sleeve nests around the right ascension axis trunnion",
    )

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )
    ctx.expect_contact(ra_head, tripod, elem_a=ra_housing, elem_b=polar_bearing)
    ctx.expect_within(
        ra_head,
        ota,
        axes="yz",
        inner_elem=dec_axis_trunnion,
        outer_elem=dec_housing,
    )
    ctx.expect_overlap(
        ota,
        ra_head,
        axes="y",
        elem_a=dec_housing,
        elem_b=dec_axis_trunnion,
        min_overlap=0.08,
    )
    ctx.expect_overlap(
        ota,
        ota,
        axes="yz",
        elem_a=tube_support,
        elem_b=tube_shell,
        min_overlap=0.05,
    )
    ctx.expect_contact(ra_head, ra_head, elem_a=ra_clutch_boss, elem_b=ra_housing)
    ctx.expect_gap(
        ota,
        ota,
        axis="z",
        max_gap=0.001,
        max_penetration=0.02,
        positive_elem=dec_clutch_boss,
        negative_elem=dec_housing,
        name="dec_clutch_boss_seats_on_housing",
    )

    ctx.expect_overlap(
        ota,
        ota,
        axes="yz",
        elem_a=tube_shell,
        elem_b=dew_shield_ring,
        min_overlap=0.012,
    )
    ctx.expect_gap(
        ota,
        ota,
        axis="x",
        max_gap=0.005,
        max_penetration=0.12,
        positive_elem=dew_shield_ring,
        negative_elem=tube_shell,
        name="dew_shield_meets_objective_end",
    )
    ctx.expect_overlap(
        ota,
        ota,
        axes="yz",
        elem_a=tube_shell,
        elem_b=focuser_barrel,
        min_overlap=0.012,
    )
    ctx.expect_gap(
        ota,
        ota,
        axis="x",
        max_gap=0.005,
        max_penetration=0.07,
        positive_elem=tube_shell,
        negative_elem=focuser_body,
        name="focuser_seats_at_eyepiece_end",
    )
    ctx.expect_gap(
        ota,
        ota,
        axis="x",
        min_gap=0.26,
        positive_elem=dew_shield_ring,
        negative_elem=focuser_barrel,
        name="tube_has_clear_front_and_rear_aperture_spacing",
    )
    ctx.expect_gap(
        ota,
        tripod,
        axis="z",
        min_gap=0.03,
        positive_elem=tube_shell,
        negative_elem=tripod_head,
        name="tube_clears_tripod_head",
    )

    with ctx.pose({ra_axis: 1.2, dec_axis: 0.8}):
        ctx.expect_gap(
            ota,
            tripod,
            axis="z",
            min_gap=0.015,
            positive_elem=focuser_barrel,
            negative_elem=tripod_head,
            name="rear_accessories_clear_tripod_when_tracking",
        )
    with ctx.pose({ra_axis: -0.9, dec_axis: -0.75}):
        ctx.expect_gap(
            ota,
            tripod,
            axis="z",
            min_gap=0.015,
            positive_elem=tube_shell,
            negative_elem=tripod_head,
            name="tube_clears_tripod_in_low_declination_pose",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
