from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    wire_from_points,
)

ASSETS = AssetContext.from_script(__file__)

BODY_RADIUS = 0.037
BODY_THICKNESS = 0.028
BODY_HALF_THICKNESS = BODY_THICKNESS * 0.5
FACE_PLATE_THICKNESS = 0.005
INNER_CAVITY_THICKNESS = BODY_THICKNESS - 2.0 * FACE_PLATE_THICKNESS
SHACKLE_RADIUS = 0.0042
SHACKLE_SPACING = 0.026
PIVOT_Z = 0.013
SHACKLE_SHOULDER_Z = 0.022
OPEN_ANGLE = math.radians(45.0)


def _arc_points(
    center_x: float,
    center_z: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int,
) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = start_angle + (end_angle - start_angle) * t
        points.append(
            (
                center_x + radius * math.cos(angle),
                0.0,
                center_z + radius * math.sin(angle),
            )
        )
    return points


def _build_shackle_mesh():
    arc = _arc_points(
        SHACKLE_SPACING * 0.5,
        SHACKLE_SHOULDER_Z,
        SHACKLE_SPACING * 0.5,
        math.pi,
        0.0,
        segments=10,
    )
    path = [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, SHACKLE_SHOULDER_Z * 0.55),
        (0.0, 0.0, SHACKLE_SHOULDER_Z),
        *arc[1:-1],
        (SHACKLE_SPACING, 0.0, SHACKLE_SHOULDER_Z),
        (SHACKLE_SPACING, 0.0, 0.012),
        (SHACKLE_SPACING, 0.0, 0.004),
    ]
    return wire_from_points(
        path,
        radius=SHACKLE_RADIUS,
        radial_segments=24,
        cap_ends=True,
        corner_mode="miter",
    )


def _build_face_plate_mesh():
    plate = CylinderGeometry(
        radius=BODY_RADIUS,
        height=FACE_PLATE_THICKNESS,
        radial_segments=72,
    ).rotate_x(math.pi / 2.0)
    opening = BoxGeometry((0.024, FACE_PLATE_THICKNESS + 0.004, 0.022)).translate(0.008, 0.0, 0.027)
    return boolean_difference(plate, opening)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="discus_padlock", assets=ASSETS)

    body_steel = model.material("body_steel", rgba=(0.62, 0.65, 0.69, 1.0))
    shackle_steel = model.material("shackle_steel", rgba=(0.82, 0.84, 0.87, 1.0))
    bezel_steel = model.material("bezel_steel", rgba=(0.48, 0.50, 0.54, 1.0))
    face_plate_mesh = mesh_from_geometry(_build_face_plate_mesh(), ASSETS.mesh_path("discus_face_plate.obj"))

    body = model.part("body")
    body.visual(
        face_plate_mesh,
        origin=Origin(
            xyz=(0.0, BODY_HALF_THICKNESS - FACE_PLATE_THICKNESS * 0.5, 0.0),
        ),
        material=body_steel,
        name="front_shell",
    )
    body.visual(
        face_plate_mesh,
        origin=Origin(
            xyz=(0.0, -BODY_HALF_THICKNESS + FACE_PLATE_THICKNESS * 0.5, 0.0),
        ),
        material=body_steel,
        name="rear_shell",
    )
    body.visual(
        Box((0.050, INNER_CAVITY_THICKNESS, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=body_steel,
        name="lower_core",
    )
    body.visual(
        Box((0.014, INNER_CAVITY_THICKNESS, 0.030)),
        origin=Origin(xyz=(-0.026, 0.0, 0.000)),
        material=body_steel,
        name="left_shield",
    )
    body.visual(
        Box((0.014, INNER_CAVITY_THICKNESS, 0.030)),
        origin=Origin(xyz=(0.026, 0.0, 0.000)),
        material=body_steel,
        name="right_shield",
    )
    body.visual(
        Box((0.010, INNER_CAVITY_THICKNESS, 0.012)),
        origin=Origin(xyz=(-0.028, 0.0, 0.022)),
        material=body_steel,
        name="upper_left_cap",
    )
    body.visual(
        Box((0.0038, INNER_CAVITY_THICKNESS, 0.018)),
        origin=Origin(xyz=(-0.0191, 0.0, 0.020)),
        material=body_steel,
        name="pivot_socket_wall",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.0025),
        origin=Origin(
            xyz=(0.0, BODY_HALF_THICKNESS + 0.00125, -0.018),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=bezel_steel,
        name="core_bezel",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_RADIUS * 2.0, BODY_THICKNESS, BODY_RADIUS * 2.0)),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    shackle = model.part("shackle")
    shackle.visual(
        mesh_from_geometry(_build_shackle_mesh(), ASSETS.mesh_path("discus_shackle.obj")),
        material=shackle_steel,
        name="shackle_form",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((SHACKLE_SPACING + SHACKLE_RADIUS * 2.0, SHACKLE_RADIUS * 2.0, 0.040)),
        mass=0.12,
        origin=Origin(xyz=(SHACKLE_SPACING * 0.5, 0.0, 0.020)),
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(-SHACKLE_SPACING * 0.5, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    body_to_shackle = object_model.get_articulation("body_to_shackle")
    shackle_form = shackle.get_visual("shackle_form")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_overlap(
        shackle,
        body,
        axes="x",
        min_overlap=0.020,
        elem_a=shackle_form,
        name="shackle_tracks_under_shielded_body_width",
    )
    ctx.expect_contact(
        shackle,
        body,
        name="shackle_anchored_in_body_at_rest",
    )
    ctx.expect_within(
        shackle,
        body,
        axes="y",
        margin=0.0,
        inner_elem=shackle_form,
        name="shackle_runs_between_body_faces",
    )

    body_aabb = ctx.part_world_aabb(body)
    shackle_aabb = ctx.part_world_aabb(shackle)
    if body_aabb is None or shackle_aabb is None:
        ctx.fail("world_bounds_available", "Expected measurable world-space bounds for body and shackle.")
        return ctx.report()

    body_size_x = body_aabb[1][0] - body_aabb[0][0]
    body_size_y = body_aabb[1][1] - body_aabb[0][1]
    body_size_z = body_aabb[1][2] - body_aabb[0][2]
    exposed_height = shackle_aabb[1][2] - body_aabb[1][2]

    ctx.check(
        "body_reads_as_round_disc",
        abs(body_size_x - body_size_z) <= 0.004 and body_size_y < body_size_x * 0.5,
        (
            f"Expected a discus-like round body; got x={body_size_x:.4f}, "
            f"y={body_size_y:.4f}, z={body_size_z:.4f}."
        ),
    )
    ctx.check(
        "shackle_only_partly_exposed_at_rest",
        0.006 <= exposed_height <= 0.016,
        f"Expected only a small exposed opening above the body; got exposed height {exposed_height:.4f} m.",
    )

    with ctx.pose({body_to_shackle: OPEN_ANGLE}):
        open_aabb = ctx.part_world_aabb(shackle)
        if open_aabb is None:
            ctx.fail("open_pose_bounds_available", "Expected measurable shackle bounds in the open pose.")
        else:
            open_height = open_aabb[1][2] - shackle_aabb[1][2]
            x_shift = shackle_aabb[1][0] - open_aabb[1][0]
            ctx.check(
                "shackle_swings_upward",
                open_height >= 0.009,
                f"Expected the open shackle to rise noticeably; got additional height {open_height:.4f} m.",
            )
            ctx.check(
                "free_leg_swings_clear_of_latch_side",
                x_shift >= 0.005,
                f"Expected the free leg to swing left out of the latch side; got x shift {x_shift:.4f} m.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
