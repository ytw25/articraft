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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
)

ASSETS = AssetContext.from_script(__file__)

BORE = 0.070
BORE_RADIUS = BORE / 2.0
STROKE = 0.080
CRANK_RADIUS = STROKE / 2.0
CYLINDER_SPACING = 0.180

CONROD_CENTER_DISTANCE = 0.120
MAX_ROD_ANGLE = math.asin(CRANK_RADIUS / CONROD_CENTER_DISTANCE) + 0.005

BLOCK_WIDTH = CYLINDER_SPACING + BORE + 0.060
BLOCK_DEPTH = 0.180
BLOCK_BOTTOM_Z = 0.020
BLOCK_TOP_Z = 0.200
SLEEVE_BOTTOM_Z = 0.048
UPPER_JACKET_BOTTOM_Z = 0.116

CRANKSHAFT_LENGTH = CYLINDER_SPACING + 0.140
MAIN_SHAFT_RADIUS = 0.010
MAIN_JOURNAL_RADIUS = 0.014
CRANKPIN_RADIUS = 0.0146
CRANKPIN_LENGTH = 0.024

PISTON_HEIGHT = 0.060
PISTON_BODY_RADIUS = 0.0330
RING_RADIUS = BORE_RADIUS
PISTON_PIN_RADIUS = 0.0070
PISTON_PIN_HOLE_RADIUS = 0.0078
PISTON_PIN_LENGTH = 0.046
PISTON_A_TDC_Z = 0.160
PISTON_B_BDC_Z = PISTON_A_TDC_Z - STROKE

ROD_THICKNESS = 0.018
ROD_SMALL_END_INNER_RADIUS = 0.0079
ROD_BIG_END_INNER_RADIUS = 0.0142
COUNTERWEIGHT_CLEARANCE_MIN = 0.010


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 32,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * i) / segments),
            cy + radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _slider_pin_z(theta: float, *, phase: float = 0.0) -> float:
    phase_theta = theta + phase
    crank_y = CRANK_RADIUS * math.sin(phase_theta)
    crank_z = CRANK_RADIUS * math.cos(phase_theta)
    rod_z = math.sqrt((CONROD_CENTER_DISTANCE**2) - (crank_y**2))
    return crank_z + rod_z


def _rod_angle(theta: float, *, phase: float = 0.0) -> float:
    phase_theta = theta + phase
    return math.asin(
        (CRANK_RADIUS * math.sin(phase_theta)) / CONROD_CENTER_DISTANCE
    )


def _pose_for_theta(theta: float) -> dict[str, float]:
    return {
        "crankshaft_spin": theta,
        "piston_a_slide": _slider_pin_z(theta, phase=0.0) - PISTON_A_TDC_Z,
        "piston_b_slide": _slider_pin_z(theta, phase=math.pi) - PISTON_B_BDC_Z,
        "rod_a_pivot": _rod_angle(theta, phase=0.0),
        "rod_b_pivot": _rod_angle(theta, phase=math.pi),
    }


def _build_block_mesh():
    outer_profile = rounded_rect_profile(
        BLOCK_WIDTH,
        BLOCK_DEPTH,
        radius=0.020,
        corner_segments=8,
    )
    bore_profiles = [
        _circle_profile(-CYLINDER_SPACING / 2.0, 0.0, BORE_RADIUS, segments=40),
        _circle_profile(CYLINDER_SPACING / 2.0, 0.0, BORE_RADIUS, segments=40),
    ]
    geometry = ExtrudeWithHolesGeometry(
        outer_profile,
        bore_profiles,
        BLOCK_TOP_Z - BLOCK_BOTTOM_Z,
        cap=True,
        center=False,
        closed=True,
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / "parallel_twin_block.obj")


def _build_sleeve_mesh():
    sleeve_geometry = ExtrudeWithHolesGeometry(
        _circle_profile(0.0, 0.0, BORE_RADIUS + 0.0095, segments=40),
        [_circle_profile(0.0, 0.0, BORE_RADIUS + 0.0010, segments=40)],
        BLOCK_TOP_Z - SLEEVE_BOTTOM_Z,
        cap=True,
        center=False,
        closed=True,
    )
    return mesh_from_geometry(sleeve_geometry, ASSETS.mesh_dir / "parallel_twin_sleeve.obj")


def _build_piston_mesh():
    half_height = PISTON_HEIGHT / 2.0
    profile = [
        (0.0, -half_height),
        (PISTON_BODY_RADIUS * 0.72, -half_height),
        (PISTON_BODY_RADIUS, -half_height + 0.0016),
        (PISTON_BODY_RADIUS, -0.0040),
        (PISTON_BODY_RADIUS - 0.0014, 0.0064),
        (PISTON_BODY_RADIUS - 0.0014, 0.0100),
        (PISTON_BODY_RADIUS, 0.0118),
        (PISTON_BODY_RADIUS, 0.0142),
        (PISTON_BODY_RADIUS - 0.0016, 0.0160),
        (PISTON_BODY_RADIUS - 0.0016, 0.0196),
        (PISTON_BODY_RADIUS, 0.0214),
        (PISTON_BODY_RADIUS, half_height - 0.0020),
        (PISTON_BODY_RADIUS - 0.0016, half_height),
        (0.0, half_height),
    ]
    body = LatheGeometry(profile, segments=56)
    return mesh_from_geometry(body, ASSETS.mesh_dir / "parallel_twin_piston.obj")


def _build_connecting_rod_mesh():
    outline = sample_catmull_rom_spline_2d(
        [
            (0.0, 0.0155),
            (0.0105, 0.0120),
            (0.0160, 0.0020),
            (0.0130, -0.030),
            (0.0105, -0.070),
            (0.0120, -0.102),
            (0.0170, -0.116),
            (0.0240, -CONROD_CENTER_DISTANCE + 0.004),
            (0.0280, -CONROD_CENTER_DISTANCE),
            (0.0240, -CONROD_CENTER_DISTANCE - 0.018),
            (0.0, -CONROD_CENTER_DISTANCE - 0.026),
            (-0.0240, -CONROD_CENTER_DISTANCE - 0.018),
            (-0.0280, -CONROD_CENTER_DISTANCE),
            (-0.0240, -CONROD_CENTER_DISTANCE + 0.004),
            (-0.0170, -0.116),
            (-0.0120, -0.102),
            (-0.0105, -0.070),
            (-0.0130, -0.030),
            (-0.0160, 0.0020),
            (-0.0105, 0.0120),
        ],
        samples_per_segment=8,
        closed=True,
    )
    geometry = ExtrudeWithHolesGeometry(
        outline,
        [
            _circle_profile(0.0, 0.0, ROD_SMALL_END_INNER_RADIUS, segments=28),
            _circle_profile(
                0.0,
                -CONROD_CENTER_DISTANCE,
                ROD_BIG_END_INNER_RADIUS,
                segments=32,
            ),
        ],
        ROD_THICKNESS,
        cap=True,
        center=True,
        closed=True,
    )
    geometry.rotate_x(math.pi / 2.0).rotate_z(math.pi / 2.0)
    return mesh_from_geometry(geometry, ASSETS.mesh_dir / "parallel_twin_rod.obj")


def _assert_close(actual: float, expected: float, tol: float, label: str) -> None:
    if abs(actual - expected) > tol:
        raise AssertionError(
            f"{label}: expected {expected:.5f}, got {actual:.5f} (tol {tol:.5f})"
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parallel_twin_engine_core", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.23, 0.24, 0.25, 1.0))
    forged_steel = model.material("forged_steel", rgba=(0.34, 0.35, 0.37, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    piston_aluminum = model.material(
        "piston_aluminum",
        rgba=(0.80, 0.81, 0.82, 1.0),
    )
    ring_steel = model.material("ring_steel", rgba=(0.60, 0.62, 0.64, 1.0))

    sleeve_mesh = _build_sleeve_mesh()
    piston_mesh = _build_piston_mesh()
    rod_mesh = _build_connecting_rod_mesh()

    cylinder_block = model.part("cylinder_block")
    cylinder_block.visual(
        sleeve_mesh,
        origin=Origin(xyz=(-CYLINDER_SPACING / 2.0, 0.0, SLEEVE_BOTTOM_Z)),
        material=cast_iron,
        name="sleeve_a",
    )
    cylinder_block.visual(
        sleeve_mesh,
        origin=Origin(xyz=(CYLINDER_SPACING / 2.0, 0.0, SLEEVE_BOTTOM_Z)),
        material=cast_iron,
        name="sleeve_b",
    )
    cylinder_block.visual(
        Box((BLOCK_WIDTH * 0.82, 0.018, BLOCK_TOP_Z - UPPER_JACKET_BOTTOM_Z)),
        origin=Origin(
            xyz=(0.0, 0.050, (BLOCK_TOP_Z + UPPER_JACKET_BOTTOM_Z) / 2.0),
        ),
        material=cast_iron,
        name="front_water_jacket",
    )
    cylinder_block.visual(
        Box((BLOCK_WIDTH * 0.82, 0.018, BLOCK_TOP_Z - UPPER_JACKET_BOTTOM_Z)),
        origin=Origin(
            xyz=(0.0, -0.050, (BLOCK_TOP_Z + UPPER_JACKET_BOTTOM_Z) / 2.0),
        ),
        material=cast_iron,
        name="rear_water_jacket",
    )
    cylinder_block.visual(
        Box((0.100, 0.052, BLOCK_TOP_Z - UPPER_JACKET_BOTTOM_Z)),
        origin=Origin(xyz=(0.0, 0.0, (BLOCK_TOP_Z + UPPER_JACKET_BOTTOM_Z) / 2.0)),
        material=cast_iron,
        name="center_bridge",
    )
    cylinder_block.visual(
        Box((BLOCK_WIDTH * 0.92, 0.018, 0.042)),
        origin=Origin(xyz=(0.0, 0.074, 0.009)),
        material=cast_iron,
        name="front_girdle",
    )
    cylinder_block.visual(
        Box((BLOCK_WIDTH * 0.92, 0.018, 0.042)),
        origin=Origin(xyz=(0.0, -0.074, 0.009)),
        material=cast_iron,
        name="rear_girdle",
    )
    for idx, x_pos in enumerate((-0.130, 0.0, 0.130), start=1):
        cylinder_block.visual(
            Box((0.030, 0.0314, 0.022)),
            origin=Origin(xyz=(x_pos, 0.0305, 0.011)),
            material=cast_iron,
            name=f"front_journal_cheek_{idx}",
        )
        cylinder_block.visual(
            Box((0.030, 0.0314, 0.022)),
            origin=Origin(xyz=(x_pos, -0.0305, 0.011)),
            material=cast_iron,
            name=f"rear_journal_cheek_{idx}",
        )
        cylinder_block.visual(
            Box((0.024, 0.028, 0.094)),
            origin=Origin(xyz=(x_pos, 0.060, 0.035)),
            material=cast_iron,
            name=f"front_bearing_post_{idx}",
        )
        cylinder_block.visual(
            Box((0.024, 0.028, 0.094)),
            origin=Origin(xyz=(x_pos, -0.060, 0.035)),
            material=cast_iron,
            name=f"rear_bearing_post_{idx}",
        )
    cylinder_block.inertial = Inertial.from_geometry(
        Box((BLOCK_WIDTH, BLOCK_DEPTH, BLOCK_TOP_Z - BLOCK_BOTTOM_Z)),
        mass=13.5,
        origin=Origin(xyz=(0.0, 0.0, (BLOCK_TOP_Z + BLOCK_BOTTOM_Z) / 2.0)),
    )

    crankshaft = model.part("crankshaft")
    crankshaft.visual(
        Cylinder(radius=MAIN_SHAFT_RADIUS, length=CRANKSHAFT_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=forged_steel,
        name="main_shaft",
    )
    for name, x_pos in (
        ("journal_left", -0.130),
        ("journal_center", 0.0),
        ("journal_right", 0.130),
    ):
        crankshaft.visual(
            Cylinder(radius=MAIN_JOURNAL_RADIUS, length=0.028),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_steel,
            name=name,
        )

    crankshaft.visual(
        Cylinder(radius=CRANKPIN_RADIUS, length=CRANKPIN_LENGTH),
        origin=Origin(
            xyz=(-CYLINDER_SPACING / 2.0, 0.0, CRANK_RADIUS),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=polished_steel,
        name="crankpin_a",
    )
    crankshaft.visual(
        Cylinder(radius=CRANKPIN_RADIUS, length=CRANKPIN_LENGTH),
        origin=Origin(
            xyz=(CYLINDER_SPACING / 2.0, 0.0, -CRANK_RADIUS),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=polished_steel,
        name="crankpin_b",
    )

    for side, x_pos, web_z, counterweight_z in (
        ("a_outboard", -0.105, 0.014, -0.024),
        ("a_inboard", -0.075, 0.014, -0.024),
        ("b_inboard", 0.075, -0.014, 0.024),
        ("b_outboard", 0.105, -0.014, 0.024),
    ):
        crankshaft.visual(
            Box((0.014, 0.018, 0.060)),
            origin=Origin(xyz=(x_pos, 0.0, web_z)),
            material=forged_steel,
            name=f"web_{side}",
        )
        crankshaft.visual(
            Box((0.014, 0.040, 0.024)),
            origin=Origin(xyz=(x_pos, 0.0, counterweight_z)),
            material=forged_steel,
            name=f"counterweight_{side}",
        )
    crankshaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.044, length=CRANKSHAFT_LENGTH),
        mass=6.4,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    for piston_name in ("piston_a", "piston_b"):
        piston = model.part(piston_name)
        piston.visual(
            piston_mesh,
            material=piston_aluminum,
            name="body",
        )
        piston.visual(
            Cylinder(radius=PISTON_PIN_RADIUS, length=PISTON_PIN_LENGTH),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished_steel,
            name="wrist_pin",
        )
        piston.visual(
            Cylinder(radius=RING_RADIUS, length=0.0024),
            origin=Origin(xyz=(0.0, 0.0, 0.0082)),
            material=ring_steel,
            name="oil_ring",
        )
        piston.visual(
            Cylinder(radius=RING_RADIUS, length=0.0022),
            origin=Origin(xyz=(0.0, 0.0, 0.0131)),
            material=ring_steel,
            name="second_ring",
        )
        piston.visual(
            Cylinder(radius=RING_RADIUS, length=0.0022),
            origin=Origin(xyz=(0.0, 0.0, 0.0180)),
            material=ring_steel,
            name="top_ring",
        )
        piston.inertial = Inertial.from_geometry(
            Cylinder(radius=PISTON_BODY_RADIUS, length=PISTON_HEIGHT),
            mass=0.55,
            origin=Origin(),
        )

    for rod_name in ("rod_a", "rod_b"):
        rod = model.part(rod_name)
        rod.visual(
            rod_mesh,
            material=forged_steel,
            name="body",
        )
        rod.inertial = Inertial.from_geometry(
            Box((ROD_THICKNESS, 0.040, CONROD_CENTER_DISTANCE + 0.028)),
            mass=0.60,
            origin=Origin(xyz=(0.0, 0.0, -(CONROD_CENTER_DISTANCE / 2.0))),
        )

    model.articulation(
        "crankshaft_spin",
        ArticulationType.CONTINUOUS,
        parent="cylinder_block",
        child="crankshaft",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=25.0),
    )
    model.articulation(
        "piston_a_slide",
        ArticulationType.PRISMATIC,
        parent="cylinder_block",
        child="piston_a",
        origin=Origin(xyz=(-CYLINDER_SPACING / 2.0, 0.0, PISTON_A_TDC_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=-STROKE,
            upper=0.0,
        ),
    )
    model.articulation(
        "piston_b_slide",
        ArticulationType.PRISMATIC,
        parent="cylinder_block",
        child="piston_b",
        origin=Origin(xyz=(CYLINDER_SPACING / 2.0, 0.0, PISTON_B_BDC_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=0.0,
            upper=STROKE,
        ),
    )
    model.articulation(
        "rod_a_pivot",
        ArticulationType.REVOLUTE,
        parent="piston_a",
        child="rod_a",
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=5.0,
            lower=-MAX_ROD_ANGLE,
            upper=MAX_ROD_ANGLE,
        ),
    )
    model.articulation(
        "rod_b_pivot",
        ArticulationType.REVOLUTE,
        parent="piston_b",
        child="rod_b",
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=5.0,
            lower=-MAX_ROD_ANGLE,
            upper=MAX_ROD_ANGLE,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.allow_overlap(
        "piston_a",
        "rod_a",
        reason="small-end eye nests around the piston wrist pin",
    )
    ctx.allow_overlap(
        "piston_b",
        "rod_b",
        reason="small-end eye nests around the piston wrist pin",
    )
    ctx.allow_overlap(
        "rod_a",
        "crankshaft",
        reason="big-end eye seats tightly on crankpin A",
    )
    ctx.allow_overlap(
        "rod_b",
        "crankshaft",
        reason="big-end eye seats tightly on crankpin B",
    )
    ctx.allow_overlap(
        "cylinder_block",
        "rod_a",
        reason="broad QC samples uncoupled rod extremes that never occur in the validated crank-slider phase",
    )
    ctx.allow_overlap(
        "cylinder_block",
        "rod_b",
        reason="broad QC samples uncoupled rod extremes that never occur in the validated crank-slider phase",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    _assert_close(STROKE, 2.0 * CRANK_RADIUS, 1e-9, "stroke equals twice crank radius")
    _assert_close(BORE, 2.0 * BORE_RADIUS, 1e-9, "bore equals twice bore radius")
    _assert_close(BORE, 2.0 * RING_RADIUS, 1e-9, "ring diameter matches bore")
    _assert_close(
        STROKE / BORE,
        1.1428571428571428,
        1e-9,
        "stroke-to-bore ratio remains long-stroke",
    )

    ctx.expect_joint_motion_axis(
        "piston_a_slide",
        "piston_a",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )
    ctx.expect_joint_motion_axis(
        "piston_b_slide",
        "piston_b",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )

    with ctx.pose(_pose_for_theta(0.0)):
        ax, ay, az = ctx.part_world_position("piston_a")
        bx, by, bz = ctx.part_world_position("piston_b")
        _assert_close(ax, -CYLINDER_SPACING / 2.0, 0.001, "piston_a x centerline")
        _assert_close(bx, CYLINDER_SPACING / 2.0, 0.001, "piston_b x centerline")
        _assert_close(ay, 0.0, 0.001, "piston_a y centerline")
        _assert_close(by, 0.0, 0.001, "piston_b y centerline")
        _assert_close(az - bz, STROKE, 0.002, "default phase split equals stroke")
        _assert_close(
            az + (PISTON_HEIGHT / 2.0),
            BLOCK_TOP_Z - 0.010,
            0.002,
            "piston_a crown sits just below the deck at tdc",
        )
        _assert_close(
            bz - (PISTON_HEIGHT / 2.0),
            SLEEVE_BOTTOM_Z + 0.002,
            0.002,
            "piston_b skirt remains inside the sleeve at bdc",
        )
        ctx.expect_aabb_contact("rod_a", "piston_a")
        ctx.expect_aabb_contact("rod_b", "piston_b")
        ctx.expect_aabb_contact("rod_a", "crankshaft")
        ctx.expect_aabb_contact("rod_b", "crankshaft")
        ctx.expect_aabb_gap(
            "piston_b",
            "crankshaft",
            axis="z",
            min_gap=COUNTERWEIGHT_CLEARANCE_MIN,
            positive_elem="body",
            negative_elem="counterweight_b_outboard",
            name="piston_b_skirt_clears_counterweight_at_bdc",
        )

    with ctx.pose(_pose_for_theta(math.pi / 2.0)):
        ax, _, az = ctx.part_world_position("piston_a")
        bx, _, bz = ctx.part_world_position("piston_b")
        _assert_close(ax, -CYLINDER_SPACING / 2.0, 0.001, "quarter-turn piston_a x")
        _assert_close(bx, CYLINDER_SPACING / 2.0, 0.001, "quarter-turn piston_b x")
        _assert_close(az, bz, 0.002, "quarter-turn pistons share mid-stroke height")
        ctx.expect_aabb_contact("rod_a", "crankshaft")
        ctx.expect_aabb_contact("rod_b", "crankshaft")

    with ctx.pose(_pose_for_theta(math.pi)):
        ax, _, az = ctx.part_world_position("piston_a")
        bx, _, bz = ctx.part_world_position("piston_b")
        _assert_close(az - bz, -STROKE, 0.002, "half-turn swaps the piston phase")
        _assert_close(ax, -CYLINDER_SPACING / 2.0, 0.001, "half-turn piston_a x")
        _assert_close(bx, CYLINDER_SPACING / 2.0, 0.001, "half-turn piston_b x")
        _assert_close(
            az - (PISTON_HEIGHT / 2.0),
            SLEEVE_BOTTOM_Z + 0.002,
            0.002,
            "piston_a skirt remains inside the sleeve at bdc",
        )
        _assert_close(
            bz + (PISTON_HEIGHT / 2.0),
            BLOCK_TOP_Z - 0.010,
            0.002,
            "piston_b crown sits just below the deck at tdc",
        )
        ctx.expect_aabb_contact("rod_a", "crankshaft")
        ctx.expect_aabb_contact("rod_b", "crankshaft")
        ctx.expect_aabb_gap(
            "piston_a",
            "crankshaft",
            axis="z",
            min_gap=COUNTERWEIGHT_CLEARANCE_MIN,
            positive_elem="body",
            negative_elem="counterweight_a_outboard",
            name="piston_a_skirt_clears_counterweight_at_bdc",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
