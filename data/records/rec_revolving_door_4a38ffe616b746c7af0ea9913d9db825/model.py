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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _add_quad(
    geom: MeshGeometry, a: int, b: int, c: int, d: int
) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _annular_sector_geometry(
    *,
    inner_radius: float,
    outer_radius: float,
    z0: float,
    z1: float,
    start_angle: float,
    end_angle: float,
    segments: int,
) -> MeshGeometry:
    geom = MeshGeometry()
    angle_span = end_angle - start_angle
    closed = abs(angle_span - 2.0 * math.pi) < 1e-6
    sample_count = segments if closed else segments + 1

    rings: list[tuple[int, int, int, int]] = []
    for i in range(sample_count):
        t = i / segments
        angle = start_angle + angle_span * t
        c = math.cos(angle)
        s = math.sin(angle)
        rings.append(
            (
                geom.add_vertex(outer_radius * c, outer_radius * s, z0),
                geom.add_vertex(outer_radius * c, outer_radius * s, z1),
                geom.add_vertex(inner_radius * c, inner_radius * s, z0),
                geom.add_vertex(inner_radius * c, inner_radius * s, z1),
            )
        )

    span_count = sample_count if closed else sample_count - 1
    for i in range(span_count):
        j = (i + 1) % sample_count
        outer_b0, outer_t0, inner_b0, inner_t0 = rings[i]
        outer_b1, outer_t1, inner_b1, inner_t1 = rings[j]
        _add_quad(geom, outer_b0, outer_b1, outer_t1, outer_t0)
        _add_quad(geom, inner_b1, inner_b0, inner_t0, inner_t1)
        _add_quad(geom, inner_t0, outer_t0, outer_t1, inner_t1)
        _add_quad(geom, outer_b0, inner_b0, inner_b1, outer_b1)

    if not closed:
        outer_b0, outer_t0, inner_b0, inner_t0 = rings[0]
        outer_b1, outer_t1, inner_b1, inner_t1 = rings[-1]
        _add_quad(geom, outer_b0, outer_t0, inner_t0, inner_b0)
        _add_quad(geom, inner_b1, inner_t1, outer_t1, outer_b1)

    return geom


def _annular_sector_mesh(
    name: str,
    *,
    inner_radius: float,
    outer_radius: float,
    z0: float,
    z1: float,
    start_angle: float,
    end_angle: float,
    segments: int = 48,
):
    return mesh_from_geometry(
        _annular_sector_geometry(
            inner_radius=inner_radius,
            outer_radius=outer_radius,
            z0=z0,
            z1=z1,
            start_angle=start_angle,
            end_angle=end_angle,
            segments=segments,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retail_revolving_door")

    aluminum = model.material("aluminum", rgba=(0.67, 0.70, 0.74, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.26, 0.29, 1.0))
    glazing = model.material("glazing", rgba=(0.70, 0.84, 0.90, 0.28))
    smoked_glass = model.material("smoked_glass", rgba=(0.56, 0.67, 0.73, 0.22))
    threshold_finish = model.material("threshold_finish", rgba=(0.39, 0.41, 0.43, 1.0))

    drum_inner_radius = 1.15
    drum_outer_radius = 1.21
    threshold_inner_radius = 1.04
    clear_wing_radius = 1.03
    door_height = 2.05
    canopy_z0 = 2.18
    canopy_z1 = 2.28
    opening_half_angle = math.radians(30.0)

    door_frame = model.part("door_frame")
    door_frame.visual(
        _annular_sector_mesh(
            "revolving_threshold_ring",
            inner_radius=threshold_inner_radius,
            outer_radius=drum_outer_radius,
            z0=0.0,
            z1=0.045,
            start_angle=0.0,
            end_angle=2.0 * math.pi,
            segments=72,
        ),
        material=threshold_finish,
        name="threshold_ring",
    )
    door_frame.visual(
        _annular_sector_mesh(
            "revolving_left_sidelite",
            inner_radius=drum_inner_radius,
            outer_radius=drum_outer_radius,
            z0=0.05,
            z1=canopy_z0,
            start_angle=opening_half_angle,
            end_angle=math.pi - opening_half_angle,
            segments=40,
        ),
        material=glazing,
        name="left_sidelite",
    )
    door_frame.visual(
        _annular_sector_mesh(
            "revolving_right_sidelite",
            inner_radius=drum_inner_radius,
            outer_radius=drum_outer_radius,
            z0=0.05,
            z1=canopy_z0,
            start_angle=math.pi + opening_half_angle,
            end_angle=2.0 * math.pi - opening_half_angle,
            segments=40,
        ),
        material=glazing,
        name="right_sidelite",
    )
    door_frame.visual(
        _annular_sector_mesh(
            "revolving_header_ring",
            inner_radius=threshold_inner_radius,
            outer_radius=drum_outer_radius,
            z0=canopy_z0,
            z1=canopy_z1,
            start_angle=0.0,
            end_angle=2.0 * math.pi,
            segments=72,
        ),
        material=aluminum,
        name="header_ring",
    )
    door_frame.visual(
        Cylinder(radius=threshold_inner_radius, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, canopy_z0 + 0.015)),
        material=smoked_glass,
        name="ceiling_disk",
    )
    door_frame.visual(
        mesh_from_geometry(
            TorusGeometry(radius=1.125, tube=0.085, radial_segments=20, tubular_segments=72),
            "revolving_motor_drive_ring",
        ),
        origin=Origin(xyz=(0.0, 0.0, 2.33)),
        material=dark_metal,
        name="motor_drive_ring",
    )

    mullion_angles = (
        opening_half_angle,
        math.pi - opening_half_angle,
        math.pi + opening_half_angle,
        2.0 * math.pi - opening_half_angle,
    )
    mullion_radius = 1.18
    for index, angle in enumerate(mullion_angles):
        door_frame.visual(
            Cylinder(radius=0.038, length=canopy_z0),
            origin=Origin(
                xyz=(mullion_radius * math.cos(angle), mullion_radius * math.sin(angle), canopy_z0 * 0.5)
            ),
            material=aluminum,
            name=f"mullion_{index}",
        )

    door_frame.inertial = Inertial.from_geometry(
        Box((2.5, 2.5, 2.42)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 1.21)),
    )

    wing_assembly = model.part("wing_assembly")
    wing_assembly.visual(
        Cylinder(radius=0.065, length=2.10),
        origin=Origin(xyz=(0.0, 0.0, 1.07)),
        material=dark_metal,
        name="central_post",
    )
    wing_assembly.visual(
        Cylinder(radius=0.11, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_metal,
        name="bottom_hub",
    )
    wing_assembly.visual(
        Cylinder(radius=0.12, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 2.15)),
        material=dark_metal,
        name="top_hub",
    )

    wing_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(wing_angles):
        wing_rotation = Origin(rpy=(0.0, 0.0, angle))
        wing_assembly.visual(
            Box((0.06, 0.07, 2.03)),
            origin=Origin(
                xyz=(0.09, 0.0, 1.055),
                rpy=wing_rotation.rpy,
            ),
            material=aluminum,
            name=f"wing_{index}_inner_stile",
        )
        wing_assembly.visual(
            Box((0.05, 0.07, 2.03)),
            origin=Origin(
                xyz=(0.995, 0.0, 1.055),
                rpy=wing_rotation.rpy,
            ),
            material=aluminum,
            name=f"wing_{index}_outer_stile",
        )
        wing_assembly.visual(
            Box((0.90, 0.055, 0.05)),
            origin=Origin(
                xyz=(0.55, 0.0, 2.015),
                rpy=wing_rotation.rpy,
            ),
            material=aluminum,
            name=f"wing_{index}_top_rail",
        )
        wing_assembly.visual(
            Box((0.90, 0.055, 0.05)),
            origin=Origin(
                xyz=(0.55, 0.0, 0.095),
                rpy=wing_rotation.rpy,
            ),
            material=aluminum,
            name=f"wing_{index}_bottom_rail",
        )
        wing_assembly.visual(
            Box((0.86, 0.028, 1.90)),
            origin=Origin(
                xyz=(0.55, 0.0, 1.05),
                rpy=wing_rotation.rpy,
            ),
            material=smoked_glass,
            name=f"wing_{index}_panel",
        )

    wing_assembly.inertial = Inertial.from_geometry(
        Box((2.10, 2.10, 2.18)),
        mass=150.0,
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
    )

    model.articulation(
        "revolving_rotation",
        ArticulationType.CONTINUOUS,
        parent=door_frame,
        child=wing_assembly,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.9),
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

    door_frame = object_model.get_part("door_frame")
    wing_assembly = object_model.get_part("wing_assembly")
    spin = object_model.get_articulation("revolving_rotation")

    def _center(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx.check(
        "revolving joint is continuous and vertical",
        spin.motion_limits is not None
        and spin.motion_limits.lower is None
        and spin.motion_limits.upper is None
        and tuple(round(v, 4) for v in spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={spin.axis}, limits={spin.motion_limits}",
    )

    with ctx.pose({spin: 0.0}):
        ctx.expect_origin_distance(
            door_frame,
            wing_assembly,
            axes="xy",
            max_dist=0.001,
            name="wing assembly stays centered in the drum",
        )
        ctx.expect_gap(
            door_frame,
            wing_assembly,
            axis="z",
            positive_elem="ceiling_disk",
            negative_elem="wing_0_panel",
            min_gap=0.10,
            max_gap=0.20,
            name="wing panels clear the overhead canopy",
        )
        ctx.expect_gap(
            wing_assembly,
            door_frame,
            axis="z",
            positive_elem="wing_0_panel",
            negative_elem="threshold_ring",
            min_gap=0.015,
            max_gap=0.06,
            name="wing panels ride just above the threshold ring",
        )
        start_aabb = ctx.part_element_world_aabb(wing_assembly, elem="wing_0_outer_stile")

    with ctx.pose({spin: math.pi / 2.0}):
        quarter_turn_aabb = ctx.part_element_world_aabb(wing_assembly, elem="wing_0_outer_stile")

    start_center = _center(start_aabb) if start_aabb is not None else None
    quarter_turn_center = _center(quarter_turn_aabb) if quarter_turn_aabb is not None else None
    start_radius = (
        math.hypot(start_center[0], start_center[1]) if start_center is not None else None
    )
    quarter_turn_radius = (
        math.hypot(quarter_turn_center[0], quarter_turn_center[1])
        if quarter_turn_center is not None
        else None
    )
    ctx.check(
        "wing assembly rotates around the central post",
        start_center is not None
        and quarter_turn_center is not None
        and start_center[0] > 0.9
        and abs(start_center[1]) < 0.08
        and quarter_turn_center[1] > 0.9
        and abs(quarter_turn_center[0]) < 0.08
        and start_radius is not None
        and quarter_turn_radius is not None
        and abs(start_radius - quarter_turn_radius) < 0.02
        and start_center[2] > 1.0
        and abs(start_center[2] - quarter_turn_center[2]) < 0.001,
        details=f"start={start_center}, quarter_turn={quarter_turn_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
