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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    def _midpoint(a, b):
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

    def _distance(a, b) -> float:
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def _rpy_for_cylinder(a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(math.hypot(dx, dy), dz)
        return (0.0, pitch, yaw)

    def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
        part.visual(
            Cylinder(radius=radius, length=_distance(a, b)),
            origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
            material=material,
            name=name,
        )

    model = ArticulatedObject(name="drill_press")

    cast_iron = model.material("cast_iron", rgba=(0.29, 0.31, 0.33, 1.0))
    enamel_green = model.material("enamel_green", rgba=(0.23, 0.35, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.21, 0.22, 0.24, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    black_bakelite = model.material("black_bakelite", rgba=(0.09, 0.09, 0.10, 1.0))
    scale_aluminum = model.material("scale_aluminum", rgba=(0.76, 0.77, 0.79, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.132, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=cast_iron,
        name="hub",
    )
    stand.visual(
        Cylinder(radius=0.062, length=0.134),
        origin=Origin(xyz=(0.0, 0.0, 0.109)),
        material=cast_iron,
        name="column_socket",
    )
    for name, angle in (
        ("front_left", math.radians(22.0)),
        ("front_right", math.radians(158.0)),
        ("rear", math.radians(270.0)),
    ):
        start = (0.076 * math.cos(angle), 0.076 * math.sin(angle), 0.064)
        end = (0.330 * math.cos(angle), 0.330 * math.sin(angle), 0.026)
        _add_member(stand, start, end, 0.020, cast_iron, name=f"{name}_leg")
        stand.visual(
            Cylinder(radius=0.050, length=0.018),
            origin=Origin(xyz=(0.340 * math.cos(angle), 0.340 * math.sin(angle), 0.009)),
            material=cast_iron,
            name=f"{name}_foot",
        )
    stand.inertial = Inertial.from_geometry(
        Box((0.82, 0.82, 0.18)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    main_column = model.part("main_column")
    main_column.visual(
        Cylinder(radius=0.040, length=1.18),
        origin=Origin(xyz=(0.0, 0.0, 0.59)),
        material=cast_iron,
        name="column_shaft",
    )
    main_column.visual(
        Cylinder(radius=0.060, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 1.145)),
        material=cast_iron,
        name="head_seat",
    )
    main_column.visual(
        Box((0.022, 0.010, 0.565)),
        origin=Origin(xyz=(0.0, 0.045, 0.565)),
        material=dark_steel,
        name="rack_strip",
    )
    main_column.visual(
        Box((0.014, 0.010, 0.565)),
        origin=Origin(xyz=(0.0, 0.047, 0.565)),
        material=machined_steel,
        name="rack_teeth",
    )
    tooth_pitch = 0.026
    for index in range(20):
        tooth_z = 0.305 + index * tooth_pitch
        main_column.visual(
            Box((0.016, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, 0.052, tooth_z)),
            material=machined_steel,
        )
    main_column.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=1.20),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
    )

    drill_head = model.part("drill_head")
    drill_head.visual(
        Cylinder(radius=0.072, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=enamel_green,
        name="head_clamp",
    )
    drill_head.visual(
        Box((0.28, 0.22, 0.24)),
        origin=Origin(xyz=(0.0, 0.045, 0.120)),
        material=enamel_green,
        name="head_body",
    )
    drill_head.visual(
        Box((0.34, 0.25, 0.12)),
        origin=Origin(xyz=(0.0, 0.015, 0.285)),
        material=enamel_green,
        name="belt_cover",
    )
    drill_head.visual(
        Cylinder(radius=0.072, length=0.28),
        origin=Origin(xyz=(0.0, -0.145, 0.240), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="motor",
    )
    drill_head.visual(
        Cylinder(radius=0.060, length=0.110),
        origin=Origin(xyz=(0.0, 0.150, 0.060), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel_green,
        name="quill_housing",
    )
    drill_head.visual(
        Cylinder(radius=0.028, length=0.210),
        origin=Origin(xyz=(0.0, 0.210, -0.080)),
        material=machined_steel,
        name="quill",
    )
    drill_head.visual(
        Cylinder(radius=0.019, length=0.075),
        origin=Origin(xyz=(0.0, 0.210, -0.220)),
        material=dark_steel,
        name="chuck",
    )
    drill_head.visual(
        Cylinder(radius=0.0035, length=0.110),
        origin=Origin(xyz=(0.0, 0.210, -0.3125)),
        material=machined_steel,
        name="drill_bit",
    )
    drill_head.visual(
        Cylinder(radius=0.018, length=0.048),
        origin=Origin(xyz=(0.148, 0.162, 0.070), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="feed_hub",
    )
    for angle in (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0):
        spoke_start = (0.170, 0.162, 0.070)
        spoke_end = (
            0.270,
            0.162 + 0.040 * math.sin(angle),
            0.070 + 0.048 * math.cos(angle),
        )
        _add_member(drill_head, spoke_start, spoke_end, 0.0075, machined_steel)
        drill_head.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=spoke_end),
            material=black_bakelite,
        )
    drill_head.inertial = Inertial.from_geometry(
        Box((0.40, 0.48, 0.72)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.03, 0.04)),
    )

    table_carriage = model.part("table_carriage")
    table_carriage.visual(
        Cylinder(radius=0.062, length=0.132),
        material=cast_iron,
        name="column_collar",
    )
    table_carriage.visual(
        Box((0.110, 0.192, 0.052)),
        origin=Origin(xyz=(0.0, 0.094, -0.004)),
        material=cast_iron,
        name="support_arm",
    )
    table_carriage.visual(
        Box((0.172, 0.066, 0.058)),
        origin=Origin(xyz=(0.0, 0.148, -0.006)),
        material=cast_iron,
        name="yoke_bridge",
    )
    table_carriage.visual(
        Box((0.024, 0.040, 0.114)),
        origin=Origin(xyz=(-0.094, 0.180, 0.000)),
        material=cast_iron,
        name="left_ear",
    )
    table_carriage.visual(
        Box((0.024, 0.040, 0.114)),
        origin=Origin(xyz=(0.094, 0.180, 0.000)),
        material=cast_iron,
        name="right_ear",
    )
    table_carriage.visual(
        Box((0.004, 0.048, 0.086)),
        origin=Origin(xyz=(-0.108, 0.180, 0.000), rpy=(0.42, 0.0, 0.0)),
        material=scale_aluminum,
        name="scale_plate",
    )
    for offset in (-0.030, -0.015, 0.0, 0.015, 0.030):
        tick_length = 0.022 if abs(offset) < 1e-9 else 0.014
        table_carriage.visual(
            Box((0.002, tick_length, 0.0025)),
            origin=Origin(
                xyz=(
                    -0.111,
                    0.180 - math.sin(0.42) * offset,
                    math.cos(0.42) * offset,
                ),
                rpy=(0.42, 0.0, 0.0),
            ),
            material=dark_steel,
        )
    table_carriage.visual(
        Box((0.034, 0.052, 0.050)),
        origin=Origin(xyz=(0.0, 0.052, -0.004)),
        material=dark_steel,
        name="pinion_housing",
    )
    table_carriage.visual(
        Cylinder(radius=0.010, length=0.068),
        origin=Origin(xyz=(0.0, 0.071, -0.004), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="crank_axis",
    )
    _add_member(
        table_carriage,
        (0.034, 0.071, -0.004),
        (0.118, 0.083, 0.000),
        0.0065,
        machined_steel,
    )
    table_carriage.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.130, 0.085, 0.002)),
        material=black_bakelite,
        name="crank_knob",
    )
    table_carriage.inertial = Inertial.from_geometry(
        Box((0.24, 0.30, 0.18)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.10, 0.0)),
    )

    table = model.part("table")
    table.visual(
        Cylinder(radius=0.028, length=0.168),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_iron,
        name="trunnion",
    )
    table.visual(
        Box((0.126, 0.130, 0.062)),
        origin=Origin(xyz=(0.0, 0.090, 0.040)),
        material=cast_iron,
        name="support_web",
    )
    table.visual(
        Cylinder(radius=0.052, length=0.056),
        origin=Origin(xyz=(0.0, 0.132, 0.052)),
        material=cast_iron,
        name="table_hub",
    )
    table.visual(
        Cylinder(radius=0.160, length=0.024),
        origin=Origin(xyz=(0.0, 0.205, 0.086)),
        material=cast_iron,
        name="table_disk",
    )
    table.visual(
        Box((0.006, 0.022, 0.003)),
        origin=Origin(xyz=(-0.082, 0.010, 0.000)),
        material=machined_steel,
        name="angle_pointer",
    )
    table.inertial = Inertial.from_geometry(
        Box((0.34, 0.38, 0.14)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.18, 0.06)),
    )

    model.articulation(
        "stand_to_column",
        ArticulationType.FIXED,
        parent=stand,
        child=main_column,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )
    model.articulation(
        "column_to_head",
        ArticulationType.FIXED,
        parent=main_column,
        child=drill_head,
        origin=Origin(xyz=(0.0, 0.0, 1.080)),
    )
    model.articulation(
        "column_to_table_carriage",
        ArticulationType.PRISMATIC,
        parent=main_column,
        child=table_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=-0.18, upper=0.0),
    )
    model.articulation(
        "table_carriage_to_table",
        ArticulationType.REVOLUTE,
        parent=table_carriage,
        child=table,
        origin=Origin(xyz=(0.0, 0.180, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    main_column = object_model.get_part("main_column")
    drill_head = object_model.get_part("drill_head")
    table_carriage = object_model.get_part("table_carriage")
    table = object_model.get_part("table")
    table_height = object_model.get_articulation("column_to_table_carriage")
    table_tilt = object_model.get_articulation("table_carriage_to_table")

    column_shaft = main_column.get_visual("column_shaft")
    head_seat = main_column.get_visual("head_seat")
    rack_teeth = main_column.get_visual("rack_teeth")
    chuck = drill_head.get_visual("chuck")
    head_clamp = drill_head.get_visual("head_clamp")
    column_collar = table_carriage.get_visual("column_collar")
    pinion_housing = table_carriage.get_visual("pinion_housing")
    left_ear = table_carriage.get_visual("left_ear")
    right_ear = table_carriage.get_visual("right_ear")
    scale_plate = table_carriage.get_visual("scale_plate")
    trunnion = table.get_visual("trunnion")
    table_disk = table.get_visual("table_disk")
    angle_pointer = table.get_visual("angle_pointer")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap(
        table_carriage,
        main_column,
        reason="table support sleeve nests around the round column for vertical adjustment.",
    )
    ctx.allow_overlap(
        table,
        table_carriage,
        reason="table trunnion nests between the cast bracket ears around the tilt pin.",
    )

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(main_column, stand, axes="xy", min_overlap=0.08, elem_a=column_shaft, elem_b=stand.get_visual("column_socket"))
    ctx.expect_overlap(drill_head, main_column, axes="xy", min_overlap=0.10, elem_a=head_clamp, elem_b=head_seat)
    ctx.expect_overlap(table_carriage, main_column, axes="xy", min_overlap=0.004, elem_a=column_collar, elem_b=column_shaft)
    ctx.expect_overlap(table_carriage, main_column, axes="xz", min_overlap=0.008, elem_a=pinion_housing, elem_b=rack_teeth)
    ctx.expect_overlap(table, table_carriage, axes="yz", min_overlap=0.003, elem_a=trunnion, elem_b=left_ear)
    ctx.expect_overlap(table, table_carriage, axes="yz", min_overlap=0.003, elem_a=trunnion, elem_b=right_ear)
    ctx.expect_overlap(table, table_carriage, axes="yz", min_overlap=0.001, elem_a=angle_pointer, elem_b=scale_plate)
    ctx.expect_gap(
        table,
        table_carriage,
        axis="x",
        min_gap=0.018,
        max_gap=0.024,
        positive_elem=angle_pointer,
        negative_elem=scale_plate,
    )
    ctx.expect_overlap(table, drill_head, axes="xy", min_overlap=0.003, elem_a=table_disk, elem_b=chuck)
    ctx.expect_gap(
        drill_head,
        table,
        axis="z",
        min_gap=0.08,
        max_gap=0.22,
        positive_elem=chuck,
        negative_elem=table_disk,
    )

    with ctx.pose({table_height: -0.12}):
        ctx.expect_overlap(table_carriage, main_column, axes="xy", min_overlap=0.004, elem_a=column_collar, elem_b=column_shaft)
        ctx.expect_overlap(table_carriage, main_column, axes="xz", min_overlap=0.008, elem_a=pinion_housing, elem_b=rack_teeth)
        ctx.expect_overlap(table, drill_head, axes="xy", min_overlap=0.003, elem_a=table_disk, elem_b=chuck)
        ctx.expect_gap(
            drill_head,
            table,
            axis="z",
            min_gap=0.24,
            max_gap=0.32,
            positive_elem=chuck,
            negative_elem=table_disk,
        )

    with ctx.pose({table_tilt: 0.42}):
        ctx.expect_overlap(table, table_carriage, axes="yz", min_overlap=0.002, elem_a=trunnion, elem_b=left_ear)
        ctx.expect_overlap(table, table_carriage, axes="yz", min_overlap=0.002, elem_a=trunnion, elem_b=right_ear)
        ctx.expect_overlap(table, table_carriage, axes="yz", min_overlap=0.001, elem_a=angle_pointer, elem_b=scale_plate)
        ctx.expect_gap(
            table,
            table_carriage,
            axis="x",
            min_gap=0.018,
            max_gap=0.024,
            positive_elem=angle_pointer,
            negative_elem=scale_plate,
        )
        ctx.expect_overlap(table, drill_head, axes="xy", min_overlap=0.003, elem_a=table_disk, elem_b=chuck)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
