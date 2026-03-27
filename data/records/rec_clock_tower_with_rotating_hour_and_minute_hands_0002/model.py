from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
os.chdir("/")

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
    model = ArticulatedObject(name="victorian_station_tower")

    sandstone = model.material("sandstone", rgba=(0.70, 0.62, 0.51, 1.0))
    limestone = model.material("limestone", rgba=(0.84, 0.79, 0.71, 1.0))
    slate = model.material("slate", rgba=(0.28, 0.30, 0.34, 1.0))
    clock_ivory = model.material("clock_ivory", rgba=(0.93, 0.91, 0.84, 1.0))
    clock_iron = model.material("clock_iron", rgba=(0.17, 0.17, 0.18, 1.0))
    aged_metal = model.material("aged_metal", rgba=(0.34, 0.35, 0.32, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.304, 0.304, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=limestone,
        name="plinth",
    )
    tower.visual(
        Box((0.270, 0.270, 0.210)),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=sandstone,
        name="base_core",
    )
    for index, z_center in enumerate((0.022, 0.064, 0.106, 0.148, 0.190)):
        tower.visual(
            Box((0.292 - 0.008 * (index % 2), 0.292 - 0.008 * (index % 2), 0.038)),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=sandstone,
            name=f"rusticated_course_{index}",
        )
    tower.visual(
        Box((0.288, 0.288, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.219)),
        material=limestone,
        name="base_transition",
    )
    tower.visual(
        Box((0.214, 0.022, 0.470)),
        origin=Origin(xyz=(0.0, -0.096, 0.459)),
        material=sandstone,
        name="rear_wall",
    )
    tower.visual(
        Box((0.022, 0.214, 0.470)),
        origin=Origin(xyz=(-0.096, 0.0, 0.459)),
        material=sandstone,
        name="left_wall",
    )
    tower.visual(
        Box((0.022, 0.214, 0.470)),
        origin=Origin(xyz=(0.096, 0.0, 0.459)),
        material=sandstone,
        name="right_wall",
    )
    tower.visual(
        Box((0.214, 0.022, 0.242)),
        origin=Origin(xyz=(0.0, 0.096, 0.345)),
        material=sandstone,
        name="front_lower_wall",
    )
    tower.visual(
        Box((0.214, 0.022, 0.078)),
        origin=Origin(xyz=(0.0, 0.096, 0.655)),
        material=sandstone,
        name="front_upper_wall",
    )
    tower.visual(
        Box((0.024, 0.238, 0.154)),
        origin=Origin(xyz=(-0.109, 0.0, 0.544)),
        material=limestone,
        name="left_belt_wrap",
    )
    tower.visual(
        Box((0.024, 0.238, 0.154)),
        origin=Origin(xyz=(0.109, 0.0, 0.544)),
        material=limestone,
        name="right_belt_wrap",
    )
    tower.visual(
        Box((0.238, 0.024, 0.154)),
        origin=Origin(xyz=(0.0, -0.109, 0.544)),
        material=limestone,
        name="rear_belt_wrap",
    )
    tower.visual(
        Cylinder(radius=0.058, length=0.050),
        origin=Origin(xyz=(0.0, 0.093, 0.544), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aged_metal,
        name="clock_seat",
    )
    tower.visual(
        Box((0.028, 0.024, 0.122)),
        origin=Origin(xyz=(-0.071, 0.119, 0.544)),
        material=limestone,
        name="clock_ring_left",
    )
    tower.visual(
        Box((0.028, 0.024, 0.122)),
        origin=Origin(xyz=(0.071, 0.119, 0.544)),
        material=limestone,
        name="clock_ring_right",
    )
    tower.visual(
        Box((0.130, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.119, 0.611)),
        material=limestone,
        name="clock_ring_top",
    )
    tower.visual(
        Box((0.130, 0.024, 0.030)),
        origin=Origin(xyz=(0.0, 0.119, 0.477)),
        material=limestone,
        name="clock_ring_bottom",
    )
    for name_suffix, x_center, z_center in (
        ("nw", -0.051, 0.593),
        ("ne", 0.051, 0.593),
        ("sw", -0.051, 0.495),
        ("se", 0.051, 0.495),
    ):
        tower.visual(
            Box((0.024, 0.024, 0.024)),
            origin=Origin(xyz=(x_center, 0.119, z_center)),
            material=limestone,
            name=f"clock_ring_diag_{name_suffix}",
        )
    for level, z_center in enumerate((0.270, 0.340, 0.410, 0.480, 0.550, 0.620)):
        span_x = 0.034 if level % 2 == 0 else 0.022
        span_y = 0.022 if level % 2 == 0 else 0.034
        x_offset = 0.107 - span_x / 2.0 if span_x > span_y else 0.107 + span_x / 2.0
        y_offset = 0.107 - span_y / 2.0 if span_y > span_x else 0.107 + span_y / 2.0
        for x_sign in (-1.0, 1.0):
            for y_sign in (-1.0, 1.0):
                tower.visual(
                    Box((span_x, span_y, 0.068)),
                    origin=Origin(xyz=(x_sign * x_offset, y_sign * y_offset, z_center)),
                    material=limestone,
                    name=f"quoin_{level}_{int(x_sign > 0)}_{int(y_sign > 0)}",
                )
    tower.visual(
        Box((0.278, 0.278, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.703)),
        material=limestone,
        name="cornice_lower",
    )
    tower.visual(
        Box((0.248, 0.248, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        material=limestone,
        name="cornice_upper",
    )
    tower.visual(
        Box((0.194, 0.194, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.753)),
        material=slate,
        name="roof_peak_lower",
    )
    tower.visual(
        Box((0.132, 0.132, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.803)),
        material=slate,
        name="roof_peak_mid",
    )
    tower.visual(
        Box((0.066, 0.066, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.858)),
        material=slate,
        name="roof_peak",
    )
    tower.visual(
        Cylinder(radius=0.004, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.948)),
        material=aged_metal,
        name="flagpole",
    )
    tower.visual(
        Sphere(radius=0.009),
        origin=Origin(xyz=(0.0, 0.0, 1.017)),
        material=aged_metal,
        name="flagpole_finial",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.320, 0.320, 1.030)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
    )

    clock_face = model.part("clock_face")
    clock_face.visual(
        Cylinder(radius=0.063, length=0.008),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=clock_ivory,
        name="clock_face_disk",
    )
    clock_face.visual(
        Cylinder(radius=0.057, length=0.024),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aged_metal,
        name="clock_mount",
    )
    clock_face.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=clock_iron,
        name="hub_boss",
    )
    clock_face.inertial = Inertial.from_geometry(
        Box((0.130, 0.034, 0.130)),
        mass=0.9,
        origin=Origin(xyz=(0.0, -0.007, 0.0)),
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Cylinder(radius=0.011, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=clock_iron,
        name="hour_cap",
    )
    hour_hand.visual(
        Box((0.012, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=clock_iron,
        name="hour_arm",
    )
    hour_hand.visual(
        Box((0.006, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=clock_iron,
        name="hour_tip",
    )
    hour_hand.visual(
        Box((0.007, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=clock_iron,
        name="hour_tail",
    )
    hour_hand.inertial = Inertial.from_geometry(
        Box((0.020, 0.004, 0.046)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Cylinder(radius=0.0085, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=clock_iron,
        name="minute_cap",
    )
    minute_hand.visual(
        Box((0.010, 0.004, 0.046)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=clock_iron,
        name="minute_arm",
    )
    minute_hand.visual(
        Box((0.005, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=clock_iron,
        name="minute_tip",
    )
    minute_hand.visual(
        Box((0.006, 0.004, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=clock_iron,
        name="minute_tail",
    )
    minute_hand.inertial = Inertial.from_geometry(
        Box((0.018, 0.004, 0.066)),
        mass=0.04,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "tower_to_clock_face",
        ArticulationType.FIXED,
        parent=tower,
        child=clock_face,
        origin=Origin(xyz=(0.0, 0.104, 0.544)),
    )
    model.articulation(
        "clock_face_to_hour_hand",
        ArticulationType.CONTINUOUS,
        parent=clock_face,
        child=hour_hand,
        origin=Origin(xyz=(0.0, 0.019, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )
    model.articulation(
        "clock_face_to_minute_hand",
        ArticulationType.CONTINUOUS,
        parent=clock_face,
        child=minute_hand,
        origin=Origin(xyz=(0.0, 0.023, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    clock_face = object_model.get_part("clock_face")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")
    hour_spin = object_model.get_articulation("clock_face_to_hour_hand")
    minute_spin = object_model.get_articulation("clock_face_to_minute_hand")

    clock_seat = tower.get_visual("clock_seat")
    roof_peak = tower.get_visual("roof_peak")
    roof_peak_lower = tower.get_visual("roof_peak_lower")
    flagpole = tower.get_visual("flagpole")
    flagpole_finial = tower.get_visual("flagpole_finial")
    cornice_lower = tower.get_visual("cornice_lower")
    cornice_upper = tower.get_visual("cornice_upper")
    clock_ring_left = tower.get_visual("clock_ring_left")
    clock_ring_right = tower.get_visual("clock_ring_right")
    clock_ring_top = tower.get_visual("clock_ring_top")
    clock_ring_bottom = tower.get_visual("clock_ring_bottom")
    rusticated_course = tower.get_visual("rusticated_course_2")
    quoin = tower.get_visual("quoin_2_1_1")
    plinth = tower.get_visual("plinth")

    clock_disk = clock_face.get_visual("clock_face_disk")
    clock_mount = clock_face.get_visual("clock_mount")
    hub_boss = clock_face.get_visual("hub_boss")

    hour_cap = hour_hand.get_visual("hour_cap")
    hour_tip = hour_hand.get_visual("hour_tip")
    minute_cap = minute_hand.get_visual("minute_cap")
    minute_tip = minute_hand.get_visual("minute_tip")

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

    ctx.expect_contact(clock_face, tower, elem_a=clock_mount, elem_b=clock_seat)
    ctx.expect_overlap(clock_face, tower, axes="xz", elem_a=clock_disk, elem_b=clock_ring_left, min_overlap=0.005)
    ctx.expect_overlap(clock_face, tower, axes="xz", elem_a=clock_disk, elem_b=clock_ring_right, min_overlap=0.005)
    ctx.expect_overlap(clock_face, tower, axes="xz", elem_a=clock_disk, elem_b=clock_ring_top, min_overlap=0.010)
    ctx.expect_overlap(clock_face, tower, axes="xz", elem_a=clock_disk, elem_b=clock_ring_bottom, min_overlap=0.010)
    ctx.expect_within(clock_face, tower, axes="xz", inner_elem=clock_disk)
    ctx.expect_contact(clock_face, hour_hand, elem_a=hub_boss, elem_b=hour_cap)
    ctx.expect_contact(hour_hand, minute_hand, elem_a=hour_cap, elem_b=minute_cap)
    ctx.expect_gap(
        minute_hand,
        hour_hand,
        axis="z",
        min_gap=0.005,
        positive_elem=minute_tip,
        negative_elem=hour_tip,
    )
    ctx.expect_within(
        hour_hand,
        clock_face,
        axes="xz",
        inner_elem=hour_tip,
        outer_elem=clock_disk,
    )
    ctx.expect_within(
        minute_hand,
        clock_face,
        axes="xz",
        inner_elem=minute_tip,
        outer_elem=clock_disk,
    )
    ctx.expect_contact(tower, tower, elem_a=quoin, elem_b=tower.get_visual("right_wall"))
    ctx.expect_within(tower, tower, axes="xy", inner_elem=rusticated_course, outer_elem=plinth)
    ctx.expect_contact(tower, tower, elem_a=plinth, elem_b=tower.get_visual("rusticated_course_0"))
    ctx.expect_contact(tower, tower, elem_a=cornice_upper, elem_b=cornice_lower)
    ctx.expect_contact(tower, tower, elem_a=roof_peak_lower, elem_b=cornice_upper)
    ctx.expect_contact(tower, tower, elem_a=flagpole, elem_b=roof_peak)
    ctx.expect_contact(tower, tower, elem_a=flagpole_finial, elem_b=flagpole)

    with ctx.pose({hour_spin: -math.pi / 3.0, minute_spin: math.pi / 2.0}):
        ctx.expect_within(
            hour_hand,
            clock_face,
            axes="xz",
            inner_elem=hour_tip,
            outer_elem=clock_disk,
        )
        ctx.expect_within(
            minute_hand,
            clock_face,
            axes="xz",
            inner_elem=minute_tip,
            outer_elem=clock_disk,
        )
        ctx.expect_contact(clock_face, hour_hand, elem_a=hub_boss, elem_b=hour_cap)
        ctx.expect_contact(hour_hand, minute_hand, elem_a=hour_cap, elem_b=minute_cap)

    with ctx.pose({hour_spin: math.pi / 6.0, minute_spin: math.pi}):
        ctx.expect_within(
            hour_hand,
            clock_face,
            axes="xz",
            inner_elem=hour_tip,
            outer_elem=clock_disk,
        )
        ctx.expect_within(
            minute_hand,
            clock_face,
            axes="xz",
            inner_elem=minute_tip,
            outer_elem=clock_disk,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
