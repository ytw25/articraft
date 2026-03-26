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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_artillery_gun")

    olive = model.material("olive_drab", rgba=(0.39, 0.43, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.56, 0.58, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.26, 1.0))

    def add_spoked_wheel(part_name: str, inner_face_y: float) -> None:
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=0.070, length=0.180),
            origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=dark_steel,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.078, length=0.018),
            origin=Origin(xyz=(0.0, inner_face_y, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
            material=dark_steel,
            name="hub_face",
        )

        rim_radius = 0.365
        segment_count = 12
        for idx in range(segment_count):
            angle = 2.0 * math.pi * idx / segment_count
            wheel.visual(
                Box((0.220, 0.030, 0.060)),
                origin=Origin(
                    xyz=(rim_radius * math.cos(angle), 0.0, rim_radius * math.sin(angle)),
                    rpy=(0.0, angle + math.pi * 0.5, 0.0),
                ),
                material=steel,
                name=f"rim_segment_{idx}",
            )

        spoke_start = 0.060
        spoke_end = 0.340
        spoke_length = spoke_end - spoke_start
        spoke_mid = 0.5 * (spoke_start + spoke_end)
        for idx in range(12):
            angle = 2.0 * math.pi * idx / 12.0
            wheel.visual(
                Cylinder(radius=0.010, length=spoke_length),
                origin=Origin(
                    xyz=(spoke_mid * math.cos(angle), 0.0, spoke_mid * math.sin(angle)),
                    rpy=(0.0, math.pi * 0.5 - angle, 0.0),
                ),
                material=steel,
                name=f"spoke_{idx}",
            )

        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.410, length=0.180),
            mass=135.0,
            origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        )

    carriage = model.part("carriage")
    carriage.visual(
        Cylinder(radius=0.034, length=1.580),
        origin=Origin(xyz=(-0.080, 0.000, 0.430), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="rear_axle",
    )
    carriage.visual(
        Box((0.300, 0.200, 0.090)),
        origin=Origin(xyz=(-0.020, 0.000, 0.455)),
        material=olive,
        name="carriage_body",
    )
    carriage.visual(
        Box((0.180, 0.080, 0.080)),
        origin=Origin(xyz=(-0.170, 0.110, 0.390)),
        material=olive,
        name="left_trail_socket",
    )
    carriage.visual(
        Box((0.180, 0.080, 0.080)),
        origin=Origin(xyz=(-0.170, -0.110, 0.390)),
        material=olive,
        name="right_trail_socket",
    )
    carriage.visual(
        Box((0.150, 0.140, 0.180)),
        origin=Origin(xyz=(0.050, 0.000, 0.565)),
        material=olive,
        name="elevating_pedestal",
    )
    carriage.visual(
        Box((0.180, 0.260, 0.050)),
        origin=Origin(xyz=(0.120, 0.000, 0.630)),
        material=olive,
        name="cradle_crossmember",
    )
    carriage.visual(
        Box((0.160, 0.060, 0.170)),
        origin=Origin(xyz=(0.150, 0.120, 0.670)),
        material=olive,
        name="left_trunnion_saddle",
    )
    carriage.visual(
        Box((0.160, 0.060, 0.170)),
        origin=Origin(xyz=(0.150, -0.120, 0.670)),
        material=olive,
        name="right_trunnion_saddle",
    )
    carriage.visual(
        Cylinder(radius=0.028, length=0.250),
        origin=Origin(xyz=(0.160, 0.000, 0.670), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_crossshaft",
    )
    carriage.visual(
        Cylinder(radius=0.042, length=0.080),
        origin=Origin(xyz=(-0.080, 0.750, 0.430), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="left_journal_cap",
    )
    carriage.visual(
        Cylinder(radius=0.042, length=0.080),
        origin=Origin(xyz=(-0.080, -0.750, 0.430), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="right_journal_cap",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((1.250, 1.450, 0.700)),
        mass=1280.0,
        origin=Origin(xyz=(-0.180, 0.000, 0.360)),
    )

    add_spoked_wheel("left_wheel", inner_face_y=-0.091)
    add_spoked_wheel("right_wheel", inner_face_y=0.091)

    left_trail = model.part("left_trail")
    left_trail.visual(
        Box((0.100, 0.070, 0.080)),
        origin=Origin(xyz=(-0.050, 0.000, 0.000)),
        material=olive,
        name="trail_mount",
    )
    left_trail.visual(
        Box((0.940, 0.060, 0.080)),
        origin=Origin(xyz=(-0.520, 0.000, -0.040)),
        material=olive,
        name="trail_beam",
    )
    left_trail.visual(
        Box((0.100, 0.120, 0.032)),
        origin=Origin(xyz=(-0.955, 0.000, -0.082), rpy=(0.0, math.radians(10.0), 0.0)),
        material=dark_steel,
        name="spade",
    )
    left_trail.inertial = Inertial.from_geometry(
        Box((1.050, 0.120, 0.110)),
        mass=165.0,
        origin=Origin(xyz=(-0.520, 0.000, -0.040)),
    )

    right_trail = model.part("right_trail")
    right_trail.visual(
        Box((0.100, 0.070, 0.080)),
        origin=Origin(xyz=(-0.050, 0.000, 0.000)),
        material=olive,
        name="trail_mount",
    )
    right_trail.visual(
        Box((0.940, 0.060, 0.080)),
        origin=Origin(xyz=(-0.520, 0.000, -0.040)),
        material=olive,
        name="trail_beam",
    )
    right_trail.visual(
        Box((0.100, 0.120, 0.032)),
        origin=Origin(xyz=(-0.955, 0.000, -0.082), rpy=(0.0, math.radians(10.0), 0.0)),
        material=dark_steel,
        name="spade",
    )
    right_trail.inertial = Inertial.from_geometry(
        Box((1.050, 0.120, 0.110)),
        mass=165.0,
        origin=Origin(xyz=(-0.520, 0.000, -0.040)),
    )

    barrel_assembly = model.part("barrel_assembly")
    barrel_assembly.visual(
        Cylinder(radius=0.022, length=0.240),
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_hub",
    )
    barrel_assembly.visual(
        Box((0.260, 0.120, 0.050)),
        origin=Origin(xyz=(0.020, 0.000, -0.020)),
        material=olive,
        name="cradle_beam",
    )
    barrel_assembly.visual(
        Box((0.220, 0.020, 0.090)),
        origin=Origin(xyz=(0.010, 0.090, 0.005)),
        material=olive,
        name="left_cradle_cheek",
    )
    barrel_assembly.visual(
        Box((0.220, 0.020, 0.090)),
        origin=Origin(xyz=(0.010, -0.090, 0.005)),
        material=olive,
        name="right_cradle_cheek",
    )
    barrel_assembly.visual(
        Box((0.260, 0.180, 0.170)),
        origin=Origin(xyz=(-0.170, 0.000, 0.055)),
        material=olive,
        name="breech_housing",
    )
    barrel_assembly.visual(
        Box((0.028, 0.034, 0.150)),
        origin=Origin(xyz=(-0.306, -0.096, 0.055)),
        material=dark_steel,
        name="breech_seat",
    )
    barrel_assembly.visual(
        Cylinder(radius=0.062, length=0.240),
        origin=Origin(xyz=(0.140, 0.000, 0.055), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="recoil_sleeve",
    )
    barrel_assembly.visual(
        Cylinder(radius=0.050, length=1.360),
        origin=Origin(xyz=(0.720, 0.000, 0.055), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=olive,
        name="rifled_barrel",
    )
    barrel_assembly.visual(
        Cylinder(radius=0.060, length=0.160),
        origin=Origin(xyz=(1.330, 0.000, 0.055), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=olive,
        name="muzzle_swell",
    )
    barrel_assembly.visual(
        Cylinder(radius=0.074, length=0.036),
        origin=Origin(xyz=(1.428, 0.000, 0.055), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=dark_steel,
        name="muzzle_brake_ring",
    )
    barrel_assembly.inertial = Inertial.from_geometry(
        Box((1.700, 0.280, 0.260)),
        mass=420.0,
        origin=Origin(xyz=(0.440, 0.000, 0.040)),
    )

    breech_block = model.part("breech_block")
    breech_block.visual(
        Cylinder(radius=0.007, length=0.150),
        material=steel,
        name="side_hinge",
    )
    breech_block.visual(
        Box((0.016, 0.040, 0.150)),
        origin=Origin(xyz=(0.000, 0.020, 0.000)),
        material=dark_steel,
        name="hinge_leaf",
    )
    breech_block.visual(
        Box((0.012, 0.140, 0.150)),
        origin=Origin(xyz=(0.000, 0.110, 0.000)),
        material=dark_steel,
        name="door_panel",
    )
    breech_block.inertial = Inertial.from_geometry(
        Box((0.020, 0.180, 0.150)),
        mass=28.0,
        origin=Origin(xyz=(0.000, 0.090, 0.000)),
    )

    model.articulation(
        "left_wheel_mount",
        ArticulationType.FIXED,
        parent=carriage,
        child="left_wheel",
        origin=Origin(xyz=(-0.080, 0.790, 0.430)),
    )
    model.articulation(
        "right_wheel_mount",
        ArticulationType.FIXED,
        parent=carriage,
        child="right_wheel",
        origin=Origin(xyz=(-0.080, -0.790, 0.430)),
    )
    model.articulation(
        "left_trail_mount",
        ArticulationType.FIXED,
        parent=carriage,
        child=left_trail,
        origin=Origin(xyz=(-0.170, 0.110, 0.390), rpy=(0.0, 0.0, -0.32)),
    )
    model.articulation(
        "right_trail_mount",
        ArticulationType.FIXED,
        parent=carriage,
        child=right_trail,
        origin=Origin(xyz=(-0.170, -0.110, 0.390), rpy=(0.0, 0.0, 0.32)),
    )
    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel_assembly,
        origin=Origin(xyz=(0.160, 0.000, 0.650)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4200.0, velocity=0.8, lower=-0.10, upper=0.30),
    )
    model.articulation(
        "breech_hinge",
        ArticulationType.REVOLUTE,
        parent=barrel_assembly,
        child=breech_block,
        origin=Origin(xyz=(-0.306, -0.096, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=1.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carriage = object_model.get_part("carriage")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    left_trail = object_model.get_part("left_trail")
    right_trail = object_model.get_part("right_trail")
    barrel_assembly = object_model.get_part("barrel_assembly")
    breech_block = object_model.get_part("breech_block")
    barrel_elevation = object_model.get_articulation("barrel_elevation")
    breech_hinge = object_model.get_articulation("breech_hinge")

    left_journal_cap = carriage.get_visual("left_journal_cap")
    right_journal_cap = carriage.get_visual("right_journal_cap")
    rear_axle = carriage.get_visual("rear_axle")
    left_socket = carriage.get_visual("left_trail_socket")
    right_socket = carriage.get_visual("right_trail_socket")
    carriage_body = carriage.get_visual("carriage_body")
    left_trunnion_saddle = carriage.get_visual("left_trunnion_saddle")
    right_trunnion_saddle = carriage.get_visual("right_trunnion_saddle")

    left_hub = left_wheel.get_visual("hub")
    right_hub = right_wheel.get_visual("hub")
    left_mount = left_trail.get_visual("trail_mount")
    right_mount = right_trail.get_visual("trail_mount")
    left_spade = left_trail.get_visual("spade")
    right_spade = right_trail.get_visual("spade")

    trunnion_hub = barrel_assembly.get_visual("trunnion_hub")
    muzzle_brake_ring = barrel_assembly.get_visual("muzzle_brake_ring")
    breech_housing = barrel_assembly.get_visual("breech_housing")
    breech_seat = barrel_assembly.get_visual("breech_seat")

    side_hinge = breech_block.get_visual("side_hinge")
    hinge_leaf = breech_block.get_visual("hinge_leaf")
    door_panel = breech_block.get_visual("door_panel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(barrel_assembly, carriage, reason="trunnion hub nests within the cradle saddles")
    ctx.allow_overlap(breech_block, barrel_assembly, reason="breech hinge sleeve nests on the side hinge pin")
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(left_wheel, right_wheel, axes="xz", max_dist=0.001)
    ctx.expect_contact(left_wheel, carriage, elem_a=left_hub, elem_b=left_journal_cap)
    ctx.expect_contact(right_wheel, carriage, elem_a=right_hub, elem_b=right_journal_cap)
    ctx.expect_contact(left_wheel, carriage, elem_a=left_hub, elem_b=rear_axle)
    ctx.expect_contact(right_wheel, carriage, elem_a=right_hub, elem_b=rear_axle)

    ctx.expect_overlap(left_trail, carriage, axes="xz", min_overlap=0.020, elem_a=left_mount, elem_b=left_socket)
    ctx.expect_overlap(right_trail, carriage, axes="xz", min_overlap=0.020, elem_a=right_mount, elem_b=right_socket)
    ctx.expect_gap(left_trail, right_trail, axis="y", min_gap=0.560, positive_elem=left_spade, negative_elem=right_spade)

    ctx.expect_contact(barrel_assembly, carriage, elem_a=trunnion_hub, elem_b=left_trunnion_saddle)
    ctx.expect_contact(barrel_assembly, carriage, elem_a=trunnion_hub, elem_b=right_trunnion_saddle)
    ctx.expect_gap(
        barrel_assembly,
        carriage,
        axis="x",
        min_gap=1.000,
        positive_elem=muzzle_brake_ring,
        negative_elem=carriage_body,
    )
    ctx.expect_overlap(breech_block, barrel_assembly, axes="yz", min_overlap=0.120, elem_a=door_panel, elem_b=breech_housing)
    ctx.expect_gap(
        barrel_assembly,
        breech_block,
        axis="x",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem=breech_housing,
        negative_elem=door_panel,
    )
    ctx.expect_contact(breech_block, barrel_assembly, elem_a=hinge_leaf, elem_b=breech_seat)

    with ctx.pose({barrel_elevation: 0.25}):
        ctx.expect_contact(barrel_assembly, carriage, elem_a=trunnion_hub, elem_b=left_trunnion_saddle)
        ctx.expect_contact(barrel_assembly, carriage, elem_a=trunnion_hub, elem_b=right_trunnion_saddle)
        ctx.expect_gap(
            barrel_assembly,
            carriage,
            axis="z",
            min_gap=0.080,
            positive_elem=muzzle_brake_ring,
            negative_elem=carriage_body,
        )

    with ctx.pose({breech_hinge: 1.10}):
        ctx.expect_contact(breech_block, barrel_assembly, elem_a=side_hinge, elem_b=breech_seat)
        ctx.expect_gap(
            barrel_assembly,
            breech_block,
            axis="x",
            min_gap=0.020,
            positive_elem=breech_housing,
            negative_elem=door_panel,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
