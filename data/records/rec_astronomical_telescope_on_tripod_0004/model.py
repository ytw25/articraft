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
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rocker_side_profile() -> list[tuple[float, float]]:
    notch_center_z = 0.255
    notch_radius = 0.128
    arc_angles_deg = [35, 5, -25, -55, -85, -115, -145, -175, -205, -215]
    arc_points = [
        (
            notch_radius * math.cos(math.radians(angle)),
            notch_center_z + notch_radius * math.sin(math.radians(angle)),
        )
        for angle in arc_angles_deg
    ]
    return [
        (-0.175, 0.0),
        (0.175, 0.0),
        (0.175, 0.175),
        (0.155, 0.215),
        (0.122, 0.278),
        *arc_points,
        (-0.122, 0.278),
        (-0.155, 0.215),
        (-0.175, 0.175),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dobsonian_reflector", assets=ASSETS)

    birch = model.material("birch_plywood", rgba=(0.72, 0.59, 0.41, 1.0))
    darker_birch = model.material("darker_birch", rgba=(0.55, 0.42, 0.28, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.12, 0.12, 0.13, 1.0))
    tube_white = model.material("tube_white", rgba=(0.93, 0.94, 0.95, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.65, 1.0))
    pad_white = model.material("pad_white", rgba=(0.92, 0.92, 0.88, 1.0))

    rocker_side_mesh = _save_mesh(
        "rocker_side_board.obj",
        ExtrudeGeometry.centered(_rocker_side_profile(), height=0.018, cap=True, closed=True),
    )
    rocker_floor_mesh = _save_mesh(
        "rocker_floor_board.obj",
        ExtrudeWithHolesGeometry(
            [(-0.170, -0.184), (0.170, -0.184), (0.170, 0.184), (-0.170, 0.184)],
            [[(-0.118, -0.138), (0.118, -0.138), (0.118, 0.138), (-0.118, 0.138)]],
            height=0.018,
            cap=True,
            center=True,
            closed=True,
        ),
    )
    tube_shell_mesh = _save_mesh(
        "optical_tube_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.154, -0.265),
                (0.156, -0.175),
                (0.157, -0.020),
                (0.156, 0.170),
                (0.159, 0.275),
            ],
            [
                (0.0, -0.265),
                (0.142, -0.252),
                (0.145, -0.155),
                (0.146, -0.015),
                (0.145, 0.170),
                (0.146, 0.266),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    ground_base = model.part("ground_base")
    ground_base.visual(
        Box((0.620, 0.620, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=birch,
        name="ground_board",
    )
    ground_base.visual(
        Cylinder(radius=0.182, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=bearing_black,
        name="azimuth_pad",
    )
    ground_base.visual(
        Box((0.100, 0.100, 0.010)),
        origin=Origin(xyz=(0.210, 0.210, 0.005)),
        material=darker_birch,
        name="foot_front_left",
    )
    ground_base.visual(
        Box((0.100, 0.100, 0.010)),
        origin=Origin(xyz=(0.210, -0.210, 0.005)),
        material=darker_birch,
        name="foot_front_right",
    )
    ground_base.visual(
        Box((0.100, 0.100, 0.010)),
        origin=Origin(xyz=(-0.210, 0.0, 0.005)),
        material=darker_birch,
        name="foot_rear",
    )
    ground_base.inertial = Inertial.from_geometry(
        Box((0.620, 0.620, 0.038)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
    )

    rocker_box = model.part("rocker_box")
    rocker_box.visual(
        Cylinder(radius=0.170, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=bearing_black,
        name="azimuth_plate",
    )
    rocker_box.visual(
        rocker_floor_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=birch,
        name="floor_board",
    )
    rocker_box.visual(
        Box((0.012, 0.368, 0.032)),
        origin=Origin(xyz=(0.170, 0.0, 0.042)),
        material=birch,
        name="front_board",
    )
    rocker_box.visual(
        Box((0.012, 0.368, 0.032)),
        origin=Origin(xyz=(-0.170, 0.0, 0.042)),
        material=birch,
        name="rear_board",
    )
    rocker_box.visual(
        rocker_side_mesh,
        origin=Origin(xyz=(0.0, 0.193, 0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=birch,
        name="left_side_board",
    )
    rocker_box.visual(
        rocker_side_mesh,
        origin=Origin(xyz=(0.0, -0.193, 0.026), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=birch,
        name="right_side_board",
    )
    rocker_box.visual(
        Box((0.052, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, 0.179, 0.281)),
        material=pad_white,
        name="left_bearing_pad",
    )
    rocker_box.visual(
        Box((0.052, 0.010, 0.038)),
        origin=Origin(xyz=(0.0, -0.179, 0.281)),
        material=pad_white,
        name="right_bearing_pad",
    )
    rocker_box.visual(
        Box((0.060, 0.368, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=darker_birch,
        name="pivot_bridge",
    )
    rocker_box.inertial = Inertial.from_geometry(
        Box((0.380, 0.402, 0.360)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )

    tube_shell_short_mesh = _save_mesh(
        "optical_tube_shell_short.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.154, -0.225),
                (0.156, -0.145),
                (0.157, -0.015),
                (0.156, 0.145),
                (0.159, 0.235),
            ],
            [
                (0.0, -0.225),
                (0.142, -0.214),
                (0.145, -0.135),
                (0.146, -0.010),
                (0.145, 0.145),
                (0.146, 0.226),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    tube_cradle = model.part("tube_cradle")
    tube_cradle.visual(
        tube_shell_short_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_white,
        name="tube_shell",
    )
    tube_cradle.visual(
        Cylinder(radius=0.140, length=0.012),
        origin=Origin(xyz=(-0.210, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="mirror_cell",
    )
    tube_cradle.visual(
        Box((0.080, 0.030, 0.130)),
        origin=Origin(xyz=(-0.010, 0.154, 0.0)),
        material=satin_black,
        name="left_saddle",
    )
    tube_cradle.visual(
        Box((0.080, 0.030, 0.130)),
        origin=Origin(xyz=(-0.010, -0.154, 0.0)),
        material=satin_black,
        name="right_saddle",
    )
    tube_cradle.visual(
        Cylinder(radius=0.112, length=0.006),
        origin=Origin(xyz=(0.0, 0.171, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_black,
        name="left_trunnion",
    )
    tube_cradle.visual(
        Cylinder(radius=0.112, length=0.006),
        origin=Origin(xyz=(0.0, -0.171, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bearing_black,
        name="right_trunnion",
    )
    tube_cradle.visual(
        Box((0.070, 0.036, 0.028)),
        origin=Origin(xyz=(0.108, 0.106, 0.117), rpy=(0.0, 0.0, 0.65)),
        material=satin_black,
        name="focuser_base",
    )
    tube_cradle.visual(
        Cylinder(radius=0.018, length=0.082),
        origin=Origin(xyz=(0.128, 0.125, 0.143), rpy=(0.85, 0.0, 0.65)),
        material=steel,
        name="eyepiece_tube",
    )
    tube_cradle.visual(
        Box((0.048, 0.020, 0.018)),
        origin=Origin(xyz=(0.152, -0.094, 0.118), rpy=(0.0, 0.0, -0.32)),
        material=steel,
        name="finder_shoe",
    )
    tube_cradle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.460),
        mass=6.2,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "azimuth_swivel",
        ArticulationType.CONTINUOUS,
        parent=ground_base,
        child=rocker_box,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.2),
    )
    model.articulation(
        "altitude_rock",
        ArticulationType.REVOLUTE,
        parent=rocker_box,
        child=tube_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.281)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-0.15,
            upper=0.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ground_base = object_model.get_part("ground_base")
    rocker_box = object_model.get_part("rocker_box")
    tube_cradle = object_model.get_part("tube_cradle")
    azimuth_swivel = object_model.get_articulation("azimuth_swivel")
    altitude_rock = object_model.get_articulation("altitude_rock")

    azimuth_pad = ground_base.get_visual("azimuth_pad")
    azimuth_plate = rocker_box.get_visual("azimuth_plate")
    floor_board = rocker_box.get_visual("floor_board")
    left_bearing_pad = rocker_box.get_visual("left_bearing_pad")
    right_bearing_pad = rocker_box.get_visual("right_bearing_pad")
    front_board = rocker_box.get_visual("front_board")
    rear_board = rocker_box.get_visual("rear_board")
    left_side_board = rocker_box.get_visual("left_side_board")
    right_side_board = rocker_box.get_visual("right_side_board")
    tube_shell = tube_cradle.get_visual("tube_shell")
    left_trunnion = tube_cradle.get_visual("left_trunnion")
    right_trunnion = tube_cradle.get_visual("right_trunnion")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # The altitude axis on a Dobsonian is a virtual line through the two side trunnions,
    # so the joint origin is intentionally centered in open space rather than near one
    # local solid feature. The exact seating checks below are the meaningful proof.
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(rocker_box, ground_base, axes="xy", min_overlap=0.10)
    ctx.expect_gap(
        rocker_box,
        ground_base,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=azimuth_plate,
        negative_elem=azimuth_pad,
        name="rocker rides on azimuth pad",
    )

    ctx.expect_within(
        tube_cradle,
        rocker_box,
        axes="y",
        inner_elem=tube_shell,
        name="tube shell stays between rocker side boards",
    )
    ctx.expect_gap(
        rocker_box,
        tube_cradle,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=left_bearing_pad,
        negative_elem=left_trunnion,
        name="left trunnion sits on left bearing pad",
    )
    ctx.expect_gap(
        tube_cradle,
        rocker_box,
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=right_trunnion,
        negative_elem=right_bearing_pad,
        name="right trunnion sits on right bearing pad",
    )
    ctx.expect_gap(
        tube_cradle,
        rocker_box,
        axis="z",
        min_gap=0.050,
        positive_elem=tube_shell,
        negative_elem=floor_board,
        name="tube clears rocker floor in rest pose",
    )
    ctx.expect_gap(
        rocker_box,
        tube_cradle,
        axis="y",
        min_gap=0.006,
        max_gap=0.016,
        positive_elem=left_side_board,
        negative_elem=left_trunnion,
        name="left trunnion runs just inside left side board",
    )
    ctx.expect_gap(
        tube_cradle,
        rocker_box,
        axis="y",
        min_gap=0.006,
        max_gap=0.016,
        positive_elem=right_trunnion,
        negative_elem=right_side_board,
        name="right trunnion runs just inside right side board",
    )

    with ctx.pose({azimuth_swivel: 1.1}):
        ctx.expect_overlap(rocker_box, ground_base, axes="xy", min_overlap=0.10)
        ctx.expect_gap(
            rocker_box,
            ground_base,
            axis="z",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem=azimuth_plate,
            negative_elem=azimuth_pad,
            name="azimuth pad remains seated while turned",
        )

    with ctx.pose({altitude_rock: 0.28}):
        ctx.expect_gap(
            rocker_box,
            tube_cradle,
            axis="y",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem=left_bearing_pad,
            negative_elem=left_trunnion,
            name="left trunnion remains seated while aimed upward",
        )
        ctx.expect_gap(
            tube_cradle,
            rocker_box,
            axis="y",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem=right_trunnion,
            negative_elem=right_bearing_pad,
            name="right trunnion remains seated while aimed upward",
        )
        ctx.expect_gap(
            rocker_box,
            tube_cradle,
            axis="y",
            min_gap=0.006,
            max_gap=0.016,
            positive_elem=left_side_board,
            negative_elem=left_trunnion,
            name="left trunnion stays nested in left side board while aimed upward",
        )
        ctx.expect_gap(
            tube_cradle,
            rocker_box,
            axis="y",
            min_gap=0.006,
            max_gap=0.016,
            positive_elem=right_trunnion,
            negative_elem=right_side_board,
            name="right trunnion stays nested in right side board while aimed upward",
        )
        ctx.expect_gap(
            tube_cradle,
            rocker_box,
            axis="z",
            min_gap=0.004,
            positive_elem=tube_shell,
            negative_elem=rear_board,
            name="tube clears rear rocker brace while aimed upward",
        )

    with ctx.pose({altitude_rock: -0.16}):
        ctx.expect_gap(
            tube_cradle,
            rocker_box,
            axis="z",
            min_gap=0.006,
            positive_elem=tube_shell,
            negative_elem=floor_board,
            name="tube still clears rocker floor nose-down",
        )
        ctx.expect_gap(
            tube_cradle,
            rocker_box,
            axis="z",
            min_gap=0.020,
            positive_elem=tube_shell,
            negative_elem=front_board,
            name="tube clears front rocker brace nose-down",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
