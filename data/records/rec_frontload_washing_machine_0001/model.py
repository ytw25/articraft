from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_h = height / 2.0
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _circle_profile(
    radius: float,
    segments: int = 48,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    mesh_dir = HERE / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    model = ArticulatedObject(name="front_load_washing_machine", assets=ASSETS)

    enamel_white = model.material("enamel_white", rgba=(0.94, 0.95, 0.96, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    knob_silver = model.material("knob_silver", rgba=(0.74, 0.75, 0.77, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.62, 0.73, 0.81, 0.28))
    black_glass = model.material("black_glass", rgba=(0.07, 0.08, 0.09, 0.92))
    rubber_gray = model.material("rubber_gray", rgba=(0.24, 0.25, 0.27, 1.0))

    depth = 0.67
    width = 0.60
    height = 0.85
    shell_t = 0.022
    back_t = 0.018
    bottom_t = 0.022
    front_panel_t = 0.018
    fascia_t = 0.028
    kick_h = 0.075
    control_h = 0.145
    front_opening_center_z = 0.42
    opening_radius = 0.170
    bezel_radius = 0.194

    mid_panel_h = height - control_h - kick_h
    panel_center_z = kick_h + mid_panel_h / 2.0

    front_panel_geom = ExtrudeWithHolesGeometry(
        outer_profile=_rect_profile(width - 2.0 * shell_t, mid_panel_h),
        hole_profiles=[
            _circle_profile(
                opening_radius,
                segments=56,
                center=(0.0, front_opening_center_z - panel_center_z),
            )
        ],
        height=front_panel_t,
        center=True,
    )
    front_panel_geom.rotate_z(math.pi / 2.0).rotate_y(math.pi / 2.0)
    front_panel_mesh = mesh_from_geometry(
        front_panel_geom,
        mesh_dir / "washer_front_panel.obj",
    )

    cabinet_bezel_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=bezel_radius,
            tube=0.018,
            radial_segments=22,
            tubular_segments=52,
        ),
        mesh_dir / "washer_cabinet_bezel.obj",
    )
    door_frame_geom = ExtrudeWithHolesGeometry(
        outer_profile=_circle_profile(0.214, segments=60, center=(0.0, -0.194)),
        hole_profiles=[_circle_profile(0.148, segments=56, center=(0.0, -0.194))],
        height=0.030,
        center=True,
    )
    door_frame_geom.rotate_z(math.pi / 2.0).rotate_y(math.pi / 2.0)
    door_frame_mesh = mesh_from_geometry(
        door_frame_geom,
        mesh_dir / "washer_door_frame.obj",
    )
    door_outer_ring_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.190,
            tube=0.022,
            radial_segments=22,
            tubular_segments=52,
        ),
        mesh_dir / "washer_door_outer_ring.obj",
    )
    door_inner_ring_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.156,
            tube=0.012,
            radial_segments=20,
            tubular_segments=48,
        ),
        mesh_dir / "washer_door_inner_ring.obj",
    )
    drum_boot_mesh = mesh_from_geometry(
        TorusGeometry(
            radius=0.165,
            tube=0.024,
            radial_segments=20,
            tubular_segments=48,
        ),
        mesh_dir / "washer_drum_boot.obj",
    )
    knob_mesh = mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, 0.000),
                (0.018, 0.000),
                (0.026, 0.003),
                (0.032, 0.010),
                (0.034, 0.020),
                (0.034, 0.028),
                (0.028, 0.034),
                (0.020, 0.038),
                (0.0, 0.038),
            ],
            segments=56,
        ),
        mesh_dir / "washer_selector_knob.obj",
    )

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((depth, shell_t, height)),
        origin=Origin(xyz=(0.0, width / 2.0 - shell_t / 2.0, height / 2.0)),
        material=enamel_white,
    )
    cabinet.visual(
        Box((depth, shell_t, height)),
        origin=Origin(xyz=(0.0, -width / 2.0 + shell_t / 2.0, height / 2.0)),
        material=enamel_white,
    )
    cabinet.visual(
        Box((depth, width - 2.0 * shell_t, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, height - shell_t / 2.0)),
        material=enamel_white,
    )
    cabinet.visual(
        Box((depth, width - 2.0 * shell_t, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=enamel_white,
    )
    cabinet.visual(
        Box((back_t, width - 2.0 * shell_t, height - 2.0 * shell_t)),
        origin=Origin(xyz=(-depth / 2.0 + back_t / 2.0, 0.0, height / 2.0)),
        material=enamel_white,
    )
    cabinet.visual(
        Box((fascia_t, width - 2.0 * shell_t, control_h)),
        origin=Origin(
            xyz=(depth / 2.0 - fascia_t / 2.0, 0.0, height - control_h / 2.0),
        ),
        material=enamel_white,
    )
    cabinet.visual(
        Box((fascia_t, width - 2.0 * shell_t, kick_h)),
        origin=Origin(xyz=(depth / 2.0 - fascia_t / 2.0, 0.0, kick_h / 2.0)),
        material=enamel_white,
    )
    cabinet.visual(
        front_panel_mesh,
        origin=Origin(xyz=(depth / 2.0 - front_panel_t / 2.0, 0.0, panel_center_z)),
        material=enamel_white,
    )
    cabinet.visual(
        cabinet_bezel_mesh,
        origin=Origin(
            xyz=(depth / 2.0 - 0.006, 0.0, front_opening_center_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_steel,
    )
    cabinet.visual(
        Box((0.006, 0.165, 0.050)),
        origin=Origin(xyz=(depth / 2.0 + 0.003, 0.025, 0.774)),
        material=black_glass,
    )
    for button_y in (0.065, 0.025, -0.015):
        cabinet.visual(
            Box((0.010, 0.026, 0.012)),
            origin=Origin(xyz=(depth / 2.0 + 0.002, button_y, 0.735)),
            material=dark_plastic,
        )
    for foot_x in (-0.250, 0.250):
        for foot_y in (-0.235, 0.235):
            cabinet.visual(
                Cylinder(radius=0.020, length=0.018),
                origin=Origin(xyz=(foot_x, foot_y, 0.009)),
                material=rubber_gray,
            )
    cabinet.inertial = Inertial.from_geometry(
        Box((depth, width, height)),
        mass=54.0,
        origin=Origin(xyz=(0.0, 0.0, height / 2.0)),
    )

    drum = model.part("drum_assembly")
    drum.visual(
        drum_boot_mesh,
        origin=Origin(
            xyz=(-0.012, 0.0, 0.175),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber_gray,
    )
    drum.visual(
        Cylinder(radius=0.168, length=0.022),
        origin=Origin(
            xyz=(-0.035, 0.0, 0.175),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_steel,
    )
    drum.visual(
        Cylinder(radius=0.156, length=0.270),
        origin=Origin(
            xyz=(-0.160, 0.0, 0.175),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_steel,
    )
    drum.visual(
        Cylinder(radius=0.158, length=0.020),
        origin=Origin(
            xyz=(-0.305, 0.0, 0.175),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_plastic,
    )
    for rib_angle in (-0.9, 0.2, 1.3):
        drum.visual(
            Box((0.190, 0.016, 0.034)),
            origin=Origin(
                xyz=(
                    -0.150,
                    0.110 * math.cos(rib_angle),
                    0.175 + 0.110 * math.sin(rib_angle),
                ),
                rpy=(0.0, 0.0, rib_angle),
            ),
            material=dark_plastic,
        )
    drum.inertial = Inertial.from_geometry(
        Cylinder(radius=0.180, length=0.320),
        mass=9.0,
        origin=Origin(
            xyz=(-0.160, 0.0, 0.175),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    door = model.part("door")
    door.visual(
        Box((0.014, 0.055, 0.240)),
        origin=Origin(xyz=(0.007, -0.010, 0.0)),
        material=dark_plastic,
    )
    door.visual(
        door_frame_mesh,
        origin=Origin(
            xyz=(0.029, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=dark_plastic,
    )
    door.visual(
        door_outer_ring_mesh,
        origin=Origin(
            xyz=(0.049, -0.194, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=satin_steel,
    )
    door.visual(
        door_inner_ring_mesh,
        origin=Origin(
            xyz=(0.034, -0.194, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_plastic,
    )
    door.visual(
        Cylinder(radius=0.145, length=0.014),
        origin=Origin(
            xyz=(0.038, -0.194, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=smoked_glass,
    )
    door.visual(
        Box((0.034, 0.050, 0.128)),
        origin=Origin(xyz=(0.050, -0.356, 0.0)),
        material=dark_plastic,
    )
    door.visual(
        Box((0.024, 0.030, 0.090)),
        origin=Origin(xyz=(0.060, -0.334, 0.0)),
        material=dark_plastic,
    )
    door.inertial = Inertial.from_geometry(
        Cylinder(radius=0.215, length=0.060),
        mass=3.5,
        origin=Origin(
            xyz=(0.047, -0.194, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(
            xyz=(0.006, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_plastic,
    )
    selector_knob.visual(
        knob_mesh,
        origin=Origin(
            xyz=(0.012, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=knob_silver,
    )
    selector_knob.visual(
        Box((0.008, 0.004, 0.010)),
        origin=Origin(xyz=(0.036, 0.0, 0.028)),
        material=dark_plastic,
    )
    selector_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.040),
        mass=0.25,
        origin=Origin(
            xyz=(0.018, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    detergent_drawer = model.part("detergent_drawer")
    detergent_drawer.visual(
        Box((0.028, 0.198, 0.094)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=enamel_white,
    )
    detergent_drawer.visual(
        Box((0.240, 0.182, 0.072)),
        origin=Origin(xyz=(-0.118, 0.0, -0.004)),
        material=enamel_white,
    )
    detergent_drawer.visual(
        Box((0.012, 0.092, 0.012)),
        origin=Origin(xyz=(0.030, 0.0, 0.006)),
        material=dark_plastic,
    )
    detergent_drawer.inertial = Inertial.from_geometry(
        Box((0.250, 0.200, 0.100)),
        mass=0.8,
        origin=Origin(xyz=(-0.105, 0.0, 0.0)),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(
            xyz=(0.005, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_plastic,
    )
    power_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.010),
        mass=0.04,
        origin=Origin(
            xyz=(0.005, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    model.articulation(
        "drum_mount",
        ArticulationType.FIXED,
        parent="cabinet",
        child="drum_assembly",
        origin=Origin(
            xyz=(depth / 2.0 - 0.028, 0.0, front_opening_center_z - opening_radius),
        ),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent="cabinet",
        child="door",
        origin=Origin(xyz=(depth / 2.0 - 0.006, bezel_radius, front_opening_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "selector_rotation",
        ArticulationType.REVOLUTE,
        parent="cabinet",
        child="selector_knob",
        origin=Origin(xyz=(depth / 2.0, -0.102, 0.755)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.0,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent="cabinet",
        child="detergent_drawer",
        origin=Origin(xyz=(depth / 2.0, 0.165, 0.758)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.45,
            lower=0.0,
            upper=0.180,
        ),
    )
    model.articulation(
        "power_button_press",
        ArticulationType.PRISMATIC,
        parent="cabinet",
        child="power_button",
        origin=Origin(xyz=(depth / 2.0, -0.235, 0.756)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.05,
            lower=-0.004,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")
    ctx.check_no_overlaps(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("drum_assembly", "cabinet")
    ctx.expect_aabb_overlap("drum_assembly", "cabinet", axes="yz", min_overlap=0.32)
    ctx.expect_aabb_gap("door", "drum_assembly", axis="x", max_gap=0.03, max_penetration=0.0)
    ctx.expect_aabb_overlap("door", "cabinet", axes="yz", min_overlap=0.38)
    ctx.expect_aabb_overlap("door", "drum_assembly", axes="yz", min_overlap=0.30)
    ctx.expect_aabb_contact("selector_knob", "cabinet")
    ctx.expect_aabb_overlap("selector_knob", "cabinet", axes="yz", min_overlap=0.06)
    ctx.expect_aabb_contact("detergent_drawer", "cabinet")
    ctx.expect_aabb_overlap("detergent_drawer", "cabinet", axes="yz", min_overlap=0.09)
    ctx.expect_aabb_contact("power_button", "cabinet")
    ctx.expect_aabb_overlap("power_button", "cabinet", axes="yz", min_overlap=0.02)
    ctx.expect_joint_motion_axis(
        "door_hinge",
        "door",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "drawer_slide",
        "detergent_drawer",
        world_axis="x",
        direction="positive",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "power_button_press",
        "power_button",
        world_axis="x",
        direction="positive",
        min_delta=0.002,
    )

    with ctx.pose(selector_rotation=-2.2):
        ctx.expect_aabb_contact("selector_knob", "cabinet")
        ctx.expect_aabb_overlap("selector_knob", "cabinet", axes="yz", min_overlap=0.06)
    with ctx.pose(selector_rotation=2.2):
        ctx.expect_aabb_contact("selector_knob", "cabinet")
        ctx.expect_aabb_overlap("selector_knob", "cabinet", axes="yz", min_overlap=0.06)
    with ctx.pose(drawer_slide=0.180):
        ctx.expect_aabb_contact("detergent_drawer", "cabinet")
        ctx.expect_aabb_overlap(
            "detergent_drawer",
            "cabinet",
            axes="yz",
            min_overlap=0.09,
        )
    with ctx.pose(door_hinge=1.45):
        ctx.expect_aabb_overlap("door", "cabinet", axes="z", min_overlap=0.38)
    with ctx.pose(power_button_press=-0.004):
        ctx.expect_aabb_contact("power_button", "cabinet")
        ctx.expect_aabb_overlap("power_button", "cabinet", axes="yz", min_overlap=0.02)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
