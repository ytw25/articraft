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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    LatheGeometry,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _xy_section(
    z: float,
    width: float,
    depth: float,
    *,
    x_center: float = 0.0,
    y_center: float = 0.0,
    exponent: float = 2.7,
    segments: int = 44,
):
    return [
        (x_center + px, y_center + py, z)
        for px, py in superellipse_profile(width, depth, exponent=exponent, segments=segments)
    ]


def _yz_section(
    x: float,
    width: float,
    height: float,
    *,
    z_center: float = 0.0,
    y_center: float = 0.0,
    exponent: float = 2.5,
    segments: int = 44,
):
    return [
        (x, y_center + py, z_center + pz)
        for py, pz in superellipse_profile(width, height, exponent=exponent, segments=segments)
    ]


def _rotate_xy(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    ca = math.cos(angle)
    sa = math.sin(angle)
    return (x * ca - y * sa, x * sa + y * ca, z)


def _mesh_path(name: str) -> str:
    return str(ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer", assets=ASSETS)

    enamel = model.material("enamel_cream", rgba=(0.92, 0.88, 0.79, 1.0))
    steel = model.material("brushed_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.86, 0.87, 0.89, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))

    pedestal_sections = [
        _xy_section(0.000, 0.255, 0.182, x_center=0.010, exponent=2.8),
        _xy_section(0.016, 0.272, 0.194, x_center=0.016, exponent=2.9),
        _xy_section(0.036, 0.245, 0.175, x_center=0.010, exponent=2.8),
        _xy_section(0.055, 0.190, 0.145, x_center=0.000, exponent=2.7),
    ]
    pedestal_mesh = mesh_from_geometry(
        repair_loft(section_loft(pedestal_sections)),
        _mesh_path("mixer_pedestal.obj"),
    )

    column_sections = [
        _xy_section(0.055, 0.128, 0.114, x_center=-0.056, exponent=2.6),
        _xy_section(0.110, 0.114, 0.102, x_center=-0.064, exponent=2.5),
        _xy_section(0.178, 0.100, 0.098, x_center=-0.076, exponent=2.4),
        _xy_section(0.236, 0.090, 0.102, x_center=-0.087, exponent=2.4),
        _xy_section(0.276, 0.082, 0.108, x_center=-0.092, exponent=2.5),
    ]
    column_mesh = mesh_from_geometry(
        repair_loft(section_loft(column_sections)),
        _mesh_path("mixer_column.obj"),
    )

    head_sections = [
        _yz_section(-0.004, 0.104, 0.096, z_center=0.066, exponent=2.4),
        _yz_section(0.050, 0.122, 0.122, z_center=0.076, exponent=2.3),
        _yz_section(0.128, 0.122, 0.118, z_center=0.074, exponent=2.3),
        _yz_section(0.205, 0.100, 0.098, z_center=0.066, exponent=2.4),
        _yz_section(0.252, 0.072, 0.078, z_center=0.058, exponent=2.6),
    ]
    head_shell_mesh = mesh_from_geometry(
        repair_loft(section_loft(head_sections)),
        _mesh_path("mixer_head_shell.obj"),
    )

    bowl_profile = [
        (0.000, 0.000),
        (0.053, 0.000),
        (0.063, 0.006),
        (0.083, 0.020),
        (0.102, 0.053),
        (0.114, 0.091),
        (0.118, 0.124),
        (0.120, 0.136),
        (0.111, 0.136),
        (0.106, 0.117),
        (0.098, 0.082),
        (0.087, 0.040),
        (0.071, 0.011),
        (0.055, 0.004),
        (0.041, 0.004),
        (0.000, 0.004),
    ]
    bowl_shell_mesh = mesh_from_geometry(
        LatheGeometry(bowl_profile, segments=56),
        _mesh_path("mixer_bowl_shell.obj"),
    )
    bowl_handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.018, 0.106, 0.036),
                (0.034, 0.145, 0.054),
                (0.036, 0.149, 0.090),
                (0.018, 0.114, 0.117),
            ],
            radius=0.006,
            samples_per_segment=18,
            radial_segments=14,
            cap_ends=True,
        ),
        _mesh_path("mixer_bowl_handle.obj"),
    )

    whisk_points = [
        (0.011, 0.000, -0.010),
        (0.024, 0.000, -0.037),
        (0.028, 0.000, -0.081),
        (0.016, 0.000, -0.132),
        (0.000, 0.000, -0.165),
        (-0.016, 0.000, -0.132),
        (-0.028, 0.000, -0.081),
        (-0.024, 0.000, -0.037),
        (-0.011, 0.000, -0.010),
    ]
    whisk_meshes = []
    for index, angle in enumerate((0.0, 0.65, 1.30, 1.95, 2.60)):
        whisk_meshes.append(
            (
                index,
                mesh_from_geometry(
                    tube_from_spline_points(
                        [_rotate_xy(point, angle) for point in whisk_points],
                        radius=0.0028,
                        samples_per_segment=20,
                        radial_segments=12,
                        cap_ends=True,
                    ),
                    _mesh_path(f"mixer_whisk_loop_{index}.obj"),
                ),
            )
        )

    base = model.part("base")
    base.visual(pedestal_mesh, material=enamel, name="pedestal_shell")
    base.visual(column_mesh, material=enamel, name="column_shell")
    base.visual(
        Cylinder(radius=0.058, length=0.010),
        origin=Origin(xyz=(0.030, 0.000, 0.060)),
        material=enamel,
        name="bowl_plate",
    )
    base.visual(
        Box((0.052, 0.102, 0.012)),
        origin=Origin(xyz=(-0.090, 0.000, 0.279)),
        material=enamel,
        name="top_saddle",
    )
    for foot_index, (x_pos, y_pos) in enumerate(
        ((-0.082, -0.060), (-0.082, 0.060), (0.094, -0.060), (0.094, 0.060))
    ):
        base.visual(
            Cylinder(radius=0.013, length=0.008),
            origin=Origin(xyz=(x_pos, y_pos, 0.004)),
            material=dark_trim,
            name=f"foot_{foot_index}",
        )

    head = model.part("head")
    head.visual(head_shell_mesh, material=enamel, name="head_shell")
    head.visual(
        Box((0.048, 0.090, 0.024)),
        origin=Origin(xyz=(0.018, 0.000, 0.012)),
        material=enamel,
        name="rear_pad",
    )
    head.visual(
        Box((0.150, 0.126, 0.006)),
        origin=Origin(xyz=(0.106, 0.000, 0.052)),
        material=chrome,
        name="head_band",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.255, 0.000, 0.050), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=chrome,
        name="attachment_hub",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.008),
        origin=Origin(xyz=(0.268, 0.000, 0.050), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=chrome,
        name="hub_cap",
    )
    head.visual(
        Box((0.054, 0.040, 0.030)),
        origin=Origin(xyz=(0.120, 0.000, 0.026)),
        material=enamel,
        name="beater_neck",
    )
    head.visual(
        Box((0.030, 0.006, 0.012)),
        origin=Origin(xyz=(0.114, -0.064, 0.078)),
        material=dark_trim,
        name="speed_lever",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.120, 0.000, 0.006)),
        material=chrome,
        name="drive_socket",
    )

    bowl = model.part("bowl")
    bowl.visual(bowl_shell_mesh, material=steel, name="bowl_shell")
    bowl.visual(
        Cylinder(radius=0.053, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=steel,
        name="bowl_foot",
    )
    bowl.visual(bowl_handle_mesh, material=steel, name="bowl_handle")

    beater = model.part("beater")
    beater.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, -0.005)),
        material=chrome,
        name="drive_collar",
    )
    beater.visual(
        Cylinder(radius=0.008, length=0.092),
        origin=Origin(xyz=(0.000, 0.000, -0.046)),
        material=chrome,
        name="shaft",
    )
    for index, whisk_mesh in whisk_meshes:
        beater.visual(whisk_mesh, material=steel, name=f"whisk_loop_{index}")

    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.090, 0.000, 0.285)),
        axis=(0.000, -1.000, 0.000),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5, lower=0.0, upper=0.85),
    )
    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.030, 0.000, 0.065)),
    )
    model.articulation(
        "head_to_beater",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.120, 0.000, 0.000)),
        axis=(0.000, 0.000, -1.000),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    head = object_model.get_part("head")
    bowl = object_model.get_part("bowl")
    beater = object_model.get_part("beater")
    head_tilt = object_model.get_articulation("base_to_head")
    beater_spin = object_model.get_articulation("head_to_beater")

    bowl_plate = base.get_visual("bowl_plate")
    top_saddle = base.get_visual("top_saddle")
    rear_pad = head.get_visual("rear_pad")
    attachment_hub = head.get_visual("attachment_hub")
    hub_cap = head.get_visual("hub_cap")
    drive_socket = head.get_visual("drive_socket")
    bowl_foot = bowl.get_visual("bowl_foot")
    drive_collar = beater.get_visual("drive_collar")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(head, beater, reason="the whisk collar nests into the shallow planetary drive socket")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_coplanar_surfaces(ignore_adjacent=True, ignore_fixed=True)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(bowl, base, elem_a=bowl_foot, elem_b=bowl_plate)
    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem=bowl_foot,
        negative_elem=bowl_plate,
        name="mixing bowl sits down on the pedestal plate",
    )
    ctx.expect_overlap(bowl, base, axes="xy", min_overlap=0.090)

    ctx.expect_contact(head, base, elem_a=rear_pad, elem_b=top_saddle)
    ctx.expect_gap(
        head,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem=rear_pad,
        negative_elem=top_saddle,
        name="tilt head rests on the rear hinge saddle",
    )
    ctx.expect_origin_distance(beater, bowl, axes="xy", max_dist=0.015)
    ctx.expect_within(beater, bowl, axes="xy")
    ctx.expect_contact(beater, head, elem_a=drive_collar, elem_b=drive_socket)
    ctx.expect_gap(
        head,
        bowl,
        axis="x",
        min_gap=0.012,
        positive_elem=hub_cap,
        name="front accessory hub projects clearly ahead of the bowl",
    )

    with ctx.pose({beater_spin: math.pi / 2.0}):
        ctx.expect_origin_distance(beater, bowl, axes="xy", max_dist=0.015)
        ctx.expect_within(beater, bowl, axes="xy")

    with ctx.pose({head_tilt: 0.75}):
        ctx.expect_gap(
            beater,
            bowl,
            axis="z",
            min_gap=0.040,
            name="raised head lifts the whisk above the bowl rim",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
