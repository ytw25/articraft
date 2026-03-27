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
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _regular_polygon(radius: float, sides: int, *, start_angle: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(start_angle + (2.0 * math.pi * index) / sides),
            radius * math.sin(start_angle + (2.0 * math.pi * index) / sides),
        )
        for index in range(sides)
    ]


def _circle_profile(radius: float, *, segments: int = 24) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cast_brass_hose_bib", assets=ASSETS)

    cast_brass = model.material("cast_brass", rgba=(0.70, 0.57, 0.31, 1.0))
    worn_brass = model.material("worn_brass", rgba=(0.62, 0.50, 0.27, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.34, 0.35, 0.37, 1.0))

    spout_pitch = math.radians(12.0)
    spout_axis_rpy = (0.0, math.pi / 2.0 + spout_pitch, 0.0)
    spout_shell_rpy = (0.0, spout_pitch, 0.0)
    spout_root = (0.015, 0.0, -0.013)

    def spout_at(distance: float, *, rise: float = 0.0) -> tuple[float, float, float]:
        return (
            spout_root[0] + distance * math.cos(spout_pitch),
            0.0,
            spout_root[2] - distance * math.sin(spout_pitch) + rise,
        )

    handle_plate_mesh = _save_mesh(
        "hose_bib_handle_plate.obj",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.064, 0.028, radius=0.013, corner_segments=10),
            [_circle_profile(0.0066, segments=24)],
            height=0.0036,
            center=True,
        ),
    )
    bonnet_hex_mesh = _save_mesh(
        "hose_bib_bonnet_hex.obj",
        ExtrudeGeometry(
            _regular_polygon(0.0145, 6, start_angle=math.pi / 6.0),
            height=0.018,
            center=True,
        ),
    )
    inlet_hex_mesh = _save_mesh(
        "hose_bib_inlet_hex.obj",
        ExtrudeGeometry(
            _regular_polygon(0.0175, 6, start_angle=math.pi / 6.0),
            height=0.013,
            center=True,
        ).rotate_y(math.pi / 2.0),
    )
    spout_shell_mesh = _save_mesh(
        "hose_bib_spout_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.0180, -0.0060),
                (0.0172, 0.0000),
                (0.0163, 0.0220),
                (0.0150, 0.0520),
                (0.0143, 0.0730),
            ],
            [
                (0.0105, -0.0010),
                (0.0106, 0.0000),
                (0.0104, 0.0220),
                (0.0100, 0.0520),
                (0.0097, 0.0730),
            ],
            segments=60,
            start_cap="flat",
            end_cap="flat",
        ).rotate_y(math.pi / 2.0),
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=Origin(xyz=(-0.062, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=worn_brass,
        name="wall_flange",
    )
    body.visual(
        Cylinder(radius=0.0105, length=0.050),
        origin=Origin(xyz=(-0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_brass,
        name="inlet_shank",
    )
    body.visual(
        inlet_hex_mesh,
        origin=Origin(xyz=(-0.017, 0.0, 0.0)),
        material=worn_brass,
        name="inlet_hex",
    )
    body.visual(
        Sphere(radius=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=cast_brass,
        name="globe_shell",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.010, 0.0, -0.009), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cast_brass,
        name="spout_waist",
    )
    body.visual(
        Cylinder(radius=0.0145, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=worn_brass,
        name="bonnet_base",
    )
    body.visual(
        bonnet_hex_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=worn_brass,
        name="bonnet_hex",
    )
    body.visual(
        Cylinder(radius=0.0095, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=worn_brass,
        name="packing_nut",
    )
    body.visual(
        Cylinder(radius=0.0058, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=dark_metal,
        name="stem_thread",
    )
    body.visual(
        spout_shell_mesh,
        origin=Origin(xyz=spout_root, rpy=spout_shell_rpy),
        material=cast_brass,
        name="spout_shell",
    )
    body.visual(
        Cylinder(radius=0.0094, length=0.066),
        origin=Origin(xyz=spout_at(0.035), rpy=spout_axis_rpy),
        material=dark_metal,
        name="spout_bore",
    )
    body.visual(
        Cylinder(radius=0.0147, length=0.022),
        origin=Origin(xyz=spout_at(0.063), rpy=spout_axis_rpy),
        material=worn_brass,
        name="thread_nose",
    )
    body.visual(
        Cylinder(radius=0.0092, length=0.014),
        origin=Origin(xyz=spout_at(0.063), rpy=spout_axis_rpy),
        material=dark_metal,
        name="nose_bore",
    )
    for index, distance in enumerate((0.055, 0.0595, 0.064, 0.0685, 0.073), start=1):
        body.visual(
            Cylinder(radius=0.0154, length=0.0018),
            origin=Origin(xyz=spout_at(distance), rpy=spout_axis_rpy),
            material=worn_brass,
            name=f"hose_thread_{index}",
        )
    body.visual(
        Cylinder(radius=0.0105, length=0.004),
        origin=Origin(xyz=spout_at(0.017, rise=0.0145)),
        material=worn_brass,
        name="breaker_seat",
    )
    body.visual(
        Cylinder(radius=0.0072, length=0.009),
        origin=Origin(xyz=spout_at(0.017, rise=0.0185)),
        material=worn_brass,
        name="breaker_boss",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.011, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=worn_brass,
        name="hub",
    )
    handle.visual(
        handle_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0052)),
        material=worn_brass,
        name="oval_plate",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.0088)),
        material=dark_metal,
        name="retaining_nut",
    )

    breaker_cap = model.part("vacuum_breaker_cap")
    breaker_cap.visual(
        Cylinder(radius=0.0103, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=worn_brass,
        name="cap_skirt",
    )
    breaker_cap.visual(
        Cylinder(radius=0.0113, length=0.0036),
        origin=Origin(xyz=(0.0, 0.0, 0.0032)),
        material=worn_brass,
        name="cap_knurl",
    )
    breaker_cap.visual(
        Sphere(radius=0.0082),
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=worn_brass,
        name="cap_top",
    )

    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "body_to_vacuum_breaker_cap",
        ArticulationType.FIXED,
        parent=body,
        child=breaker_cap,
        origin=Origin(xyz=spout_at(0.017, rise=0.0165)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    breaker_cap = object_model.get_part("vacuum_breaker_cap")
    handle_spin = object_model.get_articulation("handle_spin")

    globe_shell = body.get_visual("globe_shell")
    bonnet_hex = body.get_visual("bonnet_hex")
    stem_thread = body.get_visual("stem_thread")
    spout_shell = body.get_visual("spout_shell")
    spout_bore = body.get_visual("spout_bore")
    thread_nose = body.get_visual("thread_nose")
    nose_bore = body.get_visual("nose_bore")
    breaker_seat = body.get_visual("breaker_seat")
    breaker_boss = body.get_visual("breaker_boss")

    handle_hub = handle.get_visual("hub")
    oval_plate = handle.get_visual("oval_plate")

    cap_skirt = breaker_cap.get_visual("cap_skirt")
    cap_top = breaker_cap.get_visual("cap_top")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        breaker_cap,
        body,
        reason="vacuum-breaker cap intentionally nests over the small spout-side breaker boss",
    )
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

    ctx.expect_gap(
        handle,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem=handle_hub,
        negative_elem=stem_thread,
        name="handle hub seats on the stem thread",
    )
    ctx.expect_origin_distance(
        handle,
        body,
        axes="xy",
        max_dist=0.0015,
        name="handle stays centered on the valve stem",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="xy",
        min_overlap=0.015,
        elem_a=oval_plate,
        elem_b=bonnet_hex,
        name="oval handle spans the bonnet footprint",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="z",
        min_gap=0.020,
        positive_elem=oval_plate,
        negative_elem=bonnet_hex,
        name="oval handle sits clearly above the bonnet hardware",
    )
    ctx.expect_gap(
        body,
        body,
        axis="x",
        min_gap=0.032,
        positive_elem=thread_nose,
        negative_elem=globe_shell,
        name="spout projects clearly beyond the globe body",
    )
    ctx.expect_within(
        body,
        body,
        axes="xy",
        inner_elem=bonnet_hex,
        outer_elem=globe_shell,
        name="bonnet hardware stays tucked within the globe body footprint",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="yz",
        min_overlap=0.020,
        elem_a=thread_nose,
        elem_b=spout_shell,
        name="threaded nose continues the spout line",
    )
    ctx.expect_within(
        body,
        body,
        axes="yz",
        inner_elem=nose_bore,
        outer_elem=thread_nose,
        name="outlet bore stays nested inside the threaded nose",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="yz",
        min_overlap=0.016,
        elem_a=nose_bore,
        elem_b=thread_nose,
        name="outlet aperture is centered within the threaded nose",
    )
    ctx.expect_within(
        body,
        breaker_cap,
        axes="xy",
        inner_elem=breaker_boss,
        outer_elem=cap_skirt,
        name="vacuum breaker boss fits inside the cap skirt",
    )
    ctx.expect_contact(
        breaker_cap,
        body,
        elem_a=cap_skirt,
        elem_b=breaker_seat,
    )
    ctx.expect_gap(
        breaker_cap,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem=cap_skirt,
        negative_elem=breaker_seat,
        name="vacuum breaker cap seats on the spout-side shoulder",
    )
    ctx.expect_gap(
        body,
        breaker_cap,
        axis="x",
        min_gap=0.020,
        positive_elem=thread_nose,
        negative_elem=cap_skirt,
        name="hose-thread nose projects ahead of the vacuum breaker cap",
    )
    ctx.expect_gap(
        handle,
        breaker_cap,
        axis="z",
        min_gap=0.045,
        positive_elem=oval_plate,
        negative_elem=cap_top,
        name="handle sits well above the spout-mounted cap",
    )
    with ctx.pose({handle_spin: math.pi / 2.0}):
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0005,
            positive_elem=handle_hub,
            negative_elem=stem_thread,
            name="handle remains seated at quarter turn",
        )
        ctx.expect_gap(
            handle,
            body,
            axis="z",
            min_gap=0.020,
            positive_elem=oval_plate,
            negative_elem=bonnet_hex,
            name="handle clears the bonnet at quarter turn",
        )
        ctx.expect_gap(
            handle,
            breaker_cap,
            axis="z",
            min_gap=0.045,
            positive_elem=oval_plate,
            negative_elem=cap_top,
            name="handle clears the vacuum breaker cap at quarter turn",
        )
    with ctx.pose({handle_spin: math.pi}):
        ctx.expect_origin_distance(
            handle,
            body,
            axes="xy",
            max_dist=0.0015,
            name="handle stays centered at half turn",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
