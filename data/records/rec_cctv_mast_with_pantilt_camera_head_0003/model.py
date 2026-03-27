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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _annular_band_mesh(filename: str, outer_radius: float, inner_radius: float, height: float):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (outer_radius, -0.5 * height),
                (outer_radius, 0.5 * height),
            ],
            [
                (inner_radius, -0.5 * height),
                (inner_radius, 0.5 * height),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
        ASSETS.mesh_path(filename),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_mount_dome_head", assets=ASSETS)

    galvanized = model.material("galvanized", rgba=(0.63, 0.66, 0.69, 1.0))
    painted_gray = model.material("painted_gray", rgba=(0.43, 0.46, 0.50, 1.0))
    smoked_dome = model.material("smoked_dome", rgba=(0.36, 0.43, 0.47, 0.38))
    sensor_black = model.material("sensor_black", rgba=(0.08, 0.09, 0.10, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.12, 0.18, 0.24, 0.60))

    dome_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.056, 0.000),
                (0.055, 0.010),
                (0.050, 0.027),
                (0.040, 0.046),
                (0.022, 0.061),
                (0.000, 0.068),
            ],
            [
                (0.046, 0.004),
                (0.045, 0.013),
                (0.041, 0.028),
                (0.033, 0.045),
                (0.018, 0.059),
                (0.000, 0.064),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        ASSETS.mesh_path("dome_shell.obj"),
    )
    bearing_seat_mesh = _annular_band_mesh("bearing_seat.obj", 0.024, 0.020, 0.012)
    turret_ring_mesh = _annular_band_mesh("turret_ring.obj", 0.029, 0.020, 0.012)
    dome_skirt_mesh = _annular_band_mesh("dome_skirt.obj", 0.046, 0.028, 0.012)
    dome_flange_mesh = _annular_band_mesh("dome_flange.obj", 0.056, 0.044, 0.006)

    mount = model.part("mount")
    mount.visual(
        Box((0.110, 0.110, 0.008)),
        origin=Origin(xyz=(0.055, 0.055, 0.004)),
        material=galvanized,
        name="roof_plate",
    )
    mount.visual(
        Box((0.006, 0.110, 0.070)),
        origin=Origin(xyz=(0.003, 0.055, -0.035)),
        material=galvanized,
        name="wall_plate_x",
    )
    mount.visual(
        Box((0.110, 0.006, 0.070)),
        origin=Origin(xyz=(0.055, 0.003, -0.035)),
        material=galvanized,
        name="wall_plate_y",
    )
    mount.visual(
        Box((0.034, 0.034, 0.090)),
        origin=Origin(xyz=(0.036, 0.036, 0.053)),
        material=painted_gray,
        name="post_shaft",
    )
    mount.visual(
        Box((0.046, 0.046, 0.008)),
        origin=Origin(xyz=(0.036, 0.036, 0.102)),
        material=painted_gray,
        name="post_cap",
    )
    mount.visual(
        bearing_seat_mesh,
        origin=Origin(xyz=(0.036, 0.036, 0.112)),
        material=painted_gray,
        name="bearing_seat",
    )
    mount.inertial = Inertial.from_geometry(
        Box((0.110, 0.110, 0.188)),
        mass=3.6,
        origin=Origin(xyz=(0.055, 0.055, 0.024)),
    )

    turret = model.part("turret")
    turret.visual(
        turret_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=painted_gray,
        name="turret_ring",
    )
    turret.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=painted_gray,
        name="turret_hub",
    )
    turret.visual(
        dome_skirt_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=painted_gray,
        name="dome_skirt",
    )
    turret.visual(
        dome_flange_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=painted_gray,
        name="dome_flange",
    )
    turret.visual(
        dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=smoked_dome,
        name="dome_shell",
    )
    turret.visual(
        Box((0.006, 0.010, 0.028)),
        origin=Origin(xyz=(0.017, 0.0, 0.026)),
        material=painted_gray,
        name="lug_right",
    )
    turret.visual(
        Box((0.006, 0.010, 0.028)),
        origin=Origin(xyz=(-0.017, 0.0, 0.026)),
        material=painted_gray,
        name="lug_left",
    )
    turret.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.090),
        mass=0.75,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_gray,
        name="tilt_pin",
    )
    cradle.visual(
        Box((0.006, 0.008, 0.024)),
        origin=Origin(xyz=(-0.008, 0.0, -0.012)),
        material=painted_gray,
        name="hanger_left",
    )
    cradle.visual(
        Box((0.006, 0.008, 0.024)),
        origin=Origin(xyz=(0.008, 0.0, -0.012)),
        material=painted_gray,
        name="hanger_right",
    )
    cradle.visual(
        Box((0.020, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=painted_gray,
        name="bridge",
    )
    cradle.visual(
        Box((0.008, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=painted_gray,
        name="neck",
    )
    cradle.visual(
        Box((0.018, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=sensor_black,
        name="payload_body",
    )
    cradle.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(0.0, 0.008, -0.021), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="lens_barrel",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.022, 0.016, 0.036)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
    )

    model.articulation(
        "azimuth",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=turret,
        origin=Origin(xyz=(0.036, 0.036, 0.118)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=turret,
        child=cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.05,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    mount = object_model.get_part("mount")
    turret = object_model.get_part("turret")
    cradle = object_model.get_part("cradle")
    azimuth = object_model.get_articulation("azimuth")
    tilt = object_model.get_articulation("tilt")
    bearing_seat = mount.get_visual("bearing_seat")
    post_cap = mount.get_visual("post_cap")
    turret_ring = turret.get_visual("turret_ring")
    turret_hub = turret.get_visual("turret_hub")
    dome_flange = turret.get_visual("dome_flange")
    dome_shell = turret.get_visual("dome_shell")
    lug_left = turret.get_visual("lug_left")
    lug_right = turret.get_visual("lug_right")
    tilt_pin = cradle.get_visual("tilt_pin")
    payload_body = cradle.get_visual("payload_body")
    lens_barrel = cradle.get_visual("lens_barrel")

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

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    ctx.expect_contact(
        turret,
        mount,
        elem_a=turret_ring,
        elem_b=bearing_seat,
        name="turret_ring_seats_on_bearing",
    )
    ctx.expect_overlap(
        turret,
        mount,
        axes="xy",
        min_overlap=0.020,
        elem_a=turret_ring,
        elem_b=bearing_seat,
        name="turret_ring_centered_on_bearing",
    )
    ctx.expect_within(
        turret,
        mount,
        axes="xy",
        inner_elem=turret_hub,
        outer_elem=bearing_seat,
        name="turret_hub_stays_within_bearing_seat_footprint",
    )
    ctx.expect_gap(
        turret,
        mount,
        axis="z",
        min_gap=0.020,
        positive_elem=dome_flange,
        negative_elem=post_cap,
        name="dome_flange_stands_above_post_cap",
    )
    ctx.expect_contact(
        cradle,
        turret,
        elem_a=tilt_pin,
        elem_b=lug_left,
        name="tilt_pin_contacts_left_lug",
    )
    ctx.expect_contact(
        cradle,
        turret,
        elem_a=tilt_pin,
        elem_b=lug_right,
        name="tilt_pin_contacts_right_lug",
    )
    ctx.expect_within(
        cradle,
        turret,
        axes="xy",
        inner_elem=payload_body,
        outer_elem=dome_flange,
        name="payload_body_stays_under_dome_flange",
    )
    ctx.expect_within(
        cradle,
        turret,
        axes="xy",
        inner_elem=lens_barrel,
        outer_elem=dome_flange,
        name="lens_stays_under_dome_flange",
    )
    ctx.expect_within(
        cradle,
        turret,
        axes="xy",
        inner_elem=payload_body,
        outer_elem=dome_shell,
        name="dome_shell_covers_payload_body",
    )
    with ctx.pose({tilt: -1.00}):
        ctx.expect_within(
            cradle,
            turret,
            axes="xy",
            inner_elem=payload_body,
            outer_elem=dome_flange,
            name="payload_body_stays_under_flange_tilted_down",
        )
        ctx.expect_within(
            cradle,
            turret,
            axes="xy",
            inner_elem=lens_barrel,
            outer_elem=dome_flange,
            name="lens_stays_under_flange_tilted_down",
        )
        ctx.expect_within(
            cradle,
            turret,
            axes="xy",
            inner_elem=payload_body,
            outer_elem=dome_shell,
            name="payload_body_stays_under_dome_shell_tilted_down",
        )
        ctx.expect_within(
            cradle,
            turret,
            axes="xy",
            inner_elem=lens_barrel,
            outer_elem=dome_shell,
            name="lens_stays_under_dome_shell_tilted_down",
        )
    with ctx.pose({tilt: 0.25, azimuth: math.pi / 2.0}):
        ctx.expect_contact(
            turret,
            mount,
            elem_a=turret_ring,
            elem_b=bearing_seat,
            name="turret_ring_stays_seated_while_rotated",
        )
        ctx.expect_within(
            cradle,
            turret,
            axes="xy",
            inner_elem=lens_barrel,
            outer_elem=dome_flange,
            name="lens_stays_under_flange_in_combined_pose",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
