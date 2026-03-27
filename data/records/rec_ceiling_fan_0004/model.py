from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/")

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
    repair_loft,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    radial_segments: int = 56,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _blade_section(span_x: float, chord: float, thickness: float, z_center: float) -> list[tuple[float, float, float]]:
    half_thickness = thickness * 0.5
    return [
        (span_x, -0.50 * chord, z_center + 0.06 * half_thickness),
        (span_x, -0.22 * chord, z_center + 0.90 * half_thickness),
        (span_x, 0.10 * chord, z_center + 1.00 * half_thickness),
        (span_x, 0.42 * chord, z_center + 0.34 * half_thickness),
        (span_x, 0.50 * chord, z_center - 0.04 * half_thickness),
        (span_x, 0.20 * chord, z_center - 0.76 * half_thickness),
        (span_x, -0.10 * chord, z_center - 0.64 * half_thickness),
        (span_x, -0.46 * chord, z_center - 0.10 * half_thickness),
    ]


def _build_blade_mesh() -> MeshGeometry:
    return repair_loft(
        section_loft(
            [
                _blade_section(0.22, 0.22, 0.026, -0.010),
                _blade_section(0.62, 0.21, 0.023, -0.013),
                _blade_section(1.18, 0.18, 0.018, -0.018),
                _blade_section(1.76, 0.14, 0.012, -0.023),
                _blade_section(2.26, 0.10, 0.007, -0.027),
            ]
        )
    )


def _build_hub_ring_mesh() -> MeshGeometry:
    return _merge_geometries(
        [
            _ring_band(
                outer_radius=0.29,
                inner_radius=0.105,
                height=0.055,
                z_center=0.0,
                radial_segments=64,
            ),
            _ring_band(
                outer_radius=0.18,
                inner_radius=0.105,
                height=0.026,
                z_center=0.030,
                radial_segments=56,
            ),
            _ring_band(
                outer_radius=0.16,
                inner_radius=0.105,
                height=0.018,
                z_center=-0.040,
                radial_segments=56,
            ),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="warehouse_ceiling_fan", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.16, 0.17, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    aluminum = model.material("aluminum", rgba=(0.79, 0.82, 0.85, 1.0))

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.16, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 1.74)),
        material=graphite,
        name="ceiling_canopy",
    )
    mount.visual(
        Cylinder(radius=0.026, length=1.42),
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
        material=steel,
        name="downrod",
    )
    mount.visual(
        Cylinder(radius=0.050, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.70)),
        material=matte_black,
        name="upper_collar",
    )
    mount.visual(
        Cylinder(radius=0.052, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=graphite,
        name="lower_coupler",
    )
    mount.visual(
        Cylinder(radius=0.038, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=matte_black,
        name="rod_sleeve",
    )
    mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.026, length=1.42),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.99)),
    )

    motor = model.part("motor")
    motor.visual(
        Box((0.18, 0.14, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=graphite,
        name="hanger_block",
    )
    motor.visual(
        Box((0.050, 0.050, 0.16)),
        origin=Origin(xyz=(0.115, 0.0, 0.04)),
        material=matte_black,
        name="yoke_right",
    )
    motor.visual(
        Box((0.050, 0.050, 0.16)),
        origin=Origin(xyz=(-0.115, 0.0, 0.04)),
        material=matte_black,
        name="yoke_left",
    )
    motor.visual(
        Cylinder(radius=0.21, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=graphite,
        name="motor_shell",
    )
    motor.visual(
        Cylinder(radius=0.135, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=matte_black,
        name="gearbox_cap",
    )
    motor.visual(
        Cylinder(radius=0.09, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
        material=steel,
        name="bearing_collar",
    )
    motor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.21, length=0.18),
        mass=118.0,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        _save_mesh(_build_hub_ring_mesh(), "hub_ring.obj"),
        origin=Origin(),
        material=matte_black,
        name="hub_ring",
    )
    rotor.visual(
        _save_mesh(
            _ring_band(
                outer_radius=0.17,
                inner_radius=0.105,
                height=0.022,
                z_center=0.040,
                radial_segments=56,
            ),
            "hub_top_ring.obj",
        ),
        origin=Origin(),
        material=graphite,
        name="hub_top_ring",
    )
    rotor.visual(
        _save_mesh(
            _ring_band(
                outer_radius=0.15,
                inner_radius=0.105,
                height=0.016,
                z_center=-0.048,
                radial_segments=56,
            ),
            "hub_lower_ring.obj",
        ),
        origin=Origin(),
        material=graphite,
        name="hub_lower_ring",
    )
    rotor.visual(
        Cylinder(radius=0.105, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=graphite,
        name="hub_core",
    )

    blade_mesh = _save_mesh(_build_blade_mesh(), "blade.obj")
    for index in range(6):
        azimuth = index * math.tau / 6.0
        rotor.visual(
            blade_mesh,
            origin=Origin(rpy=(0.0, 0.0, azimuth)),
            material=aluminum,
            name=f"blade_{index}",
        )
        rotor.visual(
            Box((0.20, 0.08, 0.04)),
            origin=Origin(
                xyz=(
                    0.36 * math.cos(azimuth),
                    0.36 * math.sin(azimuth),
                    -0.018,
                ),
                rpy=(0.0, 0.0, azimuth),
            ),
            material=steel,
            name=f"blade_clamp_{index}",
        )
        rotor.visual(
            Box((0.10, 0.055, 0.006)),
            origin=Origin(
                xyz=(
                    2.30 * math.cos(azimuth),
                    2.30 * math.sin(azimuth),
                    -0.027,
                ),
                rpy=(0.0, 0.0, azimuth),
            ),
            material=aluminum,
            name=f"blade_tip_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.30, length=0.06),
        mass=74.0,
        origin=Origin(),
    )

    model.articulation(
        "mount_to_motor",
        ArticulationType.FIXED,
        parent=mount,
        child=motor,
        origin=Origin(xyz=(0.0, 0.0, 0.30)),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=motor,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, -0.17)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    motor = object_model.get_part("motor")
    rotor = object_model.get_part("rotor")
    rotor_spin = object_model.get_articulation("rotor_spin")

    ceiling_canopy = mount.get_visual("ceiling_canopy")
    lower_coupler = mount.get_visual("lower_coupler")
    hanger_block = motor.get_visual("hanger_block")
    motor_shell = motor.get_visual("motor_shell")
    gearbox_cap = motor.get_visual("gearbox_cap")
    bearing_collar = motor.get_visual("bearing_collar")
    hub_ring = rotor.get_visual("hub_ring")
    hub_top_ring = rotor.get_visual("hub_top_ring")
    hub_core = rotor.get_visual("hub_core")
    blade_clamp_0 = rotor.get_visual("blade_clamp_0")
    blade_tip_0 = rotor.get_visual("blade_tip_0")
    blade_tip_3 = rotor.get_visual("blade_tip_3")

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
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_origin_distance(
        mount,
        motor,
        axes="xy",
        max_dist=0.01,
        name="downrod stays centered over motor assembly",
    )
    ctx.expect_overlap(
        mount,
        motor,
        axes="xy",
        min_overlap=0.09,
        elem_a=lower_coupler,
        elem_b=hanger_block,
        name="lower coupler stays centered in hanger block",
    )
    ctx.expect_gap(
        motor,
        mount,
        axis="z",
        max_gap=0.02,
        max_penetration=0.02,
        positive_elem=hanger_block,
        negative_elem=lower_coupler,
        name="hanger block seats onto the long downrod coupler",
    )
    ctx.expect_gap(
        mount,
        motor,
        axis="z",
        min_gap=1.25,
        positive_elem=ceiling_canopy,
        negative_elem=gearbox_cap,
        name="long ceiling rod keeps canopy far above the motor",
    )
    ctx.expect_overlap(
        rotor,
        motor,
        axes="xy",
        min_overlap=0.16,
        elem_a=hub_ring,
        elem_b=bearing_collar,
        name="hub ring stays centered around the bearing collar",
    )
    ctx.expect_within(
        motor,
        rotor,
        axes="xy",
        inner_elem=bearing_collar,
        outer_elem=hub_ring,
        name="bearing collar stays inside the rotor hub footprint",
    )
    ctx.expect_contact(
        rotor,
        motor,
        elem_a=hub_core,
        elem_b=bearing_collar,
        name="hub core seats against the bearing collar",
    )
    ctx.expect_gap(
        motor,
        rotor,
        axis="z",
        min_gap=0.003,
        positive_elem=motor_shell,
        negative_elem=hub_top_ring,
        name="rotor hangs just below the motor housing",
    )
    ctx.expect_gap(
        rotor,
        motor,
        axis="x",
        min_gap=1.95,
        positive_elem=blade_tip_0,
        negative_elem=motor_shell,
        name="blade span extends far beyond the motor on the positive x side",
    )
    ctx.expect_gap(
        motor,
        rotor,
        axis="x",
        min_gap=1.95,
        positive_elem=motor_shell,
        negative_elem=blade_tip_3,
        name="blade span extends far beyond the motor on the negative x side",
    )
    with ctx.pose({rotor_spin: math.pi / 2.0}):
        ctx.expect_within(
            motor,
            rotor,
            axes="xy",
            inner_elem=bearing_collar,
            outer_elem=hub_ring,
            name="bearing collar stays inside the hub while spinning",
        )
        ctx.expect_gap(
            rotor,
            motor,
            axis="y",
            min_gap=1.95,
            positive_elem=blade_tip_0,
            negative_elem=motor_shell,
            name="rotated blade still reads as a warehouse-scale sweep",
        )
        ctx.expect_gap(
            motor,
            rotor,
            axis="z",
            min_gap=0.003,
            positive_elem=motor_shell,
            negative_elem=blade_clamp_0,
            name="blade root clamp stays just below the motor while spinning",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
