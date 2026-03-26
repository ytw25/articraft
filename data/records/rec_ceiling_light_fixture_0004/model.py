from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float,
    radial_segments: int = 72,
):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _build_glass_bowl_mesh():
    return LatheGeometry(
        [
            (0.046, -0.071),
            (0.070, -0.079),
            (0.105, -0.106),
            (0.132, -0.145),
            (0.145, -0.192),
            (0.140, -0.240),
            (0.112, -0.284),
            (0.066, -0.318),
            (0.018, -0.334),
            (0.000, -0.334),
            (0.000, -0.323),
            (0.061, -0.310),
            (0.104, -0.278),
            (0.131, -0.236),
            (0.136, -0.192),
            (0.124, -0.148),
            (0.098, -0.110),
            (0.052, -0.078),
            (0.028, -0.071),
        ],
        segments=88,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_flush_globe_fixture", assets=ASSETS)

    canopy_metal = model.material("canopy_metal", rgba=(0.82, 0.82, 0.80, 1.0))
    collar_metal = model.material("collar_metal", rgba=(0.74, 0.74, 0.72, 1.0))
    glass_white = model.material("glass_white", rgba=(0.96, 0.97, 0.98, 0.32))

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.108, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=canopy_metal,
        name="canopy_plate",
    )
    mount.visual(
        Cylinder(radius=0.042, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=canopy_metal,
        name="canopy_hub",
    )
    mount.visual(
        Cylinder(radius=0.024, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=canopy_metal,
        name="neck_stub",
    )
    mount.visual(
        Cylinder(radius=0.024, length=0.043),
        origin=Origin(xyz=(0.0, 0.0, -0.0415)),
        material=canopy_metal,
        name="neck_tube",
    )
    mount.visual(
        Cylinder(radius=0.036, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.067)),
        material=canopy_metal,
        name="holder_shoulder",
    )
    mount.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=canopy_metal,
        name="thread_peak_upper",
    )
    mount.visual(
        Cylinder(radius=0.0225, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=canopy_metal,
        name="thread_valley_upper",
    )
    mount.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        material=canopy_metal,
        name="thread_peak_mid",
    )
    mount.visual(
        Cylinder(radius=0.0225, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material=canopy_metal,
        name="thread_valley_lower",
    )
    mount.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.098)),
        material=canopy_metal,
        name="thread_peak_lower",
    )
    mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.108, length=0.102),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.051)),
    )

    bowl = model.part("bowl")
    bowl.visual(
        _save_mesh("glass_bowl.obj", _build_glass_bowl_mesh()),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=glass_white,
        name="glass_bowl",
    )
    bowl.visual(
        _save_mesh(
            "glass_bowl_rim.obj",
            _ring_band(
                outer_radius=0.046,
                inner_radius=0.028,
                height=0.010,
                z_center=0.010,
            ),
        ),
        material=glass_white,
        name="bowl_rim",
    )
    bowl.inertial = Inertial.from_geometry(
        Sphere(radius=0.146),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, -0.118)),
    )

    collar = model.part("collar")
    collar.visual(
        _save_mesh(
            "threaded_collar.obj",
            _ring_band(
                outer_radius=0.041,
                inner_radius=0.026,
                height=0.022,
                z_center=-0.006,
            ),
        ),
        material=collar_metal,
        name="threaded_collar",
    )
    collar.visual(
        _save_mesh(
            "retaining_lip.obj",
            _ring_band(
                outer_radius=0.046,
                inner_radius=0.028,
                height=0.010,
                z_center=0.000,
            ),
        ),
        material=collar_metal,
        name="retaining_lip",
    )
    collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.030),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
    )

    model.articulation(
        "neck_to_collar",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=collar,
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=5.0),
    )
    model.articulation(
        "collar_to_bowl",
        ArticulationType.FIXED,
        parent=collar,
        child=bowl,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("mount")
    bowl = object_model.get_part("bowl")
    collar = object_model.get_part("collar")
    collar_spin = object_model.get_articulation("neck_to_collar")
    canopy_plate = mount.get_visual("canopy_plate")
    holder_shoulder = mount.get_visual("holder_shoulder")
    thread_peak_mid = mount.get_visual("thread_peak_mid")
    glass_bowl = bowl.get_visual("glass_bowl")
    bowl_rim = bowl.get_visual("bowl_rim")
    threaded_collar = collar.get_visual("threaded_collar")
    retaining_lip = collar.get_visual("retaining_lip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.03)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance(bowl, mount, axes="xy", max_dist=0.002)
    ctx.expect_origin_distance(collar, mount, axes="xy", max_dist=0.002)
    ctx.expect_overlap(bowl, mount, axes="xy", min_overlap=0.08)
    ctx.expect_gap(
        mount,
        bowl,
        axis="z",
        min_gap=0.058,
        positive_elem=canopy_plate,
        negative_elem=glass_bowl,
        name="canopy_plate_sits_well_above_the_main_globe_volume",
    )
    ctx.expect_gap(
        mount,
        bowl,
        axis="z",
        min_gap=0.060,
        positive_elem=canopy_plate,
        negative_elem=bowl_rim,
        name="short_neck_visibly_separates_canopy_from_globe_rim",
    )
    ctx.expect_contact(
        mount,
        bowl,
        elem_a=holder_shoulder,
        elem_b=bowl_rim,
        name="glass_rim_seats_under_the_neck_shoulder",
    )
    ctx.expect_within(
        mount,
        collar,
        axes="xy",
        inner_elem=thread_peak_mid,
        outer_elem=threaded_collar,
        name="collar_ring_screws_onto_the_threaded_neck",
    )
    ctx.expect_overlap(
        collar,
        bowl,
        axes="xy",
        min_overlap=0.075,
        elem_a=retaining_lip,
        elem_b=bowl_rim,
        name="retaining_lip_spans_the_globe_opening",
    )
    ctx.expect_contact(
        collar,
        bowl,
        elem_a=retaining_lip,
        elem_b=bowl_rim,
        name="retaining_lip_supports_the_glass_bowl_from_below",
    )
    ctx.expect_gap(
        mount,
        collar,
        axis="z",
        min_gap=0.007,
        max_gap=0.010,
        positive_elem=holder_shoulder,
        negative_elem=retaining_lip,
        name="collar_gap_matches_the_captured_glass_rim_thickness",
    )
    with ctx.pose({collar_spin: math.pi / 2.0}):
        ctx.expect_within(
            mount,
            collar,
            axes="xy",
            inner_elem=thread_peak_mid,
            outer_elem=threaded_collar,
            name="collar_remains_engaged_with_the_neck_thread_when_rotated",
        )
        ctx.expect_contact(
            collar,
            bowl,
            elem_a=retaining_lip,
            elem_b=bowl_rim,
            name="globe_remains_retained_by_the_rotated_collar",
        )
        ctx.expect_origin_distance(bowl, mount, axes="xy", max_dist=0.002)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
