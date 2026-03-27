from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Inertial,
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


def _ring_mesh(name: str, outer_radius: float, inner_radius: float, height: float):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=56)
    inner = CylinderGeometry(radius=inner_radius, height=height + 0.002, radial_segments=56)
    return _save_mesh(name, boolean_difference(outer, inner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_flush_mount_ceiling_fan", assets=ASSETS)

    canopy_white = model.material("canopy_white", rgba=(0.95, 0.95, 0.94, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.92, 0.89, 1.0))
    brushed_nickel = model.material("brushed_nickel", rgba=(0.67, 0.70, 0.73, 1.0))
    bowl_glass = model.material("bowl_glass", rgba=(0.94, 0.93, 0.88, 0.62))
    walnut = model.material("walnut", rgba=(0.43, 0.29, 0.19, 1.0))
    maple = model.material("maple", rgba=(0.73, 0.61, 0.43, 1.0))

    ceiling_mount = model.part("ceiling_mount")
    ceiling_mount.visual(
        Cylinder(radius=0.090, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=canopy_white,
        name="canopy_plate",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.066, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=canopy_white,
        name="canopy_step",
    )
    ceiling_mount.visual(
        Cylinder(radius=0.050, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=canopy_white,
        name="canopy_collar",
    )
    ceiling_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.090, length=0.012),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )

    motor_housing = model.part("motor_housing")
    motor_housing.visual(
        Cylinder(radius=0.050, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=warm_white,
        name="top_mount_pad",
    )
    motor_housing.visual(
        Cylinder(radius=0.106, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=warm_white,
        name="upper_shoulder",
    )
    motor_housing.visual(
        Cylinder(radius=0.176, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=warm_white,
        name="housing_disc",
    )
    motor_housing.visual(
        Cylinder(radius=0.092, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material=warm_white,
        name="lower_skirt",
    )
    motor_housing.visual(
        Cylinder(radius=0.104, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.049)),
        material=brushed_nickel,
        name="rotor_seat",
    )
    motor_housing.visual(
        Cylinder(radius=0.060, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.043)),
        material=brushed_nickel,
        name="light_seat",
    )
    motor_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.176, length=0.053),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, -0.0265)),
    )

    light_bowl = model.part("light_bowl")
    light_bowl.visual(
        Cylinder(radius=0.056, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=brushed_nickel,
        name="bowl_rim",
    )
    light_bowl.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=bowl_glass,
        name="bowl_glass",
    )
    light_bowl.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=bowl_glass,
        name="bowl_lens",
    )
    light_bowl.inertial = Inertial.from_geometry(
        Sphere(radius=0.011),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
    )

    rotor_assembly = model.part("rotor_assembly")
    rotor_assembly.visual(
        _ring_mesh("rotor_ring.obj", outer_radius=0.108, inner_radius=0.046, height=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brushed_nickel,
        name="rotor_ring",
    )
    rotor_assembly.visual(
        Cylinder(radius=0.050, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=brushed_nickel,
        name="hub_plate",
    )
    for index in range(5):
        angle = index * (2.0 * math.pi / 5.0)
        blade_pitch = -0.06
        rotor_assembly.visual(
            Box((0.136, 0.016, 0.005)),
            origin=Origin(xyz=(0.148, 0.0, 0.004), rpy=(0.0, blade_pitch, angle)),
            material=brushed_nickel,
            name=f"blade_iron_{index}",
        )
        rotor_assembly.visual(
            Box((0.184, 0.054, 0.008)),
            origin=Origin(xyz=(0.262, 0.0, 0.014), rpy=(0.0, blade_pitch, angle)),
            material=walnut,
            name=f"blade_{index}",
        )
        rotor_assembly.visual(
            Box((0.164, 0.040, 0.002)),
            origin=Origin(xyz=(0.262, 0.0, 0.009), rpy=(0.0, blade_pitch, angle)),
            material=maple,
            name=f"reversible_face_{index}",
        )
    rotor_assembly.inertial = Inertial.from_geometry(
        Cylinder(radius=0.350, length=0.018),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )

    model.articulation(
        "mount_to_housing",
        ArticulationType.FIXED,
        parent=ceiling_mount,
        child=motor_housing,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )
    model.articulation(
        "housing_to_light",
        ArticulationType.FIXED,
        parent=motor_housing,
        child=light_bowl,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=motor_housing,
        child=rotor_assembly,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ceiling_mount = object_model.get_part("ceiling_mount")
    motor_housing = object_model.get_part("motor_housing")
    light_bowl = object_model.get_part("light_bowl")
    rotor_assembly = object_model.get_part("rotor_assembly")
    rotor_spin = object_model.get_articulation("rotor_spin")

    canopy_plate = ceiling_mount.get_visual("canopy_plate")
    canopy_step = ceiling_mount.get_visual("canopy_step")
    canopy_collar = ceiling_mount.get_visual("canopy_collar")
    top_mount_pad = motor_housing.get_visual("top_mount_pad")
    housing_disc = motor_housing.get_visual("housing_disc")
    lower_skirt = motor_housing.get_visual("lower_skirt")
    rotor_seat = motor_housing.get_visual("rotor_seat")
    light_seat = motor_housing.get_visual("light_seat")
    bowl_rim = light_bowl.get_visual("bowl_rim")
    bowl_glass = light_bowl.get_visual("bowl_glass")
    bowl_lens = light_bowl.get_visual("bowl_lens")
    rotor_ring = rotor_assembly.get_visual("rotor_ring")
    blade_iron_0 = rotor_assembly.get_visual("blade_iron_0")
    blade_0 = rotor_assembly.get_visual("blade_0")
    reversible_face_0 = rotor_assembly.get_visual("reversible_face_0")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(
        ceiling_mount,
        motor_housing,
        axes="xy",
        min_overlap=0.02,
        elem_a=canopy_plate,
        elem_b=housing_disc,
    )
    ctx.expect_within(
        ceiling_mount,
        motor_housing,
        axes="xy",
        inner_elem=canopy_step,
        outer_elem=housing_disc,
        name="canopy trim remains inside the motor housing footprint",
    )
    ctx.expect_origin_distance(
        ceiling_mount,
        motor_housing,
        axes="xy",
        max_dist=0.001,
    )
    ctx.expect_gap(
        ceiling_mount,
        motor_housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=canopy_collar,
        negative_elem=top_mount_pad,
        name="canopy plate mounts flat with no downrod gap",
    )
    ctx.expect_gap(
        motor_housing,
        light_bowl,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=light_seat,
        negative_elem=bowl_rim,
        name="light bowl is recessed into the housing bottom face",
    )
    ctx.expect_within(
        light_bowl,
        motor_housing,
        axes="xy",
        inner_elem=bowl_glass,
        outer_elem=lower_skirt,
        name="light bowl stays recessed within the housing underside footprint",
    )
    ctx.expect_gap(
        motor_housing,
        rotor_assembly,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rotor_seat,
        negative_elem=rotor_ring,
        name="rotor ring seats directly against the low-profile motor housing",
    )
    ctx.expect_overlap(
        rotor_assembly,
        motor_housing,
        axes="xy",
        min_overlap=0.012,
        elem_a=blade_iron_0,
        elem_b=housing_disc,
        name="blade irons emerge at the housing perimeter",
    )
    ctx.expect_within(
        rotor_assembly,
        rotor_assembly,
        axes="xy",
        inner_elem=reversible_face_0,
        outer_elem=blade_0,
        name="reversible blade face stays within the main blade footprint",
    )
    with ctx.pose({rotor_spin: math.pi / 5.0}):
        ctx.expect_gap(
            rotor_assembly,
            light_bowl,
            axis="z",
            min_gap=0.003,
            positive_elem=blade_0,
            negative_elem=bowl_lens,
            name="rotating blades remain above the recessed light bowl",
        )
        ctx.expect_gap(
            motor_housing,
            rotor_assembly,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=rotor_seat,
            negative_elem=rotor_ring,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
