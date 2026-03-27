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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="macro_prime_with_extension_tube", assets=ASSETS)

    satin_black = model.material("satin_black", rgba=(0.12, 0.12, 0.13, 1.0))
    matte_black = model.material("matte_black", rgba=(0.07, 0.07, 0.08, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.20, 0.21, 1.0))
    anodized_metal = model.material("anodized_metal", rgba=(0.29, 0.30, 0.32, 1.0))
    engraving_white = model.material("engraving_white", rgba=(0.86, 0.87, 0.88, 1.0))
    front_glass = model.material("front_glass", rgba=(0.36, 0.45, 0.54, 0.58))

    rear_mount = model.part("rear_mount")
    rear_mount.visual(
        Cylinder(radius=0.0365, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=anodized_metal,
        name="camera_side_ring",
    )
    rear_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0365, length=0.008),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    extension_tube = model.part("extension_tube")
    extension_tube.visual(
        Cylinder(radius=0.0325, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_black,
        name="tube_rear_step",
    )
    extension_tube.visual(
        Cylinder(radius=0.0295, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=satin_black,
        name="tube_mid_shell",
    )
    extension_tube.visual(
        Cylinder(radius=0.0332, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=satin_black,
        name="tube_front_step",
    )
    extension_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0332, length=0.048),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )

    front_mount = model.part("front_mount")
    front_mount.visual(
        Cylinder(radius=0.0390, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=anodized_metal,
        name="lens_side_ring",
    )
    front_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0390, length=0.008),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    lens_barrel = model.part("lens_barrel")
    lens_barrel.visual(
        Cylinder(radius=0.0378, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=matte_black,
        name="rear_spigot",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0430, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=matte_black,
        name="barrel_shell",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0340, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=dark_gray,
        name="front_retainer",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0310, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        material=front_glass,
        name="front_element",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0450, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=matte_black,
        name="front_rim",
    )
    lens_barrel.visual(
        Box((0.0012, 0.0006, 0.0120)),
        origin=Origin(xyz=(0.0, 0.0431, 0.0300)),
        material=engraving_white,
        name="focus_index_line",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0410, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0146)),
        material=dark_gray,
        name="focus_bearing_rear",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0410, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0454)),
        material=dark_gray,
        name="focus_bearing_front",
    )
    lens_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0450, length=0.090),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        Cylinder(radius=0.0455, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=dark_gray,
        name="focus_shell",
    )
    for mark_name, x_pos, z_pos, width, height in [
        ("distance_mark_near", -0.0046, 0.0090, 0.0018, 0.0060),
        ("distance_mark_left", -0.0022, 0.0135, 0.0020, 0.0080),
        ("distance_mark_major", 0.0000, 0.0180, 0.0048, 0.0120),
        ("distance_mark_right", 0.0022, 0.0225, 0.0020, 0.0080),
        ("distance_mark_far", 0.0046, 0.0270, 0.0018, 0.0060),
    ]:
        focus_ring.visual(
            Box((width, 0.0003, height)),
            origin=Origin(xyz=(x_pos, 0.04535, z_pos)),
            material=engraving_white,
            name=mark_name,
        )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0455, length=0.036),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    model.articulation(
        "rear_mount_to_tube",
        ArticulationType.FIXED,
        parent=rear_mount,
        child=extension_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )
    model.articulation(
        "tube_to_front_mount",
        ArticulationType.FIXED,
        parent=extension_tube,
        child=front_mount,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
    )
    model.articulation(
        "front_mount_to_barrel",
        ArticulationType.FIXED,
        parent=front_mount,
        child=lens_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )
    model.articulation(
        "focus_ring_spin",
        ArticulationType.CONTINUOUS,
        parent=lens_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    rear_mount = object_model.get_part("rear_mount")
    extension_tube = object_model.get_part("extension_tube")
    front_mount = object_model.get_part("front_mount")
    lens_barrel = object_model.get_part("lens_barrel")
    focus_ring = object_model.get_part("focus_ring")
    focus_ring_spin = object_model.get_articulation("focus_ring_spin")

    camera_side_ring = rear_mount.get_visual("camera_side_ring")
    tube_rear_step = extension_tube.get_visual("tube_rear_step")
    tube_mid_shell = extension_tube.get_visual("tube_mid_shell")
    tube_front_step = extension_tube.get_visual("tube_front_step")
    lens_side_ring = front_mount.get_visual("lens_side_ring")
    rear_spigot = lens_barrel.get_visual("rear_spigot")
    barrel_shell = lens_barrel.get_visual("barrel_shell")
    front_rim = lens_barrel.get_visual("front_rim")
    focus_bearing_rear = lens_barrel.get_visual("focus_bearing_rear")
    focus_bearing_front = lens_barrel.get_visual("focus_bearing_front")
    focus_index_line = lens_barrel.get_visual("focus_index_line")
    focus_shell = focus_ring.get_visual("focus_shell")
    distance_mark_near = focus_ring.get_visual("distance_mark_near")
    distance_mark_major = focus_ring.get_visual("distance_mark_major")
    distance_mark_far = focus_ring.get_visual("distance_mark_far")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        focus_ring,
        lens_barrel,
        reason="focus ring rides on thin hidden bearing lands around the barrel",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        extension_tube,
        rear_mount,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tube_rear_step,
        negative_elem=camera_side_ring,
    )
    ctx.expect_gap(
        front_mount,
        extension_tube,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=lens_side_ring,
        negative_elem=tube_front_step,
    )
    ctx.expect_gap(
        front_mount,
        rear_mount,
        axis="z",
        min_gap=0.035,
        max_gap=0.049,
        positive_elem=lens_side_ring,
        negative_elem=camera_side_ring,
    )
    ctx.expect_within(
        extension_tube,
        rear_mount,
        axes="xy",
        inner_elem=tube_rear_step,
        outer_elem=camera_side_ring,
    )
    ctx.expect_within(
        extension_tube,
        rear_mount,
        axes="xy",
        inner_elem=tube_mid_shell,
        outer_elem=camera_side_ring,
    )
    ctx.expect_within(
        extension_tube,
        front_mount,
        axes="xy",
        inner_elem=tube_front_step,
        outer_elem=lens_side_ring,
    )
    ctx.expect_within(
        extension_tube,
        front_mount,
        axes="xy",
        inner_elem=tube_mid_shell,
        outer_elem=lens_side_ring,
    )
    ctx.expect_gap(
        lens_barrel,
        front_mount,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rear_spigot,
        negative_elem=lens_side_ring,
    )
    ctx.expect_within(
        lens_barrel,
        focus_ring,
        axes="xy",
        inner_elem=barrel_shell,
        outer_elem=focus_shell,
    )
    ctx.expect_gap(
        lens_barrel,
        focus_ring,
        axis="z",
        min_gap=0.03,
        positive_elem=front_rim,
        negative_elem=focus_shell,
    )
    ctx.expect_overlap(
        focus_ring,
        lens_barrel,
        axes="z",
        min_overlap=0.03,
        elem_a=focus_shell,
        elem_b=barrel_shell,
    )
    ctx.expect_overlap(
        focus_ring,
        lens_barrel,
        axes="xy",
        min_overlap=0.07,
        elem_a=focus_shell,
        elem_b=focus_bearing_rear,
    )
    ctx.expect_overlap(
        focus_ring,
        lens_barrel,
        axes="xy",
        min_overlap=0.07,
        elem_a=focus_shell,
        elem_b=focus_bearing_front,
    )
    ctx.expect_overlap(
        focus_ring,
        focus_ring,
        axes="y",
        min_overlap=0.0002,
        elem_a=distance_mark_major,
        elem_b=focus_shell,
    )
    ctx.expect_overlap(
        focus_ring,
        focus_ring,
        axes="y",
        min_overlap=0.0002,
        elem_a=distance_mark_near,
        elem_b=focus_shell,
    )
    ctx.expect_overlap(
        focus_ring,
        focus_ring,
        axes="y",
        min_overlap=0.0002,
        elem_a=distance_mark_far,
        elem_b=focus_shell,
    )
    ctx.expect_overlap(
        focus_ring,
        lens_barrel,
        axes="x",
        min_overlap=0.001,
        elem_a=distance_mark_major,
        elem_b=focus_index_line,
    )
    with ctx.pose({focus_ring_spin: math.pi / 2.0}):
        ctx.expect_within(
            lens_barrel,
            focus_ring,
            axes="xy",
            inner_elem=barrel_shell,
            outer_elem=focus_shell,
        )
        ctx.expect_overlap(
            focus_ring,
            lens_barrel,
            axes="xy",
            min_overlap=0.07,
            elem_a=focus_shell,
            elem_b=focus_bearing_rear,
        )
        ctx.expect_overlap(
            focus_ring,
            lens_barrel,
            axes="xy",
            min_overlap=0.07,
            elem_a=focus_shell,
            elem_b=focus_bearing_front,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
