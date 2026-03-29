from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Material,
)


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
    Material,
    ExtrudeGeometry,
    rounded_rect_profile,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    """
    Builds a compact fold-out support arm with four rounded boxed links in series.
    The links are offset in Y to allow compact folding without overlap.
    The four joints are revolute about parallel Y-axes.
    """
    model = ArticulatedObject(name="foldable_support_arm")

    # Materials
    model.material("base_mat", color=(0.2, 0.2, 0.2))
    model.material("link_mat_1", color=(0.4, 0.4, 0.4))
    model.material("link_mat_2", color=(0.5, 0.5, 0.5))
    model.material("platform_mat", color=(0.6, 0.1, 0.1))

    # Dimensions
    base_size = (0.2, 0.15, 0.01)
    link_l = 0.1
    link_w = 0.02
    link_h = 0.02
    plat_size = (0.06, 0.06, 0.005)

    # Base Plate
    base = model.part("base")
    base_geom = ExtrudeGeometry.centered(
        rounded_rect_profile(base_size[0], base_size[1], 0.01), base_size[2]
    )
    base.visual(
        mesh_from_geometry(base_geom, "base_plate_mesh"),
        origin=Origin(xyz=(0, 0, 0.005)),
        material="base_mat",
        name="base_plate",
    )
    # Mounting post to connect the base plate to the first arm joint
    base.visual(
        Cylinder(radius=0.01, length=0.04),
        origin=Origin(xyz=(-0.08, -0.03, 0.03)),
        material="base_mat",
        name="mounting_post",
    )
    base.inertial = Inertial.from_geometry(
        Box(base_size), mass=2.0, origin=Origin(xyz=(0, 0, 0.005))
    )

    # Links
    links = []
    link_profile = rounded_rect_profile(link_w, link_h, 0.004)
    # Extrude along X (local length)
    # ExtrudeGeometry extrudes along Z, so we rotate it
    link_geom_raw = ExtrudeGeometry.centered(link_profile, link_l).rotate_y(math.pi / 2)
    link_mesh = mesh_from_geometry(link_geom_raw, "link_mesh")

    for i in range(4):
        link_name = f"link_{i+1}"
        link = model.part(link_name)
        # Each link is a rounded box.
        # The joint is at local (0,0,0). Visual is centered in Y and Z.
        link.visual(
            link_mesh,
            origin=Origin(xyz=(link_l / 2, 0, 0)),
            material="link_mat_1" if i % 2 == 0 else "link_mat_2",
            name=f"{link_name}_visual",
        )
        link.inertial = Inertial.from_geometry(
            Box((link_l, link_w, link_h)),
            mass=0.3,
            origin=Origin(xyz=(link_l / 2, 0, 0)),
        )
        links.append(link)

    # Articulations
    # All joints are revolute about the Y axis.
    # Joint 1: Base to Link 1. Located near one end of the base.
    # Raised Z to 0.05 so links and platform don't overlap base plate even when flipped.
    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(xyz=(-0.08, -0.03, 0.05)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(
            effort=10, velocity=2, lower=-math.pi / 2, upper=math.pi / 2
        ),
    )

    # Joints between links
    # Each link is offset by link_w (0.02) in Y from its parent.
    # This creates a side-by-side stacking when folded.
    for i in range(3):
        model.articulation(
            f"link_{i+1}_to_link_{i+2}",
            ArticulationType.REVOLUTE,
            parent=links[i],
            child=links[i+1],
            origin=Origin(xyz=(link_l, link_w, 0)),
            axis=(0, 1, 0),
            motion_limits=MotionLimits(
                effort=10, velocity=2, lower=-math.pi, upper=math.pi
            ),
        )

    # Platform
    platform = model.part("platform")
    plat_geom = ExtrudeGeometry.centered(
        rounded_rect_profile(plat_size[0], plat_size[1], 0.005), plat_size[2]
    )
    platform.visual(
        mesh_from_geometry(plat_geom, "platform_mesh"),
        origin=Origin(xyz=(0, 0, 0.0025)),
        material="platform_mat",
        name="platform_visual",
    )
    # Post to connect the platform to the last arm link
    platform.visual(
        Cylinder(radius=0.01, length=0.02),
        origin=Origin(xyz=(0, 0, -0.01)),
        material="platform_mat",
        name="platform_post",
    )
    platform.inertial = Inertial.from_geometry(
        Box(plat_size), mass=0.2, origin=Origin(xyz=(0, 0, 0.0025))
    )

    model.articulation(
        "link_4_to_platform",
        ArticulationType.FIXED,
        parent=links[3],
        child=platform,
        origin=Origin(xyz=(link_l, 0, 0.02)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    l1 = object_model.get_part("link_1")
    l2 = object_model.get_part("link_2")
    l3 = object_model.get_part("link_3")
    l4 = object_model.get_part("link_4")
    platform = object_model.get_part("platform")

    j1 = object_model.get_articulation("base_to_link_1")
    j2 = object_model.get_articulation("link_1_to_link_2")
    j3 = object_model.get_articulation("link_2_to_link_3")
    j4 = object_model.get_articulation("link_3_to_link_4")

    # Allow intentional overlaps at the hinge points
    ctx.allow_overlap(base, l1, reason="Hinge contact")
    ctx.allow_overlap(l4, platform, reason="Hinge contact")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    # Test extended pose reach
    with ctx.pose({j1: 0, j2: 0, j3: 0, j4: 0}):
        ctx.expect_origin_distance(base, platform, axes="x", min_dist=0.25, max_dist=0.45)

    # Test folded pose (accordion fold)
    # L1: 0, L2: pi, L3: pi, L4: pi
    with ctx.pose({j1: 0, j2: math.pi, j3: math.pi, j4: math.pi}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_folded")
        # All links should be above the base plate specifically
        for link in [l1, l2, l3, l4]:
            ctx.expect_gap(
                link,
                base,
                axis="z",
                min_gap=0.0,
                negative_elem="base_plate",
                max_penetration=1e-12,
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
