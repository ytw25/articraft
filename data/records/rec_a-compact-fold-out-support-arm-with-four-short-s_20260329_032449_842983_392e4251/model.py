from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_arm")

    # Dimensions
    base_size = (0.12, 0.1, 0.01)
    link_len = 0.08
    link_width = 0.02
    link_height = 0.015
    plat_size = (0.04, 0.04, 0.005)

    # Materials
    model.material("metal", rgba=(0.6, 0.6, 0.65, 1.0))
    model.material("dark_base", rgba=(0.15, 0.15, 0.18, 1.0))
    model.material("accent", rgba=(0.2, 0.5, 0.8, 1.0))

    # Base Plate
    base = model.part("base")
    base.visual(
        Box(base_size),
        origin=Origin(xyz=(0, 0, base_size[2] / 2)),
        material="dark_base",
        name="base_plate_visual",
    )
    # Add a decorative "slot" or recessed area on the base
    base.visual(
        Box((base_size[0] - 0.01, base_size[1] - 0.01, 0.002)),
        origin=Origin(xyz=(0, 0, base_size[2])),
        material="accent",
        name="base_detail",
    )
    base.inertial = Inertial.from_geometry(
        Box(base_size), mass=1.0, origin=Origin(xyz=(0, 0, base_size[2] / 2))
    )

    # Links
    y_offsets = [-0.03, -0.01, 0.01, 0.03]
    links = []
    for i in range(4):
        name = f"link_{i+1}"
        link = model.part(name)
        # Main link body
        link.visual(
            Box((link_len, link_width, link_height)),
            origin=Origin(xyz=(link_len / 2, 0, 0)),
            material="metal",
            name=f"{name}_visual",
        )
        # Hinge knuckles
        link.visual(
            Cylinder(radius=link_width / 2, length=link_height + 0.001),
            origin=Origin(xyz=(0, 0, 0)),
            material="metal",
            name=f"{name}_knuckle_1",
        )
        link.visual(
            Cylinder(radius=link_width / 2, length=link_height + 0.001),
            origin=Origin(xyz=(link_len, 0, 0)),
            material="metal",
            name=f"{name}_knuckle_2",
        )
        # Joint pin visual
        link.visual(
            Cylinder(radius=0.003, length=link_height + 0.005),
            origin=Origin(xyz=(0, 0, 0)),
            material="accent",
            name=f"{name}_pin",
        )

        link.inertial = Inertial.from_geometry(
            Box((link_len, link_width, link_height)),
            mass=0.1,
            origin=Origin(xyz=(link_len / 2, 0, 0)),
        )
        links.append(link)

    # Platform Bracket
    platform = model.part("platform")
    # A more interesting shape for the platform: a base with a small lip
    platform.visual(
        Box(plat_size),
        origin=Origin(xyz=(plat_size[0] / 2, 0, 0)),
        material="metal",
        name="platform_base",
    )
    platform.visual(
        Box((0.005, plat_size[1], 0.01)),
        origin=Origin(xyz=(plat_size[0], 0, 0.005)),
        material="metal",
        name="platform_lip",
    )
    platform.inertial = Inertial.from_geometry(
        Box(plat_size), mass=0.05, origin=Origin(xyz=(plat_size[0] / 2, 0, 0))
    )

    # Articulations
    # All joints have rpy=(0, 0, pi) to create a zigzag folding pattern.
    # This keeps all links over the base at joint angle 0.

    # Joint 1: Base to Link 1
    # Axis Y. Pivot at x=0.04 on base. link_1 points towards -X.
    model.articulation(
        "joint_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(xyz=(0.04, -0.03, base_size[2]), rpy=(0, 0, 3.14159)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10, velocity=2, lower=0, upper=3.14159),
    )

    # Joint 2: Link 1 to Link 2
    # link_1 has Y_1 = -Y_base. To get link_2 at Y_base = -0.01 (delta +0.02),
    # we need delta_y_1 = -0.02.
    model.articulation(
        "joint_2",
        ArticulationType.REVOLUTE,
        parent=links[0],
        child=links[1],
        origin=Origin(xyz=(link_len, -0.02, 0), rpy=(0, 0, 3.14159)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10, velocity=2, lower=0, upper=3.14159),
    )

    # Joint 3: Link 2 to Link 3
    # link_2 has Y_2 = Y_base. To get link_3 at Y_base = 0.01 (delta +0.02),
    # we need delta_y_2 = 0.02.
    model.articulation(
        "joint_3",
        ArticulationType.REVOLUTE,
        parent=links[1],
        child=links[2],
        origin=Origin(xyz=(link_len, 0.02, 0), rpy=(0, 0, 3.14159)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10, velocity=2, lower=0, upper=3.14159),
    )

    # Joint 4: Link 3 to Link 4
    # link_3 has Y_3 = -Y_base. To get link_4 at Y_base = 0.03 (delta +0.02),
    # we need delta_y_3 = -0.02.
    model.articulation(
        "joint_4",
        ArticulationType.REVOLUTE,
        parent=links[2],
        child=links[3],
        origin=Origin(xyz=(link_len, -0.02, 0), rpy=(0, 0, 3.14159)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10, velocity=2, lower=0, upper=3.14159),
    )

    # Platform to Link 4 (Fixed)
    model.articulation(
        "joint_platform",
        ArticulationType.FIXED,
        parent=links[3],
        child=platform,
        origin=Origin(xyz=(link_len, 0, 0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("base")
    link1 = object_model.get_part("link_1")
    link2 = object_model.get_part("link_2")
    link3 = object_model.get_part("link_3")
    link4 = object_model.get_part("link_4")
    platform = object_model.get_part("platform")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    # Check that in rest pose they are stacked/parallel on the base
    # (Joint positions default to 0)
    ctx.expect_overlap(link1, base, axes="xy", min_overlap=0.01)
    ctx.expect_overlap(link4, base, axes="xy", min_overlap=0.01)

    # Check connection sequence
    ctx.expect_contact(base, link1)
    ctx.expect_contact(link1, link2)
    ctx.expect_contact(link2, link3)
    ctx.expect_contact(link3, link4)
    ctx.expect_contact(link4, platform)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
