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
    MeshGeometry,
    BoxGeometry,
    CylinderGeometry,
    mesh_from_geometry,
)


def build_link_geometry(length: float, width: float, height: float, next_y_offset: float) -> MeshGeometry:
    """Builds a link with integrated knuckles and a beam."""
    # Start knuckle at (0, 0, 0)
    geom = CylinderGeometry(radius=width / 2, height=height)
    
    # Main beam from knuckle to knuckle
    beam = BoxGeometry((length, width, height)).translate(length / 2, 0, 0)
    geom.merge(beam)
    
    # End knuckle and connector
    if next_y_offset != 0:
        # Cross-connector at the end of the beam
        connector = BoxGeometry((width, abs(next_y_offset), height)).translate(
            length, next_y_offset / 2, 0
        )
        geom.merge(connector)
        # End knuckle where the next link will attach
        end_knuckle = CylinderGeometry(radius=width / 2, height=height).translate(
            length, next_y_offset, 0
        )
        geom.merge(end_knuckle)
    else:
        # Final link end knuckle
        end_knuckle = CylinderGeometry(radius=width / 2, height=height).translate(
            length, 0, 0
        )
        geom.merge(end_knuckle)
        
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_arm")

    # Materials
    metal = model.material("metal_grey", color=(0.3, 0.3, 0.35))
    base_mat = model.material("base_silver", color=(0.7, 0.7, 0.7))
    bracket_mat = model.material("bracket_black", color=(0.15, 0.15, 0.15))

    # Base Plate
    base = model.part("base")
    base.visual(
        Box((0.08, 0.08, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=base_mat,
        name="base_plate",
    )
    base.inertial = Inertial.from_geometry(Box((0.08, 0.08, 0.01)), mass=0.5)

    # Links
    link_length = 0.1
    link_width = 0.02
    link_height = 0.02
    next_offsets = [0.025, -0.025, 0.025, 0.0]

    links = []
    for i in range(4):
        l = model.part(f"link_{i+1}")
        geom = build_link_geometry(
            link_length, link_width, link_height, next_offsets[i]
        )
        l.visual(
            mesh_from_geometry(geom, f"link_{i+1}_mesh"),
            origin=Origin(xyz=(0, 0, 0.01)),
            material=metal,
            name=f"link_{i+1}_visual",
        )
        l.inertial = Inertial.from_geometry(
            Box((link_length + link_width, link_width + abs(next_offsets[i]), link_height)),
            mass=0.1,
            origin=Origin(xyz=(link_length / 2, next_offsets[i] / 2, 0.01)),
        )
        links.append(l)

    # Platform
    platform = model.part("platform")
    platform.visual(
        Box((0.04, 0.04, 0.005)),
        origin=Origin(xyz=(0, 0, 0.0225)),
        material=bracket_mat,
        name="platform_bracket",
    )
    # Add a small vertical "lip" to the bracket for realism
    platform.visual(
        Box((0.005, 0.04, 0.01)),
        origin=Origin(xyz=(-0.0175, 0, 0.025)),
        material=bracket_mat,
        name="bracket_lip",
    )
    platform.inertial = Inertial.from_geometry(Box((0.04, 0.04, 0.005)), mass=0.05)

    # Articulations
    # J1: Base to Link 1
    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(xyz=(-0.03, 0.0, 0.01)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=10, velocity=1, lower=0.0, upper=3.1),
    )

    # J2-J4: Link to Link
    for i in range(3):
        model.articulation(
            f"link_{i+1}_to_link_{i+2}",
            ArticulationType.REVOLUTE,
            parent=links[i],
            child=links[i+1],
            origin=Origin(xyz=(link_length, next_offsets[i], 0)),
            axis=(0, 1, 0),
            motion_limits=MotionLimits(effort=10, velocity=1, lower=-3.1, upper=0.0)
            if (i % 2 == 0)
            else MotionLimits(effort=10, velocity=1, lower=0.0, upper=3.1),
        )

    # Fixed Platform at the end of Link 4
    model.articulation(
        "link_4_to_platform",
        ArticulationType.FIXED,
        parent=links[3],
        child=platform,
        origin=Origin(xyz=(link_length, 0, 0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("base")
    l1 = object_model.get_part("link_1")
    l2 = object_model.get_part("link_2")
    l3 = object_model.get_part("link_3")
    l4 = object_model.get_part("link_4")
    platform = object_model.get_part("platform")

    # Allow intentional hinge overlaps
    ctx.allow_overlap(l1, l2, reason="Intentional hinge knuckle overlap")
    ctx.allow_overlap(l2, l3, reason="Intentional hinge knuckle overlap")
    ctx.allow_overlap(l3, l4, reason="Intentional hinge knuckle overlap")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    # Check connectivity
    ctx.expect_contact(base, l1)
    ctx.expect_contact(l1, l2)
    ctx.expect_contact(l4, platform)

    # Check that it's sitting on the base
    ctx.expect_gap(l1, base, axis="z", min_gap=0, max_gap=0.001)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
