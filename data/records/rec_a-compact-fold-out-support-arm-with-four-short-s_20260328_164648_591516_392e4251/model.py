from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    """Builds a refined compact fold-out support arm with realistic boxed links."""
    model = ArticulatedObject(name="fold_out_support_arm")

    # Dimensions
    base_dim = (0.09, 0.09, 0.015)
    link_len = 0.10
    link_width = 0.032
    link_thick = 0.022
    bracket_dim = (0.07, 0.07, 0.008)

    # Materials
    brushed_alu = model.material("brushed_aluminum", rgba=(0.8, 0.8, 0.82, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.15, 0.15, 0.16, 1.0))

    # 1. Base Plate
    # Boxed base with mounting holes and beveled edges
    base_shape = (
        cq.Workplane("XY")
        .box(*base_dim)
        .edges("|Z").fillet(0.008)
        .edges(">Z").chamfer(0.002)
        # Mounting holes
        .faces(">Z").workplane()
        .rect(base_dim[0] - 0.02, base_dim[1] - 0.02, forConstruction=True)
        .vertices()
        .hole(0.006)
    )
    base = model.part("base")
    base.visual(mesh_from_cadquery(base_shape, "base_plate"), material=brushed_alu)
    base.inertial = Inertial.from_geometry(Box(base_dim), mass=0.6)

    # 2. Refined Boxed Links
    def make_boxed_link(length, width, thickness, name):
        """Creates a structural boxed link with weight-reduction pockets and hinge pins."""
        # Main body
        link = (
            cq.Workplane("XY")
            .box(length, width, thickness, centered=(False, True, True))
            .edges("|Z").fillet(width / 2.01)
        )
        
        # Side pockets (weight reduction)
        pocket_w = width * 0.7
        pocket_l = length - width
        link = (
            link.faces(">Y").workplane()
            .center(length / 2, 0)
            .rect(pocket_l, thickness * 0.7)
            .cutBlind(-width * 0.15)
            .faces("<Y").workplane()
            .center(length / 2, 0)
            .rect(pocket_l, thickness * 0.7)
            .cutBlind(-width * 0.15)
        )
        
        # End slot for joint (clevis look)
        slot_w = thickness * 0.5
        link = (
            link.faces(">X").workplane()
            .rect(width, slot_w)
            .cutBlind(-width * 0.4)
        )
        
        return link

    prev_part = base
    prev_joint_pos = (0.0, 0.0, base_dim[2] / 2)

    for i in range(1, 5):
        link_name = f"link_{i}"
        link_shape = make_boxed_link(link_len, link_width, link_thick, link_name)
        
        link = model.part(link_name)
        link.visual(mesh_from_cadquery(link_shape, link_name), material=anodized_black)
        link.inertial = Inertial.from_geometry(Box((link_len, link_width, link_thick)), mass=0.12)
        
        model.articulation(
            f"joint_{i}",
            ArticulationType.REVOLUTE,
            parent=prev_part,
            child=link,
            origin=Origin(xyz=prev_joint_pos),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=15.0, velocity=2.5, lower=-2.8, upper=2.8),
        )
        
        prev_part = link
        prev_joint_pos = (link_len, 0.0, 0.0)

    # 3. Platform Bracket
    # Reinforced platform with a grid of mounting holes
    platform_shape = (
        cq.Workplane("XY")
        .box(*bracket_dim)
        .edges("|Z").fillet(0.006)
        # Recessed center for mounting surface
        .faces(">Z").workplane()
        .rect(bracket_dim[0] - 0.01, bracket_dim[1] - 0.01)
        .cutBlind(-0.002)
        # Mounting hole grid
        .faces(">Z").workplane()
        .rect(0.04, 0.04, forConstruction=True)
        .vertices()
        .hole(0.005)
    )
    platform = model.part("platform")
    platform.visual(
        mesh_from_cadquery(platform_shape, "platform_bracket"),
        origin=Origin(xyz=(bracket_dim[0] / 2, 0.0, 0.0)),
        material=brushed_alu
    )
    platform.inertial = Inertial.from_geometry(Box(bracket_dim), mass=0.25)
    
    model.articulation(
        "link4_to_platform",
        ArticulationType.FIXED,
        parent=prev_part,
        child=platform,
        origin=Origin(xyz=(link_len, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    """Runs validation tests for the support arm."""
    ctx = TestContext(object_model)

    # Lookup
    base = object_model.get_part("base")
    link1 = object_model.get_part("link_1")
    link4 = object_model.get_part("link_4")
    platform = object_model.get_part("platform")
    j1 = object_model.get_articulation("joint_1")

    # Baseline
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    
    # Junction allowances
    ctx.allow_overlap(base, link1, reason="hinge junction")
    for i in range(1, 4):
        ctx.allow_overlap(f"link_{i}", f"link_{i+1}", reason="hinge junction")
    ctx.allow_overlap(link4, platform, reason="fixed mount")

    ctx.fail_if_parts_overlap_in_current_pose()

    # Reach test
    with ctx.pose({j1: 0}):
        # Link 1 extends 0.1m, others follow. 
        # Platform origin is at end of link 4.
        ctx.expect_origin_gap(platform, base, axis="x", min_gap=0.35)

    return ctx.report()


object_model = build_object_model()
# >>> USER_CODE_END

object_model = build_object_model()
