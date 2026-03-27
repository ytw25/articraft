from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path
import math
from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    Inertial,
    MotionLimits,
    Origin,
    Material,
    TestContext,
    TestReport,
    MeshGeometry,
    BoxGeometry,
    CylinderGeometry,
    SphereGeometry,
    LatheGeometry,
    SectionLoftSpec,
    section_loft,
    repair_loft,
    tube_from_spline_points,
    mesh_from_geometry,
    superellipse_profile,
    ExtrudeGeometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_stand_mixer", assets=ASSETS)

    # Materials
    red_paint = model.material("empire_red", rgba=(0.7, 0.1, 0.1, 1.0))
    chrome = model.material("chrome", rgba=(0.8, 0.8, 0.8, 1.0))
    steel = model.material("brushed_steel", rgba=(0.6, 0.6, 0.6, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.1, 0.1, 0.1, 1.0))

    # --- Base and Pedestal ---
    base_pedestal = model.part("base_pedestal")

    # The base plate (teardrop shape)
    base_profile = superellipse_profile(0.22, 0.36, exponent=2.5, segments=64)
    base_profile = [(x, y + 0.12) for x, y in base_profile]
    base_geom = ExtrudeGeometry.from_z0(base_profile, 0.04)

    # The vertical pedestal/neck
    # Rises from the back of the base
    neck_sections = [
        [(x, y + 0.05, 0.04) for x, y in superellipse_profile(0.12, 0.10, exponent=3.0, segments=32)],
        [(x, y + 0.05, 0.15) for x, y in superellipse_profile(0.10, 0.08, exponent=3.0, segments=32)],
        [(x, y + 0.05, 0.30) for x, y in superellipse_profile(0.08, 0.06, exponent=3.0, segments=32)],
    ]
    neck_geom = section_loft(neck_sections)

    base_pedestal.visual(mesh_from_geometry(base_geom, ASSETS.mesh_path("base.obj")), material=red_paint, name="base_plate")
    base_pedestal.visual(mesh_from_geometry(neck_geom, ASSETS.mesh_path("neck.obj")), material=red_paint, name="neck_column")

    # Internal hinge pin for proximity check
    base_pedestal.visual(Cylinder(radius=0.02, length=0.12), origin=Origin(xyz=(0, 0.05, 0.30), rpy=(0, 1.57, 0)), material=chrome, name="base_hinge_pin")

    # Bowl seat (circular indent on the base)
    bowl_seat_vis = base_pedestal.visual(
        Cylinder(radius=0.09, length=0.01),
        origin=Origin(xyz=(0, 0.22, 0.045)),
        material=chrome,
        name="bowl_seat"
    )

    base_pedestal.inertial = Inertial.from_geometry(Box((0.22, 0.36, 0.3)), mass=5.0, origin=Origin(xyz=(0, 0.15, 0.15)))

    # --- Head ---
    head = model.part("head")

    # The head is a horizontal rounded enclosure
    # Shifted sections to ensure the hinge point (0,0,0) is well-embedded
    # Slightly shallower height to clear bowl rim (0.225)
    head_sections = [
        [(x, -0.08, z) for x, z in superellipse_profile(0.08, 0.09, exponent=2.5, segments=32)], 
        [(x, -0.02, z + 0.01) for x, z in superellipse_profile(0.11, 0.12, exponent=2.5, segments=32)], 
        [(x, 0.10, z + 0.02) for x, z in superellipse_profile(0.13, 0.14, exponent=2.5, segments=32)], 
        [(x, 0.23, z + 0.015) for x, z in superellipse_profile(0.11, 0.12, exponent=2.5, segments=32)], 
        [(x, 0.28, z + 0.005) for x, z in superellipse_profile(0.07, 0.07, exponent=4.0, segments=32)], 
    ]
    head_geom = section_loft(head_sections)
    head_shell_vis = head.visual(mesh_from_geometry(head_geom, ASSETS.mesh_path("head.obj")), material=red_paint, name="head_shell")

    # Internal hinge pin for proximity check
    head.visual(Cylinder(radius=0.02, length=0.12), origin=Origin(xyz=(0, 0, 0), rpy=(0, 1.57, 0)), material=chrome, name="head_hinge_pin")

    # Attachment hub (where the beater goes)
    hub_housing_vis = head.visual(
        Cylinder(radius=0.04, length=0.10),
        origin=Origin(xyz=(0, 0.21, -0.02)), # Extended to ensure connection to shell
        material=chrome,
        name="hub_housing"
    )

    # Controls: Speed lever (side)
    head.visual(Cylinder(radius=0.01, length=0.02), origin=Origin(xyz=(0.06, 0.07, 0.02), rpy=(0, 1.57, 0)), material=chrome, name="speed_lever_base")
    head.visual(Sphere(radius=0.012), origin=Origin(xyz=(0.08, 0.07, 0.02)), material=black_plastic, name="speed_lever_knob")

    # Tilt lock lever (other side)
    head.visual(Cylinder(radius=0.01, length=0.02), origin=Origin(xyz=(-0.06, 0.07, 0.02), rpy=(0, -1.57, 0)), material=chrome, name="tilt_lock_base")
    head.visual(Sphere(radius=0.012), origin=Origin(xyz=(-0.08, 0.07, 0.02)), material=black_plastic, name="tilt_lock_knob")

    head.inertial = Inertial.from_geometry(Box((0.15, 0.35, 0.15)), mass=3.0, origin=Origin(xyz=(0, 0.1, 0)))

    # --- Bowl ---
    bowl = model.part("bowl")
    bowl_profile = [
        (0.0, 0.0),
        (0.08, 0.0),
        (0.11, 0.05),
        (0.12, 0.18),
        (0.125, 0.185),
        (0.115, 0.185),
        (0.115, 0.175),
        (0.105, 0.05),
        (0.075, 0.01),
        (0.0, 0.01),
    ]
    bowl_geom = LatheGeometry(bowl_profile, segments=48)
    bowl_shell_vis = bowl.visual(mesh_from_geometry(bowl_geom, ASSETS.mesh_path("bowl.obj")), material=steel, name="bowl_shell")
    
    # Bowl handle - moved further inward to ensure connection
    handle_path = [
        (0.11, 0.0, 0.14),
        (0.15, 0.0, 0.10),
        (0.11, 0.0, 0.06),
    ]
    handle_geom = tube_from_spline_points(handle_path, radius=0.008)
    bowl.visual(mesh_from_geometry(handle_geom, ASSETS.mesh_path("bowl_handle.obj")), material=steel, name="bowl_handle")

    bowl.inertial = Inertial.from_geometry(Cylinder(radius=0.12, length=0.2), mass=1.0, origin=Origin(xyz=(0, 0, 0.09)))

    # --- Beater ---
    beater = model.part("beater")
    # Shifted down slightly to avoid hub overlap
    beater_frame_points = [
        (0, 0, -0.01),
        (0, 0, -0.06),
        (0.04, 0, -0.11),
        (0, 0, -0.16),
        (-0.04, 0, -0.11),
        (0, 0, -0.06),
    ]
    beater_geom = tube_from_spline_points(beater_frame_points, radius=0.005, closed_spline=True)
    cross_bar = tube_from_spline_points([(0,0,-0.06), (0,0,-0.16)], radius=0.004)
    beater_geom.merge(cross_bar)

    beater_wire_vis = beater.visual(mesh_from_geometry(beater_geom, ASSETS.mesh_path("beater.obj")), material=chrome, name="beater_wire")
    beater.inertial = Inertial.from_geometry(Box((0.08, 0.02, 0.15)), mass=0.2, origin=Origin(xyz=(0, 0, -0.085)))

    # --- Articulations ---

    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=base_pedestal,
        child=head,
        origin=Origin(xyz=(0, 0.05, 0.30)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=0.8),
    )

    model.articulation(
        "beater_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0, 0.21, -0.02)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10.0, velocity=10.0),
    )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base_pedestal,
        child=bowl,
        origin=Origin(xyz=(0, 0.22, 0.04)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    base = object_model.get_part("base_pedestal")
    head = object_model.get_part("head")
    bowl = object_model.get_part("bowl")
    beater = object_model.get_part("beater")
    tilt_hinge = object_model.get_articulation("tilt_hinge")

    # Resolve visuals
    neck_column = base.get_visual("neck_column")
    head_shell = head.get_visual("head_shell")
    bowl_seat = base.get_visual("bowl_seat")
    bowl_shell = bowl.get_visual("bowl_shell")
    hub_housing = head.get_visual("hub_housing")
    beater_wire = beater.get_visual("beater_wire")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()

    # Allow overlap at the hinge and beater mount
    ctx.allow_overlap(head, base, reason="Head pivots inside neck column")
    ctx.allow_overlap(beater, head, reason="Beater mounts into hub")

    ctx.check_articulation_overlaps(max_pose_samples=16)
    ctx.warn_if_overlaps(max_pose_samples=16, ignore_adjacent=True, ignore_fixed=True)

    # Bowl should be on the base seat
    ctx.expect_overlap(bowl, base, axes="xy", min_overlap=0.1)
    ctx.expect_gap(bowl, base, axis="z", max_gap=0.005, max_penetration=0.01, positive_elem=bowl_shell, negative_elem=bowl_seat)

    # Beater should be inside or near the bowl when head is down
    with ctx.pose({tilt_hinge: 0.0}):
        ctx.expect_within(beater, bowl, axes="xy")

    # Test tilt up
    with ctx.pose({tilt_hinge: 0.8}):
        ctx.expect_gap(beater, bowl, axis="z", min_gap=0.05)

    return ctx.report()


object_model = build_object_model()


# >>> USER_CODE_END

object_model = build_object_model()
