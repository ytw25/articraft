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
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_union,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_mixer", assets=ASSETS)

    # Materials
    body_metal = model.material("body_metal", rgba=(0.7, 0.1, 0.1, 1.0))
    bowl_metal = model.material("bowl_metal", rgba=(0.9, 0.9, 0.9, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.1, 0.1, 0.1, 1.0))

    # 1. Base Unit
    base_unit = model.part("base_unit")

    # Base Plate: Extruded rounded rect
    bp_profile = rounded_rect_profile(0.35, 0.22, 0.05)
    bp_geom = ExtrudeGeometry(bp_profile, 0.04)
    bp_mesh = mesh_from_geometry(bp_geom, ASSETS.mesh_path("base_plate.obj"))
    base_unit.visual(
        bp_mesh, origin=Origin(xyz=(0.075, 0.0, 0.02)), material=body_metal, name="base_plate"
    )

    # Neck: Section Loft with integrated Lever
    neck_s0 = [(x, y, 0.0) for x, y in rounded_rect_profile(0.1, 0.12, 0.03)]
    neck_s1 = [(x, y, 0.18) for x, y in rounded_rect_profile(0.08, 0.1, 0.03)]
    neck_s2 = [(x, y, 0.35) for x, y in rounded_rect_profile(0.06, 0.08, 0.025)]
    neck_geom = section_loft([neck_s0, neck_s1, neck_s2])
    
    # Merge lever into neck geometry (using a cylinder for a better lever look)
    lever_geom = CylinderGeometry(radius=0.008, height=0.05).rotate_x(math.pi/2).translate(-0.02, 0.04, 0.21)
    neck_geom = boolean_union(neck_geom, lever_geom)
    
    neck_mesh = mesh_from_geometry(neck_geom, ASSETS.mesh_path("neck.obj"))
    base_unit.visual(
        neck_mesh, origin=Origin(xyz=(-0.06, 0.0, 0.04)), material=body_metal, name="neck"
    )

    base_unit.inertial = Inertial.from_geometry(Box((0.35, 0.22, 0.39)), mass=6.0)

    # 2. Bowl
    bowl = model.part("bowl")
    outer_prof = [(0.02, 0.0), (0.06, 0.01), (0.1, 0.06), (0.1, 0.15), (0.105, 0.16)]
    inner_prof = [(0.0, 0.005), (0.055, 0.015), (0.095, 0.06), (0.095, 0.155)]
    bowl_geom = LatheGeometry.from_shell_profiles(outer_prof, inner_prof, segments=48)
    bowl_mesh = mesh_from_geometry(bowl_geom, ASSETS.mesh_path("bowl.obj"))
    bowl.visual(bowl_mesh, material=bowl_metal, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(Cylinder(radius=0.1, length=0.16), mass=0.8)

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base_unit,
        child=bowl,
        origin=Origin(xyz=(0.1, 0.0, 0.04)),
    )

    # 3. Head
    head = model.part("head")
    def yz_section(w, h, r, x):
        return [(x, y, z) for z, y in rounded_rect_profile(h, w, r)]

    head_s0 = yz_section(0.08, 0.1, 0.03, 0.0)
    head_s1 = yz_section(0.14, 0.16, 0.05, 0.15)
    head_s2 = yz_section(0.1, 0.12, 0.04, 0.35)
    head_geom = section_loft([head_s0, head_s1, head_s2])
    
    # Merge dial into head geometry
    dial_geom = CylinderGeometry(radius=0.03, height=0.01).rotate_y(math.pi/2).translate(0.15, 0.07, 0.0)
    head_geom = boolean_union(head_geom, dial_geom)
    
    head_mesh = mesh_from_geometry(head_geom, ASSETS.mesh_path("head.obj"))
    head.visual(head_mesh, material=body_metal, name="head_shell")
    head.inertial = Inertial.from_geometry(Box((0.35, 0.16, 0.16)), mass=4.0, origin=Origin(xyz=(0.175, 0, 0)))

    # Tilt joint: Axis (0, -1, 0) tilts UP for positive rotation in RHS
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=base_unit,
        child=head,
        origin=Origin(xyz=(-0.06, 0.0, 0.39)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0, velocity=1.0, lower=0.0, upper=math.radians(65)
        ),
    )

    # 4. Hub
    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.04, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=plastic_black,
        name="hub_shell",
    )
    hub.inertial = Inertial.from_geometry(Cylinder(radius=0.04, length=0.05), mass=0.3)

    model.articulation(
        "head_to_hub",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=hub,
        origin=Origin(xyz=(0.25, 0.0, -0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=15.0),
    )

    # 5. Whisk
    whisk = model.part("whisk")
    whisk_wires = []
    num_wires = 8
    for i in range(num_wires):
        angle = i * math.pi / num_wires
        c, s = math.cos(angle), math.sin(angle)
        pts = [
            (0.0, 0.0, 0.0),
            (0.02 * c, 0.02 * s, -0.02),
            (0.04 * c, 0.04 * s, -0.05),
            (0.04 * c, 0.04 * s, -0.08),
            (0.0, 0.0, -0.10),
        ]
        whisk_wires.append(tube_from_spline_points(pts, radius=0.002))

    final_whisk_geom = whisk_wires[0]
    for w_geom in whisk_wires[1:]:
        final_whisk_geom.merge(w_geom)

    whisk_mesh = mesh_from_geometry(final_whisk_geom, ASSETS.mesh_path("whisk.obj"))
    whisk.visual(whisk_mesh, material=bowl_metal, name="whisk_shell")
    whisk.inertial = Inertial.from_geometry(Cylinder(radius=0.05, length=0.10), mass=0.2)

    model.articulation(
        "hub_to_whisk",
        ArticulationType.FIXED,
        parent=hub,
        child=whisk,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    base_unit = object_model.get_part("base_unit")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    hub = object_model.get_part("hub")
    whisk = object_model.get_part("whisk")

    neck_to_head = object_model.get_articulation("neck_to_head")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    # Legitimate overlaps
    ctx.allow_overlap(base_unit, head, reason="Hinge interpenetration")
    ctx.allow_overlap(bowl, whisk, reason="Whisk inside bowl")
    ctx.allow_overlap(head, hub, reason="Hub seated in head")
    ctx.fail_if_parts_overlap_in_current_pose()

    # Attachment checks
    ctx.expect_contact(bowl, base_unit)
    ctx.expect_contact(head, base_unit)
    ctx.expect_contact(hub, head)
    ctx.expect_contact(whisk, hub)

    # Bowl containment
    ctx.expect_within(bowl, base_unit, axes="xy")

    # Whisk in bowl check at rest
    ctx.expect_overlap(whisk, bowl, axes="xy", min_overlap=0.01)

    # Movement checks
    # Positive rotation with axis (0, -1, 0) tilts UP
    with ctx.pose({neck_to_head: math.radians(60)}):
        ctx.expect_gap(whisk, bowl, axis="z", min_gap=0.05)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
