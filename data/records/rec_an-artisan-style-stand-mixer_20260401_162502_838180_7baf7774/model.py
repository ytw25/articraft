from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
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
    LatheGeometry,
    CylinderGeometry,
    tube_from_spline_points,
    mesh_from_geometry,
    section_loft,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stand_mixer")

    # Materials
    metal_paint = model.material("metal_paint", rgba=(0.8, 0.1, 0.1, 1.0))  # Classic red
    chrome = model.material("chrome", rgba=(0.8, 0.8, 0.8, 1.0))

    # --- Body (Base + Column) ---
    body = model.part("body")

    # Base plate: rounded rectangle
    base_size = (0.36, 0.22, 0.04)
    base_geom = section_loft(
        [
            [
                (x, y, 0.0)
                for x, y in rounded_rect_profile(base_size[0], base_size[1], 0.05)
            ],
            [
                (x, y, base_size[2])
                for x, y in rounded_rect_profile(base_size[0] - 0.01, base_size[1] - 0.01, 0.04)
            ],
        ]
    )
    # Add a seat for the bowl
    base_geom.merge(
        CylinderGeometry(0.08, 0.01).translate(0.06, 0.0, base_size[2])
    )
    body.visual(
        mesh_from_geometry(base_geom, "base_plate"),
        material=metal_paint,
        name="base_plate",
    )

    # Column: tapered pillar at the back
    column_height = 0.28
    column_sections = [
        # Bottom
        [
            (x - 0.12, y, base_size[2])
            for x, y in rounded_rect_profile(0.12, 0.14, 0.04)
        ],
        # Middle
        [
            (x - 0.12, y, column_height * 0.5)
            for x, y in rounded_rect_profile(0.10, 0.12, 0.03)
        ],
        # Top
        [
            (x - 0.12, y, column_height)
            for x, y in rounded_rect_profile(0.08, 0.10, 0.03)
        ],
    ]
    column_geom = section_loft(column_sections)
    # Add a hinge boss at the top
    column_geom.merge(
        CylinderGeometry(0.04, 0.12).rotate_x(math.pi / 2).translate(-0.12, 0.0, column_height)
    )
    body.visual(
        mesh_from_geometry(column_geom, "column"),
        material=metal_paint,
        name="column",
    )

    body.inertial = Inertial.from_geometry(Box(base_size), mass=5.0)

    # --- Bowl ---
    bowl = model.part("bowl")
    # Simple bowl shape using Lathe
    bowl_radius = 0.11
    bowl_height = 0.16
    bowl_profile = [
        (0.0, 0.0),
        (0.06, 0.01),
        (0.10, 0.06),
        (bowl_radius, bowl_height),
        (bowl_radius + 0.005, bowl_height),  # Rim
    ]
    bowl_inner_profile = [
        (0.0, 0.002),
        (0.058, 0.012),
        (0.098, 0.062),
        (bowl_radius - 0.002, bowl_height - 0.002),
    ]
    bowl_geom = LatheGeometry.from_shell_profiles(bowl_profile, bowl_inner_profile)
    # Position bowl on base
    bowl.visual(
        mesh_from_geometry(bowl_geom, "bowl_shell"),
        origin=Origin(xyz=(0.06, 0.0, base_size[2])),
        material=chrome,
        name="bowl_shell",
    )
    bowl.inertial = Inertial.from_geometry(Cylinder(bowl_radius, bowl_height), mass=1.0)

    # --- Head ---
    head = model.part("head")
    # Head housing: horizontal rounded shape
    head_length = 0.32
    head_width = 0.14
    head_height = 0.12
    # Define sections for a horizontal loft along X
    head_sections = [
        [(-0.14, y, z) for y, z in rounded_rect_profile(0.12, 0.12, 0.04)],
        [(0.06, y, z + 0.02) for y, z in rounded_rect_profile(0.16, 0.14, 0.06)],
        [(0.28, y, z - 0.02) for y, z in rounded_rect_profile(0.14, 0.12, 0.05)],
    ]
    from sdk import SectionLoftSpec, LoftSection
    head_spec = SectionLoftSpec(
        sections=tuple(LoftSection(points=tuple(tuple(float(v) for v in p) for p in s)) for s in head_sections),
        cap=True,
        solid=True
    )
    head_geom = section_loft(head_spec).translate(0, 0, 0.02)
    # Add beater hub
    head_geom.merge(
        CylinderGeometry(0.025, 0.1).translate(0.18, 0.0, 0.0)
    )

    head.visual(
        mesh_from_geometry(head_geom, "head_housing"),
        material=metal_paint,
        name="head_housing",
    )
    head.inertial = Inertial.from_geometry(Box((head_length, head_width, head_height)), mass=3.0)

    # Tilt Articulation
    # Hinge is at the top of the column
    model.articulation(
        "tilt_head",
        ArticulationType.REVOLUTE,
        parent=body,
        child=head,
        origin=Origin(xyz=(-0.12, 0.0, column_height)),
        axis=(0.0, -1.0, 0.0),  # Positive rotates head UP
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.8),
    )

    # --- Beater (Whisk) ---
    beater = model.part("beater")
    # Whisk wires
    whisk_wires = []
    num_wires = 6
    for i in range(num_wires):
        angle = (i / num_wires) * math.pi * 2
        c = math.cos(angle)
        s = math.sin(angle)
        points = [
            (0.0, 0.0, 0.0),
            (0.03 * c, 0.03 * s, -0.04),
            (0.05 * c, 0.05 * s, -0.09),
            (0.03 * c, 0.03 * s, -0.14),
            (0.0, 0.0, -0.15),
        ]
        whisk_wires.append(tube_from_spline_points(points, radius=0.002))

    # Merge whisk wires
    final_whisk_geom = whisk_wires[0]
    for w in whisk_wires[1:]:
        final_whisk_geom.merge(w)

    # Add central shaft
    final_whisk_geom.merge(
        CylinderGeometry(0.008, 0.04).translate(0, 0, -0.02)
    )

    beater.visual(
        mesh_from_geometry(final_whisk_geom, "whisk"),
        material=chrome,
        name="whisk",
    )
    beater.inertial = Inertial.from_geometry(Cylinder(0.05, 0.15), mass=0.2)

    # Beater Articulation
    model.articulation(
        "beater_rotation",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=beater,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),  # Attached to hub
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=10.0),
    )

    # --- Speed Knob ---
    knob = model.part("knob")
    from sdk import CapsuleGeometry
    knob_geom = CapsuleGeometry(radius=0.015, length=0.01).rotate_x(math.pi / 2)
    knob.visual(
        mesh_from_geometry(knob_geom, "knob_visual"),
        material=chrome,
        name="knob_visual",
    )
    knob.inertial = Inertial.from_geometry(Cylinder(0.02, 0.015), mass=0.05)

    model.articulation(
        "speed_control",
        ArticulationType.REVOLUTE,
        parent=head,
        child=knob,
        origin=Origin(xyz=(0.0, 0.096, 0.04)),  # Side of head, flush
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=0.0, upper=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    head = object_model.get_part("head")
    beater = object_model.get_part("beater")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Allow intentional overlaps for mechanical fits
    ctx.allow_overlap(
        head, body,
        reason="The head housing intentionally encloses the hinge boss on the column."
    )
    ctx.allow_overlap(
        beater, head,
        reason="The beater shaft is inserted into the hub socket in the head."
    )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    return ctx.report()


object_model = build_object_model()


# >>> USER_CODE_END

object_model = build_object_model()
