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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="microscope")

    # Colors / Materials
    metal_dark = model.material("metal_dark", rgba=(0.15, 0.15, 0.15, 1.0))
    metal_light = model.material("metal_light", rgba=(0.75, 0.75, 0.75, 1.0))
    chrome = model.material("chrome", rgba=(0.9, 0.9, 0.9, 1.0))

    # 1. Base
    base = model.part("base")
    base_geom = ExtrudeGeometry(rounded_rect_profile(0.16, 0.20, 0.02), 0.03)
    base.visual(
        mesh_from_geometry(base_geom, "base_plate_mesh"),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=metal_dark,
        name="base_plate",
    )
    base.inertial = Inertial.from_geometry(Box((0.16, 0.20, 0.03)), mass=2.0, origin=Origin(xyz=(0.0, 0.0, 0.015)))

    # 2. Column (Fixed to Base)
    column = model.part("column")
    column.visual(
        Box((0.04, 0.04, 0.30)),
        origin=Origin(xyz=(0.0, -0.07, 0.15)),
        material=metal_dark,
        name="column_post",
    )
    model.articulation(
        "base_to_column",
        ArticulationType.FIXED,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    # 3. Stage (Fixed to Column)
    stage = model.part("stage")
    # Stage with central light hole
    stage_outer = rounded_rect_profile(0.12, 0.12, 0.01)
    # Simple square hole for light
    stage_hole = [[-0.01, -0.01], [0.01, -0.01], [0.01, 0.01], [-0.01, 0.01]]
    stage_geom = ExtrudeWithHolesGeometry(stage_outer, [stage_hole], 0.01)
    
    stage.visual(
        mesh_from_geometry(stage_geom, "stage_platform_mesh"),
        origin=Origin(xyz=(0.0, 0.06, 0.005)),
        material=metal_dark,
        name="stage_platform",
    )
    stage.visual(
        Box((0.04, 0.05, 0.02)),
        origin=Origin(xyz=(0.0, -0.02, 0.005)),
        material=metal_dark,
        name="stage_arm",
    )
    model.articulation(
        "column_to_stage",
        ArticulationType.FIXED,
        parent=column,
        child=stage,
        origin=Origin(xyz=(0.0, -0.05, 0.12)),
    )

    # 4. Carriage (Prismatic on Column)
    carriage = model.part("carriage")
    carriage.visual(
        Box((0.06, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, 0.03, 0.0)),
        material=metal_light,
        name="carriage_body",
    )
    carriage.visual(
        Cylinder(radius=0.015, length=0.15),
        origin=Origin(xyz=(0.0, 0.06, 0.05)),
        material=metal_dark,
        name="optical_tube",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.04),
        origin=Origin(xyz=(0.0, 0.06, 0.145)),
        material=metal_dark,
        name="eyepiece_tube",
    )
    # Turret
    carriage.visual(
        Cylinder(radius=0.03, length=0.02),
        origin=Origin(xyz=(0.0, 0.06, -0.035)),
        material=metal_light,
        name="objective_turret",
    )
    # Objectives (3 small cylinders)
    for i, angle in enumerate([0, 2.09, 4.18]):
        ox = 0.015 * math.cos(angle)
        oy = 0.015 * math.sin(angle)
        carriage.visual(
            Cylinder(radius=0.006, length=0.025),
            origin=Origin(xyz=(ox, 0.06 + oy, -0.055)),
            material=chrome,
            name=f"objective_{i}",
        )

    model.articulation(
        "focus_slider",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.05, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.1, lower=-0.04, upper=0.04),
    )

    # 5. Focus Knobs (Continuous on Carriage)
    knobs = model.part("focus_knobs")
    knobs.visual(
        Cylinder(radius=0.02, length=0.015),
        origin=Origin(xyz=(-0.04, 0.0, 0.0), rpy=(0.0, 1.57, 0.0)),
        material=metal_dark,
        name="knob_left",
    )
    knobs.visual(
        Cylinder(radius=0.02, length=0.015),
        origin=Origin(xyz=(0.04, 0.0, 0.0), rpy=(0.0, 1.57, 0.0)),
        material=metal_dark,
        name="knob_right",
    )
    knobs.visual(
        Cylinder(radius=0.005, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.57, 0.0)),
        material=chrome,
        name="knob_shaft",
    )

    model.articulation(
        "focus_knobs_joint",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=knobs,
        origin=Origin(xyz=(0.0, 0.03, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    stage = object_model.get_part("stage")
    carriage = object_model.get_part("carriage")
    knobs = object_model.get_part("focus_knobs")

    ctx.allow_overlap(stage, column, reason="Stage arm mounts to column")
    ctx.allow_overlap(knobs, carriage, reason="Knob shaft passes through carriage body")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(stage, base, axis="z", min_gap=0.05)
    ctx.expect_gap(carriage, column, axis="y", min_gap=-0.001, max_gap=0.01)

    return ctx.report()


object_model = build_object_model()
# >>> USER_CODE_END

object_model = build_object_model()
