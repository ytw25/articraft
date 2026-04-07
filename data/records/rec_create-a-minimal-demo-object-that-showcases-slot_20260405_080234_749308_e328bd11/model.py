from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

PANEL_SIZE = (0.18, 0.09)
THICKNESS = 0.004


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slot_pattern_panel_demo")
    finish = model.material("filter_gray", rgba=(0.66, 0.70, 0.73, 1.0))

    filter_face = model.part("filter_face")
    filter_face.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                PANEL_SIZE,
                THICKNESS,
                slot_size=(0.024, 0.006),
                pitch=(0.032, 0.016),
                frame=0.010,
                corner_radius=0.004,
                slot_angle_deg=18.0,
                stagger=True,
            ),
            "filter_face",
        ),
        material=finish,
        name="filter_face",
    )
    filter_face.inertial = Inertial.from_geometry(Box((PANEL_SIZE[0], PANEL_SIZE[1], THICKNESS)), mass=0.20)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    filter_face = object_model.get_part("filter_face")
    ctx.check("filter_face_part_present", filter_face is not None, "Expected a single filter_face part.")
    if filter_face is None:
        return ctx.report()

    ctx.check(
        "filter_face_visual_present",
        filter_face.get_visual("filter_face") is not None,
        "Expected a mesh-backed filter_face visual.",
    )
    aabb = ctx.part_world_aabb(filter_face)
    ctx.check("filter_face_aabb_present", aabb is not None, "Expected a world AABB for the filter face.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    ctx.check(
        "filter_face_overall_size",
        abs(size[0] - PANEL_SIZE[0]) <= 0.004
        and abs(size[1] - PANEL_SIZE[1]) <= 0.004
        and abs(size[2] - THICKNESS) <= 0.002,
        f"size={size!r}",
    )
    ctx.check(
        "filter_face_centered",
        max(abs(value) for value in center) <= 0.002,
        f"center={center!r}",
    )
    return ctx.report()


object_model = build_object_model()
