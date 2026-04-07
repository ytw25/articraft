from __future__ import annotations

from sdk import (
    ArticulatedObject,
    Box,
    Inertial,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

PANEL_SIZE = (0.16, 0.10)
THICKNESS = 0.004


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="perforated_panel_demo")
    finish = model.material("speaker_black", rgba=(0.12, 0.13, 0.14, 1.0))

    speaker_face = model.part("speaker_face")
    speaker_face.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                PANEL_SIZE,
                THICKNESS,
                hole_diameter=0.006,
                pitch=(0.012, 0.012),
                frame=0.010,
                corner_radius=0.004,
                stagger=True,
            ),
            "speaker_face",
        ),
        material=finish,
        name="speaker_face",
    )
    speaker_face.inertial = Inertial.from_geometry(Box((PANEL_SIZE[0], PANEL_SIZE[1], THICKNESS)), mass=0.18)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    speaker_face = object_model.get_part("speaker_face")
    ctx.check("speaker_face_part_present", speaker_face is not None, "Expected a single speaker_face part.")
    if speaker_face is None:
        return ctx.report()

    ctx.check(
        "speaker_face_visual_present",
        speaker_face.get_visual("speaker_face") is not None,
        "Expected a mesh-backed speaker_face visual.",
    )
    aabb = ctx.part_world_aabb(speaker_face)
    ctx.check("speaker_face_aabb_present", aabb is not None, "Expected a world AABB for the speaker face.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    center = tuple(float((maxs[i] + mins[i]) * 0.5) for i in range(3))
    ctx.check(
        "speaker_face_overall_size",
        abs(size[0] - PANEL_SIZE[0]) <= 0.004
        and abs(size[1] - PANEL_SIZE[1]) <= 0.004
        and abs(size[2] - THICKNESS) <= 0.002,
        f"size={size!r}",
    )
    ctx.check(
        "speaker_face_centered",
        max(abs(value) for value in center) <= 0.002,
        f"center={center!r}",
    )
    return ctx.report()


object_model = build_object_model()
