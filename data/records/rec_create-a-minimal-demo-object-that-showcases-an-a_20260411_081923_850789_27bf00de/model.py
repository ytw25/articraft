from __future__ import annotations

from sdk import (
    ArticulatedObject,
    BezelCutout,
    BezelEdgeFeature,
    BezelGeometry,
    BezelMounts,
    Box,
    Inertial,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

OUTER_SIZE = (0.112, 0.082)
DEPTH = 0.012


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="asym_panel_bezel_demo")
    finish = model.material("panel_bezel_blue", rgba=(0.20, 0.28, 0.36, 1.0))

    bezel = model.part("asym_panel_bezel")
    bezel.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.070, 0.044),
                OUTER_SIZE,
                DEPTH,
                outer_corner_radius=0.008,
                wall=(0.012, 0.020, 0.010, 0.014),
                mounts=BezelMounts(
                    style="bosses",
                    hole_count=4,
                    hole_diameter=0.003,
                    boss_diameter=0.007,
                    setback=0.004,
                ),
                cutouts=(BezelCutout(edge="right", width=0.016, depth=0.004),),
                edge_features=(BezelEdgeFeature(style="notch", edge="bottom", size=0.004, extent=0.016),),
            ),
            "asym_panel_bezel",
        ),
        material=finish,
        name="asym_panel_bezel",
    )
    bezel.inertial = Inertial.from_geometry(Box((OUTER_SIZE[0], OUTER_SIZE[1], 0.016)), mass=0.12)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bezel = object_model.get_part("asym_panel_bezel")
    ctx.check("asym_panel_bezel_part_present", bezel is not None, "Expected an asym_panel_bezel part.")
    if bezel is None:
        return ctx.report()

    ctx.check(
        "asym_panel_bezel_visual_present",
        bezel.get_visual("asym_panel_bezel") is not None,
        "Expected a mesh-backed asym_panel_bezel visual.",
    )
    aabb = ctx.part_world_aabb(bezel)
    ctx.check("asym_panel_bezel_aabb_present", aabb is not None, "Expected a world AABB for the bezel.")
    if aabb is None:
        return ctx.report()

    mins, maxs = aabb
    size = tuple(float(maxs[i] - mins[i]) for i in range(3))
    ctx.check("asym_panel_bezel_width", 0.108 <= size[0] <= 0.118, f"size={size!r}")
    ctx.check("asym_panel_bezel_height", 0.078 <= size[1] <= 0.086, f"size={size!r}")
    ctx.check("asym_panel_bezel_depth", 0.010 <= size[2] <= 0.018, f"size={size!r}")
    return ctx.report()


object_model = build_object_model()
