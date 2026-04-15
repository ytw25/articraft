from __future__ import annotations

from sdk import ArticulatedObject, Box, Origin, TestContext, TestReport

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="small_cube")
    cube = model.part("cube")
    cube.visual(
        Box((0.1, 0.1, 0.1)),
        origin=Origin(xyz=(0, 0, 0.05)),
        name="cube_visual",
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cube = object_model.get_part("cube")
    ctx.expect_overlap(cube, cube, axes="xyz", min_overlap=0.1, name="cube_size_check")
    return ctx.report()


object_model = build_object_model()
