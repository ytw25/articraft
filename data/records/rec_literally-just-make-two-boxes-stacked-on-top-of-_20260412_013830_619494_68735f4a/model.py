from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacked_boxes")

    # Bottom box
    bottom = model.part("bottom_box")
    bottom.visual(
        Box((0.2, 0.2, 0.1)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="bottom_visual",
    )

    # Top box
    top = model.part("top_box")
    top.visual(
        Box((0.2, 0.2, 0.1)),
        # The top box part frame will be at the hinge line: (x=-0.1, y=0, z=0.1)
        # So its visual center (0, 0, 0.15) in world is (0.1, 0, 0.05) in part frame.
        origin=Origin(xyz=(0.1, 0.0, 0.05)),
        name="top_visual",
    )

    # Hinge articulation
    model.articulation(
        "hinge",
        ArticulationType.REVOLUTE,
        parent=bottom,
        child=top,
        origin=Origin(xyz=(-0.1, 0.0, 0.1)),
        axis=(0.0, 1.0, 0.0),  # Rotate around Y axis
        motion_limits=MotionLimits(lower=0.0, upper=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottom = object_model.get_part("bottom_box")
    top = object_model.get_part("top_box")
    hinge = object_model.get_articulation("hinge")

    # At q=0, top should be exactly on top of bottom
    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(top, bottom, axis="z", max_gap=0.001, max_penetration=0.0)
        ctx.expect_overlap(top, bottom, axes="xy", min_overlap=0.19)

    return ctx.report()


object_model = build_object_model()
