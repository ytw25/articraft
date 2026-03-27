from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os
import math

_ORIG_GETCWD = os.getcwd

if "__file__" in globals() and not os.path.isabs(__file__):
    __file__ = "/" + __file__.lstrip("./")


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except FileNotFoundError:
            pass
        return "/"


os.getcwd = _safe_getcwd
_safe_getcwd()

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
)
SCRIPT_DIR = "/"

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heat_pump_dryer")

    cabinet_white = model.material("cabinet_white", rgba=(0.92, 0.93, 0.94, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.26, 0.28, 0.30, 1.0))
    charcoal = model.material("charcoal", rgba=(0.14, 0.15, 0.17, 1.0))
    glass = model.material("glass", rgba=(0.56, 0.72, 0.80, 0.35))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    tray_grey = model.material("tray_grey", rgba=(0.78, 0.80, 0.82, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.018, 0.660, 0.850)),
        origin=Origin(xyz=(-0.291, 0.000, 0.425)),
        material=cabinet_white,
        name="left_side_wall",
    )
    cabinet.visual(
        Box((0.018, 0.660, 0.850)),
        origin=Origin(xyz=(0.291, 0.000, 0.425)),
        material=cabinet_white,
        name="right_side_wall",
    )
    cabinet.visual(
        Box((0.564, 0.660, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.841)),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((0.564, 0.660, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=cabinet_white,
        name="bottom_panel",
    )
    cabinet.visual(
        Box((0.564, 0.018, 0.814)),
        origin=Origin(xyz=(0.000, -0.321, 0.425)),
        material=cabinet_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((0.564, 0.018, 0.188)),
        origin=Origin(xyz=(0.000, 0.320, 0.756)),
        material=cabinet_white,
        name="top_header",
    )
    cabinet.visual(
        Box((0.070, 0.018, 0.424)),
        origin=Origin(xyz=(-0.247, 0.320, 0.450)),
        material=cabinet_white,
        name="left_front_jamb",
    )
    cabinet.visual(
        Box((0.070, 0.018, 0.424)),
        origin=Origin(xyz=(0.247, 0.320, 0.450)),
        material=cabinet_white,
        name="right_front_jamb",
    )
    cabinet.visual(
        Box((0.424, 0.018, 0.022)),
        origin=Origin(xyz=(0.000, 0.320, 0.227)),
        material=cabinet_white,
        name="door_sill",
    )
    cabinet.visual(
        Box((0.336, 0.018, 0.220)),
        origin=Origin(xyz=(0.114, 0.320, 0.128)),
        material=cabinet_white,
        name="lower_right_panel",
    )
    cabinet.visual(
        Box((0.564, 0.018, 0.050)),
        origin=Origin(xyz=(0.000, 0.320, 0.025)),
        material=cabinet_white,
        name="bottom_kick",
    )
    cabinet.visual(
        Box((0.228, 0.018, 0.020)),
        origin=Origin(xyz=(-0.168, 0.320, 0.139)),
        material=cabinet_white,
        name="tray_top_frame",
    )
    cabinet.visual(
        Box((0.024, 0.018, 0.079)),
        origin=Origin(xyz=(-0.270, 0.320, 0.0895)),
        material=cabinet_white,
        name="tray_left_frame",
    )
    cabinet.visual(
        Box((0.024, 0.018, 0.079)),
        origin=Origin(xyz=(-0.066, 0.320, 0.0895)),
        material=cabinet_white,
        name="tray_right_frame",
    )
    cabinet.visual(
        Cylinder(radius=0.212, length=0.050),
        origin=Origin(
            xyz=(0.000, 0.284, 0.450),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=charcoal,
        name="drum_throat",
    )
    cabinet.visual(
        Box((0.040, 0.050, 0.120)),
        origin=Origin(xyz=(-0.206, 0.301, 0.450)),
        material=charcoal,
        name="left_drum_bridge",
    )
    cabinet.visual(
        Box((0.040, 0.050, 0.120)),
        origin=Origin(xyz=(0.206, 0.301, 0.450)),
        material=charcoal,
        name="right_drum_bridge",
    )
    cabinet.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(-0.253, 0.346, 0.590)),
        material=satin_metal,
        name="upper_door_knuckle",
    )
    cabinet.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(-0.253, 0.346, 0.290)),
        material=satin_metal,
        name="lower_door_knuckle",
    )
    cabinet.visual(
        Box((0.018, 0.016, 0.050)),
        origin=Origin(xyz=(-0.274, 0.338, 0.590)),
        material=satin_metal,
        name="upper_knuckle_leaf",
    )
    cabinet.visual(
        Box((0.018, 0.016, 0.050)),
        origin=Origin(xyz=(-0.274, 0.338, 0.290)),
        material=satin_metal,
        name="lower_knuckle_leaf",
    )
    cabinet.visual(
        Box((0.018, 0.016, 0.350)),
        origin=Origin(xyz=(-0.274, 0.338, 0.440)),
        material=satin_metal,
        name="door_hinge_spine",
    )
    cabinet.visual(
        Cylinder(radius=0.006, length=0.055),
        origin=Origin(xyz=(0.300, -0.050, 0.500)),
        material=satin_metal,
        name="lint_hinge_barrel",
    )
    cabinet.visual(
        Box((0.010, 0.032, 0.055)),
        origin=Origin(xyz=(0.289, -0.034, 0.500)),
        material=satin_metal,
        name="lint_hinge_leaf",
    )
    cabinet.visual(
        Box((0.180, 0.214, 0.008)),
        origin=Origin(xyz=(-0.170, 0.213, 0.0655)),
        material=dark_grey,
        name="drain_track",
    )
    cabinet.visual(
        Box((0.178, 0.010, 0.038)),
        origin=Origin(xyz=(-0.170, 0.114, 0.087)),
        material=dark_grey,
        name="drain_track_stop",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.600, 0.660, 0.850)),
        mass=56.0,
        origin=Origin(xyz=(0.000, 0.000, 0.425)),
    )

    main_door = model.part("main_door")
    main_door.visual(
        Cylinder(radius=0.245, length=0.026),
        origin=Origin(
            xyz=(0.253, -0.003, 0.000),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_grey,
        name="door_ring",
    )
    main_door.visual(
        Cylinder(radius=0.216, length=0.012),
        origin=Origin(
            xyz=(0.253, -0.011, 0.000),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=charcoal,
        name="door_inner_trim",
    )
    main_door.visual(
        Cylinder(radius=0.182, length=0.010),
        origin=Origin(
            xyz=(0.253, -0.006, 0.000),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=glass,
        name="door_glass",
    )
    main_door.visual(
        Box((0.024, 0.030, 0.110)),
        origin=Origin(xyz=(0.448, 0.010, 0.000)),
        material=trim_grey,
        name="door_pull",
    )
    main_door.visual(
        Box((0.052, 0.018, 0.388)),
        origin=Origin(xyz=(0.068, 0.000, 0.000)),
        material=trim_grey,
        name="door_hinge_stile",
    )
    main_door.visual(
        Box((0.018, 0.014, 0.056)),
        origin=Origin(xyz=(0.021, 0.000, 0.150)),
        material=satin_metal,
        name="upper_hinge_tab",
    )
    main_door.visual(
        Box((0.018, 0.014, 0.056)),
        origin=Origin(xyz=(0.021, 0.000, -0.150)),
        material=satin_metal,
        name="lower_hinge_tab",
    )
    main_door.visual(
        Box((0.026, 0.014, 0.056)),
        origin=Origin(xyz=(0.043, 0.000, 0.150)),
        material=satin_metal,
        name="upper_hinge_leaf",
    )
    main_door.visual(
        Box((0.026, 0.014, 0.056)),
        origin=Origin(xyz=(0.043, 0.000, -0.150)),
        material=satin_metal,
        name="lower_hinge_leaf",
    )
    main_door.inertial = Inertial.from_geometry(
        Box((0.520, 0.050, 0.550)),
        mass=6.8,
        origin=Origin(xyz=(0.253, 0.000, 0.000)),
    )

    lint_filter_door = model.part("lint_filter_door")
    lint_filter_door.visual(
        Box((0.008, 0.220, 0.110)),
        origin=Origin(xyz=(0.010, 0.110, 0.000)),
        material=cabinet_white,
        name="lint_panel",
    )
    lint_filter_door.visual(
        Box((0.010, 0.032, 0.055)),
        origin=Origin(xyz=(0.011, 0.016, 0.000)),
        material=satin_metal,
        name="lint_hinge_leaf",
    )
    lint_filter_door.visual(
        Box((0.012, 0.022, 0.060)),
        origin=Origin(xyz=(0.016, 0.188, 0.000)),
        material=trim_grey,
        name="lint_pull",
    )
    lint_filter_door.inertial = Inertial.from_geometry(
        Box((0.014, 0.220, 0.110)),
        mass=0.35,
        origin=Origin(xyz=(0.009, 0.110, 0.000)),
    )

    drain_tray = model.part("drain_tray")
    drain_tray.visual(
        Box((0.170, 0.206, 0.055)),
        origin=Origin(xyz=(0.000, 0.000, 0.0275)),
        material=tray_grey,
        name="tray_bin",
    )
    drain_tray.visual(
        Box((0.174, 0.004, 0.063)),
        origin=Origin(xyz=(0.000, 0.105, 0.0275)),
        material=trim_grey,
        name="tray_face",
    )
    drain_tray.visual(
        Box((0.060, 0.014, 0.018)),
        origin=Origin(xyz=(0.000, 0.096, 0.0275)),
        material=charcoal,
        name="tray_handle",
    )
    drain_tray.inertial = Inertial.from_geometry(
        Box((0.170, 0.210, 0.060)),
        mass=0.85,
        origin=Origin(xyz=(0.000, 0.000, 0.0275)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=main_door,
        origin=Origin(xyz=(-0.253, 0.346, 0.440)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.8,
            lower=0.0,
            upper=2.1,
        ),
    )
    model.articulation(
        "lint_filter_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lint_filter_door,
        origin=Origin(xyz=(0.300, -0.050, 0.500)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.4,
            lower=-1.25,
            upper=0.0,
        ),
    )
    model.articulation(
        "drain_tray_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drain_tray,
        origin=Origin(xyz=(-0.170, 0.225, 0.0695)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.25,
            lower=0.0,
            upper=0.140,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=SCRIPT_DIR or "/")
    cabinet = object_model.get_part("cabinet")
    main_door = object_model.get_part("main_door")
    lint_filter_door = object_model.get_part("lint_filter_door")
    drain_tray = object_model.get_part("drain_tray")

    door_hinge = object_model.get_articulation("door_hinge")
    lint_hinge = object_model.get_articulation("lint_filter_hinge")
    tray_slide = object_model.get_articulation("drain_tray_slide")

    door_sill = cabinet.get_visual("door_sill")
    right_front_jamb = cabinet.get_visual("right_front_jamb")
    right_side_wall = cabinet.get_visual("right_side_wall")
    upper_knuckle = cabinet.get_visual("upper_door_knuckle")
    lower_knuckle = cabinet.get_visual("lower_door_knuckle")
    lint_barrel = cabinet.get_visual("lint_hinge_barrel")
    drain_track = cabinet.get_visual("drain_track")
    tray_top_frame = cabinet.get_visual("tray_top_frame")

    door_ring = main_door.get_visual("door_ring")
    door_glass = main_door.get_visual("door_glass")
    door_pull = main_door.get_visual("door_pull")
    upper_hinge_tab = main_door.get_visual("upper_hinge_tab")
    lower_hinge_tab = main_door.get_visual("lower_hinge_tab")

    lint_panel = lint_filter_door.get_visual("lint_panel")
    lint_leaf = lint_filter_door.get_visual("lint_hinge_leaf")
    lint_pull = lint_filter_door.get_visual("lint_pull")

    tray_bin = drain_tray.get_visual("tray_bin")
    tray_face = drain_tray.get_visual("tray_face")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(max_pose_samples=128)
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        main_door,
        cabinet,
        axes="xz",
        elem_a=door_ring,
        min_overlap=0.40,
    )
    ctx.expect_within(
        main_door,
        main_door,
        axes="xz",
        inner_elem=door_glass,
        outer_elem=door_ring,
    )
    ctx.expect_gap(
        main_door,
        cabinet,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=door_ring,
        negative_elem=door_sill,
    )
    ctx.expect_contact(main_door, cabinet, elem_a=upper_hinge_tab, elem_b=upper_knuckle)
    ctx.expect_contact(main_door, cabinet, elem_a=lower_hinge_tab, elem_b=lower_knuckle)

    ctx.expect_gap(
        lint_filter_door,
        cabinet,
        axis="x",
        max_gap=0.007,
        max_penetration=0.0,
        positive_elem=lint_panel,
        negative_elem=right_side_wall,
    )
    ctx.expect_within(
        lint_filter_door,
        cabinet,
        axes="yz",
        inner_elem=lint_panel,
        outer_elem=right_side_wall,
    )
    ctx.expect_contact(lint_filter_door, cabinet, elem_a=lint_leaf, elem_b=lint_barrel)

    ctx.expect_gap(
        drain_tray,
        cabinet,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=tray_bin,
        negative_elem=drain_track,
    )
    ctx.expect_overlap(
        drain_tray,
        cabinet,
        axes="xz",
        elem_a=tray_face,
        min_overlap=0.06,
    )
    ctx.expect_gap(
        main_door,
        drain_tray,
        axis="z",
        min_gap=0.06,
        positive_elem=door_ring,
        negative_elem=tray_face,
    )
    ctx.expect_gap(
        lint_filter_door,
        drain_tray,
        axis="z",
        min_gap=0.25,
        positive_elem=lint_panel,
        negative_elem=tray_face,
    )
    ctx.expect_gap(
        lint_filter_door,
        drain_tray,
        axis="x",
        min_gap=0.38,
        positive_elem=lint_panel,
        negative_elem=tray_face,
    )

    with ctx.pose({door_hinge: 1.2}):
        ctx.expect_gap(
            main_door,
            cabinet,
            axis="y",
            min_gap=0.18,
            positive_elem=door_pull,
            negative_elem=right_front_jamb,
        )
        ctx.expect_contact(main_door, cabinet, elem_a=upper_hinge_tab, elem_b=upper_knuckle)

    with ctx.pose({lint_hinge: -1.0}):
        ctx.expect_gap(
            lint_filter_door,
            cabinet,
            axis="x",
            min_gap=0.06,
            positive_elem=lint_pull,
            negative_elem=right_side_wall,
        )
        ctx.expect_contact(lint_filter_door, cabinet, elem_a=lint_leaf, elem_b=lint_barrel)

    with ctx.pose({tray_slide: 0.120}):
        ctx.expect_gap(
            drain_tray,
            cabinet,
            axis="y",
            min_gap=0.10,
            positive_elem=tray_face,
            negative_elem=tray_top_frame,
        )
        ctx.expect_gap(
            drain_tray,
            cabinet,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=tray_bin,
            negative_elem=drain_track,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
