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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RING_SEGMENTS = (
    ("east", 0.0),
    ("ene", math.pi / 8.0),
    ("ne", math.pi / 4.0),
    ("nne", 3.0 * math.pi / 8.0),
    ("north", math.pi / 2.0),
    ("nnw", 5.0 * math.pi / 8.0),
    ("nw", 3.0 * math.pi / 4.0),
    ("wnw", 7.0 * math.pi / 8.0),
    ("west", math.pi),
    ("wsw", 9.0 * math.pi / 8.0),
    ("sw", 5.0 * math.pi / 4.0),
    ("ssw", 11.0 * math.pi / 8.0),
    ("south", 3.0 * math.pi / 2.0),
    ("sse", 13.0 * math.pi / 8.0),
    ("se", 7.0 * math.pi / 4.0),
    ("ese", 15.0 * math.pi / 8.0),
)

ASSETS = AssetContext.from_script(__file__)


def _add_ring_boxes(
    part,
    prefix: str,
    *,
    center_x: float,
    center_z: float,
    radius: float,
    radial_thickness: float,
    depth: float,
    material,
    y: float,
) -> None:
    tangent = 2.0 * radius * math.sin(math.pi / len(RING_SEGMENTS)) * 1.12
    for label, angle in RING_SEGMENTS:
        part.visual(
            Box((tangent, depth, radial_thickness)),
            origin=Origin(
                xyz=(
                    center_x + radius * math.cos(angle),
                    y,
                    center_z + radius * math.sin(angle),
                ),
                rpy=(0.0, (math.pi / 2.0) - angle, 0.0),
            ),
            material=material,
            name=f"{prefix}_{label}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_front_load_washer", assets=ASSETS)

    cabinet_white = model.material("cabinet_white", rgba=(0.95, 0.96, 0.97, 1.0))
    appliance_grey = model.material("appliance_grey", rgba=(0.77, 0.79, 0.82, 1.0))
    drum_shadow = model.material("drum_shadow", rgba=(0.18, 0.19, 0.21, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    hinge_grey = model.material("hinge_grey", rgba=(0.56, 0.58, 0.61, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.66, 0.79, 0.86, 0.30))
    rail_grey = model.material("rail_grey", rgba=(0.69, 0.70, 0.73, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.595, 0.585, 0.022)),
        origin=Origin(xyz=(0.0, 0.2925, 0.011)),
        material=cabinet_white,
        name="base_plinth",
    )
    cabinet.visual(
        Box((0.022, 0.585, 0.801)),
        origin=Origin(xyz=(-0.2865, 0.2925, 0.4225)),
        material=cabinet_white,
        name="left_side",
    )
    cabinet.visual(
        Box((0.022, 0.585, 0.801)),
        origin=Origin(xyz=(0.2865, 0.2925, 0.4225)),
        material=cabinet_white,
        name="right_side",
    )
    cabinet.visual(
        Box((0.551, 0.585, 0.022)),
        origin=Origin(xyz=(0.0, 0.2925, 0.834)),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((0.551, 0.018, 0.801)),
        origin=Origin(xyz=(0.0, 0.576, 0.4225)),
        material=cabinet_white,
        name="rear_panel",
    )
    cabinet.visual(
        Box((0.595, 0.028, 0.090)),
        origin=Origin(xyz=(0.0, 0.014, 0.755)),
        material=appliance_grey,
        name="control_band",
    )
    cabinet.visual(
        Box((0.102, 0.028, 0.596)),
        origin=Origin(xyz=(-0.236, 0.014, 0.390)),
        material=cabinet_white,
        name="left_front_stile",
    )
    cabinet.visual(
        Box((0.102, 0.028, 0.596)),
        origin=Origin(xyz=(0.236, 0.014, 0.390)),
        material=cabinet_white,
        name="right_front_stile",
    )
    cabinet.visual(
        Box((0.430, 0.028, 0.120)),
        origin=Origin(xyz=(-0.075, 0.014, 0.118)),
        material=cabinet_white,
        name="lower_front_band",
    )
    cabinet.visual(
        Box((0.100, 0.028, 0.044)),
        origin=Origin(xyz=(0.242, 0.014, 0.170)),
        material=cabinet_white,
        name="filter_top_band",
    )
    cabinet.visual(
        Box((0.056, 0.028, 0.164)),
        origin=Origin(xyz=(0.269, 0.014, 0.108)),
        material=cabinet_white,
        name="filter_side_band",
    )
    cabinet.visual(
        Box((0.154, 0.012, 0.060)),
        origin=Origin(xyz=(-0.173, 0.006, 0.717)),
        material=drum_shadow,
        name="drawer_bay_face",
    )
    cabinet.visual(
        Box((0.080, 0.012, 0.080)),
        origin=Origin(xyz=(0.242, 0.006, 0.118)),
        material=drum_shadow,
        name="filter_bay_face",
    )
    cabinet.visual(
        Cylinder(radius=0.155, length=0.344),
        origin=Origin(xyz=(0.0, 0.182, 0.405), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=drum_shadow,
        name="cavity_liner",
    )
    cabinet.visual(
        Cylinder(radius=0.145, length=0.028),
        origin=Origin(xyz=(0.0, 0.359, 0.405), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="drum_back",
    )
    cabinet.visual(
        Box((0.121, 0.060, 0.060)),
        origin=Origin(xyz=(-0.215, 0.230, 0.405)),
        material=drum_shadow,
        name="left_tub_support",
    )
    cabinet.visual(
        Box((0.121, 0.060, 0.060)),
        origin=Origin(xyz=(0.215, 0.230, 0.405)),
        material=drum_shadow,
        name="right_tub_support",
    )
    _add_ring_boxes(
        cabinet,
        "seal_outer",
        center_x=0.0,
        center_z=0.405,
        radius=0.173,
        radial_thickness=0.026,
        depth=0.012,
        material=appliance_grey,
        y=-0.006,
    )
    _add_ring_boxes(
        cabinet,
        "seal_inner",
        center_x=0.0,
        center_z=0.405,
        radius=0.154,
        radial_thickness=0.015,
        depth=0.010,
        material=appliance_grey,
        y=-0.009,
    )
    cabinet.visual(
        Box((0.012, 0.014, 0.040)),
        origin=Origin(xyz=(-0.182, -0.007, 0.501)),
        material=hinge_grey,
        name="upper_hinge_mount",
    )
    cabinet.visual(
        Box((0.012, 0.014, 0.040)),
        origin=Origin(xyz=(-0.182, -0.007, 0.309)),
        material=hinge_grey,
        name="lower_hinge_mount",
    )
    cabinet.visual(
        Box((0.016, 0.136, 0.008)),
        origin=Origin(xyz=(-0.173, 0.074, 0.684)),
        material=rail_grey,
        name="drawer_center_rail",
    )
    cabinet.visual(
        Box((0.070, 0.104, 0.086)),
        origin=Origin(xyz=(0.242, 0.058, 0.118)),
        material=drum_shadow,
        name="filter_cavity",
    )
    cabinet.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.242, 0.026, 0.118), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rail_grey,
        name="filter_cap",
    )
    cabinet.visual(
        Box((0.010, 0.012, 0.024)),
        origin=Origin(xyz=(0.210, 0.006, 0.144)),
        material=hinge_grey,
        name="filter_hinge_mount_upper",
    )
    cabinet.visual(
        Box((0.010, 0.012, 0.024)),
        origin=Origin(xyz=(0.210, 0.006, 0.092)),
        material=hinge_grey,
        name="filter_hinge_mount_lower",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((0.595, 0.585, 0.845)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.2925, 0.4225)),
    )

    door = model.part("door")
    _add_ring_boxes(
        door,
        "door_outer",
        center_x=0.188,
        center_z=0.0,
        radius=0.159,
        radial_thickness=0.082,
        depth=0.028,
        material=appliance_grey,
        y=-0.014,
    )
    _add_ring_boxes(
        door,
        "door_inner",
        center_x=0.188,
        center_z=0.0,
        radius=0.136,
        radial_thickness=0.020,
        depth=0.014,
        material=trim_dark,
        y=-0.007,
    )
    door.visual(
        Cylinder(radius=0.109, length=0.006),
        origin=Origin(xyz=(0.188, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=glass_tint,
        name="door_glass",
    )
    door.visual(
        Box((0.110, 0.014, 0.050)),
        origin=Origin(xyz=(0.045, -0.007, 0.0)),
        material=hinge_grey,
        name="hinge_bridge",
    )
    door.visual(
        Box((0.012, 0.012, 0.040)),
        origin=Origin(xyz=(0.006, -0.006, 0.096)),
        material=hinge_grey,
        name="upper_hinge_leaf",
    )
    door.visual(
        Box((0.012, 0.012, 0.040)),
        origin=Origin(xyz=(0.006, -0.006, -0.096)),
        material=hinge_grey,
        name="lower_hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.0, -0.010, 0.096)),
        material=hinge_grey,
        name="upper_knuckle",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.0, -0.010, -0.096)),
        material=hinge_grey,
        name="lower_knuckle",
    )
    door.visual(
        Box((0.022, 0.024, 0.060)),
        origin=Origin(xyz=(0.343, -0.026, 0.0)),
        material=trim_dark,
        name="handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.410, 0.060, 0.420)),
        mass=4.8,
        origin=Origin(xyz=(0.188, -0.015, 0.0)),
    )

    drawer = model.part("detergent_drawer")
    drawer.visual(
        Box((0.150, 0.018, 0.054)),
        origin=Origin(xyz=(0.0, -0.009, 0.0)),
        material=appliance_grey,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.052, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.021, -0.016)),
        material=trim_dark,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.122, 0.118, 0.040)),
        origin=Origin(xyz=(0.0, 0.059, 0.0)),
        material=cabinet_white,
        name="drawer_body",
    )
    drawer.visual(
        Box((0.024, 0.112, 0.006)),
        origin=Origin(xyz=(0.0, 0.059, -0.014)),
        material=rail_grey,
        name="drawer_guide",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.150, 0.138, 0.064)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.060, -0.004)),
    )

    filter_door = model.part("pump_filter_door")
    filter_door.visual(
        Box((0.078, 0.016, 0.078)),
        origin=Origin(xyz=(0.039, -0.003, 0.0)),
        material=appliance_grey,
        name="filter_panel",
    )
    filter_door.visual(
        Box((0.010, 0.010, 0.022)),
        origin=Origin(xyz=(0.005, 0.0, 0.026)),
        material=hinge_grey,
        name="filter_leaf_upper",
    )
    filter_door.visual(
        Box((0.010, 0.010, 0.022)),
        origin=Origin(xyz=(0.005, 0.0, -0.026)),
        material=hinge_grey,
        name="filter_leaf_lower",
    )
    filter_door.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=hinge_grey,
        name="filter_knuckle_upper",
    )
    filter_door.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=hinge_grey,
        name="filter_knuckle_lower",
    )
    filter_door.visual(
        Box((0.010, 0.014, 0.024)),
        origin=Origin(xyz=(0.072, -0.010, 0.0)),
        material=trim_dark,
        name="filter_pull",
    )
    filter_door.inertial = Inertial.from_geometry(
        Box((0.080, 0.020, 0.080)),
        mass=0.2,
        origin=Origin(xyz=(0.040, -0.008, 0.0)),
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.188, -0.014, 0.405)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.4, lower=0.0, upper=1.30),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(-0.173, 0.0, 0.717)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.25, lower=0.0, upper=0.110),
    )
    model.articulation(
        "filter_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=filter_door,
        origin=Origin(xyz=(0.205, -0.005, 0.118)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.20),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    drawer = object_model.get_part("detergent_drawer")
    filter_door = object_model.get_part("pump_filter_door")

    door_hinge = object_model.get_articulation("door_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    filter_hinge = object_model.get_articulation("filter_hinge")

    cavity_liner = cabinet.get_visual("cavity_liner")
    seal_outer_north = cabinet.get_visual("seal_outer_north")
    drawer_bay_face = cabinet.get_visual("drawer_bay_face")
    drawer_center_rail = cabinet.get_visual("drawer_center_rail")
    upper_hinge_mount = cabinet.get_visual("upper_hinge_mount")
    lower_hinge_mount = cabinet.get_visual("lower_hinge_mount")
    filter_bay_face = cabinet.get_visual("filter_bay_face")
    filter_hinge_mount_upper = cabinet.get_visual("filter_hinge_mount_upper")
    filter_hinge_mount_lower = cabinet.get_visual("filter_hinge_mount_lower")

    door_glass = door.get_visual("door_glass")
    door_outer_north = door.get_visual("door_outer_north")
    upper_hinge_leaf = door.get_visual("upper_hinge_leaf")
    lower_hinge_leaf = door.get_visual("lower_hinge_leaf")
    upper_knuckle = door.get_visual("upper_knuckle")
    lower_knuckle = door.get_visual("lower_knuckle")

    drawer_front = drawer.get_visual("drawer_front")
    drawer_guide = drawer.get_visual("drawer_guide")

    filter_panel = filter_door.get_visual("filter_panel")
    filter_leaf_upper = filter_door.get_visual("filter_leaf_upper")
    filter_leaf_lower = filter_door.get_visual("filter_leaf_lower")
    filter_knuckle_upper = filter_door.get_visual("filter_knuckle_upper")
    filter_knuckle_lower = filter_door.get_visual("filter_knuckle_lower")
    filter_pull = filter_door.get_visual("filter_pull")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        drawer,
        cabinet,
        reason="detergent drawer telescopes inside the recessed dispenser pocket around the fixed center guide rail",
    )
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(door, cabinet, axes="xz", elem_a=door_glass, elem_b=cavity_liner, min_overlap=0.04)
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        max_gap=0.012,
        max_penetration=0.0,
        positive_elem=seal_outer_north,
        negative_elem=door_outer_north,
    )
    ctx.expect_contact(door, cabinet, elem_a=upper_hinge_leaf, elem_b=upper_hinge_mount)
    ctx.expect_contact(door, cabinet, elem_a=lower_hinge_leaf, elem_b=lower_hinge_mount)
    ctx.expect_contact(door, cabinet, elem_a=upper_knuckle, elem_b=upper_hinge_mount)
    ctx.expect_contact(door, cabinet, elem_a=lower_knuckle, elem_b=lower_hinge_mount)

    ctx.expect_gap(
        drawer,
        door,
        axis="z",
        min_gap=0.140,
        positive_elem=drawer_front,
        negative_elem=door_glass,
    )
    ctx.expect_overlap(drawer, cabinet, axes="xz", elem_a=drawer_front, elem_b=drawer_bay_face, min_overlap=0.006)
    ctx.expect_gap(
        cabinet,
        drawer,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=drawer_bay_face,
        negative_elem=drawer_front,
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="xy",
        elem_a=drawer_guide,
        elem_b=drawer_center_rail,
        min_overlap=0.010,
    )
    ctx.expect_gap(
        drawer,
        cabinet,
        axis="z",
        max_gap=0.013,
        max_penetration=0.0,
        positive_elem=drawer_guide,
        negative_elem=drawer_center_rail,
    )

    ctx.expect_gap(
        door,
        filter_door,
        axis="z",
        min_gap=0.120,
        positive_elem=door_glass,
        negative_elem=filter_panel,
    )
    ctx.expect_overlap(filter_door, cabinet, axes="xz", elem_a=filter_panel, elem_b=filter_bay_face, min_overlap=0.004)
    ctx.expect_gap(
        cabinet,
        filter_door,
        axis="y",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=filter_bay_face,
        negative_elem=filter_panel,
    )
    ctx.expect_contact(filter_door, cabinet, elem_a=filter_leaf_upper, elem_b=filter_hinge_mount_upper)
    ctx.expect_contact(filter_door, cabinet, elem_a=filter_leaf_lower, elem_b=filter_hinge_mount_lower)
    ctx.expect_contact(filter_door, cabinet, elem_a=filter_knuckle_upper, elem_b=filter_hinge_mount_upper)
    ctx.expect_contact(filter_door, cabinet, elem_a=filter_knuckle_lower, elem_b=filter_hinge_mount_lower)

    with ctx.pose({door_hinge: 1.15}):
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            min_gap=0.080,
            positive_elem=seal_outer_north,
            negative_elem=door_outer_north,
        )

    with ctx.pose({drawer_slide: 0.095}):
        ctx.expect_gap(
            cabinet,
            drawer,
            axis="y",
            min_gap=0.085,
            positive_elem=drawer_bay_face,
            negative_elem=drawer_front,
        )
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="xy",
            elem_a=drawer_guide,
            elem_b=drawer_center_rail,
            min_overlap=0.010,
        )
        ctx.expect_gap(
            drawer,
            cabinet,
            axis="z",
            max_gap=0.013,
            max_penetration=0.0,
            positive_elem=drawer_guide,
            negative_elem=drawer_center_rail,
        )

    with ctx.pose({filter_hinge: 1.05}):
        ctx.expect_gap(
            cabinet,
            filter_door,
            axis="y",
            min_gap=0.032,
            positive_elem=filter_bay_face,
            negative_elem=filter_pull,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
