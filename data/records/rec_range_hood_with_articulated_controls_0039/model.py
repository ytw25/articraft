from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _write_knob_mesh():
    return mesh_from_geometry(
        LatheGeometry(
            [
                (0.0, -0.014),
                (0.0175, -0.014),
                (0.0215, -0.010),
                (0.0220, 0.006),
                (0.0205, 0.011),
                (0.0185, 0.014),
                (0.0, 0.014),
            ],
            segments=48,
        ),
        ASSETS.asset_root / "range_hood_knob.obj",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.80, 0.82, 0.84, 1.0))
    charcoal = model.material("charcoal", rgba=(0.17, 0.18, 0.19, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_marker = model.material("satin_marker", rgba=(0.78, 0.80, 0.82, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.55
    canopy_height = 0.18
    shell_thickness = 0.015

    chimney_width = 0.32
    chimney_depth = 0.26
    chimney_height = 0.34
    chimney_center_y = (canopy_depth - chimney_depth) / 2.0

    hood_body = model.part("hood_body")
    hood_body.inertial = Inertial.from_geometry(
        Box((canopy_width, canopy_depth, canopy_height + chimney_height)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, (canopy_height + chimney_height) / 2.0)),
    )

    hood_body.visual(
        Box((canopy_width, canopy_depth, shell_thickness)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height - shell_thickness / 2.0)),
        material=stainless,
        name="canopy_top",
    )
    hood_body.visual(
        Box((shell_thickness, canopy_depth, canopy_height)),
        origin=Origin(
            xyz=(-canopy_width / 2.0 + shell_thickness / 2.0, 0.0, canopy_height / 2.0)
        ),
        material=stainless,
        name="canopy_left_side",
    )
    hood_body.visual(
        Box((shell_thickness, canopy_depth, canopy_height)),
        origin=Origin(
            xyz=(canopy_width / 2.0 - shell_thickness / 2.0, 0.0, canopy_height / 2.0)
        ),
        material=stainless,
        name="canopy_right_side",
    )
    hood_body.visual(
        Box((canopy_width, shell_thickness, canopy_height)),
        origin=Origin(
            xyz=(0.0, -canopy_depth / 2.0 + shell_thickness / 2.0, canopy_height / 2.0)
        ),
        material=stainless,
        name="front_panel",
    )
    hood_body.visual(
        Box((0.78, 0.006, 0.082)),
        origin=Origin(xyz=(-0.02, -canopy_depth / 2.0 + 0.003, 0.080)),
        material=charcoal,
        name="control_fascia",
    )
    hood_body.visual(
        Box((canopy_width, shell_thickness, canopy_height)),
        origin=Origin(
            xyz=(0.0, canopy_depth / 2.0 - shell_thickness / 2.0, canopy_height / 2.0)
        ),
        material=stainless,
        name="rear_panel",
    )

    filter_width = 0.28
    filter_depth = 0.40
    filter_thickness = 0.008
    filter_z = canopy_height - shell_thickness - filter_thickness / 2.0
    hood_body.visual(
        Box((filter_width, filter_depth, filter_thickness)),
        origin=Origin(xyz=(-0.15, 0.0, filter_z)),
        material=charcoal,
        name="left_filter",
    )
    hood_body.visual(
        Box((filter_width, filter_depth, filter_thickness)),
        origin=Origin(xyz=(0.15, 0.0, filter_z)),
        material=charcoal,
        name="right_filter",
    )

    chimney_base_z = canopy_height
    chimney_center_z = chimney_base_z + chimney_height / 2.0
    hood_body.visual(
        Box((chimney_width, chimney_depth, shell_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y,
                chimney_base_z + chimney_height - shell_thickness / 2.0,
            )
        ),
        material=stainless,
        name="chimney_top",
    )
    hood_body.visual(
        Box((shell_thickness, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                -chimney_width / 2.0 + shell_thickness / 2.0,
                chimney_center_y,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_left_side",
    )
    hood_body.visual(
        Box((shell_thickness, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                chimney_width / 2.0 - shell_thickness / 2.0,
                chimney_center_y,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_right_side",
    )
    hood_body.visual(
        Box((chimney_width, shell_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y - chimney_depth / 2.0 + shell_thickness / 2.0,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_front",
    )
    hood_body.visual(
        Box((chimney_width, shell_thickness, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                chimney_center_y + chimney_depth / 2.0 - shell_thickness / 2.0,
                chimney_center_z,
            )
        ),
        material=stainless,
        name="chimney_back",
    )

    knob_radius = 0.022
    knob_length = 0.028
    bezel_radius = 0.024
    bezel_length = 0.004
    knob_y = -canopy_depth / 2.0
    knob_z = 0.078
    knob_mesh = _write_knob_mesh()
    knob_specs = (
        ("knob_left_outer", -0.305),
        ("knob_left_mid", -0.220),
        ("knob_left_inner", -0.135),
        ("knob_right", 0.290),
    )

    for knob_name, knob_x in knob_specs:
        knob = model.part(knob_name)
        knob.inertial = Inertial.from_geometry(
            Box((bezel_radius * 2.0, knob_length, bezel_radius * 2.0)),
            mass=0.18,
            origin=Origin(xyz=(0.0, -knob_length / 2.0, 0.0)),
        )
        knob.visual(
            Cylinder(radius=bezel_radius, length=bezel_length),
            origin=Origin(xyz=(0.0, -bezel_length / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="base_ring",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, -knob_length / 2.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=matte_black,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.004, 0.012)),
            origin=Origin(xyz=(0.0, -knob_length - 0.002, knob_radius * 0.6)),
            material=satin_marker,
            name="indicator",
        )

        model.articulation(
            f"hood_to_{knob_name}",
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(knob_x, knob_y, knob_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=10.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hood_body = object_model.get_part("hood_body")
    knob_left_outer = object_model.get_part("knob_left_outer")
    knob_left_mid = object_model.get_part("knob_left_mid")
    knob_left_inner = object_model.get_part("knob_left_inner")
    knob_right = object_model.get_part("knob_right")

    knobs = {
        "knob_left_outer": knob_left_outer,
        "knob_left_mid": knob_left_mid,
        "knob_left_inner": knob_left_inner,
        "knob_right": knob_right,
    }
    joints = {
        name: object_model.get_articulation(f"hood_to_{name}")
        for name in knobs
    }

    def dims_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple(upper[i] - lower[i] for i in range(3))

    def center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    hood_dims = dims_from_aabb(ctx.part_world_aabb(hood_body))
    ctx.check(
        "hood_body_present_with_realistic_width",
        hood_dims is not None and 0.86 <= hood_dims[0] <= 0.94,
        details=f"expected width near 0.90 m, got {hood_dims}",
    )
    ctx.check(
        "hood_body_present_with_realistic_depth",
        hood_dims is not None and 0.53 <= hood_dims[1] <= 0.57,
        details=f"expected deep canopy near 0.55 m, got {hood_dims}",
    )
    ctx.check(
        "hood_body_present_with_realistic_height",
        hood_dims is not None and 0.50 <= hood_dims[2] <= 0.54,
        details=f"expected canopy plus short chimney around 0.52 m tall, got {hood_dims}",
    )

    canopy_top_aabb = ctx.part_element_world_aabb(hood_body, elem="canopy_top")
    chimney_front_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_front")
    chimney_left_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_left_side")
    chimney_right_aabb = ctx.part_element_world_aabb(hood_body, elem="chimney_right_side")
    control_fascia_aabb = ctx.part_element_world_aabb(hood_body, elem="control_fascia")
    chimney_front_dims = dims_from_aabb(chimney_front_aabb)
    control_fascia_dims = dims_from_aabb(control_fascia_aabb)
    ctx.check(
        "chimney_starts_above_canopy",
        canopy_top_aabb is not None
        and chimney_front_aabb is not None
        and chimney_front_aabb[0][2] >= canopy_top_aabb[0][2],
        details="upper chimney should rise from the canopy top instead of floating separately",
    )
    ctx.check(
        "chimney_box_is_narrow",
        chimney_left_aabb is not None
        and chimney_right_aabb is not None
        and 0.28 <= chimney_right_aabb[1][0] - chimney_left_aabb[0][0] <= 0.36,
        details="chimney box should be substantially narrower than the canopy",
    )
    ctx.check(
        "chimney_height_is_short",
        chimney_front_dims is not None and 0.30 <= chimney_front_dims[2] <= 0.36,
        details=f"expected short upper chimney box, got {chimney_front_dims}",
    )
    ctx.check(
        "control_fascia_is_prominent",
        control_fascia_dims is not None
        and 0.74 <= control_fascia_dims[0] <= 0.82
        and 0.07 <= control_fascia_dims[2] <= 0.09,
        details=f"expected a broad front control strip for the knob layout, got {control_fascia_dims}",
    )

    knob_positions = {name: ctx.part_world_position(part) for name, part in knobs.items()}
    ctx.check(
        "knob_layout_is_left_heavy",
        all(
            knob_positions[name] is not None and knob_positions[name][0] < 0.0
            for name in ("knob_left_outer", "knob_left_mid", "knob_left_inner")
        )
        and knob_positions["knob_right"] is not None
        and knob_positions["knob_right"][0] > 0.20,
        details=f"expected three knobs left of center and one isolated far right, got {knob_positions}",
    )
    ctx.expect_origin_gap(
        knob_left_mid,
        knob_left_outer,
        axis="x",
        min_gap=0.07,
        max_gap=0.10,
        name="left_cluster_outer_gap",
    )
    ctx.expect_origin_gap(
        knob_left_inner,
        knob_left_mid,
        axis="x",
        min_gap=0.07,
        max_gap=0.10,
        name="left_cluster_inner_gap",
    )
    ctx.expect_origin_gap(
        knob_right,
        knob_left_inner,
        axis="x",
        min_gap=0.35,
        max_gap=0.48,
        name="isolated_right_knob_gap",
    )

    for knob_name, knob in knobs.items():
        joint = joints[knob_name]
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_is_continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint.name} should be continuous, got {joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name}_axis_is_front_to_back",
            joint.axis == (0.0, 1.0, 0.0),
            details=f"{joint.name} should rotate about +Y, got {joint.axis}",
        )
        ctx.check(
            f"{joint.name}_has_unbounded_rotation",
            limits is not None and limits.lower is None and limits.upper is None,
            details=f"{joint.name} should be continuous without angular bounds, got {limits}",
        )
        ctx.expect_contact(
            knob,
            hood_body,
            elem_a="base_ring",
            elem_b="front_panel",
            name=f"{knob_name}_mounted_to_front_panel",
        )
        ctx.expect_overlap(
            knob,
            hood_body,
            axes="xz",
            elem_a="base_ring",
            elem_b="front_panel",
            min_overlap=0.035,
            name=f"{knob_name}_footprint_overlaps_front_panel",
        )

        rest_center = center_from_aabb(ctx.part_element_world_aabb(knob, elem="indicator"))
        with ctx.pose({joint: pi / 2.0}):
            turned_center = center_from_aabb(ctx.part_element_world_aabb(knob, elem="indicator"))
            ctx.expect_contact(
                knob,
                hood_body,
                elem_a="base_ring",
                elem_b="front_panel",
                name=f"{joint.name}_turned_contact",
            )
            ctx.fail_if_isolated_parts(name=f"{joint.name}_turned_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_turned_no_overlap")
        moved_distance = None
        if rest_center is not None and turned_center is not None:
            moved_distance = sqrt(
                (turned_center[0] - rest_center[0]) ** 2
                + (turned_center[2] - rest_center[2]) ** 2
            )
        ctx.check(
            f"{joint.name}_indicator_moves_when_rotated",
            moved_distance is not None and moved_distance >= 0.012,
            details=f"indicator should visibly move when {joint.name} rotates, got {moved_distance}",
        )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
