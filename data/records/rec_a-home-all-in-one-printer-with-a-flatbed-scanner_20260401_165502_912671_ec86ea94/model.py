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


def _rounded_slab_mesh(
    width: float,
    depth: float,
    thickness: float,
    radius: float,
    *,
    y_back_edge_at_zero: bool = False,
    name: str,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, depth, radius),
        thickness,
    )
    if y_back_edge_at_zero:
        geom.translate(0.0, -depth * 0.5, thickness * 0.5)
    else:
        geom.translate(0.0, 0.0, thickness * 0.5)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="home_all_in_one_printer")

    body_dark = model.material("body_dark", rgba=(0.22, 0.23, 0.25, 1.0))
    lid_dark = model.material("lid_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.72, 0.73, 0.75, 1.0))
    liner_white = model.material("liner_white", rgba=(0.90, 0.91, 0.92, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.18, 0.24, 0.28, 0.55))
    roller_rubber = model.material("roller_rubber", rgba=(0.16, 0.16, 0.17, 1.0))
    accent_black = model.material("accent_black", rgba=(0.10, 0.10, 0.11, 1.0))

    body = model.part("body")

    top_frame_outer = rounded_rect_profile(0.45, 0.33, 0.022)
    top_frame_inner = rounded_rect_profile(0.326, 0.232, 0.014)
    top_frame_geom = ExtrudeWithHolesGeometry(
        top_frame_outer,
        [top_frame_inner],
        0.013,
    )
    top_frame_geom.translate(0.0, 0.0, 0.1695)
    top_frame_mesh = mesh_from_geometry(top_frame_geom, "printer_top_frame")

    body.visual(
        Box((0.45, 0.36, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=body_dark,
        name="bottom_pan",
    )
    body.visual(
        Box((0.022, 0.36, 0.126)),
        origin=Origin(xyz=(-0.214, 0.0, 0.088)),
        material=body_dark,
        name="left_wall",
    )
    body.visual(
        Box((0.022, 0.36, 0.126)),
        origin=Origin(xyz=(0.214, 0.0, 0.088)),
        material=body_dark,
        name="right_wall",
    )
    body.visual(
        Box((0.45, 0.024, 0.126)),
        origin=Origin(xyz=(0.0, 0.168, 0.088)),
        material=body_dark,
        name="rear_wall",
    )
    body.visual(
        Box((0.057, 0.032, 0.121)),
        origin=Origin(xyz=(-0.1965, -0.164, 0.0855)),
        material=body_dark,
        name="front_left_cheek",
    )
    body.visual(
        Box((0.057, 0.032, 0.121)),
        origin=Origin(xyz=(0.1965, -0.164, 0.0855)),
        material=body_dark,
        name="front_right_cheek",
    )
    body.visual(
        Box((0.338, 0.032, 0.045)),
        origin=Origin(xyz=(0.0, -0.164, 0.1535)),
        material=body_dark,
        name="front_brow",
    )
    body.visual(
        Box((0.45, 0.09, 0.026)),
        origin=Origin(xyz=(0.0, 0.095, 0.163)),
        material=body_dark,
        name="upper_rear_block",
    )
    body.visual(
        top_frame_mesh,
        material=body_dark,
        name="top_frame",
    )
    body.visual(
        Box((0.332, 0.238, 0.011)),
        origin=Origin(xyz=(0.0, -0.005, 0.1685)),
        material=body_dark,
        name="scanner_support",
    )
    body.visual(
        Box((0.312, 0.218, 0.002)),
        origin=Origin(xyz=(0.0, -0.005, 0.175)),
        material=glass_dark,
        name="scanner_glass",
    )
    body.visual(
        Box((0.094, 0.035, 0.003)),
        origin=Origin(xyz=(0.128, -0.120, 0.1775)),
        material=accent_black,
        name="control_panel",
    )
    body.visual(
        Box((0.028, 0.030, 0.012)),
        origin=Origin(xyz=(-0.189, 0.165, 0.170)),
        material=body_dark,
        name="rear_hinge_left",
    )
    body.visual(
        Box((0.028, 0.030, 0.012)),
        origin=Origin(xyz=(0.189, 0.165, 0.170)),
        material=body_dark,
        name="rear_hinge_right",
    )
    body.visual(
        Box((0.028, 0.020, 0.018)),
        origin=Origin(xyz=(-0.166, -0.182, 0.022)),
        material=body_dark,
        name="tray_hinge_left",
    )
    body.visual(
        Box((0.028, 0.020, 0.018)),
        origin=Origin(xyz=(0.166, -0.182, 0.022)),
        material=body_dark,
        name="tray_hinge_right",
    )
    body.visual(
        Box((0.356, 0.152, 0.008)),
        origin=Origin(xyz=(0.0, -0.069, 0.072)),
        material=tray_gray,
        name="paper_shelf",
    )
    body.visual(
        Box((0.026, 0.050, 0.050)),
        origin=Origin(xyz=(-0.190, -0.120, 0.094)),
        material=body_dark,
        name="roller_bracket_left",
    )
    body.visual(
        Box((0.026, 0.050, 0.050)),
        origin=Origin(xyz=(0.190, -0.120, 0.094)),
        material=body_dark,
        name="roller_bracket_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.45, 0.36, 0.188)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
    )

    lid = model.part("scanner_lid")
    lid_top_mesh = _rounded_slab_mesh(
        0.434,
        0.334,
        0.012,
        0.018,
        y_back_edge_at_zero=True,
        name="scanner_lid_top",
    )
    lid.visual(
        lid_top_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=lid_dark,
        name="top_shell",
    )
    lid.visual(
        Box((0.392, 0.320, 0.004)),
        origin=Origin(xyz=(0.0, -0.160, 0.004)),
        material=liner_white,
        name="underside_pad",
    )
    lid.visual(
        Box((0.430, 0.020, 0.022)),
        origin=Origin(xyz=(0.0, -0.324, 0.011)),
        material=lid_dark,
        name="front_rim",
    )
    lid.visual(
        Box((0.020, 0.334, 0.022)),
        origin=Origin(xyz=(-0.207, -0.167, 0.011)),
        material=lid_dark,
        name="left_rim",
    )
    lid.visual(
        Box((0.020, 0.334, 0.022)),
        origin=Origin(xyz=(0.207, -0.167, 0.011)),
        material=lid_dark,
        name="right_rim",
    )
    lid.visual(
        Box((0.340, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, -0.007, 0.008)),
        material=lid_dark,
        name="rear_bridge",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.350),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lid_dark,
        name="hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.434, 0.334, 0.024)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -0.167, 0.012)),
    )

    tray = model.part("output_tray")
    tray.visual(
        Cylinder(radius=0.007, length=0.304),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tray_gray,
        name="hinge_barrel",
    )
    tray.visual(
        Box((0.334, 0.004, 0.102)),
        origin=Origin(xyz=(0.0, -0.002, 0.056)),
        material=tray_gray,
        name="tray_panel",
    )
    tray.visual(
        Box((0.014, 0.014, 0.088)),
        origin=Origin(xyz=(-0.160, 0.007, 0.049)),
        material=tray_gray,
        name="left_lip",
    )
    tray.visual(
        Box((0.014, 0.014, 0.088)),
        origin=Origin(xyz=(0.160, 0.007, 0.049)),
        material=tray_gray,
        name="right_lip",
    )
    tray.visual(
        Box((0.280, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.007, 0.101)),
        material=tray_gray,
        name="front_stop",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.334, 0.018, 0.110)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.002, 0.055)),
    )

    roller = model.part("feed_roller")
    roller.visual(
        Cylinder(radius=0.013, length=0.282),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=roller_rubber,
        name="drum",
    )
    roller.visual(
        Cylinder(radius=0.004, length=0.036),
        origin=Origin(xyz=(-0.159, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_black,
        name="left_shaft",
    )
    roller.visual(
        Cylinder(radius=0.004, length=0.036),
        origin=Origin(xyz=(0.159, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_black,
        name="right_shaft",
    )
    roller.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.318),
        mass=0.18,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "body_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.165, 0.178)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "body_to_output_tray",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -0.187, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "body_to_feed_roller",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=roller,
        origin=Origin(xyz=(0.0, -0.120, 0.094)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=20.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("scanner_lid")
    tray = object_model.get_part("output_tray")
    roller = object_model.get_part("feed_roller")

    lid_hinge = object_model.get_articulation("body_to_scanner_lid")
    tray_hinge = object_model.get_articulation("body_to_output_tray")
    roller_joint = object_model.get_articulation("body_to_feed_roller")

    rear_hinge_left = body.get_visual("rear_hinge_left")
    rear_hinge_right = body.get_visual("rear_hinge_right")
    tray_hinge_left = body.get_visual("tray_hinge_left")
    tray_hinge_right = body.get_visual("tray_hinge_right")
    roller_bracket_left = body.get_visual("roller_bracket_left")
    roller_bracket_right = body.get_visual("roller_bracket_right")
    scanner_glass = body.get_visual("scanner_glass")
    paper_shelf = body.get_visual("paper_shelf")
    front_brow = body.get_visual("front_brow")

    lid_barrel = lid.get_visual("hinge_barrel")
    lid_top = lid.get_visual("top_shell")
    lid_pad = lid.get_visual("underside_pad")

    tray_barrel = tray.get_visual("hinge_barrel")
    tray_panel = tray.get_visual("tray_panel")

    roller_left_shaft = roller.get_visual("left_shaft")
    roller_right_shaft = roller.get_visual("right_shaft")
    roller_drum = roller.get_visual("drum")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

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

    ctx.check(
        "scanner lid hinge is rear horizontal revolute",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.axis == (-1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > 1.1,
        details=f"type={lid_hinge.articulation_type}, axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "output tray hinge is front horizontal revolute",
        tray_hinge.articulation_type == ArticulationType.REVOLUTE
        and abs(tray_hinge.axis[0]) == 1.0
        and tray_hinge.axis[1] == 0.0
        and tray_hinge.axis[2] == 0.0
        and tray_hinge.motion_limits is not None
        and tray_hinge.motion_limits.lower == 0.0
        and tray_hinge.motion_limits.upper is not None
        and tray_hinge.motion_limits.upper > 1.2,
        details=f"type={tray_hinge.articulation_type}, axis={tray_hinge.axis}, limits={tray_hinge.motion_limits}",
    )
    ctx.check(
        "feed roller spins continuously on a horizontal axle",
        roller_joint.articulation_type == ArticulationType.CONTINUOUS
        and roller_joint.axis == (1.0, 0.0, 0.0)
        and roller_joint.motion_limits is not None
        and roller_joint.motion_limits.lower is None
        and roller_joint.motion_limits.upper is None,
        details=f"type={roller_joint.articulation_type}, axis={roller_joint.axis}, limits={roller_joint.motion_limits}",
    )

    with ctx.pose({lid_hinge: 0.0, tray_hinge: 0.0}):
        ctx.expect_contact(
            body,
            lid,
            elem_a=rear_hinge_left,
            elem_b=lid_barrel,
            name="lid barrel contacts left rear hinge mount",
        )
        ctx.expect_contact(
            body,
            lid,
            elem_a=rear_hinge_right,
            elem_b=lid_barrel,
            name="lid barrel contacts right rear hinge mount",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=lid_pad,
            negative_elem=scanner_glass,
            min_gap=0.002,
            max_gap=0.008,
            name="closed lid sits just above the scanner glass",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=lid_top,
            elem_b=scanner_glass,
            min_overlap=0.20,
            name="lid covers the flatbed scanner glass",
        )
        ctx.expect_contact(
            body,
            tray,
            elem_a=tray_hinge_left,
            elem_b=tray_barrel,
            name="tray barrel contacts left hinge support",
        )
        ctx.expect_contact(
            body,
            tray,
            elem_a=tray_hinge_right,
            elem_b=tray_barrel,
            name="tray barrel contacts right hinge support",
        )
        ctx.expect_gap(
            body,
            tray,
            axis="z",
            positive_elem=front_brow,
            negative_elem=tray_panel,
            min_gap=0.001,
            max_gap=0.004,
            name="closed tray nests just below the front brow",
        )
        ctx.expect_contact(
            body,
            roller,
            elem_a=roller_bracket_left,
            elem_b=roller_left_shaft,
            name="left roller shaft seats on the left bracket",
        )
        ctx.expect_contact(
            body,
            roller,
            elem_a=roller_bracket_right,
            elem_b=roller_right_shaft,
            name="right roller shaft seats on the right bracket",
        )
        ctx.expect_gap(
            roller,
            body,
            axis="z",
            positive_elem=roller_drum,
            negative_elem=paper_shelf,
            min_gap=0.004,
            max_gap=0.008,
            name="feed roller sits just above the paper shelf",
        )

        closed_lid_aabb = ctx.part_world_aabb(lid)
        closed_tray_aabb = ctx.part_world_aabb(tray)
        roller_rest_pos = ctx.part_world_position(roller)

    lid_upper = 0.0 if lid_hinge.motion_limits is None or lid_hinge.motion_limits.upper is None else lid_hinge.motion_limits.upper
    tray_upper = 0.0 if tray_hinge.motion_limits is None or tray_hinge.motion_limits.upper is None else tray_hinge.motion_limits.upper

    with ctx.pose({lid_hinge: lid_upper, tray_hinge: tray_upper, roller_joint: math.pi / 2.0}):
        open_lid_aabb = ctx.part_world_aabb(lid)
        open_tray_aabb = ctx.part_world_aabb(tray)
        roller_spun_pos = ctx.part_world_position(roller)

    ctx.check(
        "scanner lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.12,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "output tray folds downward and outward",
        closed_tray_aabb is not None
        and open_tray_aabb is not None
        and open_tray_aabb[0][1] < closed_tray_aabb[0][1] - 0.08
        and open_tray_aabb[1][2] < closed_tray_aabb[1][2] - 0.07,
        details=f"closed={closed_tray_aabb}, open={open_tray_aabb}",
    )
    ctx.check(
        "feed roller spins in place without translating",
        roller_rest_pos is not None
        and roller_spun_pos is not None
        and max(abs(a - b) for a, b in zip(roller_rest_pos, roller_spun_pos)) < 1e-6,
        details=f"rest={roller_rest_pos}, spun={roller_spun_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
