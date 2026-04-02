from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_multifunction_laser_printer")

    housing_light = model.material("housing_light", rgba=(0.86, 0.88, 0.89, 1.0))
    housing_mid = model.material("housing_mid", rgba=(0.73, 0.76, 0.79, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.23, 0.25, 0.28, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.78, 0.80, 0.82, 1.0))
    glass = model.material("glass", rgba=(0.34, 0.50, 0.57, 0.35))

    body_width = 0.43
    body_depth = 0.39
    engine_height = 0.215
    scanner_base_depth = 0.345
    scanner_base_height = 0.090
    top_surface_z = engine_height + scanner_base_height
    back_y = body_depth * 0.5 - 0.013
    front_y = -body_depth * 0.5

    printer_body = model.part("printer_body")
    printer_body.visual(
        Box((body_width, body_depth, engine_height)),
        origin=Origin(xyz=(0.0, 0.0, engine_height * 0.5)),
        material=housing_light,
        name="engine_shell",
    )
    printer_body.visual(
        Box((0.41, scanner_base_depth, scanner_base_height)),
        origin=Origin(xyz=(0.0, 0.0075, engine_height + scanner_base_height * 0.5)),
        material=housing_mid,
        name="scanner_base",
    )
    printer_body.visual(
        Box((0.35, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, back_y, top_surface_z - 0.010)),
        material=housing_mid,
        name="rear_hinge_block",
    )

    platen_outer_w = 0.352
    platen_outer_d = 0.262
    platen_inner_w = 0.302
    platen_inner_d = 0.212
    platen_y = 0.028
    frame_thickness = 0.004
    frame_rail = 0.025
    frame_z = top_surface_z - frame_thickness * 0.5

    printer_body.visual(
        Box((platen_outer_w, frame_rail, frame_thickness)),
        origin=Origin(
            xyz=(0.0, platen_y + (platen_outer_d - frame_rail) * 0.5, frame_z)
        ),
        material=trim_dark,
        name="platen_frame_rear",
    )
    printer_body.visual(
        Box((platen_outer_w, frame_rail, frame_thickness)),
        origin=Origin(
            xyz=(0.0, platen_y - (platen_outer_d - frame_rail) * 0.5, frame_z)
        ),
        material=trim_dark,
        name="platen_frame_front",
    )
    printer_body.visual(
        Box((frame_rail, platen_inner_d, frame_thickness)),
        origin=Origin(
            xyz=(-(platen_inner_w + frame_rail) * 0.5, platen_y, frame_z)
        ),
        material=trim_dark,
        name="platen_frame_left",
    )
    printer_body.visual(
        Box((frame_rail, platen_inner_d, frame_thickness)),
        origin=Origin(
            xyz=((platen_inner_w + frame_rail) * 0.5, platen_y, frame_z)
        ),
        material=trim_dark,
        name="platen_frame_right",
    )
    printer_body.visual(
        Box((platen_inner_w, platen_inner_d, 0.002)),
        origin=Origin(xyz=(0.0, platen_y, top_surface_z - 0.001)),
        material=glass,
        name="platen_glass",
    )
    printer_body.visual(
        Box((0.13, 0.028, 0.004)),
        origin=Origin(xyz=(0.105, -0.145, top_surface_z - 0.002)),
        material=trim_dark,
        name="control_panel",
    )
    printer_body.visual(
        Box((0.31, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, front_y + 0.003, 0.212)),
        material=trim_dark,
        name="output_slot",
    )
    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            printer_body.visual(
                Box((0.050, 0.050, 0.006)),
                origin=Origin(
                    xyz=(sign_x * 0.155, sign_y * 0.130, 0.003),
                ),
                material=trim_dark,
                name=f"foot_{'l' if sign_x < 0 else 'r'}_{'f' if sign_y < 0 else 'b'}",
            )
    printer_body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, top_surface_z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, top_surface_z * 0.5)),
    )

    lid_width = 0.422
    lid_depth = 0.300
    lid_thickness = 0.022

    scanner_lid = model.part("scanner_lid")
    scanner_lid.visual(
        Box((lid_width, lid_depth, lid_thickness)),
        origin=Origin(xyz=(0.0, -lid_depth * 0.5, lid_thickness * 0.5)),
        material=housing_light,
        name="lid_shell",
    )
    scanner_lid.visual(
        Box((0.360, 0.065, 0.014)),
        origin=Origin(xyz=(0.0, -0.038, lid_thickness + 0.007)),
        material=housing_mid,
        name="lid_rear_riser",
    )
    scanner_lid.visual(
        Box((0.160, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, -0.290, lid_thickness - 0.001)),
        material=housing_mid,
        name="lid_front_grip",
    )
    scanner_lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, 0.040)),
        mass=3.0,
        origin=Origin(xyz=(0.0, -lid_depth * 0.5, 0.020)),
    )

    tray_width = 0.340
    tray_depth = 0.110

    output_tray = model.part("output_tray")
    output_tray.visual(
        Box((tray_width, 0.008, tray_depth)),
        origin=Origin(xyz=(0.0, -0.004, -tray_depth * 0.5)),
        material=tray_gray,
        name="door_panel",
    )
    output_tray.visual(
        Box((0.332, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.005, -0.008)),
        material=tray_gray,
        name="tray_hinge_beam",
    )
    output_tray.visual(
        Box((0.012, 0.012, 0.092)),
        origin=Origin(xyz=(-0.164, -0.014, -0.064)),
        material=tray_gray,
        name="tray_left_fence",
    )
    output_tray.visual(
        Box((0.012, 0.012, 0.092)),
        origin=Origin(xyz=(0.164, -0.014, -0.064)),
        material=tray_gray,
        name="tray_right_fence",
    )
    output_tray.visual(
        Box((0.286, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.014, -0.105)),
        material=tray_gray,
        name="tray_front_stop",
    )
    output_tray.visual(
        Box((0.240, 0.004, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, -0.020)),
        material=tray_gray,
        name="tray_slider_bridge",
    )
    output_tray.inertial = Inertial.from_geometry(
        Box((tray_width, 0.020, tray_depth)),
        mass=0.42,
        origin=Origin(xyz=(0.0, -0.010, -tray_depth * 0.5)),
    )

    support_width = 0.280
    support_length = 0.090

    output_support = model.part("output_support")
    output_support.visual(
        Box((support_width, 0.004, support_length)),
        origin=Origin(xyz=(0.0, -0.010, -support_length * 0.5)),
        material=tray_gray,
        name="support_plate",
    )
    output_support.visual(
        Box((0.010, 0.008, 0.060)),
        origin=Origin(xyz=(-0.126, -0.012, -0.032)),
        material=tray_gray,
        name="support_left_guide",
    )
    output_support.visual(
        Box((0.010, 0.008, 0.060)),
        origin=Origin(xyz=(0.126, -0.012, -0.032)),
        material=tray_gray,
        name="support_right_guide",
    )
    output_support.inertial = Inertial.from_geometry(
        Box((support_width, 0.016, support_length)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.010, -support_length * 0.5)),
    )

    model.articulation(
        "body_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=printer_body,
        child=scanner_lid,
        origin=Origin(xyz=(0.0, back_y, top_surface_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "body_to_output_tray",
        ArticulationType.REVOLUTE,
        parent=printer_body,
        child=output_tray,
        origin=Origin(xyz=(0.0, front_y, 0.192)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "tray_to_output_support",
        ArticulationType.PRISMATIC,
        parent=output_tray,
        child=output_support,
        origin=Origin(xyz=(0.0, -0.006, -0.015)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.10,
            lower=0.0,
            upper=0.055,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    printer_body = object_model.get_part("printer_body")
    scanner_lid = object_model.get_part("scanner_lid")
    output_tray = object_model.get_part("output_tray")
    output_support = object_model.get_part("output_support")
    lid_joint = object_model.get_articulation("body_to_scanner_lid")
    tray_joint = object_model.get_articulation("body_to_output_tray")
    support_joint = object_model.get_articulation("tray_to_output_support")
    lid_upper = lid_joint.motion_limits.upper if lid_joint.motion_limits else 0.0
    tray_upper = tray_joint.motion_limits.upper if tray_joint.motion_limits else 0.0
    support_upper = support_joint.motion_limits.upper if support_joint.motion_limits else 0.0

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

    with ctx.pose({lid_joint: 0.0, tray_joint: 0.0, support_joint: 0.0}):
        ctx.expect_gap(
            scanner_lid,
            printer_body,
            axis="z",
            max_gap=0.0025,
            max_penetration=0.0,
            name="closed scanner lid seats on the printer top",
        )
        ctx.expect_gap(
            printer_body,
            output_tray,
            axis="y",
            positive_elem="engine_shell",
            negative_elem="door_panel",
            max_gap=0.0015,
            max_penetration=0.0,
            name="closed output tray sits flush with the front panel",
        )

    with ctx.pose({tray_joint: tray_upper, support_joint: 0.0}):
        ctx.expect_within(
            output_support,
            output_tray,
            axes="x",
            margin=0.0015,
            name="retracted support stays guided between the tray rails",
        )
        ctx.expect_overlap(
            output_support,
            output_tray,
            axes="y",
            min_overlap=0.080,
            name="retracted support remains substantially nested in the open tray",
        )
        ctx.expect_overlap(
            output_support,
            output_tray,
            axes="z",
            min_overlap=0.016,
            name="retracted support stays vertically aligned with the open tray",
        )

    with ctx.pose({tray_joint: tray_upper, support_joint: support_upper}):
        ctx.expect_within(
            output_support,
            output_tray,
            axes="x",
            margin=0.0015,
            name="extended support remains laterally guided by the tray rails",
        )
        ctx.expect_overlap(
            output_support,
            output_tray,
            axes="y",
            min_overlap=0.035,
            name="extended support retains fore-aft insertion in the tray",
        )
        ctx.expect_overlap(
            output_support,
            output_tray,
            axes="z",
            min_overlap=0.018,
            name="extended support stays vertically aligned with the open tray",
        )

    closed_lid_aabb = ctx.part_world_aabb(scanner_lid)
    with ctx.pose({lid_joint: lid_upper}):
        opened_lid_aabb = ctx.part_world_aabb(scanner_lid)

    def _extent(aabb, axis: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis] - aabb[0][axis]

    ctx.check(
        "scanner lid rotates from flat cover to raised open pose",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and _extent(closed_lid_aabb, 2) is not None
        and _extent(opened_lid_aabb, 2) is not None
        and _extent(opened_lid_aabb, 2) > _extent(closed_lid_aabb, 2) + 0.12,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )

    closed_tray_aabb = ctx.part_world_aabb(output_tray)
    with ctx.pose({tray_joint: tray_upper}):
        opened_tray_aabb = ctx.part_world_aabb(output_tray)
    ctx.check(
        "output tray rotates from vertical front door to horizontal shelf",
        closed_tray_aabb is not None
        and opened_tray_aabb is not None
        and _extent(opened_tray_aabb, 1) is not None
        and _extent(closed_tray_aabb, 1) is not None
        and _extent(opened_tray_aabb, 2) is not None
        and _extent(closed_tray_aabb, 2) is not None
        and _extent(opened_tray_aabb, 1) > _extent(closed_tray_aabb, 1) + 0.070
        and _extent(opened_tray_aabb, 2) < _extent(closed_tray_aabb, 2) - 0.070,
        details=f"closed={closed_tray_aabb}, opened={opened_tray_aabb}",
    )

    with ctx.pose({tray_joint: tray_upper, support_joint: 0.0}):
        retracted_support_aabb = ctx.part_world_aabb(output_support)
    with ctx.pose({tray_joint: tray_upper, support_joint: support_upper}):
        extended_support_aabb = ctx.part_world_aabb(output_support)
    ctx.check(
        "telescoping support slides outward from the open tray",
        retracted_support_aabb is not None
        and extended_support_aabb is not None
        and extended_support_aabb[0][1] < retracted_support_aabb[0][1] - 0.050,
        details=f"retracted={retracted_support_aabb}, extended={extended_support_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
