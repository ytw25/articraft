from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BARREL_OUTER_RADIUS = 0.032
BARREL_INNER_RADIUS = 0.029
BARREL_HEIGHT = 0.603
BARREL_BASE_Z = 0.040
TOP_CAP_Z = 0.635

ROD_RADIUS = 0.0055
ROD_LENGTH = 0.655
ROD_CENTER_Z = -0.1925
PISTON_RADIUS = 0.012
PISTON_LENGTH = 0.028
PISTON_CENTER_Z = -0.519
SLIDE_TRAVEL = 0.18

HOOK_PIVOT_X = 0.092
HOOK_PIVOT_Z = 0.084


def make_base_frame() -> cq.Workplane:
    foot_thickness = 0.018
    hub_radius = 0.050
    leg_offset = 0.106
    leg_length = 0.162
    leg_width = 0.054

    base = cq.Workplane("XY").circle(hub_radius).extrude(foot_thickness)
    for angle in (0, 120, 240):
        leg = (
            cq.Workplane("XY")
            .transformed(rotate=(0, 0, angle))
            .center(leg_offset, 0.0)
            .slot2D(leg_length, leg_width)
            .extrude(foot_thickness)
        )
        base = base.union(leg)

    lower_collar = (
        cq.Workplane("XY")
        .circle(0.041)
        .extrude(0.020)
        .translate((0.0, 0.0, foot_thickness - 0.002))
    )
    upper_collar = (
        cq.Workplane("XY")
        .circle(0.037)
        .extrude(0.020)
        .translate((0.0, 0.0, foot_thickness + 0.014))
    )
    return base.union(lower_collar).union(upper_collar)


def make_barrel_shell() -> cq.Workplane:
    ferrule = (
        cq.Workplane("XY")
        .circle(BARREL_OUTER_RADIUS + 0.004)
        .circle(BARREL_INNER_RADIUS)
        .extrude(0.024)
    )
    tube = (
        cq.Workplane("XY")
        .circle(BARREL_OUTER_RADIUS)
        .circle(BARREL_INNER_RADIUS)
        .extrude(BARREL_HEIGHT)
        .translate((0.0, 0.0, 0.018))
    )
    return ferrule.union(tube).translate((0.0, 0.0, BARREL_BASE_Z))


def make_top_cap() -> cq.Workplane:
    cap_ring = (
        cq.Workplane("XY")
        .circle(0.040)
        .circle(0.0072)
        .extrude(0.012)
    )
    guide_boss = (
        cq.Workplane("XY")
        .circle(0.014)
        .circle(0.0075)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.010))
    )
    return cap_ring.union(guide_boss).translate((0.0, 0.0, TOP_CAP_Z))


def make_hook_bracket() -> cq.Workplane:
    web = (
        cq.Workplane("XZ")
        .moveTo(0.036, 0.034)
        .lineTo(0.036, 0.056)
        .lineTo(0.050, 0.068)
        .lineTo(0.068, 0.078)
        .lineTo(0.082, 0.082)
        .lineTo(0.082, 0.070)
        .lineTo(0.060, 0.060)
        .lineTo(0.046, 0.048)
        .lineTo(0.044, 0.034)
        .close()
        .extrude(0.0045, both=True)
    )
    bracket = web
    for y_center in (-0.007, 0.007):
        bridge = (
            cq.Workplane("XZ")
            .center(0.084, 0.079)
            .rect(0.016, 0.010)
            .extrude(0.003, both=True)
            .translate((0.0, y_center, 0.0))
        )
        ear = (
            cq.Workplane("XZ")
            .center(HOOK_PIVOT_X, HOOK_PIVOT_Z)
            .circle(0.0075)
            .extrude(0.003, both=True)
            .translate((0.0, y_center, 0.0))
        )
        bracket = bracket.union(bridge).union(ear)
    return bracket


def make_handle() -> cq.Workplane:
    crossbar = (
        cq.Workplane("YZ")
        .circle(0.014)
        .extrude(0.115, both=True)
        .translate((0.0, 0.0, 0.155))
    )
    stem = (
        cq.Workplane("XY")
        .circle(0.012)
        .extrude(0.050)
        .translate((0.0, 0.0, 0.105))
    )
    center_boss = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.012)
        .translate((0.0, 0.0, 0.143))
    )
    return crossbar.union(stem).union(center_boss)


def make_piston_head() -> cq.Workplane:
    core = cq.Workplane("XY").circle(PISTON_RADIUS).extrude(PISTON_LENGTH)
    top_land = (
        cq.Workplane("XY")
        .circle(0.014)
        .extrude(0.008)
        .translate((0.0, 0.0, PISTON_LENGTH - 0.006))
    )
    return core.union(top_land)


def make_rod_stop_collar() -> cq.Workplane:
    return cq.Workplane("XY").circle(0.018).extrude(0.008)


def make_hook_bracket() -> cq.Workplane:
    bracket = None
    for y_center in (-0.0065, 0.0065):
        cheek = (
            cq.Workplane("XZ")
            .moveTo(0.036, 0.034)
            .lineTo(0.036, 0.056)
            .lineTo(0.050, 0.068)
            .lineTo(0.068, 0.078)
            .lineTo(0.080, 0.082)
            .lineTo(0.080, 0.070)
            .lineTo(0.060, 0.060)
            .lineTo(0.046, 0.048)
            .lineTo(0.044, 0.034)
            .close()
            .extrude(0.0025, both=True)
            .translate((0.0, y_center, 0.0))
        )
        bridge = (
            cq.Workplane("XZ")
            .center(0.086, 0.080)
            .rect(0.013, 0.008)
            .extrude(0.0025, both=True)
            .translate((0.0, y_center, 0.0))
        )
        ear = (
            cq.Workplane("XZ")
            .center(HOOK_PIVOT_X, HOOK_PIVOT_Z)
            .circle(0.0075)
            .extrude(0.0025, both=True)
            .translate((0.0, y_center, 0.0))
        )
        side = cheek.union(bridge).union(ear)
        bracket = side if bracket is None else bracket.union(side)
    return bracket


def make_hook() -> cq.Workplane:
    knuckle = cq.Workplane("XZ").circle(0.0066).extrude(0.004, both=True)
    hook_arm = (
        cq.Workplane("XZ")
        .moveTo(0.001, 0.006)
        .lineTo(0.001, -0.018)
        .lineTo(0.018, -0.040)
        .lineTo(0.040, -0.052)
        .lineTo(0.050, -0.044)
        .lineTo(0.034, -0.030)
        .lineTo(0.018, -0.010)
        .lineTo(0.012, 0.004)
        .close()
        .extrude(0.004, both=True)
    )
    return knuckle.union(hook_arm)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bicycle_track_pump")

    frame_mat = model.material("frame_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    barrel_mat = model.material("barrel_silver", rgba=(0.80, 0.82, 0.84, 1.0))
    rod_mat = model.material("rod_chrome", rgba=(0.88, 0.89, 0.90, 1.0))
    grip_mat = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))
    accent_mat = model.material("accent_red", rgba=(0.78, 0.18, 0.12, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(make_base_frame(), "track_pump_base_frame"),
        material=frame_mat,
        name="base_frame",
    )
    body.visual(
        mesh_from_cadquery(make_barrel_shell(), "track_pump_barrel_shell"),
        material=barrel_mat,
        name="barrel_shell",
    )
    body.visual(
        mesh_from_cadquery(make_top_cap(), "track_pump_top_cap"),
        material=frame_mat,
        name="top_cap",
    )
    body.visual(
        mesh_from_cadquery(make_hook_bracket(), "track_pump_hook_bracket"),
        material=frame_mat,
        name="hook_bracket",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=ROD_RADIUS, length=ROD_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, ROD_CENTER_Z)),
        material=rod_mat,
        name="rod",
    )
    plunger.visual(
        mesh_from_cadquery(make_piston_head(), "track_pump_piston_head"),
        origin=Origin(xyz=(0.0, 0.0, PISTON_CENTER_Z)),
        material=frame_mat,
        name="piston",
    )
    plunger.visual(
        mesh_from_cadquery(make_handle(), "track_pump_handle"),
        material=grip_mat,
        name="handle",
    )
    plunger.visual(
        mesh_from_cadquery(make_rod_stop_collar(), "track_pump_rod_stop_collar"),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=grip_mat,
        name="stop_collar",
    )

    hose_hook = model.part("hose_hook")
    hose_hook.visual(
        mesh_from_cadquery(make_hook(), "track_pump_hose_hook"),
        material=accent_mat,
        name="hook",
    )

    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.654)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.60,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_hose_hook",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hose_hook,
        origin=Origin(xyz=(HOOK_PIVOT_X, 0.0, HOOK_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    part_names = {part.name for part in object_model.parts}
    articulation_names = {joint.name for joint in object_model.articulations}

    ctx.check(
        "critical parts present",
        {"body", "plunger", "hose_hook"}.issubset(part_names),
        details=f"parts={sorted(part_names)}",
    )
    ctx.check(
        "critical articulations present",
        {"body_to_plunger", "body_to_hose_hook"}.issubset(articulation_names),
        details=f"articulations={sorted(articulation_names)}",
    )

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

    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    hose_hook = object_model.get_part("hose_hook")
    plunger_slide = object_model.get_articulation("body_to_plunger")
    hook_hinge = object_model.get_articulation("body_to_hose_hook")

    ctx.check(
        "plunger slide follows the barrel axis",
        plunger_slide.axis == (0.0, 0.0, 1.0),
        details=f"axis={plunger_slide.axis}",
    )
    ctx.check(
        "hook hinge uses a side-mounted horizontal axis",
        hook_hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={hook_hinge.axis}",
    )

    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="rod",
        outer_elem="barrel_shell",
        margin=0.0,
        name="rod stays centered in the barrel envelope at rest",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="rod",
        elem_b="barrel_shell",
        min_overlap=0.45,
        name="rod remains deeply inserted into the barrel at rest",
    )
    ctx.expect_gap(
        plunger,
        body,
        axis="z",
        positive_elem="handle",
        negative_elem="top_cap",
        min_gap=0.10,
        name="handle clears the top cap",
    )
    ctx.expect_contact(
        hose_hook,
        body,
        contact_tol=1e-4,
        name="hook knuckle is supported by the body lugs",
    )

    rest_slide_pos = ctx.part_world_position(plunger)
    slide_upper = plunger_slide.motion_limits.upper
    with ctx.pose({plunger_slide: slide_upper}):
        ctx.expect_within(
            plunger,
            body,
            axes="xy",
            inner_elem="rod",
            outer_elem="barrel_shell",
            margin=0.0,
            name="rod stays centered when the pump is pulled up",
        )
        ctx.expect_overlap(
            plunger,
            body,
            axes="z",
            elem_a="rod",
            elem_b="barrel_shell",
            min_overlap=0.27,
            name="rod keeps retained insertion at full stroke",
        )
        extended_slide_pos = ctx.part_world_position(plunger)

    ctx.check(
        "plunger rises through its stroke",
        rest_slide_pos is not None
        and extended_slide_pos is not None
        and extended_slide_pos[2] > rest_slide_pos[2] + 0.15,
        details=f"rest={rest_slide_pos}, extended={extended_slide_pos}",
    )

    closed_hook_aabb = ctx.part_world_aabb(hose_hook)
    hook_upper = hook_hinge.motion_limits.upper
    with ctx.pose({hook_hinge: hook_upper}):
        open_hook_aabb = ctx.part_world_aabb(hose_hook)
        ctx.expect_contact(
            hose_hook,
            body,
            contact_tol=1e-4,
            name="hook stays mounted when swung open",
        )

    ctx.check(
        "hook swings upward and outward from the base",
        closed_hook_aabb is not None
        and open_hook_aabb is not None
        and open_hook_aabb[1][0] > closed_hook_aabb[1][0] + 0.01
        and open_hook_aabb[1][2] > closed_hook_aabb[1][2] + 0.03,
        details=f"closed={closed_hook_aabb}, open={open_hook_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
