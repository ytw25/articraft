from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk_hybrid import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOP_THICKNESS = 0.030
WING_LENGTH = 1.60
WING_DEPTH = 0.75

FRAME_PLATE_T = 0.010
FRAME_BEAM_W = 0.060
FRAME_BEAM_H = 0.030
FRAME_BEAM_TOP_Z = -FRAME_PLATE_T

SLEEVE_W = 0.095
SLEEVE_D = 0.065
SLEEVE_H = 0.350
SLEEVE_WALL = 0.007
SLEEVE_PLATE_X = 0.160
SLEEVE_PLATE_Y = 0.120

INNER_COLUMN_W = 0.077
INNER_COLUMN_D = 0.047
INNER_COLUMN_L = 0.700
LEG_TRAVEL = 0.270

FOOT_H = 0.022
FOOT_LONG = 0.690
FOOT_SHORT = 0.090

FRONT_LEFT_POS = (0.18, 0.18)
FRONT_RIGHT_POS = (1.42, 0.18)
REAR_POS = (0.18, 1.42)

INNER_CORNER_SUPPORT = 0.66

HANDSET_LEN = 0.135
HANDSET_DEPTH = 0.058
HANDSET_H = 0.022
HANDSET_MOUNT_X = 1.45
HANDSET_MOUNT_Y = 0.018

PADDLE_BARREL_R = 0.0045
PADDLE_BARREL_L = 0.110
PADDLE_PIVOT_Y = 0.003
PADDLE_PIVOT_Z = -0.009
PADDLE_TILT = 0.23


def _box_wp(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _fuse_all(items: list[cq.Workplane]) -> cq.Workplane:
    fused = items[0]
    for item in items[1:]:
        fused = fused.union(item)
    return fused


def _make_desktop_shape() -> cq.Workplane:
    front_wing = cq.Workplane("XY").box(
        WING_LENGTH,
        WING_DEPTH,
        TOP_THICKNESS,
        centered=(False, False, False),
    )
    left_wing = cq.Workplane("XY").box(
        WING_DEPTH,
        WING_LENGTH,
        TOP_THICKNESS,
        centered=(False, False, False),
    )
    desktop = front_wing.union(left_wing)
    return desktop


def _make_sleeve_shape(x: float, y: float) -> cq.Workplane:
    left_wall = _box_wp(
        (SLEEVE_WALL, SLEEVE_D, SLEEVE_H),
        (x - (SLEEVE_W - SLEEVE_WALL) / 2.0, y, -SLEEVE_H / 2.0),
    )
    right_wall = _box_wp(
        (SLEEVE_WALL, SLEEVE_D, SLEEVE_H),
        (x + (SLEEVE_W - SLEEVE_WALL) / 2.0, y, -SLEEVE_H / 2.0),
    )
    front_wall = _box_wp(
        (SLEEVE_W - 2.0 * SLEEVE_WALL, SLEEVE_WALL, SLEEVE_H),
        (x, y - (SLEEVE_D - SLEEVE_WALL) / 2.0, -SLEEVE_H / 2.0),
    )
    rear_wall = _box_wp(
        (SLEEVE_W - 2.0 * SLEEVE_WALL, SLEEVE_WALL, SLEEVE_H),
        (x, y + (SLEEVE_D - SLEEVE_WALL) / 2.0, -SLEEVE_H / 2.0),
    )
    top_plate = _box_wp(
        (SLEEVE_PLATE_X, SLEEVE_PLATE_Y, FRAME_PLATE_T),
        (x, y, -FRAME_PLATE_T / 2.0),
    )
    return _fuse_all([left_wall, right_wall, front_wall, rear_wall, top_plate])


def _make_frame_rails_shape() -> cq.Workplane:
    beam_z = -FRAME_BEAM_H / 2.0
    items = [
        _box_wp(
            (1.00, 0.100, FRAME_BEAM_H),
            (0.80, 0.31, beam_z),
        ),
        _box_wp(
            (0.100, 1.00, FRAME_BEAM_H),
            (0.31, 0.80, beam_z),
        ),
        _box_wp(
            (0.100, 0.44, FRAME_BEAM_H),
            (1.31, 0.53, beam_z),
        ),
        _box_wp(
            (0.44, 0.100, FRAME_BEAM_H),
            (0.53, 1.31, beam_z),
        ),
        _box_wp(
            (0.22, 0.100, FRAME_BEAM_H),
            (1.20, 0.31, beam_z),
        ),
        _box_wp(
            (0.100, 0.22, FRAME_BEAM_H),
            (0.31, 1.20, beam_z),
        ),
    ]
    return _fuse_all(items)


def _make_foot_shape(x_len: float, y_len: float) -> cq.Workplane:
    foot = _box_wp((x_len, y_len, FOOT_H), (0.0, 0.0, -(INNER_COLUMN_L + FOOT_H / 2.0)))
    return foot


def _make_handset_body_shape() -> cq.Workplane:
    body = _box_wp(
        (HANDSET_LEN, HANDSET_DEPTH, HANDSET_H),
        (0.0, HANDSET_DEPTH / 2.0, -HANDSET_H / 2.0),
    )
    mount_pad = _box_wp(
        (HANDSET_LEN - 0.020, 0.026, 0.004),
        (0.0, 0.022, -0.002),
    )
    groove = (
        cq.Workplane("YZ")
        .circle(PADDLE_BARREL_R)
        .extrude(PADDLE_BARREL_L / 2.0 + 0.010, both=True)
        .translate((0.0, PADDLE_PIVOT_Y, PADDLE_PIVOT_Z))
    )
    front_slot = _box_wp(
        (HANDSET_LEN - 0.020, 0.014, 0.018),
        (0.0, 0.004, -0.016),
    )
    return body.union(mount_pad).cut(groove).cut(front_slot)


def _make_paddle_shape() -> cq.Workplane:
    barrel = cq.Workplane("YZ").circle(PADDLE_BARREL_R).extrude(PADDLE_BARREL_L / 2.0, both=True)
    web = _box_wp((0.090, 0.009, 0.010), (0.0, -0.0035, -0.0085))
    blade = _box_wp((0.108, 0.018, 0.020), (0.0, -0.012, -0.015))
    lip = _box_wp((0.094, 0.010, 0.006), (0.0, -0.021, -0.023))
    paddle = barrel.union(web).union(blade).union(lip)
    return paddle


def _add_leg_part(
    model: ArticulatedObject,
    *,
    name: str,
    foot_size: tuple[float, float],
    material: str,
) -> None:
    leg = model.part(name)
    leg.visual(
        Box((INNER_COLUMN_W, INNER_COLUMN_D, INNER_COLUMN_L)),
        origin=Origin(xyz=(0.0, 0.0, -INNER_COLUMN_L / 2.0)),
        material=material,
        name="inner_column",
    )
    leg.visual(
        mesh_from_cadquery(_make_foot_shape(*foot_size), f"{name}_foot"),
        material=material,
        name="foot",
    )


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_sleeve_part(model: ArticulatedObject, *, name: str, material: str) -> None:
    sleeve = model.part(name)
    _add_box_visual(
        sleeve,
        size=(SLEEVE_PLATE_X, SLEEVE_PLATE_Y, FRAME_PLATE_T),
        center=(0.0, 0.0, -FRAME_PLATE_T / 2.0),
        material=material,
        name="top_plate",
    )
    _add_box_visual(
        sleeve,
        size=(SLEEVE_WALL, SLEEVE_D, SLEEVE_H),
        center=(-(SLEEVE_W - SLEEVE_WALL) / 2.0, 0.0, -SLEEVE_H / 2.0),
        material=material,
        name="left_wall",
    )
    _add_box_visual(
        sleeve,
        size=(SLEEVE_WALL, SLEEVE_D, SLEEVE_H),
        center=((SLEEVE_W - SLEEVE_WALL) / 2.0, 0.0, -SLEEVE_H / 2.0),
        material=material,
        name="right_wall",
    )
    _add_box_visual(
        sleeve,
        size=(SLEEVE_W - 2.0 * SLEEVE_WALL, SLEEVE_WALL, SLEEVE_H),
        center=(0.0, -(SLEEVE_D - SLEEVE_WALL) / 2.0, -SLEEVE_H / 2.0),
        material=material,
        name="front_wall",
    )
    _add_box_visual(
        sleeve,
        size=(SLEEVE_W - 2.0 * SLEEVE_WALL, SLEEVE_WALL, SLEEVE_H),
        center=(0.0, (SLEEVE_D - SLEEVE_WALL) / 2.0, -SLEEVE_H / 2.0),
        material=material,
        name="rear_wall",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="corner_standing_desk")

    model.material("walnut_top", rgba=(0.55, 0.38, 0.23, 1.0))
    model.material("dark_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("leg_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("handset_plastic", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("paddle_plastic", rgba=(0.18, 0.19, 0.21, 1.0))

    desktop = model.part("desktop")
    desktop.visual(
        mesh_from_cadquery(_make_desktop_shape(), "desktop_top"),
        material="walnut_top",
        name="desktop_top",
    )

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_rails_shape(), "frame_rails"),
        material="dark_steel",
        name="frame_rails",
    )

    _add_sleeve_part(model, name="front_left_sleeve", material="dark_steel")
    _add_sleeve_part(model, name="front_right_sleeve", material="dark_steel")
    _add_sleeve_part(model, name="rear_sleeve", material="dark_steel")

    _add_leg_part(
        model,
        name="front_left_leg",
        foot_size=(FOOT_SHORT, FOOT_LONG),
        material="leg_steel",
    )
    _add_leg_part(
        model,
        name="front_right_leg",
        foot_size=(FOOT_SHORT, FOOT_LONG),
        material="leg_steel",
    )
    _add_leg_part(
        model,
        name="rear_leg",
        foot_size=(FOOT_LONG, FOOT_SHORT),
        material="leg_steel",
    )

    handset = model.part("handset")
    _add_box_visual(
        handset,
        size=(HANDSET_LEN, HANDSET_DEPTH, HANDSET_H),
        center=(0.0, HANDSET_DEPTH / 2.0, -HANDSET_H / 2.0),
        material="handset_plastic",
        name="handset_body",
    )
    _add_box_visual(
        handset,
        size=(HANDSET_LEN - 0.020, 0.026, 0.004),
        center=(0.0, 0.020, -0.002),
        material="handset_plastic",
        name="mount_pad",
    )

    paddle = model.part("handset_paddle")
    _add_box_visual(
        paddle,
        size=(0.110, 0.020, 0.016),
        center=(0.0, -0.010, -0.010),
        material="paddle_plastic",
        name="paddle",
    )

    model.articulation(
        "desktop_to_frame",
        ArticulationType.FIXED,
        parent=desktop,
        child=frame,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_front_left_sleeve",
        ArticulationType.FIXED,
        parent=frame,
        child="front_left_sleeve",
        origin=Origin(xyz=(FRONT_LEFT_POS[0], FRONT_LEFT_POS[1], 0.0)),
    )
    model.articulation(
        "frame_to_front_right_sleeve",
        ArticulationType.FIXED,
        parent=frame,
        child="front_right_sleeve",
        origin=Origin(xyz=(FRONT_RIGHT_POS[0], FRONT_RIGHT_POS[1], 0.0)),
    )
    model.articulation(
        "frame_to_rear_sleeve",
        ArticulationType.FIXED,
        parent=frame,
        child="rear_sleeve",
        origin=Origin(xyz=(REAR_POS[0], REAR_POS[1], 0.0)),
    )
    model.articulation(
        "front_left_sleeve_to_front_left_leg",
        ArticulationType.PRISMATIC,
        parent="front_left_sleeve",
        child="front_left_leg",
        origin=Origin(xyz=(0.0, 0.0, -FRAME_PLATE_T)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.050, lower=0.0, upper=LEG_TRAVEL),
    )
    model.articulation(
        "front_right_sleeve_to_front_right_leg",
        ArticulationType.PRISMATIC,
        parent="front_right_sleeve",
        child="front_right_leg",
        origin=Origin(xyz=(0.0, 0.0, -FRAME_PLATE_T)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.050, lower=0.0, upper=LEG_TRAVEL),
    )
    model.articulation(
        "rear_sleeve_to_rear_leg",
        ArticulationType.PRISMATIC,
        parent="rear_sleeve",
        child="rear_leg",
        origin=Origin(xyz=(0.0, 0.0, -FRAME_PLATE_T)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=800.0, velocity=0.050, lower=0.0, upper=LEG_TRAVEL),
    )
    model.articulation(
        "desktop_to_handset",
        ArticulationType.FIXED,
        parent=desktop,
        child=handset,
        origin=Origin(xyz=(HANDSET_MOUNT_X, HANDSET_MOUNT_Y, 0.0)),
    )
    model.articulation(
        "handset_to_paddle",
        ArticulationType.REVOLUTE,
        parent=handset,
        child=paddle,
        origin=Origin(xyz=(0.0, PADDLE_PIVOT_Y, PADDLE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-PADDLE_TILT, upper=PADDLE_TILT),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desktop = object_model.get_part("desktop")
    frame = object_model.get_part("frame")
    front_left_sleeve = object_model.get_part("front_left_sleeve")
    front_right_sleeve = object_model.get_part("front_right_sleeve")
    rear_sleeve = object_model.get_part("rear_sleeve")
    front_left_leg = object_model.get_part("front_left_leg")
    front_right_leg = object_model.get_part("front_right_leg")
    rear_leg = object_model.get_part("rear_leg")
    handset = object_model.get_part("handset")
    paddle = object_model.get_part("handset_paddle")

    front_left_joint = object_model.get_articulation("front_left_sleeve_to_front_left_leg")
    front_right_joint = object_model.get_articulation("front_right_sleeve_to_front_right_leg")
    rear_joint = object_model.get_articulation("rear_sleeve_to_rear_leg")
    paddle_joint = object_model.get_articulation("handset_to_paddle")

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

    ctx.expect_contact(front_left_sleeve, desktop, name="front left sleeve top plate contacts the desktop")
    ctx.expect_contact(front_right_sleeve, desktop, name="front right sleeve top plate contacts the desktop")
    ctx.expect_contact(rear_sleeve, desktop, name="rear sleeve top plate contacts the desktop")
    ctx.expect_contact(frame, desktop, name="underframe bears against the desktop underside")
    ctx.expect_contact(handset, desktop, name="handset mounts directly below the desktop")
    ctx.expect_contact(paddle, handset, name="paddle stays carried by the handset body")

    desktop_aabb = ctx.part_world_aabb(desktop)
    handset_pos = ctx.part_world_position(handset)
    ctx.check(
        "handset sits below the right front edge",
        desktop_aabb is not None
        and handset_pos is not None
        and handset_pos[0] > desktop_aabb[1][0] - 0.25
        and handset_pos[1] < desktop_aabb[0][1] + 0.06
        and handset_pos[2] < desktop_aabb[0][2] + 0.001,
        details=f"desktop_aabb={desktop_aabb}, handset_pos={handset_pos}",
    )

    for leg, sleeve, label in (
        (front_left_leg, front_left_sleeve, "front left"),
        (front_right_leg, front_right_sleeve, "front right"),
        (rear_leg, rear_sleeve, "rear"),
    ):
        ctx.expect_origin_distance(
            leg,
            sleeve,
            axes="xy",
            max_dist=0.001,
            name=f"{label} leg stays centered on its sleeve axis at rest",
        )
        ctx.expect_overlap(
            leg,
            sleeve,
            axes="z",
            min_overlap=0.32,
            name=f"{label} leg keeps deep insertion at seated height",
        )

    rest_positions = {
        "front_left": ctx.part_world_position(front_left_leg),
        "front_right": ctx.part_world_position(front_right_leg),
        "rear": ctx.part_world_position(rear_leg),
    }
    with ctx.pose(
        {
            front_left_joint: LEG_TRAVEL,
            front_right_joint: LEG_TRAVEL,
            rear_joint: LEG_TRAVEL,
        }
    ):
        for leg, sleeve, label in (
            (front_left_leg, front_left_sleeve, "front left"),
            (front_right_leg, front_right_sleeve, "front right"),
            (rear_leg, rear_sleeve, "rear"),
        ):
            ctx.expect_origin_distance(
                leg,
                sleeve,
                axes="xy",
                max_dist=0.001,
                name=f"{label} leg stays centered on its sleeve axis when extended",
            )
            ctx.expect_overlap(
                leg,
                sleeve,
                axes="z",
                min_overlap=0.06,
                name=f"{label} leg retains insertion at full standing height",
            )
        extended_positions = {
            "front_left": ctx.part_world_position(front_left_leg),
            "front_right": ctx.part_world_position(front_right_leg),
            "rear": ctx.part_world_position(rear_leg),
        }

    for key, label in (
        ("front_left", "front left"),
        ("front_right", "front right"),
        ("rear", "rear"),
    ):
        rest = rest_positions[key]
        extended = extended_positions[key]
        ctx.check(
            f"{label} leg extends downward",
            rest is not None and extended is not None and extended[2] < rest[2] - 0.20,
            details=f"rest={rest}, extended={extended}",
        )

    neutral_paddle_aabb = ctx.part_world_aabb(paddle)
    with ctx.pose({paddle_joint: PADDLE_TILT}):
        pressed_paddle_aabb = ctx.part_world_aabb(paddle)
    ctx.check(
        "positive paddle motion tilts the switch downward",
        neutral_paddle_aabb is not None
        and pressed_paddle_aabb is not None
        and pressed_paddle_aabb[0][2] < neutral_paddle_aabb[0][2] - 0.004,
        details=f"neutral={neutral_paddle_aabb}, pressed={pressed_paddle_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
