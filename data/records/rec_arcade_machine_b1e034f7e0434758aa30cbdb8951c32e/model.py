from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cocktail_table_arcade_machine")

    cabinet_paint = model.material("cabinet_paint", rgba=(0.17, 0.16, 0.15, 1.0))
    black_trim = model.material("black_trim", rgba=(0.07, 0.07, 0.08, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.33, 0.39, 0.42, 0.38))
    screen_black = model.material("screen_black", rgba=(0.05, 0.06, 0.07, 1.0))
    screen_glow = model.material("screen_glow", rgba=(0.12, 0.53, 0.68, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.72, 0.73, 0.75, 1.0))
    button_red = model.material("button_red", rgba=(0.78, 0.18, 0.17, 1.0))
    button_yellow = model.material("button_yellow", rgba=(0.88, 0.72, 0.15, 1.0))
    button_blue = model.material("button_blue", rgba=(0.20, 0.45, 0.83, 1.0))
    joystick_black = model.material("joystick_black", rgba=(0.09, 0.09, 0.10, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))

    cabinet = model.part("cabinet")

    cabinet.visual(
        Box((0.90, 0.60, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=black_trim,
        name="base_plinth",
    )

    cabinet.visual(
        Box((0.91, 0.03, 0.56)),
        origin=Origin(xyz=(0.0, 0.305, 0.34)),
        material=cabinet_paint,
        name="left_body_wall",
    )
    cabinet.visual(
        Box((0.91, 0.03, 0.56)),
        origin=Origin(xyz=(0.0, -0.305, 0.34)),
        material=cabinet_paint,
        name="right_body_wall",
    )

    cabinet.visual(
        Box((0.03, 0.58, 0.56)),
        origin=Origin(xyz=(-0.455, 0.0, 0.34)),
        material=cabinet_paint,
        name="rear_body_wall",
    )

    cabinet.visual(
        Box((0.03, 0.58, 0.16)),
        origin=Origin(xyz=(0.455, 0.0, 0.14)),
        material=cabinet_paint,
        name="front_wall_lower",
    )
    cabinet.visual(
        Box((0.03, 0.58, 0.22)),
        origin=Origin(xyz=(0.455, 0.0, 0.51)),
        material=cabinet_paint,
        name="front_wall_upper",
    )
    cabinet.visual(
        Box((0.03, 0.39, 0.18)),
        origin=Origin(xyz=(0.455, -0.095, 0.31)),
        material=cabinet_paint,
        name="front_wall_left_jamb",
    )
    cabinet.visual(
        Box((0.03, 0.03, 0.18)),
        origin=Origin(xyz=(0.455, 0.275, 0.31)),
        material=cabinet_paint,
        name="front_wall_right_jamb",
    )

    cabinet.visual(
        Box((0.78, 0.12, 0.026)),
        origin=Origin(xyz=(0.0, 0.23, 0.608)),
        material=black_trim,
        name="player_panel_pos",
    )
    cabinet.visual(
        Box((0.78, 0.12, 0.026)),
        origin=Origin(xyz=(0.0, -0.23, 0.608)),
        material=black_trim,
        name="player_panel_neg",
    )
    cabinet.visual(
        Box((0.11, 0.58, 0.026)),
        origin=Origin(xyz=(0.405, 0.0, 0.608)),
        material=black_trim,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((0.11, 0.58, 0.026)),
        origin=Origin(xyz=(-0.405, 0.0, 0.608)),
        material=black_trim,
        name="rear_top_rail",
    )

    cabinet.visual(
        Box((0.78, 0.03, 0.12)),
        origin=Origin(xyz=(0.0, 0.165, 0.54)),
        material=cabinet_paint,
        name="screen_well_left",
    )
    cabinet.visual(
        Box((0.78, 0.03, 0.12)),
        origin=Origin(xyz=(0.0, -0.165, 0.54)),
        material=cabinet_paint,
        name="screen_well_right",
    )
    cabinet.visual(
        Box((0.03, 0.30, 0.12)),
        origin=Origin(xyz=(0.39, 0.0, 0.54)),
        material=cabinet_paint,
        name="screen_well_front",
    )
    cabinet.visual(
        Box((0.03, 0.30, 0.12)),
        origin=Origin(xyz=(-0.39, 0.0, 0.54)),
        material=cabinet_paint,
        name="screen_well_rear",
    )

    cabinet.visual(
        Box((0.76, 0.02, 0.022)),
        origin=Origin(xyz=(0.0, 0.17, 0.609)),
        material=black_trim,
        name="glass_lip_left",
    )
    cabinet.visual(
        Box((0.76, 0.02, 0.022)),
        origin=Origin(xyz=(0.0, -0.17, 0.609)),
        material=black_trim,
        name="glass_lip_right",
    )
    cabinet.visual(
        Box((0.02, 0.32, 0.022)),
        origin=Origin(xyz=(0.38, 0.0, 0.609)),
        material=black_trim,
        name="glass_lip_front",
    )
    cabinet.visual(
        Box((0.02, 0.32, 0.022)),
        origin=Origin(xyz=(-0.38, 0.0, 0.609)),
        material=black_trim,
        name="glass_lip_rear",
    )
    cabinet.visual(
        Box((0.58, 0.34, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.495)),
        material=black_trim,
        name="screen_shelf",
    )
    cabinet.visual(
        Box((0.50, 0.03, 0.028)),
        origin=Origin(xyz=(0.0, 0.125, 0.524)),
        material=black_trim,
        name="screen_support_left",
    )
    cabinet.visual(
        Box((0.50, 0.03, 0.028)),
        origin=Origin(xyz=(0.0, -0.125, 0.524)),
        material=black_trim,
        name="screen_support_right",
    )

    def add_player_controls(side_sign: float, prefix: str) -> None:
        panel_y = 0.23 * side_sign
        panel_label_y = 0.19 * side_sign

        cabinet.visual(
            Box((0.36, 0.10, 0.003)),
            origin=Origin(xyz=(-0.05, panel_y, 0.6225)),
            material=brushed_metal,
            name=f"{prefix}_overlay",
        )

        if side_sign > 0.0:
            cabinet.visual(
                Box((0.09, 0.06, 0.003)),
                origin=Origin(xyz=(0.25, panel_label_y, 0.6225)),
                material=brushed_metal,
                name="selector_mount_plate",
            )

        cabinet.visual(
            Cylinder(radius=0.020, length=0.012),
            origin=Origin(xyz=(-0.18, panel_label_y, 0.629), rpy=(0.0, 0.0, 0.0)),
            material=joystick_black,
            name=f"{prefix}_joystick_base",
        )
        cabinet.visual(
            Cylinder(radius=0.005, length=0.058),
            origin=Origin(xyz=(-0.18, panel_label_y, 0.652)),
            material=joystick_black,
            name=f"{prefix}_joystick_shaft",
        )
        cabinet.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=(-0.18, panel_label_y, 0.689)),
            material=button_red if side_sign > 0.0 else button_blue,
            name=f"{prefix}_joystick_ball",
        )

        button_xs = (-0.04, 0.02, 0.08)
        button_materials = (button_red, button_yellow, button_blue)
        for index, (button_x, button_material) in enumerate(zip(button_xs, button_materials)):
            cabinet.visual(
                Cylinder(radius=0.015, length=0.010),
                origin=Origin(xyz=(button_x, panel_label_y, 0.629)),
                material=button_material,
                name=f"{prefix}_button_{index}",
            )

    add_player_controls(1.0, "player_one")
    add_player_controls(-1.0, "player_two")

    cabinet.inertial = Inertial.from_geometry(
        Box((0.94, 0.64, 0.68)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
    )

    glass_top = model.part("glass_top")
    glass_top.visual(
        Box((0.76, 0.32, 0.006)),
        material=smoked_glass,
        name="glass_panel",
    )
    glass_top.inertial = Inertial.from_geometry(
        Box((0.76, 0.32, 0.006)),
        mass=6.0,
    )
    model.articulation(
        "cabinet_to_glass_top",
        ArticulationType.FIXED,
        parent=cabinet,
        child=glass_top,
        origin=Origin(xyz=(0.0, 0.0, 0.622)),
    )

    screen_module = model.part("screen_module")
    screen_module.visual(
        Box((0.54, 0.28, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=screen_black,
        name="screen_bezel",
    )
    screen_module.visual(
        Box((0.48, 0.22, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=screen_glow,
        name="screen_face",
    )
    screen_module.inertial = Inertial.from_geometry(
        Box((0.54, 0.28, 0.038)),
        mass=9.0,
    )
    model.articulation(
        "cabinet_to_screen_module",
        ArticulationType.FIXED,
        parent=cabinet,
        child=screen_module,
        origin=Origin(xyz=(0.0, 0.0, 0.557)),
    )

    coin_door = model.part("coin_door")
    coin_door.visual(
        Box((0.018, 0.16, 0.18)),
        origin=Origin(xyz=(0.009, 0.0, -0.09)),
        material=brushed_metal,
        name="coin_door_panel",
    )
    coin_door.visual(
        Box((0.004, 0.06, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, -0.035)),
        material=screen_black,
        name="coin_slot",
    )
    coin_door.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.020, 0.0, -0.115), rpy=(0.0, 1.57079632679, 0.0)),
        material=screen_black,
        name="coin_return_pull",
    )
    coin_door.inertial = Inertial.from_geometry(
        Box((0.018, 0.16, 0.18)),
        mass=1.2,
        origin=Origin(xyz=(0.009, 0.0, -0.09)),
    )
    model.articulation(
        "cabinet_to_coin_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=coin_door,
        origin=Origin(xyz=(0.47, 0.18, 0.40)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=knob_black,
        name="selector_collar",
    )
    selector_knob.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=brushed_metal,
        name="selector_shaft",
    )
    selector_knob.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=knob_black,
        name="selector_body",
    )
    selector_knob.visual(
        Cylinder(radius=0.016, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0405)),
        material=brushed_metal,
        name="selector_cap",
    )
    selector_knob.inertial = Inertial.from_geometry(
        Box((0.044, 0.044, 0.045)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(0.25, 0.19, 0.624)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    cabinet = object_model.get_part("cabinet")
    glass_top = object_model.get_part("glass_top")
    screen_module = object_model.get_part("screen_module")
    coin_door = object_model.get_part("coin_door")
    selector_knob = object_model.get_part("selector_knob")
    coin_hinge = object_model.get_articulation("cabinet_to_coin_door")
    knob_joint = object_model.get_articulation("cabinet_to_selector_knob")

    cabinet_visual_names = {visual.name for visual in cabinet.visuals if visual.name is not None}
    ctx.check(
        "cabinet has mirrored long-side control panels",
        {"player_panel_pos", "player_panel_neg"} <= cabinet_visual_names,
        details=f"visuals={sorted(cabinet_visual_names)}",
    )

    ctx.check(
        "coin door uses a front horizontal hinge under one side rail",
        coin_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 6) for value in coin_hinge.axis) == (0.0, -1.0, 0.0)
        and coin_hinge.motion_limits is not None
        and coin_hinge.motion_limits.lower == 0.0
        and coin_hinge.motion_limits.upper is not None
        and coin_hinge.motion_limits.upper >= 1.2
        and coin_hinge.origin.xyz[0] > 0.44
        and coin_hinge.origin.xyz[1] > 0.10,
        details=f"axis={coin_hinge.axis}, origin={coin_hinge.origin.xyz}, limits={coin_hinge.motion_limits}",
    )

    ctx.check(
        "selector knob spins on a vertical continuous shaft",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in knob_joint.axis) == (0.0, 0.0, 1.0)
        and knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower is None
        and knob_joint.motion_limits.upper is None
        and knob_joint.origin.xyz[1] > 0.12,
        details=f"axis={knob_joint.axis}, origin={knob_joint.origin.xyz}, limits={knob_joint.motion_limits}",
    )

    with ctx.pose({coin_hinge: 0.0}):
        ctx.expect_contact(
            coin_door,
            cabinet,
            name="coin door closes flush to cabinet front",
        )
        ctx.expect_gap(
            glass_top,
            screen_module,
            axis="z",
            min_gap=0.03,
            max_gap=0.05,
            name="glass sits above recessed screen",
        )
        ctx.expect_overlap(
            glass_top,
            screen_module,
            axes="xy",
            min_overlap=0.28,
            name="screen stays under the glass opening",
        )
        ctx.expect_contact(
            selector_knob,
            cabinet,
            name="selector knob is mounted to the player panel",
        )
        closed_door_aabb = ctx.part_world_aabb(coin_door)

    with ctx.pose({coin_hinge: 1.2, knob_joint: 1.7}):
        opened_door_aabb = ctx.part_world_aabb(coin_door)
        spun_knob_position = ctx.part_world_position(selector_knob)

    rest_knob_position = ctx.part_world_position(selector_knob)
    ctx.check(
        "coin door swings outward when opened",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][0] > closed_door_aabb[1][0] + 0.08,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )
    ctx.check(
        "selector knob rotates in place",
        rest_knob_position is not None
        and spun_knob_position is not None
        and all(
            abs(rest_coordinate - spun_coordinate) < 1e-9
            for rest_coordinate, spun_coordinate in zip(rest_knob_position, spun_knob_position)
        ),
        details=f"rest={rest_knob_position}, spun={spun_knob_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
