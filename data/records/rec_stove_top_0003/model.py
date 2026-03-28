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
    DomeGeometry,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)

BODY_WIDTH = 0.900
BODY_DEPTH = 0.520
PLATE_WIDTH = 0.892
PLATE_DEPTH = 0.512
PLATE_THICKNESS = 0.006
PLATE_CENTER_Z = 0.055
PLATE_TOP_Z = PLATE_CENTER_Z + PLATE_THICKNESS * 0.5
PANEL_WIDTH = 0.790
PANEL_HEIGHT = 0.048
PANEL_THICKNESS = 0.006
PANEL_CENTER_Y = -0.257
PANEL_CENTER_Z = 0.026
KNOB_Z = 0.026

BURNER_SPECS = (
    {
        "name": "left_rear",
        "position": (-0.255, 0.132),
        "kind": "regular",
        "hole_radius": 0.035,
        "knob_x": -0.290,
    },
    {
        "name": "left_front",
        "position": (-0.225, -0.098),
        "kind": "regular",
        "hole_radius": 0.035,
        "knob_x": -0.145,
    },
    {
        "name": "center_wok",
        "position": (0.000, 0.032),
        "kind": "center",
        "hole_radius": 0.050,
        "knob_x": 0.000,
    },
    {
        "name": "right_front",
        "position": (0.225, -0.098),
        "kind": "regular",
        "hole_radius": 0.035,
        "knob_x": 0.145,
    },
    {
        "name": "right_rear",
        "position": (0.255, 0.132),
        "kind": "regular",
        "hole_radius": 0.035,
        "knob_x": 0.290,
    },
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 28,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * index / segments),
            cy + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _build_top_plate_mesh():
    outer_profile = rounded_rect_profile(
        PLATE_WIDTH,
        PLATE_DEPTH,
        radius=0.018,
        corner_segments=10,
    )
    hole_profiles = [
        _circle_profile(spec["hole_radius"], center=spec["position"], segments=36) for spec in BURNER_SPECS
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            hole_profiles,
            PLATE_THICKNESS,
            cap=True,
            center=True,
            closed=True,
        ),
        ASSETS.mesh_path("premium_stovetop_plate.obj"),
    )


def _build_control_panel_mesh(knob_xs: tuple[float, ...]):
    outer_profile = rounded_rect_profile(
        PANEL_WIDTH,
        PANEL_HEIGHT,
        radius=0.006,
        corner_segments=6,
    )
    hole_profiles = [_circle_profile(0.0065, center=(knob_x, 0.0), segments=24) for knob_x in knob_xs]
    geom = ExtrudeWithHolesGeometry(
        outer_profile,
        hole_profiles,
        PANEL_THICKNESS,
        cap=True,
        center=True,
        closed=True,
    )
    geom.rotate_x(-math.pi * 0.5)
    return mesh_from_geometry(geom, ASSETS.mesh_path("premium_stovetop_control_panel.obj"))


def _build_burner_bowl_mesh(
    mesh_name: str,
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=40,
            start_cap="flat",
            end_cap="flat",
        ),
        ASSETS.mesh_path(mesh_name),
    )


def _build_knob_shell_mesh():
    return mesh_from_geometry(
        LatheGeometry(
            [
                (0.000, 0.000),
                (0.0135, 0.000),
                (0.0175, 0.0025),
                (0.0185, 0.0075),
                (0.0178, 0.0135),
                (0.0160, 0.0185),
                (0.0132, 0.0230),
                (0.000, 0.0230),
            ],
            segments=44,
            closed=True,
        ),
        ASSETS.mesh_path("premium_stovetop_knob_shell.obj"),
    )


def _add_pair_grate(
    part,
    *,
    width: float,
    depth: float,
    foot_height: float,
    foot_span_x: float,
    foot_span_y: float,
    support_y: tuple[float, float],
    material,
) -> None:
    rail = 0.016
    bar_h = 0.008
    foot = 0.014
    frame_z = foot_height + bar_h * 0.5
    foot_z = foot_height * 0.5

    part.visual(
        Box((width, rail, bar_h)),
        origin=Origin(xyz=(0.0, -(depth * 0.5 - rail * 0.5), frame_z)),
        material=material,
        name="front_rail",
    )
    part.visual(
        Box((width, rail, bar_h)),
        origin=Origin(xyz=(0.0, depth * 0.5 - rail * 0.5, frame_z)),
        material=material,
        name="rear_rail",
    )
    part.visual(
        Box((rail, depth, bar_h)),
        origin=Origin(xyz=(-(width * 0.5 - rail * 0.5), 0.0, frame_z)),
        material=material,
        name="left_rail",
    )
    part.visual(
        Box((rail, depth, bar_h)),
        origin=Origin(xyz=(width * 0.5 - rail * 0.5, 0.0, frame_z)),
        material=material,
        name="right_rail",
    )
    part.visual(
        Box((0.018, depth - rail, bar_h)),
        origin=Origin(xyz=(0.0, 0.0, frame_z)),
        material=material,
        name="center_spine",
    )
    part.visual(
        Box((width - rail, 0.014, bar_h)),
        origin=Origin(xyz=(0.0, support_y[0], frame_z)),
        material=material,
        name="front_crossbar",
    )
    part.visual(
        Box((width - rail, 0.014, bar_h)),
        origin=Origin(xyz=(0.0, support_y[1], frame_z)),
        material=material,
        name="rear_crossbar",
    )

    foot_positions = {
        "front_left_foot": (-(foot_span_x * 0.5), -(foot_span_y * 0.5), foot_z),
        "front_right_foot": (foot_span_x * 0.5, -(foot_span_y * 0.5), foot_z),
        "rear_left_foot": (-(foot_span_x * 0.5), foot_span_y * 0.5, foot_z),
        "rear_right_foot": (foot_span_x * 0.5, foot_span_y * 0.5, foot_z),
    }
    for name, xyz in foot_positions.items():
        part.visual(Box((foot, foot, foot_height)), origin=Origin(xyz=xyz), material=material, name=name)


def _add_center_grate(part, *, size: float, foot_height: float, material) -> None:
    rail = 0.016
    bar_h = 0.008
    foot = 0.014
    frame_z = foot_height + bar_h * 0.5
    foot_z = foot_height * 0.5
    half = size * 0.5

    part.visual(
        Box((size, rail, bar_h)),
        origin=Origin(xyz=(0.0, -(half - rail * 0.5), frame_z)),
        material=material,
        name="front_rail",
    )
    part.visual(
        Box((size, rail, bar_h)),
        origin=Origin(xyz=(0.0, half - rail * 0.5, frame_z)),
        material=material,
        name="rear_rail",
    )
    part.visual(
        Box((rail, size, bar_h)),
        origin=Origin(xyz=(-(half - rail * 0.5), 0.0, frame_z)),
        material=material,
        name="left_rail",
    )
    part.visual(
        Box((rail, size, bar_h)),
        origin=Origin(xyz=((half - rail * 0.5), 0.0, frame_z)),
        material=material,
        name="right_rail",
    )
    part.visual(
        Box((size - rail, 0.014, bar_h)),
        origin=Origin(xyz=(0.0, 0.0, frame_z)),
        material=material,
        name="center_crossbar",
    )
    part.visual(
        Box((0.014, size - rail, bar_h)),
        origin=Origin(xyz=(0.0, 0.0, frame_z)),
        material=material,
        name="center_spine",
    )

    foot_positions = {
        "front_left_foot": (-(size * 0.5 - 0.022), -(size * 0.5 - 0.022), foot_z),
        "front_right_foot": (size * 0.5 - 0.022, -(size * 0.5 - 0.022), foot_z),
        "rear_left_foot": (-(size * 0.5 - 0.022), size * 0.5 - 0.022, foot_z),
        "rear_right_foot": (size * 0.5 - 0.022, size * 0.5 - 0.022, foot_z),
    }
    for name, xyz in foot_positions.items():
        part.visual(Box((foot, foot, foot_height)), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_stove_top", assets=ASSETS)

    body_graphite = model.material("body_graphite", rgba=(0.15, 0.16, 0.17, 1.0))
    panel_graphite = model.material("panel_graphite", rgba=(0.11, 0.12, 0.13, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    burner_cast = model.material("burner_cast", rgba=(0.20, 0.20, 0.21, 1.0))
    burner_metal = model.material("burner_metal", rgba=(0.47, 0.48, 0.50, 1.0))
    grate_iron = model.material("grate_iron", rgba=(0.12, 0.12, 0.13, 1.0))
    knob_black = model.material("knob_black", rgba=(0.13, 0.13, 0.14, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.63, 0.64, 0.66, 1.0))

    regular_bowl_mesh = _build_burner_bowl_mesh(
        "regular_burner_bowl.obj",
        outer_profile=[
            (0.018, -0.021),
            (0.026, -0.018),
            (0.031, -0.010),
            (0.033, -0.003),
            (0.033, 0.000),
        ],
        inner_profile=[
            (0.000, -0.018),
            (0.018, -0.018),
            (0.022, -0.010),
            (0.026, -0.003),
            (0.029, 0.000),
        ],
    )
    center_bowl_mesh = _build_burner_bowl_mesh(
        "center_burner_bowl.obj",
        outer_profile=[
            (0.025, -0.024),
            (0.036, -0.020),
            (0.043, -0.011),
            (0.046, -0.003),
            (0.046, 0.000),
        ],
        inner_profile=[
            (0.000, -0.020),
            (0.025, -0.020),
            (0.031, -0.011),
            (0.036, -0.003),
            (0.041, 0.000),
        ],
    )
    regular_cap_mesh = mesh_from_geometry(
        DomeGeometry(radius=(0.020, 0.020, 0.0065), radial_segments=30, height_segments=12, closed=True),
        ASSETS.mesh_path("regular_burner_cap.obj"),
    )
    center_cap_mesh = mesh_from_geometry(
        DomeGeometry(radius=(0.026, 0.026, 0.0085), radial_segments=32, height_segments=12, closed=True),
        ASSETS.mesh_path("center_burner_cap.obj"),
    )
    top_plate_mesh = _build_top_plate_mesh()
    control_panel_mesh = _build_control_panel_mesh(tuple(spec["knob_x"] for spec in BURNER_SPECS))
    knob_shell_mesh = _build_knob_shell_mesh()

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.840, 0.400, 0.030)),
        origin=Origin(xyz=(0.000, 0.030, 0.015)),
        material=body_graphite,
        name="main_body",
    )
    chassis.visual(
        Box((0.760, 0.048, 0.048)),
        origin=Origin(xyz=(0.000, -0.194, 0.024)),
        material=body_graphite,
        name="front_mount_rail",
    )
    chassis.visual(
        Box((0.038, 0.092, 0.048)),
        origin=Origin(xyz=(-0.401, -0.205, 0.024)),
        material=body_graphite,
        name="left_cheek",
    )
    chassis.visual(
        Box((0.038, 0.092, 0.048)),
        origin=Origin(xyz=(0.401, -0.205, 0.024)),
        material=body_graphite,
        name="right_cheek",
    )
    chassis.visual(
        Box((0.022, 0.490, 0.004)),
        origin=Origin(xyz=(-0.435, 0.000, 0.050)),
        material=body_graphite,
        name="left_support_ledge",
    )
    chassis.visual(
        Box((0.022, 0.490, 0.004)),
        origin=Origin(xyz=(0.435, 0.000, 0.050)),
        material=body_graphite,
        name="right_support_ledge",
    )
    chassis.visual(
        Box((0.790, 0.022, 0.004)),
        origin=Origin(xyz=(0.000, 0.245, 0.050)),
        material=body_graphite,
        name="rear_support_ledge",
    )
    chassis.visual(
        Box((0.790, 0.022, 0.004)),
        origin=Origin(xyz=(0.000, -0.245, 0.050)),
        material=body_graphite,
        name="front_support_ledge",
    )
    chassis.visual(
        Box((0.024, 0.490, 0.020)),
        origin=Origin(xyz=(-0.423, 0.000, 0.040)),
        material=body_graphite,
        name="left_support_web",
    )
    chassis.visual(
        Box((0.024, 0.490, 0.020)),
        origin=Origin(xyz=(0.423, 0.000, 0.040)),
        material=body_graphite,
        name="right_support_web",
    )
    chassis.visual(
        Box((0.790, 0.024, 0.020)),
        origin=Origin(xyz=(0.000, 0.238, 0.040)),
        material=body_graphite,
        name="rear_support_web",
    )
    chassis.visual(
        Box((0.790, 0.030, 0.020)),
        origin=Origin(xyz=(0.000, -0.231, 0.040)),
        material=body_graphite,
        name="front_support_web",
    )
    tab_centers = (-0.362, -0.218, -0.072, 0.072, 0.218, 0.362)
    tab_names = (
        "panel_tab_left",
        "panel_tab_left_mid",
        "panel_tab_inner_left",
        "panel_tab_inner_right",
        "panel_tab_right_mid",
        "panel_tab_right",
    )
    for tab_name, tab_x in zip(tab_names, tab_centers):
        chassis.visual(
            Box((0.040, 0.040, 0.024)),
            origin=Origin(xyz=(tab_x, -0.234, 0.026)),
            material=body_graphite,
            name=tab_name,
        )

    top_plate = model.part("top_plate")
    top_plate.visual(top_plate_mesh, material=satin_steel, name="plate_skin")
    model.articulation(
        "chassis_to_top_plate",
        ArticulationType.FIXED,
        parent=chassis,
        child=top_plate,
        origin=Origin(xyz=(0.000, 0.000, PLATE_CENTER_Z)),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(control_panel_mesh, material=panel_graphite, name="panel_skin")
    control_panel.visual(
        Box((0.778, 0.002, 0.004)),
        origin=Origin(xyz=(0.000, -0.003, 0.018)),
        material=satin_steel,
        name="satin_break_strip",
    )
    model.articulation(
        "chassis_to_control_panel",
        ArticulationType.FIXED,
        parent=chassis,
        child=control_panel,
        origin=Origin(xyz=(0.000, PANEL_CENTER_Y, PANEL_CENTER_Z)),
    )

    regular_crown_radius = 0.027
    for spec in BURNER_SPECS:
        burner = model.part(f"{spec['name']}_burner")
        if spec["kind"] == "center":
            burner.visual(center_bowl_mesh, origin=Origin(xyz=(0.000, 0.000, 0.003)), material=burner_metal, name="bowl_shell")
            burner.visual(
                Cylinder(radius=0.060, length=0.004),
                origin=Origin(xyz=(0.000, 0.000, 0.005)),
                material=burner_metal,
                name="trim_ring",
            )
            burner.visual(
                Cylinder(radius=0.038, length=0.010),
                origin=Origin(xyz=(0.000, 0.000, 0.011)),
                material=burner_cast,
                name="burner_crown",
            )
            burner.visual(
                Cylinder(radius=0.022, length=0.008),
                origin=Origin(xyz=(0.000, 0.000, 0.015)),
                material=burner_cast,
                name="inner_crown",
            )
            burner.visual(center_cap_mesh, origin=Origin(xyz=(0.000, 0.000, 0.017)), material=burner_cast, name="burner_cap")
        else:
            burner.visual(regular_bowl_mesh, origin=Origin(xyz=(0.000, 0.000, 0.003)), material=burner_metal, name="bowl_shell")
            burner.visual(
                Cylinder(radius=0.044, length=0.004),
                origin=Origin(xyz=(0.000, 0.000, 0.005)),
                material=burner_metal,
                name="trim_ring",
            )
            burner.visual(
                Cylinder(radius=regular_crown_radius, length=0.010),
                origin=Origin(xyz=(0.000, 0.000, 0.011)),
                material=burner_cast,
                name="burner_crown",
            )
            burner.visual(
                regular_cap_mesh,
                origin=Origin(xyz=(0.000, 0.000, 0.016)),
                material=burner_cast,
                name="burner_cap",
            )
        model.articulation(
            f"top_plate_to_{spec['name']}_burner",
            ArticulationType.FIXED,
            parent=top_plate,
            child=burner,
            origin=Origin(xyz=(spec["position"][0], spec["position"][1], 0.000)),
        )

        knob = model.part(f"{spec['name']}_knob")
        knob.visual(
            Cylinder(radius=0.0045, length=0.022),
            origin=Origin(xyz=(0.000, 0.004, 0.000), rpy=(math.pi * 0.5, 0.000, 0.000)),
            material=shaft_steel,
            name="shaft",
        )
        knob.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(0.000, 0.0045, 0.000), rpy=(math.pi * 0.5, 0.000, 0.000)),
            material=shaft_steel,
            name="rear_collar",
        )
        knob.visual(
            knob_shell_mesh,
            origin=Origin(xyz=(0.000, -0.005, 0.000), rpy=(math.pi * 0.5, 0.000, 0.000)),
            material=knob_black,
            name="knob_body",
        )
        knob.visual(
            Cylinder(radius=0.0105, length=0.0015),
            origin=Origin(xyz=(0.000, -0.0275, 0.000), rpy=(math.pi * 0.5, 0.000, 0.000)),
            material=satin_steel,
            name="face_cap",
        )
        knob.visual(
            Box((0.003, 0.004, 0.010)),
            origin=Origin(xyz=(0.000, -0.026, 0.010)),
            material=satin_steel,
            name="indicator",
        )
        model.articulation(
            f"control_panel_to_{spec['name']}_knob",
            ArticulationType.REVOLUTE,
            parent=control_panel,
            child=knob,
            origin=Origin(xyz=(spec["knob_x"], 0.000, KNOB_Z - PANEL_CENTER_Z)),
            axis=(0.000, 1.000, 0.000),
            motion_limits=MotionLimits(
                effort=0.6,
                velocity=4.0,
                lower=0.000,
                upper=5.000,
            ),
        )

    left_grate = model.part("left_grate")
    _add_pair_grate(
        left_grate,
        width=0.182,
        depth=0.330,
        foot_height=0.022,
        foot_span_x=0.134,
        foot_span_y=0.286,
        support_y=(-0.115, 0.115),
        material=grate_iron,
    )
    model.articulation(
        "top_plate_to_left_grate",
        ArticulationType.FIXED,
        parent=top_plate,
        child=left_grate,
        origin=Origin(xyz=(-0.240, 0.017, PLATE_TOP_Z - PLATE_CENTER_Z)),
    )

    right_grate = model.part("right_grate")
    _add_pair_grate(
        right_grate,
        width=0.182,
        depth=0.330,
        foot_height=0.022,
        foot_span_x=0.134,
        foot_span_y=0.286,
        support_y=(-0.115, 0.115),
        material=grate_iron,
    )
    model.articulation(
        "top_plate_to_right_grate",
        ArticulationType.FIXED,
        parent=top_plate,
        child=right_grate,
        origin=Origin(xyz=(0.240, 0.017, PLATE_TOP_Z - PLATE_CENTER_Z)),
    )

    center_grate = model.part("center_grate")
    _add_center_grate(center_grate, size=0.196, foot_height=0.025, material=grate_iron)
    model.articulation(
        "top_plate_to_center_grate",
        ArticulationType.FIXED,
        parent=top_plate,
        child=center_grate,
        origin=Origin(xyz=(0.000, 0.032, PLATE_TOP_Z - PLATE_CENTER_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    chassis = object_model.get_part("chassis")
    top_plate = object_model.get_part("top_plate")
    control_panel = object_model.get_part("control_panel")
    left_rear_burner = object_model.get_part("left_rear_burner")
    left_front_burner = object_model.get_part("left_front_burner")
    center_wok_burner = object_model.get_part("center_wok_burner")
    right_front_burner = object_model.get_part("right_front_burner")
    right_rear_burner = object_model.get_part("right_rear_burner")
    left_grate = object_model.get_part("left_grate")
    center_grate = object_model.get_part("center_grate")
    right_grate = object_model.get_part("right_grate")
    left_rear_knob = object_model.get_part("left_rear_knob")
    left_front_knob = object_model.get_part("left_front_knob")
    center_wok_knob = object_model.get_part("center_wok_knob")
    right_front_knob = object_model.get_part("right_front_knob")
    right_rear_knob = object_model.get_part("right_rear_knob")
    left_rear_knob_joint = object_model.get_articulation("control_panel_to_left_rear_knob")

    plate_skin = top_plate.get_visual("plate_skin")
    panel_skin = control_panel.get_visual("panel_skin")
    front_support_ledge = chassis.get_visual("front_support_ledge")
    rear_support_ledge = chassis.get_visual("rear_support_ledge")
    left_support_ledge = chassis.get_visual("left_support_ledge")
    right_support_ledge = chassis.get_visual("right_support_ledge")
    panel_tab_left = chassis.get_visual("panel_tab_left")
    panel_tab_inner_left = chassis.get_visual("panel_tab_inner_left")
    panel_tab_right = chassis.get_visual("panel_tab_right")
    left_front_trim = left_front_burner.get_visual("trim_ring")
    left_rear_trim = left_rear_burner.get_visual("trim_ring")
    center_trim = center_wok_burner.get_visual("trim_ring")
    right_front_trim = right_front_burner.get_visual("trim_ring")
    right_rear_trim = right_rear_burner.get_visual("trim_ring")
    left_front_cap = left_front_burner.get_visual("burner_cap")
    left_rear_cap = left_rear_burner.get_visual("burner_cap")
    center_cap = center_wok_burner.get_visual("burner_cap")
    right_front_cap = right_front_burner.get_visual("burner_cap")
    right_rear_cap = right_rear_burner.get_visual("burner_cap")
    left_front_crossbar = left_grate.get_visual("front_crossbar")
    left_rear_crossbar = left_grate.get_visual("rear_crossbar")
    center_crossbar = center_grate.get_visual("center_crossbar")
    right_front_crossbar = right_grate.get_visual("front_crossbar")
    right_rear_crossbar = right_grate.get_visual("rear_crossbar")
    left_rear_shaft = left_rear_knob.get_visual("shaft")
    left_front_shaft = left_front_knob.get_visual("shaft")
    center_wok_shaft = center_wok_knob.get_visual("shaft")
    right_front_shaft = right_front_knob.get_visual("shaft")
    right_rear_shaft = right_rear_knob.get_visual("shaft")
    left_rear_body = left_rear_knob.get_visual("knob_body")
    left_front_body = left_front_knob.get_visual("knob_body")
    center_wok_body = center_wok_knob.get_visual("knob_body")
    right_front_body = right_front_knob.get_visual("knob_body")
    right_rear_body = right_rear_knob.get_visual("knob_body")
    left_rear_collar = left_rear_knob.get_visual("rear_collar")
    left_front_collar = left_front_knob.get_visual("rear_collar")
    center_wok_collar = center_wok_knob.get_visual("rear_collar")
    right_front_collar = right_front_knob.get_visual("rear_collar")
    right_rear_collar = right_rear_knob.get_visual("rear_collar")
    left_rear_indicator = left_rear_knob.get_visual("indicator")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=18, name="knob articulation clearance sweep")

    ctx.expect_contact(top_plate, chassis, elem_a=plate_skin, elem_b=front_support_ledge, name="top plate seats on front ledge")
    ctx.expect_contact(top_plate, chassis, elem_a=plate_skin, elem_b=rear_support_ledge, name="top plate seats on rear ledge")
    ctx.expect_contact(top_plate, chassis, elem_a=plate_skin, elem_b=left_support_ledge, name="top plate seats on left ledge")
    ctx.expect_contact(top_plate, chassis, elem_a=plate_skin, elem_b=right_support_ledge, name="top plate seats on right ledge")

    ctx.expect_contact(control_panel, chassis, elem_a=panel_skin, elem_b=panel_tab_left, name="control panel mounts at left tab")
    ctx.expect_contact(
        control_panel,
        chassis,
        elem_a=panel_skin,
        elem_b=panel_tab_inner_left,
        name="control panel mounts at inner tab",
    )
    ctx.expect_contact(control_panel, chassis, elem_a=panel_skin, elem_b=panel_tab_right, name="control panel mounts at right tab")

    ctx.expect_contact(
        left_front_burner,
        top_plate,
        elem_a=left_front_trim,
        elem_b=plate_skin,
        name="left front burner trim seats on cooktop plate",
    )
    ctx.expect_contact(
        left_rear_burner,
        top_plate,
        elem_a=left_rear_trim,
        elem_b=plate_skin,
        name="left rear burner trim seats on cooktop plate",
    )
    ctx.expect_contact(
        center_wok_burner,
        top_plate,
        elem_a=center_trim,
        elem_b=plate_skin,
        name="center wok burner trim seats on cooktop plate",
    )
    ctx.expect_contact(
        right_front_burner,
        top_plate,
        elem_a=right_front_trim,
        elem_b=plate_skin,
        name="right front burner trim seats on cooktop plate",
    )
    ctx.expect_contact(
        right_rear_burner,
        top_plate,
        elem_a=right_rear_trim,
        elem_b=plate_skin,
        name="right rear burner trim seats on cooktop plate",
    )

    ctx.expect_contact(left_grate, top_plate, elem_b=plate_skin, name="left grate feet contact cooktop plate")
    ctx.expect_contact(center_grate, top_plate, elem_b=plate_skin, name="center grate feet contact cooktop plate")
    ctx.expect_contact(right_grate, top_plate, elem_b=plate_skin, name="right grate feet contact cooktop plate")

    ctx.expect_gap(
        left_grate,
        left_front_burner,
        axis="z",
        min_gap=0.002,
        positive_elem=left_front_crossbar,
        negative_elem=left_front_cap,
        name="left front grate clears burner cap",
    )
    ctx.expect_gap(
        left_grate,
        left_rear_burner,
        axis="z",
        min_gap=0.002,
        positive_elem=left_rear_crossbar,
        negative_elem=left_rear_cap,
        name="left rear grate clears burner cap",
    )
    ctx.expect_gap(
        center_grate,
        center_wok_burner,
        axis="z",
        min_gap=0.002,
        positive_elem=center_crossbar,
        negative_elem=center_cap,
        name="center grate clears wok burner cap",
    )
    ctx.expect_gap(
        right_grate,
        right_front_burner,
        axis="z",
        min_gap=0.002,
        positive_elem=right_front_crossbar,
        negative_elem=right_front_cap,
        name="right front grate clears burner cap",
    )
    ctx.expect_gap(
        right_grate,
        right_rear_burner,
        axis="z",
        min_gap=0.002,
        positive_elem=right_rear_crossbar,
        negative_elem=right_rear_cap,
        name="right rear grate clears burner cap",
    )

    knob_checks = (
        ("left rear", left_rear_knob, left_rear_shaft, left_rear_collar, left_rear_body),
        ("left front", left_front_knob, left_front_shaft, left_front_collar, left_front_body),
        ("center wok", center_wok_knob, center_wok_shaft, center_wok_collar, center_wok_body),
        ("right front", right_front_knob, right_front_shaft, right_front_collar, right_front_body),
        ("right rear", right_rear_knob, right_rear_shaft, right_rear_collar, right_rear_body),
    )
    for label, knob_part, shaft, collar, body in knob_checks:
        ctx.expect_overlap(
            knob_part,
            control_panel,
            axes="xz",
            elem_a=shaft,
            elem_b=panel_skin,
            min_overlap=0.008,
            name=f"{label} knob shaft aligns with control panel bore",
        )
        ctx.expect_contact(
            knob_part,
            control_panel,
            elem_a=collar,
            elem_b=panel_skin,
            name=f"{label} knob collar captures control panel face",
        )
        ctx.expect_gap(
            control_panel,
            knob_part,
            axis="y",
            min_gap=0.001,
            max_gap=0.004,
            positive_elem=panel_skin,
            negative_elem=body,
            name=f"{label} knob body sits just proud of the panel",
        )

    ctx.expect_origin_distance(center_wok_burner, center_wok_knob, axes="x", max_dist=0.001, name="center knob aligns to wok burner axis")

    burner_positions = {
        "left_front": ctx.part_world_position(left_front_burner),
        "left_rear": ctx.part_world_position(left_rear_burner),
        "center_wok": ctx.part_world_position(center_wok_burner),
        "right_front": ctx.part_world_position(right_front_burner),
        "right_rear": ctx.part_world_position(right_rear_burner),
    }
    knob_positions = {
        "left_rear": ctx.part_world_position(left_rear_knob),
        "left_front": ctx.part_world_position(left_front_knob),
        "center_wok": ctx.part_world_position(center_wok_knob),
        "right_front": ctx.part_world_position(right_front_knob),
        "right_rear": ctx.part_world_position(right_rear_knob),
    }

    burner_layout_ok = (
        burner_positions["left_front"] is not None
        and burner_positions["left_rear"] is not None
        and burner_positions["center_wok"] is not None
        and burner_positions["right_front"] is not None
        and burner_positions["right_rear"] is not None
        and abs(burner_positions["center_wok"][0]) < 0.005
        and burner_positions["left_rear"][1] > burner_positions["left_front"][1] + 0.18
        and burner_positions["right_rear"][1] > burner_positions["right_front"][1] + 0.18
        and burner_positions["left_front"][0] < burner_positions["center_wok"][0] - 0.18
        and burner_positions["right_front"][0] > burner_positions["center_wok"][0] + 0.18
    )
    ctx.check("burner layout follows premium five-zone plan", burner_layout_ok, details=str(burner_positions))

    knob_row_ok = (
        knob_positions["left_rear"] is not None
        and knob_positions["left_front"] is not None
        and knob_positions["center_wok"] is not None
        and knob_positions["right_front"] is not None
        and knob_positions["right_rear"] is not None
        and knob_positions["left_rear"][0] < knob_positions["left_front"][0] < knob_positions["center_wok"][0]
        < knob_positions["right_front"][0] < knob_positions["right_rear"][0]
        and max(abs(position[1] - PANEL_CENTER_Y) for position in knob_positions.values()) < 1e-6
    )
    ctx.check("front knob row stays ordered and centered on the control fascia", knob_row_ok, details=str(knob_positions))

    def _center_x(aabb):
        if aabb is None:
            return None
        return (aabb[0][0] + aabb[1][0]) * 0.5

    rest_indicator_x = _center_x(ctx.part_element_world_aabb(left_rear_knob, elem=left_rear_indicator))
    with ctx.pose({left_rear_knob_joint: 1.2}):
        turned_indicator_x = _center_x(ctx.part_element_world_aabb(left_rear_knob, elem=left_rear_indicator))
    indicator_motion_ok = (
        rest_indicator_x is not None
        and turned_indicator_x is not None
        and turned_indicator_x > rest_indicator_x + 0.007
    )
    ctx.check(
        "left rear knob indicator swings around the shaft axis",
        indicator_motion_ok,
        details=f"rest_x={rest_indicator_x}, turned_x={turned_indicator_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
