from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import tempfile
from pathlib import Path

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


def _stable_asset_root() -> Path:
    spec = globals().get("__spec__")
    origin = getattr(spec, "origin", None)
    for candidate in (origin, globals().get("__file__")):
        if isinstance(candidate, str) and os.path.isabs(candidate):
            return Path(candidate).parent
    return Path(tempfile.gettempdir())


HERE = _stable_asset_root()
ASSETS = AssetContext(HERE)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="film_scanner_flatbed", assets=ASSETS)

    body_light = model.material("body_light", rgba=(0.88, 0.89, 0.90, 1.0))
    body_dark = model.material("body_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    charcoal = model.material("charcoal", rgba=(0.11, 0.12, 0.13, 1.0))
    glass = model.material("glass", rgba=(0.60, 0.76, 0.84, 0.42))
    frosted = model.material("frosted", rgba=(0.88, 0.91, 0.93, 0.72))
    hinge_dark = model.material("hinge_dark", rgba=(0.36, 0.38, 0.41, 1.0))
    accent = model.material("accent", rgba=(0.72, 0.74, 0.76, 1.0))

    body_w = 0.36
    body_d = 0.27
    body_h = 0.048
    rail_h = 0.010
    top_z = body_h + rail_h
    glass_w = 0.255
    glass_d = 0.175

    lid_w = 0.344
    lid_d = 0.248
    lid_t = 0.013
    lid_joint_y = 0.126
    lid_joint_z = 0.0665

    base = model.part("base")
    base.visual(
        Box((body_w, body_d, body_h)),
        origin=Origin(xyz=(0.0, 0.0, body_h / 2.0)),
        material=body_light,
        name="lower_shell",
    )
    base.visual(
        Box((body_w, 0.042, rail_h)),
        origin=Origin(xyz=(0.0, -(glass_d / 2.0 + 0.042 / 2.0), body_h + rail_h / 2.0)),
        material=body_light,
        name="front_rail",
    )
    base.visual(
        Box((body_w, 0.048, rail_h)),
        origin=Origin(xyz=(0.0, glass_d / 2.0 + 0.048 / 2.0, body_h + rail_h / 2.0)),
        material=body_light,
        name="rear_rail",
    )
    side_rail_w = (body_w - glass_w) / 2.0
    base.visual(
        Box((side_rail_w, glass_d, rail_h)),
        origin=Origin(xyz=(-(glass_w / 2.0 + side_rail_w / 2.0), 0.0, body_h + rail_h / 2.0)),
        material=body_light,
        name="left_rail",
    )
    base.visual(
        Box((side_rail_w, glass_d, rail_h)),
        origin=Origin(xyz=(glass_w / 2.0 + side_rail_w / 2.0, 0.0, body_h + rail_h / 2.0)),
        material=body_light,
        name="right_rail",
    )
    base.visual(
        Box((glass_w, glass_d, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, top_z - 0.003)),
        material=glass,
        name="platen_glass",
    )
    base.visual(
        Box((0.328, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -(body_d / 2.0 + 0.006), 0.023)),
        material=body_dark,
        name="front_panel",
    )

    port_x = 0.078
    port_cavity_size = (0.058, 0.018, 0.022)
    for side_name, x in (("left", -port_x), ("right", port_x)):
        base.visual(
            Box(port_cavity_size),
            origin=Origin(xyz=(x, -0.126, 0.022)),
            material=charcoal,
            name=f"{side_name}_port_cavity",
        )

    hinge_y = lid_joint_y
    hinge_z = lid_joint_z
    hinge_radius = 0.0065
    hinge_len_outer = 0.009
    hinge_len_inner = 0.009
    base_hinge_centers = {
        "left": (-0.156, -0.138),
        "right": (0.138, 0.156),
    }
    for side_name, xs in base_hinge_centers.items():
        for idx, x in enumerate(xs, start=1):
            base.visual(
                Box((0.009, 0.012, 0.012)),
                origin=Origin(xyz=(x, hinge_y + 0.006, hinge_z - 0.0075)),
                material=body_dark,
                name=f"{side_name}_hinge_riser_{idx}",
            )
            base.visual(
                Cylinder(radius=hinge_radius, length=hinge_len_outer if idx == 1 else hinge_len_inner),
                origin=Origin(
                    xyz=(x, hinge_y, hinge_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hinge_dark,
                name=f"{side_name}_base_knuckle_{idx}",
            )

    door_leaf_w = 0.004
    for side_name, x in (("left", -0.112), ("right", 0.112)):
        base.visual(
            Box((door_leaf_w, 0.004, 0.030)),
            origin=Origin(xyz=(x, -0.1485, 0.022)),
            material=accent,
            name=f"{side_name}_base_leaf",
        )
        base.visual(
            Cylinder(radius=0.0026, length=0.010),
            origin=Origin(xyz=(x, -0.1475, 0.014)),
            material=hinge_dark,
            name=f"{side_name}_base_knuckle_lower",
        )
        base.visual(
            Cylinder(radius=0.0026, length=0.010),
            origin=Origin(xyz=(x, -0.1475, 0.030)),
            material=hinge_dark,
            name=f"{side_name}_base_knuckle_upper",
        )

    base.inertial = Inertial.from_geometry(
        Box((body_w, body_d, top_z)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, top_z / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, -(lid_d / 2.0 + 0.010), -0.002)),
        material=body_light,
        name="outer_shell",
    )
    lid.visual(
        Box((0.286, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, -0.236, -0.0065)),
        material=body_dark,
        name="front_pad",
    )
    lid.visual(
        Box((0.282, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.040, -0.0065)),
        material=body_dark,
        name="rear_liner",
    )
    lid.visual(
        Box((0.020, 0.190, 0.004)),
        origin=Origin(xyz=(-0.134, -0.132, -0.0065)),
        material=body_dark,
        name="left_liner",
    )
    lid.visual(
        Box((0.020, 0.190, 0.004)),
        origin=Origin(xyz=(0.134, -0.132, -0.0065)),
        material=body_dark,
        name="right_liner",
    )
    lid.visual(
        Box((0.286, 0.210, 0.002)),
        origin=Origin(xyz=(0.0, -0.132, -0.0085)),
        material=accent,
        name="adapter_seat",
    )

    for side_name, x in (("left", -0.147), ("right", 0.147)):
        lid.visual(
            Box((0.006, 0.006, 0.012)),
            origin=Origin(xyz=(x, -0.009, -0.003)),
            material=body_dark,
            name=f"{side_name}_hinge_arm",
        )
    lid_knuckles = {"left": (-0.147,), "right": (0.147,)}
    for side_name, xs in lid_knuckles.items():
        for idx, x in enumerate(xs, start=1):
            lid.visual(
                Cylinder(radius=0.006, length=0.008),
                origin=Origin(
                    xyz=(x, 0.0, 0.0),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hinge_dark,
                name=f"{side_name}_lid_knuckle_{idx}",
            )

    lid_snap_y = -0.080
    for side_name, x in (("left", -0.144), ("right", 0.144)):
        lid.visual(
            Box((0.007, 0.024, 0.008)),
            origin=Origin(xyz=(x, lid_snap_y, -0.004)),
            material=accent,
            name=f"{side_name}_snap_bracket",
        )
        lid.visual(
            Box((0.011, 0.016, 0.002)),
            origin=Origin(xyz=(x + (0.0045 if side_name == "left" else -0.0045), lid_snap_y, -0.0015)),
            material=accent,
            name=f"{side_name}_snap_lip",
        )

    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, 0.018)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -(lid_d / 2.0 + 0.010), -0.002)),
    )

    adapter_panel = model.part("adapter_panel")
    panel_tab_y = 0.052
    adapter_panel.visual(
        Box((0.266, 0.196, 0.003)),
        origin=Origin(),
        material=frosted,
        name="panel_sheet",
    )
    adapter_panel.visual(
        Box((0.008, 0.022, 0.006)),
        origin=Origin(xyz=(-0.137, panel_tab_y, 0.0015)),
        material=accent,
        name="left_tab",
    )
    adapter_panel.visual(
        Box((0.008, 0.022, 0.006)),
        origin=Origin(xyz=(0.137, panel_tab_y, 0.0015)),
        material=accent,
        name="right_tab",
    )
    adapter_panel.visual(
        Box((0.230, 0.160, 0.0015)),
        origin=Origin(xyz=(0.0, -0.008, 0.00225)),
        material=glass,
        name="light_window",
    )
    adapter_panel.inertial = Inertial.from_geometry(
        Box((0.266, 0.196, 0.006)),
        mass=0.25,
        origin=Origin(),
    )

    left_door = model.part("left_port_door")
    left_door.visual(
        Box((0.060, 0.004, 0.030)),
        origin=Origin(xyz=(0.031, 0.0015, 0.0)),
        material=body_dark,
        name="door_panel",
    )
    left_door.visual(
        Box((0.012, 0.003, 0.010)),
        origin=Origin(xyz=(0.048, -0.0015, -0.004)),
        material=accent,
        name="finger_pull",
    )
    left_door.visual(
        Box((0.004, 0.004, 0.030)),
        origin=Origin(xyz=(0.002, -0.002, 0.0)),
        material=accent,
        name="hinge_leaf",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((0.060, 0.008, 0.030)),
        mass=0.08,
        origin=Origin(xyz=(0.031, 0.0015, 0.0)),
    )

    right_door = model.part("right_port_door")
    right_door.visual(
        Box((0.060, 0.004, 0.030)),
        origin=Origin(xyz=(-0.031, 0.0015, 0.0)),
        material=body_dark,
        name="door_panel",
    )
    right_door.visual(
        Box((0.012, 0.003, 0.010)),
        origin=Origin(xyz=(-0.048, -0.0015, -0.004)),
        material=accent,
        name="finger_pull",
    )
    right_door.visual(
        Box((0.004, 0.004, 0.030)),
        origin=Origin(xyz=(-0.002, -0.002, 0.0)),
        material=accent,
        name="hinge_leaf",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((0.060, 0.008, 0.030)),
        mass=0.08,
        origin=Origin(xyz=(-0.031, 0.0015, 0.0)),
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, lid_joint_y, lid_joint_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-1.35, upper=0.0),
    )
    model.articulation(
        "adapter_mount",
        ArticulationType.FIXED,
        parent=lid,
        child=adapter_panel,
        origin=Origin(xyz=(0.0, -0.132, -0.007)),
    )
    left_door_hinge = model.articulation(
        "left_port_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_door,
        origin=Origin(xyz=(-0.112, -0.1505, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.5, lower=-1.25, upper=0.0),
    )
    right_door_hinge = model.articulation(
        "right_port_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_door,
        origin=Origin(xyz=(0.112, -0.1505, 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    adapter_panel = object_model.get_part("adapter_panel")
    left_door = object_model.get_part("left_port_door")
    right_door = object_model.get_part("right_port_door")

    lid_hinge = object_model.get_articulation("lid_hinge")
    left_port_hinge = object_model.get_articulation("left_port_hinge")
    right_port_hinge = object_model.get_articulation("right_port_hinge")

    platen_glass = base.get_visual("platen_glass")
    front_rail = base.get_visual("front_rail")
    front_panel = base.get_visual("front_panel")
    left_port_cavity = base.get_visual("left_port_cavity")
    right_port_cavity = base.get_visual("right_port_cavity")
    left_base_leaf = base.get_visual("left_base_leaf")
    right_base_leaf = base.get_visual("right_base_leaf")
    left_base_knuckle_lower = base.get_visual("left_base_knuckle_lower")
    right_base_knuckle_lower = base.get_visual("right_base_knuckle_lower")
    left_base_knuckle_upper = base.get_visual("left_base_knuckle_upper")
    right_base_knuckle_upper = base.get_visual("right_base_knuckle_upper")
    left_lid_knuckle = lid.get_visual("left_lid_knuckle_1")
    right_lid_knuckle = lid.get_visual("right_lid_knuckle_1")
    left_base_knuckle = base.get_visual("left_base_knuckle_1")
    right_base_knuckle = base.get_visual("right_base_knuckle_1")
    front_pad = lid.get_visual("front_pad")
    adapter_sheet = adapter_panel.get_visual("panel_sheet")
    left_tab = adapter_panel.get_visual("left_tab")
    right_tab = adapter_panel.get_visual("right_tab")
    left_snap_bracket = lid.get_visual("left_snap_bracket")
    right_snap_bracket = lid.get_visual("right_snap_bracket")
    left_snap_lip = lid.get_visual("left_snap_lip")
    right_snap_lip = lid.get_visual("right_snap_lip")
    left_door_panel = left_door.get_visual("door_panel")
    right_door_panel = right_door.get_visual("door_panel")
    left_door_leaf = left_door.get_visual("hinge_leaf")
    right_door_leaf = right_door.get_visual("hinge_leaf")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(lid, base, axes="xy", min_overlap=0.08)
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        max_gap=0.0025,
        max_penetration=0.0,
        positive_elem=front_pad,
        negative_elem=front_rail,
        name="lid_front_edge_seats_on_base",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="yz",
        min_overlap=0.00008,
        elem_a=left_lid_knuckle,
        elem_b=left_base_knuckle,
        name="left_rear_knuckle_hinge_present",
    )
    ctx.expect_overlap(
        lid,
        base,
        axes="yz",
        min_overlap=0.00008,
        elem_a=right_lid_knuckle,
        elem_b=right_base_knuckle,
        name="right_rear_knuckle_hinge_present",
    )

    ctx.expect_within(adapter_panel, lid, axes="xy", inner_elem=adapter_sheet, outer_elem=lid.get_visual("adapter_seat"))
    ctx.expect_overlap(adapter_panel, lid, axes="yz", min_overlap=0.005, elem_a=left_tab, elem_b=left_snap_bracket)
    ctx.expect_overlap(adapter_panel, lid, axes="yz", min_overlap=0.005, elem_a=right_tab, elem_b=right_snap_bracket)
    ctx.expect_contact(adapter_panel, lid, elem_a=left_tab, elem_b=left_snap_lip)
    ctx.expect_contact(adapter_panel, lid, elem_a=right_tab, elem_b=right_snap_lip)

    ctx.expect_overlap(left_door, base, axes="xz", min_overlap=0.001, elem_a=left_door_panel, elem_b=left_port_cavity)
    ctx.expect_overlap(right_door, base, axes="xz", min_overlap=0.001, elem_a=right_door_panel, elem_b=right_port_cavity)
    ctx.expect_gap(
        base,
        left_door,
        axis="y",
        max_gap=0.0025,
        max_penetration=0.0001,
        positive_elem=front_panel,
        negative_elem=left_door_panel,
        name="left_port_door_sits_flush_when_closed",
    )
    ctx.expect_gap(
        base,
        right_door,
        axis="y",
        max_gap=0.0025,
        max_penetration=0.0001,
        positive_elem=front_panel,
        negative_elem=right_door_panel,
        name="right_port_door_sits_flush_when_closed",
    )
    ctx.expect_contact(left_door, base, elem_a=left_door_leaf, elem_b=left_base_leaf)
    ctx.expect_contact(right_door, base, elem_a=right_door_leaf, elem_b=right_base_leaf)
    ctx.expect_overlap(left_door, base, axes="xz", min_overlap=0.00004, elem_a=left_door_leaf, elem_b=left_base_knuckle_lower)
    ctx.expect_overlap(left_door, base, axes="xz", min_overlap=0.00004, elem_a=left_door_leaf, elem_b=left_base_knuckle_upper)
    ctx.expect_overlap(right_door, base, axes="xz", min_overlap=0.00004, elem_a=right_door_leaf, elem_b=right_base_knuckle_lower)
    ctx.expect_overlap(right_door, base, axes="xz", min_overlap=0.00004, elem_a=right_door_leaf, elem_b=right_base_knuckle_upper)

    with ctx.pose({lid_hinge: -1.2}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            min_gap=0.09,
            positive_elem=front_pad,
            negative_elem=platen_glass,
            name="lid_opens_clear_of_platen",
        )
        ctx.expect_within(adapter_panel, lid, axes="xy", inner_elem=adapter_sheet, outer_elem=lid.get_visual("adapter_seat"))

    with ctx.pose({left_port_hinge: -1.1, right_port_hinge: 1.1}):
        ctx.expect_gap(
            base,
            left_door,
            axis="y",
            min_gap=0.018,
            positive_elem=front_panel,
            negative_elem=left_door.get_visual("finger_pull"),
            name="left_port_door_swings_outward",
        )
        ctx.expect_gap(
            base,
            right_door,
            axis="y",
            min_gap=0.018,
            positive_elem=front_panel,
            negative_elem=right_door.get_visual("finger_pull"),
            name="right_port_door_swings_outward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
