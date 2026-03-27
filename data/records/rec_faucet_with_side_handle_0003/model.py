from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

_ORIGINAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIGINAL_GETCWD()
    except FileNotFoundError:
        os.chdir("/tmp")
        return _ORIGINAL_GETCWD()


os.getcwd = _safe_getcwd
os.chdir(_safe_getcwd())

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

_SPEC = globals().get("__spec__")
_LOADER = globals().get("__loader__")
_MODULE_ORIGIN = getattr(_SPEC, "origin", None)
if not _MODULE_ORIGIN and _LOADER and hasattr(_LOADER, "get_filename"):
    try:
        _MODULE_ORIGIN = _LOADER.get_filename(__name__)
    except Exception:
        _MODULE_ORIGIN = None
ASSET_ROOT = (
    os.path.dirname(_MODULE_ORIGIN)
    if isinstance(_MODULE_ORIGIN, str) and os.path.isabs(_MODULE_ORIGIN)
    else "/tmp"
)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_wall_faucet")

    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.38, 0.40, 0.43, 1.0))
    black = model.material("black", rgba=(0.16, 0.17, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.012, 0.240, 0.170)),
        origin=Origin(xyz=(0.006, 0.0, 0.020)),
        material=steel,
        name="wall_plate",
    )
    body.visual(
        Box((0.040, 0.112, 0.056)),
        origin=Origin(xyz=(0.024, 0.0, 0.016)),
        material=dark_steel,
        name="valve_block",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.044),
        origin=Origin(xyz=(0.029, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="valve_body",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.059, 0.0, 0.062), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="mixing_boss",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.059, -0.018, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="spout_left_trunnion",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.059, 0.018, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="spout_right_trunnion",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(0.074, -0.012, 0.062), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lever_left_ear",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.010),
        origin=Origin(xyz=(0.074, 0.012, 0.062), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lever_right_ear",
    )

    for side_name, y_sign in (("left", -1.0), ("right", 1.0)):
        y_pos = y_sign * 0.062
        body.visual(
            Cylinder(radius=0.010, length=0.066),
            origin=Origin(xyz=(-0.022, y_pos, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"{side_name}_supply_pipe",
        )
        body.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(0.010, y_pos, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"{side_name}_supply_shoulder",
        )
        for thread_index, x_pos in enumerate((-0.044, -0.036, -0.028, -0.020), start=1):
            body.visual(
                Cylinder(radius=0.0116, length=0.003),
                origin=Origin(xyz=(x_pos, y_pos, -0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=steel,
                name=f"{side_name}_thread_{thread_index}",
            )

    body.inertial = Inertial.from_geometry(
        Box((0.120, 0.240, 0.180)),
        mass=4.8,
        origin=Origin(xyz=(0.020, 0.0, 0.018)),
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.0085, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_barrel",
    )
    spout.visual(
        Cylinder(radius=0.0115, length=0.020),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pivot_hub",
    )
    spout_path = [
        (0.018, 0.0, 0.000),
        (0.066, 0.0, -0.004),
        (0.134, 0.0, -0.022),
        (0.194, 0.0, -0.066),
        (0.218, 0.0, -0.098),
    ]
    for index, (start_pt, end_pt) in enumerate(zip(spout_path[:-1], spout_path[1:]), start=1):
        spout.visual(
            Cylinder(radius=0.0105, length=_distance(start_pt, end_pt)),
            origin=Origin(
                xyz=_midpoint(start_pt, end_pt),
                rpy=_rpy_for_cylinder(start_pt, end_pt),
            ),
            material=steel,
            name=f"spout_run_{index}",
        )
    spout.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.218, 0.0, -0.108)),
        material=dark_steel,
        name="outlet_collar",
    )
    spout.visual(
        Cylinder(radius=0.009, length=0.050),
        origin=Origin(xyz=(0.218, 0.0, -0.142)),
        material=steel,
        name="outlet_tip",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.270, 0.040, 0.170)),
        mass=1.4,
        origin=Origin(xyz=(0.132, 0.0, -0.058)),
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.0058, length=0.014),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_barrel",
    )
    lever.visual(
        Box((0.014, 0.010, 0.060)),
        origin=Origin(xyz=(0.022, 0.0, 0.018), rpy=(0.0, 0.95, 0.0)),
        material=steel,
        name="lever_stem",
    )
    lever.visual(
        Box((0.040, 0.018, 0.010)),
        origin=Origin(xyz=(0.046, 0.0, 0.044), rpy=(0.0, 0.22, 0.0)),
        material=black,
        name="lever_grip",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.090, 0.030, 0.110)),
        mass=0.45,
        origin=Origin(xyz=(0.036, 0.0, 0.046)),
    )

    model.articulation(
        "body_to_spout",
        ArticulationType.REVOLUTE,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.060, 0.0, 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=-0.30,
            upper=0.55,
        ),
    )
    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.076, 0.0, 0.062)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=-0.60,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSET_ROOT)
    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("lever")
    spout_hinge = object_model.get_articulation("body_to_spout")
    lever_hinge = object_model.get_articulation("body_to_lever")

    wall_plate = body.get_visual("wall_plate")
    mixing_boss = body.get_visual("mixing_boss")
    spout_left_trunnion = body.get_visual("spout_left_trunnion")
    spout_right_trunnion = body.get_visual("spout_right_trunnion")
    lever_left_ear = body.get_visual("lever_left_ear")
    lever_right_ear = body.get_visual("lever_right_ear")
    left_supply_shoulder = body.get_visual("left_supply_shoulder")
    right_supply_shoulder = body.get_visual("right_supply_shoulder")
    valve_body = body.get_visual("valve_body")
    left_thread_1 = body.get_visual("left_thread_1")
    right_thread_1 = body.get_visual("right_thread_1")

    spout_barrel = spout.get_visual("pivot_barrel")
    spout_hub = spout.get_visual("pivot_hub")
    outlet_tip = spout.get_visual("outlet_tip")
    lever_barrel = lever.get_visual("pivot_barrel")
    lever_grip = lever.get_visual("lever_grip")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(
        spout,
        body,
        elem_a=spout_barrel,
        elem_b=spout_left_trunnion,
        name="spout_left_pivot_seated",
    )
    ctx.expect_contact(
        spout,
        body,
        elem_a=spout_barrel,
        elem_b=spout_right_trunnion,
        name="spout_right_pivot_seated",
    )
    ctx.expect_contact(
        lever,
        body,
        elem_a=lever_barrel,
        elem_b=lever_left_ear,
        name="lever_left_pivot_seated",
    )
    ctx.expect_contact(
        lever,
        body,
        elem_a=lever_barrel,
        elem_b=lever_right_ear,
        name="lever_right_pivot_seated",
    )
    ctx.expect_gap(
        spout,
        body,
        axis="x",
        max_gap=0.016,
        max_penetration=0.0,
        positive_elem=spout_hub,
        negative_elem=valve_body,
        name="spout_hub_stays_close_to_valve_body",
    )
    ctx.expect_gap(
        lever,
        body,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=lever_barrel,
        negative_elem=mixing_boss,
        name="lever_pin_sits_at_mixing_boss",
    )
    ctx.expect_gap(
        spout,
        body,
        axis="x",
        min_gap=0.170,
        positive_elem=outlet_tip,
        negative_elem=wall_plate,
        name="spout_projects_well_out_from_wall",
    )
    ctx.expect_gap(
        body,
        spout,
        axis="z",
        min_gap=0.080,
        positive_elem=mixing_boss,
        negative_elem=outlet_tip,
        name="outlet_hangs_below_valve_boss",
    )
    ctx.expect_gap(
        lever,
        body,
        axis="z",
        min_gap=0.004,
        positive_elem=lever_grip,
        negative_elem=mixing_boss,
        name="lever_grip_sits_above_boss",
    )
    ctx.expect_gap(
        body,
        body,
        axis="y",
        min_gap=0.090,
        positive_elem=right_supply_shoulder,
        negative_elem=left_supply_shoulder,
        name="dual_supply_inlets_flank_centerline",
    )
    ctx.expect_gap(
        body,
        body,
        axis="x",
        min_gap=0.040,
        positive_elem=wall_plate,
        negative_elem=left_thread_1,
        name="left_supply_threads_project_behind_wall_plate",
    )
    ctx.expect_gap(
        body,
        body,
        axis="x",
        min_gap=0.040,
        positive_elem=wall_plate,
        negative_elem=right_thread_1,
        name="right_supply_threads_project_behind_wall_plate",
    )
    ctx.expect_overlap(
        body,
        body,
        axes="xz",
        min_overlap=0.015,
        elem_a=left_supply_shoulder,
        elem_b=right_supply_shoulder,
        name="supply_inlets_share_common_height_and_depth",
    )

    with ctx.pose({spout_hinge: 0.45}):
        ctx.expect_contact(
            spout,
            body,
            elem_a=spout_barrel,
            elem_b=spout_left_trunnion,
            name="raised_spout_left_pivot_stays_seated",
        )
        ctx.expect_contact(
            spout,
            body,
            elem_a=spout_barrel,
            elem_b=spout_right_trunnion,
            name="raised_spout_right_pivot_stays_seated",
        )
        ctx.expect_gap(
            spout,
            body,
            axis="x",
            min_gap=0.135,
            positive_elem=outlet_tip,
            negative_elem=wall_plate,
            name="raised_spout_still_projects_forward",
        )

    with ctx.pose({lever_hinge: -0.40}):
        ctx.expect_contact(
            lever,
            body,
            elem_a=lever_barrel,
            elem_b=lever_left_ear,
            name="opened_lever_left_pivot_stays_seated",
        )
        ctx.expect_contact(
            lever,
            body,
            elem_a=lever_barrel,
            elem_b=lever_right_ear,
            name="opened_lever_right_pivot_stays_seated",
        )
        ctx.expect_gap(
            lever,
            body,
            axis="x",
            min_gap=0.030,
            positive_elem=lever_grip,
            negative_elem=wall_plate,
            name="opened_lever_pulls_forward_from_wall_plate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
