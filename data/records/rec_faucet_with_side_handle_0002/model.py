from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except OSError:
            pass
        return "/"


os.getcwd = _safe_getcwd

_REAL_PATH_ABSOLUTE = pathlib.Path.absolute
_REAL_PATH_RESOLVE = pathlib.Path.resolve
_REAL_PATH_CWD = pathlib.Path.cwd


def _safe_path_absolute(self):
    try:
        return _REAL_PATH_ABSOLUTE(self)
    except FileNotFoundError:
        if self.is_absolute():
            return self
        return pathlib.Path("/").joinpath(self)


def _safe_path_resolve(self, strict=False):
    try:
        return _REAL_PATH_RESOLVE(self, strict=strict)
    except FileNotFoundError:
        fallback = _safe_path_absolute(self)
        if strict and not fallback.exists():
            raise
        return fallback


@classmethod
def _safe_path_cwd(cls):
    try:
        return _REAL_PATH_CWD()
    except FileNotFoundError:
        return cls("/")


pathlib.Path.absolute = _safe_path_absolute
pathlib.Path.resolve = _safe_path_resolve
pathlib.Path.cwd = _safe_path_cwd

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

VALVE_CENTER_X = 0.103
BODY_RADIUS = 0.028
BODY_HEIGHT = 0.092
BODY_BASE_Z = -0.062
BODY_CENTER_Z = BODY_BASE_Z + BODY_HEIGHT / 2.0
BODY_TOP_Z = BODY_BASE_Z + BODY_HEIGHT
BRIDGE_RADIUS = 0.0115
BRIDGE_LENGTH = 0.152
BRIDGE_SHANK_RADIUS = 0.0105
BRIDGE_SHANK_LENGTH = VALVE_CENTER_X - BRIDGE_LENGTH / 2.0
SOCKET_RADIUS = 0.014
SOCKET_LENGTH = 0.022
SOCKET_CENTER_OFFSET = 0.016
FITTING_RADIUS = 0.016
FITTING_LENGTH = 0.014
FITTING_CENTER_Z = BODY_TOP_Z + FITTING_LENGTH / 2.0
HANDLE_JOINT_Z = BODY_TOP_Z + FITTING_LENGTH
STEM_RADIUS = 0.0075
STEM_LENGTH = 0.024
STEM_CENTER_Z = HANDLE_JOINT_Z + STEM_LENGTH / 2.0


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _yz_segment_origin(
    y0: float,
    z0: float,
    y1: float,
    z1: float,
) -> tuple[Origin, float]:
    dy = y1 - y0
    dz = z1 - z0
    length = math.hypot(dy, dz)
    pitch = -math.atan2(dy, dz)
    return Origin(xyz=(0.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0), rpy=(pitch, 0.0, 0.0)), length


def _add_valve_body(part, inward_sign: float, material) -> None:
    part.visual(
        Cylinder(radius=0.035, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, BODY_BASE_Z + 0.004)),
        material=material,
        name="escutcheon",
    )
    part.visual(
        Cylinder(radius=BODY_RADIUS, length=BODY_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BODY_CENTER_Z)),
        material=material,
        name="barrel",
    )
    part.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z - 0.005)),
        material=material,
        name="upper_collar",
    )
    part.visual(
        Cylinder(radius=SOCKET_RADIUS, length=SOCKET_LENGTH),
        origin=Origin(
            xyz=(inward_sign * SOCKET_CENTER_OFFSET, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name="bridge_socket",
    )
    part.visual(
        Cylinder(radius=FITTING_RADIUS, length=FITTING_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, FITTING_CENTER_Z)),
        material=material,
        name="stem_fitting",
    )
    part.visual(
        Cylinder(radius=STEM_RADIUS, length=STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, STEM_CENTER_Z)),
        material=material,
        name="stem",
    )


def _add_cross_handle(part, metal, accent) -> None:
    part.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=metal,
        name="hub",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.015), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="arm_x",
    )
    part.visual(
        Cylinder(radius=0.006, length=0.074),
        origin=Origin(xyz=(0.0, 0.0, 0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="arm_y",
    )
    for name, xyz in (
        ("grip_x_pos", (0.037, 0.0, 0.018)),
        ("grip_x_neg", (-0.037, 0.0, 0.018)),
        ("grip_y_pos", (0.0, 0.037, 0.018)),
        ("grip_y_neg", (0.0, -0.037, 0.018)),
    ):
        part.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(xyz=xyz),
            material=metal,
            name=name,
        )
    part.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=accent,
        name="index_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_style_faucet", assets=ASSETS)

    chrome = model.material("polished_chrome", rgba=(0.79, 0.81, 0.84, 1.0))
    porcelain = model.material("porcelain_cap", rgba=(0.95, 0.95, 0.93, 1.0))

    bridge = model.part("bridge")
    bridge.visual(
        Cylinder(radius=BRIDGE_RADIUS, length=BRIDGE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="bridge_tube",
    )
    bridge.visual(
        Cylinder(radius=BRIDGE_SHANK_RADIUS, length=BRIDGE_SHANK_LENGTH),
        origin=Origin(
            xyz=(-(BRIDGE_LENGTH + BRIDGE_SHANK_LENGTH) / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="left_bridge_shank",
    )
    bridge.visual(
        Cylinder(radius=BRIDGE_SHANK_RADIUS, length=BRIDGE_SHANK_LENGTH),
        origin=Origin(
            xyz=((BRIDGE_LENGTH + BRIDGE_SHANK_LENGTH) / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=chrome,
        name="right_bridge_shank",
    )
    bridge.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=chrome,
        name="spout_mount",
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.0155, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=chrome,
        name="spout_base",
    )
    spout.visual(
        _save_mesh(
            "bridge_faucet_spout_gooseneck.obj",
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.024),
                    (0.0, 0.0, 0.072),
                    (0.0, 0.018, 0.118),
                    (0.0, 0.066, 0.138),
                    (0.0, 0.109, 0.112),
                    (0.0, 0.128, 0.046),
                ],
                radius=0.0108,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
                up_hint=(1.0, 0.0, 0.0),
            ),
        ),
        material=chrome,
        name="spout_gooseneck",
    )
    spout.visual(
        Cylinder(radius=0.0090, length=0.070),
        origin=Origin(xyz=(0.0, 0.128, 0.039)),
        material=chrome,
        name="spout_drop",
    )
    spout.visual(
        Cylinder(radius=0.0102, length=0.010),
        origin=Origin(xyz=(0.0, 0.128, 0.004)),
        material=chrome,
        name="outlet_tip",
    )
    spout.visual(
        Cylinder(radius=0.0085, length=0.014),
        origin=Origin(xyz=(0.0, 0.128, 0.013)),
        material=chrome,
        name="aerator_tip",
    )

    left_valve = model.part("left_valve_body")
    right_valve = model.part("right_valve_body")
    _add_valve_body(left_valve, inward_sign=1.0, material=chrome)
    _add_valve_body(right_valve, inward_sign=-1.0, material=chrome)

    left_handle = model.part("left_handle")
    right_handle = model.part("right_handle")
    _add_cross_handle(left_handle, metal=chrome, accent=porcelain)
    _add_cross_handle(right_handle, metal=chrome, accent=porcelain)

    model.articulation(
        "bridge_to_spout",
        ArticulationType.FIXED,
        parent=bridge,
        child=spout,
        origin=Origin(),
    )
    model.articulation(
        "bridge_to_left_valve",
        ArticulationType.FIXED,
        parent=bridge,
        child=left_valve,
        origin=Origin(xyz=(-VALVE_CENTER_X, 0.0, 0.0)),
    )
    model.articulation(
        "bridge_to_right_valve",
        ArticulationType.FIXED,
        parent=bridge,
        child=right_valve,
        origin=Origin(xyz=(VALVE_CENTER_X, 0.0, 0.0)),
    )
    model.articulation(
        "left_handle_turn",
        ArticulationType.REVOLUTE,
        parent=left_valve,
        child=left_handle,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "right_handle_turn",
        ArticulationType.REVOLUTE,
        parent=right_valve,
        child=right_handle,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.2,
            upper=1.2,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    spout = object_model.get_part("spout")
    left_valve = object_model.get_part("left_valve_body")
    right_valve = object_model.get_part("right_valve_body")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    left_turn = object_model.get_articulation("left_handle_turn")
    right_turn = object_model.get_articulation("right_handle_turn")

    bridge_tube = bridge.get_visual("bridge_tube")
    left_bridge_shank = bridge.get_visual("left_bridge_shank")
    right_bridge_shank = bridge.get_visual("right_bridge_shank")
    spout_mount = bridge.get_visual("spout_mount")
    spout_base = spout.get_visual("spout_base")
    spout_tip = spout.get_visual("outlet_tip")
    left_socket = left_valve.get_visual("bridge_socket")
    right_socket = right_valve.get_visual("bridge_socket")
    left_fitting = left_valve.get_visual("stem_fitting")
    right_fitting = right_valve.get_visual("stem_fitting")
    left_stem = left_valve.get_visual("stem")
    right_stem = right_valve.get_visual("stem")
    left_hub = left_handle.get_visual("hub")
    right_hub = right_handle.get_visual("hub")

    ctx.allow_overlap(
        left_handle,
        left_valve,
        reason="left cross-handle hub clamps around the protruding valve stem",
    )
    ctx.allow_overlap(
        right_handle,
        right_valve,
        reason="right cross-handle hub clamps around the protruding valve stem",
    )

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

    ctx.expect_contact(bridge, left_valve, elem_a=bridge_tube, elem_b=left_socket)
    ctx.expect_contact(bridge, right_valve, elem_a=bridge_tube, elem_b=right_socket)
    ctx.expect_overlap(bridge, left_valve, axes="xz", min_overlap=0.010, elem_a=left_bridge_shank, elem_b=left_socket)
    ctx.expect_overlap(bridge, right_valve, axes="xz", min_overlap=0.010, elem_a=right_bridge_shank, elem_b=right_socket)
    ctx.expect_gap(
        spout,
        bridge,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=spout_base,
        negative_elem=spout_mount,
    )
    ctx.expect_origin_distance(spout, bridge, axes="xy", max_dist=0.001)
    ctx.expect_gap(right_valve, left_valve, axis="x", min_gap=0.130)
    ctx.expect_gap(spout, bridge, axis="y", min_gap=0.040, positive_elem=spout_tip, negative_elem=bridge_tube)

    ctx.expect_origin_distance(left_handle, left_valve, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(right_handle, right_valve, axes="xy", max_dist=0.001)
    ctx.expect_within(left_valve, left_handle, axes="xy", inner_elem=left_stem, outer_elem=left_hub)
    ctx.expect_within(right_valve, right_handle, axes="xy", inner_elem=right_stem, outer_elem=right_hub)
    ctx.expect_gap(
        left_handle,
        left_valve,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_hub,
        negative_elem=left_fitting,
    )
    ctx.expect_gap(
        right_handle,
        right_valve,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_hub,
        negative_elem=right_fitting,
    )
    ctx.expect_gap(left_handle, bridge, axis="z", min_gap=0.020)
    ctx.expect_gap(right_handle, bridge, axis="z", min_gap=0.020)
    ctx.expect_gap(spout, left_handle, axis="x", min_gap=0.030)
    ctx.expect_gap(right_handle, spout, axis="x", min_gap=0.030)

    with ctx.pose({left_turn: 0.75, right_turn: -0.75}):
        ctx.expect_within(left_valve, left_handle, axes="xy", inner_elem=left_stem, outer_elem=left_hub)
        ctx.expect_within(right_valve, right_handle, axes="xy", inner_elem=right_stem, outer_elem=right_hub)
        ctx.expect_gap(
            left_handle,
            left_valve,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=left_hub,
            negative_elem=left_fitting,
        )
        ctx.expect_gap(
            right_handle,
            right_valve,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=right_hub,
            negative_elem=right_fitting,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
