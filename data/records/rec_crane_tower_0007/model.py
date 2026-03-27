from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

# The compile harness can keep a module path that later gets normalized through
# pathlib, even after its original working directory has disappeared. Capture a
# stable cwd while it exists and provide a fallback getcwd so later stdlib path
# normalization cannot fail with FileNotFoundError.
_REAL_GETCWD = os.getcwd
try:
    _STABLE_CWD = _REAL_GETCWD()
except FileNotFoundError:
    _STABLE_CWD = "/"


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        return _STABLE_CWD


os.getcwd = _safe_getcwd
pathlib.os.getcwd = _safe_getcwd

_REAL_PATH_ABSOLUTE = pathlib.Path.absolute


def _safe_path_absolute(self):
    try:
        return _REAL_PATH_ABSOLUTE(self)
    except FileNotFoundError:
        if self.is_absolute():
            return self
        return pathlib.Path(_STABLE_CWD) / self


pathlib.Path.absolute = _safe_path_absolute
pathlib.Path.cwd = classmethod(lambda cls: cls(_safe_getcwd()))
if "__file__" in globals() and not os.path.isabs(__file__):
    __file__ = os.path.join(_STABLE_CWD, __file__)

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="self_erecting_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(0.89, 0.75, 0.16, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.21, 0.23, 0.26, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    concrete = model.material("concrete", rgba=(0.64, 0.64, 0.61, 1.0))
    warning_red = model.material("warning_red", rgba=(0.76, 0.14, 0.12, 1.0))
    cable = model.material("cable", rgba=(0.14, 0.14, 0.16, 1.0))

    def midpoint(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)

    def distance(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> float:
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)

    def rpy_for_member(
        a: tuple[float, float, float], b: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        yaw = math.atan2(dy, dx)
        pitch = math.atan2(math.hypot(dx, dy), dz)
        return (0.0, pitch, yaw)

    def add_member(
        part,
        a: tuple[float, float, float],
        b: tuple[float, float, float],
        radius: float,
        material,
        name: str | None = None,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=distance(a, b)),
            origin=Origin(xyz=midpoint(a, b), rpy=rpy_for_member(a, b)),
            material=material,
            name=name,
        )

    def add_square_ring(
        part,
        *,
        half_span: float,
        width: float,
        z: float,
        thickness: float,
        material,
        prefix: str,
    ) -> None:
        part.visual(
            Box((thickness, width, thickness)),
            origin=Origin(xyz=(half_span, 0.0, z)),
            material=material,
            name=f"{prefix}_right",
        )
        part.visual(
            Box((thickness, width, thickness)),
            origin=Origin(xyz=(-half_span, 0.0, z)),
            material=material,
            name=f"{prefix}_left",
        )
        part.visual(
            Box((width, thickness, thickness)),
            origin=Origin(xyz=(0.0, half_span, z)),
            material=material,
            name=f"{prefix}_front",
        )
        part.visual(
            Box((width, thickness, thickness)),
            origin=Origin(xyz=(0.0, -half_span, z)),
            material=material,
            name=f"{prefix}_back",
        )

    chassis = model.part("chassis")
    chassis.visual(
        Box((2.60, 0.28, 0.14)),
        origin=Origin(xyz=(-0.05, 0.0, 0.07)),
        material=dark_grey,
        name="main_beam",
    )
    chassis.visual(
        Box((1.70, 0.52, 0.06)),
        origin=Origin(xyz=(0.18, 0.0, 0.17)),
        material=steel,
        name="upper_deck",
    )
    chassis.visual(
        Box((0.28, 1.14, 0.07)),
        origin=Origin(xyz=(-0.42, 0.0, 0.175)),
        material=steel,
        name="rear_outrigger_beam",
    )
    chassis.visual(
        Box((0.28, 1.14, 0.07)),
        origin=Origin(xyz=(0.58, 0.0, 0.175)),
        material=steel,
        name="front_outrigger_beam",
    )
    chassis.visual(
        Box((0.62, 0.44, 0.30)),
        origin=Origin(xyz=(0.70, 0.0, 0.32)),
        material=dark_grey,
        name="machinery_house",
    )
    chassis.visual(
        Box((0.18, 0.18, 0.126)),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=steel,
        name="pivot_pedestal",
    )
    chassis.visual(
        Box((0.12, 0.03, 0.18)),
        origin=Origin(xyz=(0.0, -0.105, 0.41)),
        material=steel,
        name="pivot_cheek_left",
    )
    chassis.visual(
        Box((0.12, 0.03, 0.18)),
        origin=Origin(xyz=(0.0, 0.105, 0.41)),
        material=steel,
        name="pivot_cheek_right",
    )
    chassis.visual(
        Box((0.16, 0.12, 0.18)),
        origin=Origin(xyz=(-1.18, 0.0, 0.23)),
        material=dark_grey,
        name="transport_saddle_post",
    )
    chassis.visual(
        Box((0.24, 0.22, 0.06)),
        origin=Origin(xyz=(-1.18, 0.0, 0.35)),
        material=steel,
        name="transport_saddle",
    )
    chassis.visual(
        Box((0.10, 0.10, 0.05)),
        origin=Origin(xyz=(-0.98, 0.0, 0.165)),
        material=concrete,
        name="counter_pad",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((2.70, 1.18, 0.75)),
        mass=22.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.375)),
    )

    mast_outer = model.part("mast_outer")
    mast_outer.visual(
        Cylinder(radius=0.037, length=0.24),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    mast_outer.visual(
        Box((0.18, 0.22, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=dark_grey,
        name="base_knuckle",
    )
    for x in (-0.105, 0.105):
        for y in (-0.105, 0.105):
            mast_outer.visual(
                Box((0.03, 0.03, 1.42)),
                origin=Origin(xyz=(x, y, 0.84)),
                material=crane_yellow,
            )
    for i, z in enumerate((0.20, 0.58, 0.96, 1.34, 1.52)):
        add_square_ring(
            mast_outer,
            half_span=0.105,
            width=0.24,
            z=z,
            thickness=0.03,
            material=crane_yellow,
            prefix=f"outer_ring_{i}",
        )
    mast_outer.visual(
        Box((0.008, 0.10, 0.16)),
        origin=Origin(xyz=(0.080, 0.0, 1.46)),
        material=steel,
        name="sleeve_pad_right",
    )
    mast_outer.visual(
        Box((0.008, 0.10, 0.16)),
        origin=Origin(xyz=(-0.080, 0.0, 1.46)),
        material=steel,
        name="sleeve_pad_left",
    )
    mast_outer.visual(
        Box((0.10, 0.008, 0.16)),
        origin=Origin(xyz=(0.0, 0.080, 1.46)),
        material=steel,
        name="sleeve_pad_front",
    )
    mast_outer.visual(
        Box((0.10, 0.008, 0.16)),
        origin=Origin(xyz=(0.0, -0.080, 1.46)),
        material=steel,
        name="sleeve_pad_back",
    )
    mast_outer.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(-0.090, 0.0, 1.46), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="top_guide_bushing_left",
    )
    mast_outer.visual(
        Cylinder(radius=0.018, length=0.04),
        origin=Origin(xyz=(0.090, 0.0, 1.46), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="top_guide_bushing_right",
    )
    mast_outer.visual(
        Box((0.06, 0.16, 0.18)),
        origin=Origin(xyz=(0.011, 0.0, 1.18)),
        material=dark_grey,
        name="transport_rest",
    )
    mast_outer.inertial = Inertial.from_geometry(
        Box((0.34, 0.34, 1.62)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
    )

    mast_inner = model.part("mast_inner")
    for x in (-0.065, 0.065):
        for y in (-0.065, 0.065):
            mast_inner.visual(
                Box((0.024, 0.024, 1.46)),
                origin=Origin(xyz=(x, y, 0.73)),
                material=crane_yellow,
            )
    for i, z in enumerate((0.06, 0.32, 0.58, 0.84, 1.10, 1.36)):
        add_square_ring(
            mast_inner,
            half_span=0.065,
            width=0.154,
            z=z,
            thickness=0.024,
            material=crane_yellow,
            prefix=f"inner_ring_{i}",
        )
    mast_inner.visual(
        Box((0.18, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.50)),
        material=steel,
        name="head_cap",
    )
    mast_inner.visual(
        Box((0.006, 0.06, 0.14)),
        origin=Origin(xyz=(0.071, 0.0, 0.38)),
        material=steel,
        name="guide_strip_right",
    )
    mast_inner.visual(
        Box((0.006, 0.06, 0.14)),
        origin=Origin(xyz=(-0.071, 0.0, 0.38)),
        material=steel,
        name="guide_strip_left",
    )
    mast_inner.visual(
        Box((0.06, 0.006, 0.14)),
        origin=Origin(xyz=(0.0, 0.071, 0.38)),
        material=steel,
        name="guide_strip_front",
    )
    mast_inner.visual(
        Box((0.06, 0.006, 0.14)),
        origin=Origin(xyz=(0.0, -0.071, 0.38)),
        material=steel,
        name="guide_strip_back",
    )
    mast_inner.visual(
        Box((0.05, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=warning_red,
        name="lock_lug",
    )
    mast_inner.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, 1.60)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
    )

    jib = model.part("jib")
    jib.visual(
        Box((0.24, 0.18, 0.10)),
        origin=Origin(xyz=(0.02, 0.0, 0.05)),
        material=steel,
        name="jib_mount_block",
    )
    jib.visual(
        Box((0.16, 0.16, 0.24)),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=crane_yellow,
        name="mast_head_frame",
    )
    jib.visual(
        Box((2.12, 0.04, 0.04)),
        origin=Origin(xyz=(1.00, -0.075, -0.03)),
        material=steel,
        name="rail_left",
    )
    jib.visual(
        Box((2.12, 0.04, 0.04)),
        origin=Origin(xyz=(1.00, 0.075, -0.03)),
        material=steel,
        name="rail_right",
    )
    jib.visual(
        Box((2.10, 0.12, 0.04)),
        origin=Origin(xyz=(1.02, 0.0, 0.02)),
        material=steel,
        name="bottom_chord",
    )
    jib.visual(
        Box((1.96, 0.04, 0.04)),
        origin=Origin(xyz=(1.06, 0.0, 0.28)),
        material=crane_yellow,
        name="top_chord",
    )
    jib.visual(
        Box((0.16, 0.12, 0.12)),
        origin=Origin(xyz=(2.08, 0.0, 0.10)),
        material=warning_red,
        name="jib_tip_marker",
    )
    jib.visual(
        Box((0.38, 0.14, 0.14)),
        origin=Origin(xyz=(-0.34, 0.0, 0.12)),
        material=dark_grey,
        name="ballast_pack",
    )
    add_member(jib, (0.04, -0.06, 0.28), (2.02, -0.05, 0.08), 0.010, crane_yellow)
    add_member(jib, (0.04, 0.06, 0.28), (2.02, 0.05, 0.08), 0.010, crane_yellow)
    add_member(jib, (0.18, 0.0, 0.34), (2.08, 0.0, 0.14), 0.010, crane_yellow)
    add_member(jib, (-0.16, 0.0, 0.24), (-0.52, 0.0, 0.10), 0.009, crane_yellow)
    add_member(jib, (0.0, 0.0, 0.40), (1.20, 0.0, 0.30), 0.004, cable)
    add_member(jib, (0.0, 0.0, 0.40), (2.04, 0.0, 0.16), 0.004, cable)
    jib.inertial = Inertial.from_geometry(
        Box((2.80, 0.36, 0.56)),
        mass=5.0,
        origin=Origin(xyz=(0.90, 0.0, 0.18)),
    )

    trolley = model.part("trolley")
    trolley.visual(
        Box((0.17, 0.11, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.13)),
        material=dark_grey,
        name="trolley_body",
    )
    trolley.visual(
        Box((0.18, 0.05, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
        material=steel,
        name="upper_yoke",
    )
    trolley.visual(
        Box((0.20, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=steel,
        name="carriage_crossbeam",
    )
    trolley.visual(
        Box((0.14, 0.024, 0.04)),
        origin=Origin(xyz=(0.0, -0.075, -0.045)),
        material=steel,
        name="roller_left",
    )
    trolley.visual(
        Box((0.14, 0.024, 0.04)),
        origin=Origin(xyz=(0.0, 0.075, -0.045)),
        material=steel,
        name="roller_right",
    )
    trolley.visual(
        Box((0.04, 0.014, 0.095)),
        origin=Origin(xyz=(-0.05, -0.06, -0.088)),
        material=steel,
        name="hanger_left_front",
    )
    trolley.visual(
        Box((0.04, 0.014, 0.095)),
        origin=Origin(xyz=(0.05, -0.06, -0.088)),
        material=steel,
        name="hanger_left_rear",
    )
    trolley.visual(
        Box((0.04, 0.014, 0.095)),
        origin=Origin(xyz=(-0.05, 0.06, -0.088)),
        material=steel,
        name="hanger_right_front",
    )
    trolley.visual(
        Box((0.04, 0.014, 0.095)),
        origin=Origin(xyz=(0.05, 0.06, -0.088)),
        material=steel,
        name="hanger_right_rear",
    )
    trolley.visual(
        Cylinder(radius=0.008, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, -0.36)),
        material=cable,
        name="hoist_line",
    )
    trolley.visual(
        Box((0.10, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, -0.72)),
        material=warning_red,
        name="hook_block",
    )
    trolley.inertial = Inertial.from_geometry(
        Box((0.22, 0.16, 0.92)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.36)),
    )

    lock_pin = model.part("lock_pin")
    lock_pin.visual(
        Cylinder(radius=0.016, length=0.20),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="pin_shaft",
    )
    lock_pin.visual(
        Box((0.05, 0.09, 0.02)),
        origin=Origin(xyz=(-0.115, 0.0, 0.0)),
        material=warning_red,
        name="pin_handle",
    )
    lock_pin.inertial = Inertial.from_geometry(
        Box((0.26, 0.10, 0.04)),
        mass=0.18,
        origin=Origin(),
    )

    model.articulation(
        "mast_erection",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=mast_outer,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=100.0,
            velocity=0.4,
            lower=-math.pi / 2.0,
            upper=0.02,
        ),
    )
    model.articulation(
        "mast_telescoping",
        ArticulationType.PRISMATIC,
        parent=mast_outer,
        child=mast_inner,
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.35,
            lower=-0.56,
            upper=0.0,
        ),
    )
    model.articulation(
        "mast_to_jib",
        ArticulationType.FIXED,
        parent=mast_inner,
        child=jib,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
    )
    model.articulation(
        "trolley_travel",
        ArticulationType.PRISMATIC,
        parent=jib,
        child=trolley,
        origin=Origin(xyz=(1.00, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.8,
            lower=-0.72,
            upper=0.78,
        ),
    )
    model.articulation(
        "lock_pin_slide",
        ArticulationType.PRISMATIC,
        parent=mast_outer,
        child=lock_pin,
        origin=Origin(xyz=(0.0, 0.0, 1.46)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.12,
            lower=-0.10,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    mast_outer = object_model.get_part("mast_outer")
    mast_inner = object_model.get_part("mast_inner")
    jib = object_model.get_part("jib")
    trolley = object_model.get_part("trolley")
    lock_pin = object_model.get_part("lock_pin")

    mast_erection = object_model.get_articulation("mast_erection")
    mast_telescoping = object_model.get_articulation("mast_telescoping")
    trolley_travel = object_model.get_articulation("trolley_travel")
    lock_pin_slide = object_model.get_articulation("lock_pin_slide")

    pivot_cheek_left = chassis.get_visual("pivot_cheek_left")
    pivot_pedestal = chassis.get_visual("pivot_pedestal")
    transport_saddle = chassis.get_visual("transport_saddle")

    pivot_barrel = mast_outer.get_visual("pivot_barrel")
    sleeve_pad_right = mast_outer.get_visual("sleeve_pad_right")
    top_guide_bushing_right = mast_outer.get_visual("top_guide_bushing_right")
    transport_rest = mast_outer.get_visual("transport_rest")

    head_cap = mast_inner.get_visual("head_cap")
    guide_strip_right = mast_inner.get_visual("guide_strip_right")
    lock_lug = mast_inner.get_visual("lock_lug")

    jib_mount_block = jib.get_visual("jib_mount_block")
    rail_left = jib.get_visual("rail_left")
    rail_right = jib.get_visual("rail_right")

    roller_left = trolley.get_visual("roller_left")
    roller_right = trolley.get_visual("roller_right")

    pin_shaft = lock_pin.get_visual("pin_shaft")

    ctx.allow_overlap(
        jib,
        trolley,
        reason="The travel trolley wheel shoes ride clamped against the jib bottom-rail running surfaces.",
    )
    ctx.allow_overlap(
        mast_outer,
        mast_inner,
        reason="The telescoping mast section remains nested inside the outer sleeve over its travel.",
    )
    ctx.allow_overlap(
        chassis,
        mast_outer,
        reason="The erection hinge barrel nests between the chassis cheek plates.",
    )
    ctx.allow_overlap(
        lock_pin,
        mast_outer,
        reason="The locking pin slides inside the top-sleeve guide bushings.",
    )
    ctx.allow_overlap(
        lock_pin,
        mast_inner,
        reason="The locking pin passes through the mast lock lug at working height.",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(
        mast_outer,
        chassis,
        axes="xz",
        min_overlap=0.06,
        elem_a=pivot_barrel,
        elem_b=pivot_cheek_left,
        name="pivot_barrel_aligned_with_cheek_plate",
    )
    ctx.expect_contact(
        mast_outer,
        chassis,
        elem_a=pivot_barrel,
        elem_b=pivot_pedestal,
        name="mast_hinge_seated_on_pivot_pedestal",
    )
    ctx.expect_within(
        mast_inner,
        mast_outer,
        axes="xy",
        name="inner_mast_within_outer_sleeve_plan",
    )
    ctx.expect_gap(
        mast_outer,
        mast_inner,
        axis="x",
        max_gap=0.004,
        max_penetration=0.0,
        positive_elem=sleeve_pad_right,
        negative_elem=guide_strip_right,
        name="top_sleeve_guides_inner_mast",
    )
    ctx.expect_gap(
        jib,
        mast_inner,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=jib_mount_block,
        negative_elem=head_cap,
        name="jib_mount_block_seated_on_mast_head",
    )
    ctx.expect_gap(
        jib,
        trolley,
        axis="z",
        min_gap=-0.03,
        max_gap=0.002,
        positive_elem=rail_left,
        negative_elem=roller_left,
        name="trolley_left_roller_seated_under_rail",
    )
    ctx.expect_gap(
        jib,
        trolley,
        axis="z",
        min_gap=-0.03,
        max_gap=0.002,
        positive_elem=rail_right,
        negative_elem=roller_right,
        name="trolley_right_roller_seated_under_rail",
    )
    ctx.expect_overlap(
        jib,
        trolley,
        axes="yz",
        min_overlap=0.02,
        elem_a=rail_left,
        elem_b=roller_left,
        name="trolley_left_bogie_nested_on_left_rail",
    )
    ctx.expect_overlap(
        jib,
        trolley,
        axes="yz",
        min_overlap=0.02,
        elem_a=rail_right,
        elem_b=roller_right,
        name="trolley_right_bogie_nested_on_right_rail",
    )
    ctx.expect_overlap(
        lock_pin,
        mast_outer,
        axes="yz",
        min_overlap=0.02,
        elem_a=pin_shaft,
        elem_b=top_guide_bushing_right,
        name="lock_pin_aligned_with_top_sleeve_bushing",
    )
    ctx.expect_overlap(
        lock_pin,
        mast_inner,
        axes="yz",
        min_overlap=0.028,
        elem_a=pin_shaft,
        elem_b=lock_lug,
        name="lock_pin_crosses_mast_lock_lug",
    )

    with ctx.pose({trolley_travel: -0.68}):
        ctx.expect_gap(
            jib,
            trolley,
            axis="z",
            min_gap=-0.03,
            max_gap=0.002,
            positive_elem=rail_left,
            negative_elem=roller_left,
            name="trolley_stays_on_rail_at_inner_limit",
        )
        ctx.expect_overlap(
            jib,
            trolley,
            axes="yz",
            min_overlap=0.02,
            elem_a=rail_left,
            elem_b=roller_left,
            name="trolley_left_bogie_stays_nested_at_inner_limit",
        )
    with ctx.pose({trolley_travel: 0.72}):
        ctx.expect_gap(
            jib,
            trolley,
            axis="z",
            min_gap=-0.03,
            max_gap=0.002,
            positive_elem=rail_right,
            negative_elem=roller_right,
            name="trolley_stays_on_rail_at_outer_limit",
        )
        ctx.expect_overlap(
            jib,
            trolley,
            axes="yz",
            min_overlap=0.02,
            elem_a=rail_right,
            elem_b=roller_right,
            name="trolley_right_bogie_stays_nested_at_outer_limit",
        )

    with ctx.pose(
        {
            mast_erection: -math.pi / 2.0,
            mast_telescoping: -0.56,
            lock_pin_slide: -0.09,
        }
    ):
        ctx.expect_gap(
            mast_outer,
            chassis,
            axis="z",
            max_gap=0.003,
            max_penetration=0.0,
            positive_elem=transport_rest,
            negative_elem=transport_saddle,
            name="stored_mast_rests_on_transport_saddle",
        )
        ctx.expect_overlap(
            mast_outer,
            chassis,
            axes="xy",
            min_overlap=0.08,
            elem_a=transport_rest,
            elem_b=transport_saddle,
            name="stored_mast_transport_rest_centered_on_saddle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
