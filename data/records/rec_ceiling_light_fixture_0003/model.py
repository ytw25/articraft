from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except FileNotFoundError:
            pass
        return "/"


os.getcwd = _safe_getcwd
_safe_getcwd()

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

TRACK_LENGTH = 1.20
TRACK_WIDTH = 0.08
TOP_THICKNESS = 0.008
SIDE_THICKNESS = 0.008
SIDE_HEIGHT = 0.042
LIP_WIDTH = 0.019
LIP_THICKNESS = 0.006
SLOT_WIDTH = 0.026

CANOPY_X = (-0.42, 0.42)
LAMP_SPECS = (
    ("1", -0.32, -0.06, 0.06),
    ("2", 0.00, -0.06, 0.06),
    ("3", 0.32, -0.06, 0.06),
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_lighting_bar")

    satin_black = model.material("satin_black", color=(0.14, 0.14, 0.15, 1.0))
    matte_graphite = model.material("matte_graphite", color=(0.24, 0.25, 0.27, 1.0))
    warm_white = model.material("warm_white", color=(0.92, 0.92, 0.90, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", color=(0.74, 0.75, 0.77, 1.0))
    copper_strip = model.material("copper_strip", color=(0.76, 0.48, 0.28, 1.0))
    lens_glass = model.material("lens_glass", color=(0.84, 0.88, 0.93, 0.38))

    track = model.part("track")
    track.visual(
        Box((TRACK_LENGTH, TRACK_WIDTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=satin_black,
        name="top_web",
    )
    track.visual(
        Box((TRACK_LENGTH, SIDE_THICKNESS, SIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.036, 0.000)),
        material=satin_black,
        name="side_right",
    )
    track.visual(
        Box((TRACK_LENGTH, SIDE_THICKNESS, SIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.036, 0.000)),
        material=satin_black,
        name="side_left",
    )
    track.visual(
        Box((TRACK_LENGTH, LIP_WIDTH, LIP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0225, -0.019)),
        material=matte_graphite,
        name="lip_right",
    )
    track.visual(
        Box((TRACK_LENGTH, LIP_WIDTH, LIP_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.0225, -0.019)),
        material=matte_graphite,
        name="lip_left",
    )
    track.visual(
        Box((TRACK_LENGTH - 0.04, 0.012, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
        material=copper_strip,
        name="contact_strip",
    )

    for side_name, canopy_x in zip(("left", "right"), CANOPY_X):
        canopy = model.part(f"canopy_{side_name}")
        canopy.visual(
            Cylinder(radius=0.055, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.016)),
            material=brushed_aluminum,
            name="cap",
        )
        canopy.visual(
            Cylinder(radius=0.015, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=brushed_aluminum,
            name="stem",
        )
        model.articulation(
            f"track_to_canopy_{side_name}",
            ArticulationType.FIXED,
            parent=track,
            child=canopy,
            origin=Origin(xyz=(canopy_x, 0.0, 0.025)),
        )

    for lamp_id, x0, lower, upper in LAMP_SPECS:
        carriage = model.part(f"carriage_{lamp_id}")
        carriage.visual(
            Box((0.052, 0.038, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, 0.000)),
            material=matte_graphite,
            name="shoulder",
        )
        carriage.visual(
            Box((0.022, SLOT_WIDTH - 0.006, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.013)),
            material=matte_graphite,
            name="neck",
        )
        carriage.visual(
            Box((0.048, 0.042, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, 0.028)),
            material=matte_graphite,
            name="adapter",
        )
        carriage.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(xyz=(0.012, 0.0, -0.010)),
            material=brushed_aluminum,
            name="lock_knob",
        )
        carriage.visual(
            Cylinder(radius=0.005, length=0.074),
            origin=Origin(xyz=(0.0, 0.0, -0.041)),
            material=brushed_aluminum,
            name="pendant_stem",
        )
        carriage.visual(
            Box((0.014, 0.082, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.083)),
            material=matte_graphite,
            name="yoke_bridge",
        )
        carriage.visual(
            Box((0.010, 0.008, 0.068)),
            origin=Origin(xyz=(0.0, 0.038, -0.117)),
            material=matte_graphite,
            name="yoke_right",
        )
        carriage.visual(
            Box((0.010, 0.008, 0.068)),
            origin=Origin(xyz=(0.0, -0.038, -0.117)),
            material=matte_graphite,
            name="yoke_left",
        )
        carriage.visual(
            Cylinder(radius=0.003, length=0.068),
            origin=Origin(xyz=(0.0, 0.0, -0.117), rpy=(1.57079632679, 0.0, 0.0)),
            material=brushed_aluminum,
            name="pivot_axle",
        )

        head = model.part(f"lamp_{lamp_id}")
        head.visual(
            Cylinder(radius=0.004, length=0.068),
            origin=Origin(rpy=(1.57079632679, 0.0, 0.0)),
            material=brushed_aluminum,
            name="trunnion",
        )
        head.visual(
            Cylinder(radius=0.017, length=0.030),
            origin=Origin(xyz=(0.0, 0.0, -0.017)),
            material=matte_graphite,
            name="hub",
        )
        head.visual(
            Cylinder(radius=0.030, length=0.090),
            origin=Origin(xyz=(0.0, 0.0, -0.066)),
            material=warm_white,
            name="body_shell",
        )
        head.visual(
            Cylinder(radius=0.033, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.111)),
            material=brushed_aluminum,
            name="bezel",
        )
        head.visual(
            Cylinder(radius=0.025, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, -0.1175)),
            material=lens_glass,
            name="lens",
        )

        model.articulation(
            f"track_to_carriage_{lamp_id}",
            ArticulationType.PRISMATIC,
            parent=track,
            child=carriage,
            origin=Origin(xyz=(x0, 0.0, -0.026)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=0.40,
                lower=lower,
                upper=upper,
            ),
        )
        model.articulation(
            f"carriage_{lamp_id}_to_lamp_{lamp_id}",
            ArticulationType.REVOLUTE,
            parent=carriage,
            child=head,
            origin=Origin(xyz=(0.0, 0.0, -0.117)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=2.0,
                lower=-0.95,
                upper=0.70,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    track = object_model.get_part("track")
    top_web = track.get_visual("top_web")
    lip_left = track.get_visual("lip_left")
    lip_right = track.get_visual("lip_right")
    contact_strip = track.get_visual("contact_strip")

    for lamp_id, _, _, _ in LAMP_SPECS:
        carriage = object_model.get_part(f"carriage_{lamp_id}")
        lamp = object_model.get_part(f"lamp_{lamp_id}")
        ctx.allow_overlap(
            track,
            carriage,
            reason="track adapter nests inside the channel while the clamp shoulder hooks under the slot lips",
        )
        ctx.allow_overlap(
            carriage,
            lamp,
            reason="lamp trunnion is captured by the yoke cheeks to form the pivoting head mount",
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
    ctx.expect_overlap(
        track,
        track,
        axes="x",
        min_overlap=1.10,
        elem_a=contact_strip,
        elem_b=top_web,
        name="contact_strip_runs_nearly_full_track_length",
    )

    for side_name in ("left", "right"):
        canopy = object_model.get_part(f"canopy_{side_name}")
        canopy_cap = canopy.get_visual("cap")
        canopy_stem = canopy.get_visual("stem")
        ctx.expect_overlap(canopy, track, axes="xy", min_overlap=0.004, elem_a=canopy_cap)
        ctx.expect_gap(
            canopy,
            track,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=canopy_stem,
            negative_elem=top_web,
            name=f"{side_name}_canopy_seated_on_track",
        )

    for lamp_id, _, _, _ in LAMP_SPECS:
        carriage = object_model.get_part(f"carriage_{lamp_id}")
        lamp = object_model.get_part(f"lamp_{lamp_id}")
        slide = object_model.get_articulation(f"track_to_carriage_{lamp_id}")
        pitch = object_model.get_articulation(f"carriage_{lamp_id}_to_lamp_{lamp_id}")

        shoulder = carriage.get_visual("shoulder")
        adapter = carriage.get_visual("adapter")
        lock_knob = carriage.get_visual("lock_knob")
        yoke_left = carriage.get_visual("yoke_left")
        yoke_right = carriage.get_visual("yoke_right")
        pivot_axle = carriage.get_visual("pivot_axle")
        body_shell = lamp.get_visual("body_shell")
        trunnion = lamp.get_visual("trunnion")

        ctx.expect_origin_distance(carriage, track, axes="y", max_dist=0.001)
        ctx.expect_within(carriage, track, axes="x")
        ctx.expect_gap(
            track,
            carriage,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=lip_left,
            negative_elem=shoulder,
            name=f"lamp_{lamp_id}_left_lip_clamp_seat",
        )
        ctx.expect_gap(
            track,
            carriage,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=lip_right,
            negative_elem=shoulder,
            name=f"lamp_{lamp_id}_right_lip_clamp_seat",
        )
        ctx.expect_gap(
            track,
            carriage,
            axis="z",
            min_gap=0.004,
            positive_elem=contact_strip,
            negative_elem=adapter,
            name=f"lamp_{lamp_id}_adapter_clears_contact_strip",
        )
        ctx.expect_gap(
            track,
            carriage,
            axis="z",
            min_gap=0.006,
            positive_elem=lip_left,
            negative_elem=lock_knob,
            name=f"lamp_{lamp_id}_lock_knob_hangs_below_track",
        )
        ctx.expect_contact(
            carriage,
            lamp,
            elem_a=yoke_left,
            elem_b=trunnion,
            name=f"lamp_{lamp_id}_left_yoke_contact",
        )
        ctx.expect_contact(
            carriage,
            lamp,
            elem_a=yoke_right,
            elem_b=trunnion,
            name=f"lamp_{lamp_id}_right_yoke_contact",
        )
        ctx.expect_within(
            carriage,
            lamp,
            axes="xz",
            inner_elem=pivot_axle,
            outer_elem=trunnion,
            name=f"lamp_{lamp_id}_pivot_axle_nested_in_trunnion",
        )
        ctx.expect_gap(
            carriage,
            lamp,
            axis="z",
            min_gap=0.08,
            positive_elem=shoulder,
            negative_elem=body_shell,
            name=f"lamp_{lamp_id}_reads_as_pendant",
        )

        with ctx.pose({slide: 0.85 * slide.motion_limits.upper, pitch: 0.55}):
            ctx.expect_within(carriage, track, axes="x", name=f"lamp_{lamp_id}_stays_on_track_at_forward_pose")
            ctx.expect_gap(
                track,
                carriage,
                axis="z",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem=lip_left,
                negative_elem=shoulder,
                name=f"lamp_{lamp_id}_left_lip_stays_seated_forward",
            )
            ctx.expect_contact(
                carriage,
                lamp,
                elem_a=yoke_right,
                elem_b=trunnion,
                name=f"lamp_{lamp_id}_pivot_stays_engaged_forward",
            )

        with ctx.pose({slide: 0.85 * slide.motion_limits.lower, pitch: -0.60}):
            ctx.expect_within(carriage, track, axes="x", name=f"lamp_{lamp_id}_stays_on_track_at_back_pose")
            ctx.expect_gap(
                track,
                carriage,
                axis="z",
                max_gap=0.001,
                max_penetration=0.0,
                positive_elem=lip_right,
                negative_elem=shoulder,
                name=f"lamp_{lamp_id}_right_lip_stays_seated_back",
            )
            ctx.expect_gap(
                carriage,
                lamp,
                axis="z",
                min_gap=0.06,
                positive_elem=shoulder,
                negative_elem=body_shell,
                name=f"lamp_{lamp_id}_still_reads_as_pendant_when_tilted",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
