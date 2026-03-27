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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rooftop_floodlight", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.34, 0.37, 0.39, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    galvanized = model.material("galvanized", rgba=(0.56, 0.58, 0.60, 1.0))
    glass = model.material("glass", rgba=(0.74, 0.84, 0.92, 0.32))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.180, 0.140, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=galvanized,
        name="base_plate",
    )
    base.visual(
        Box((0.080, 0.060, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=painted_steel,
        name="pedestal_block",
    )
    base.visual(
        Cylinder(radius=0.036, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=dark_steel,
        name="bearing_collar",
    )
    base.visual(
        Box((0.020, 0.020, 0.010)),
        origin=Origin(xyz=(0.060, 0.045, 0.009)),
        material=galvanized,
        name="front_right_anchor",
    )
    base.visual(
        Box((0.020, 0.020, 0.010)),
        origin=Origin(xyz=(-0.060, 0.045, 0.009)),
        material=galvanized,
        name="front_left_anchor",
    )
    base.visual(
        Box((0.020, 0.020, 0.010)),
        origin=Origin(xyz=(0.060, -0.045, 0.009)),
        material=galvanized,
        name="rear_right_anchor",
    )
    base.visual(
        Box((0.020, 0.020, 0.010)),
        origin=Origin(xyz=(-0.060, -0.045, 0.009)),
        material=galvanized,
        name="rear_left_anchor",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.180, 0.140, 0.060)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    head = model.part("swivel_head")
    head.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=galvanized,
        name="swivel_flange",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=painted_steel,
        name="column_shaft",
    )
    head.visual(
        Box((0.042, 0.042, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.138)),
        material=dark_steel,
        name="column_head",
    )
    head.visual(
        Box((0.042, 0.012, 0.050)),
        origin=Origin(xyz=(0.016, 0.0, 0.158)),
        material=dark_steel,
        name="t_bracket_web",
    )
    head.visual(
        Box((0.030, 0.006, 0.040)),
        origin=Origin(xyz=(0.030, 0.016, 0.153), rpy=(0.0, math.pi / 4.0, 0.0)),
        material=dark_steel,
        name="left_gusset",
    )
    head.visual(
        Box((0.030, 0.006, 0.040)),
        origin=Origin(xyz=(0.030, -0.016, 0.153), rpy=(0.0, math.pi / 4.0, 0.0)),
        material=dark_steel,
        name="right_gusset",
    )
    head.visual(
        Cylinder(radius=0.011, length=0.152),
        origin=Origin(xyz=(0.081, 0.0, 0.168), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=painted_steel,
        name="arm_tube",
    )
    head.visual(
        Box((0.022, 0.122, 0.022)),
        origin=Origin(xyz=(0.128, 0.0, 0.168)),
        material=dark_steel,
        name="yoke_crown",
    )
    head.visual(
        Box((0.074, 0.004, 0.076)),
        origin=Origin(xyz=(0.168, -0.059, 0.122)),
        material=painted_steel,
        name="left_yoke_plate",
    )
    head.visual(
        Box((0.074, 0.004, 0.076)),
        origin=Origin(xyz=(0.168, 0.059, 0.122)),
        material=painted_steel,
        name="right_yoke_plate",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.230, 0.124, 0.184)),
        mass=4.2,
        origin=Origin(xyz=(0.115, 0.0, 0.092)),
    )

    lamp = model.part("lamp")
    lamp.visual(
        Box((0.094, 0.102, 0.064)),
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
        material=painted_steel,
        name="housing_shell",
    )
    lamp.visual(
        Box((0.064, 0.086, 0.010)),
        origin=Origin(xyz=(0.016, 0.0, 0.027)),
        material=painted_steel,
        name="top_taper",
    )
    lamp.visual(
        Box((0.006, 0.096, 0.074)),
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        material=black,
        name="front_bezel",
    )
    lamp.visual(
        Box((0.003, 0.088, 0.064)),
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
        material=glass,
        name="lens",
    )
    lamp.visual(
        Box((0.012, 0.060, 0.044)),
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
        material=dark_steel,
        name="rear_cap",
    )
    lamp.visual(
        Box((0.024, 0.092, 0.004)),
        origin=Origin(xyz=(0.060, 0.0, 0.031)),
        material=painted_steel,
        name="visor",
    )
    lamp.visual(
        Box((0.022, 0.006, 0.034)),
        origin=Origin(xyz=(0.0, -0.053, 0.0)),
        material=dark_steel,
        name="left_ear",
    )
    lamp.visual(
        Box((0.022, 0.006, 0.034)),
        origin=Origin(xyz=(0.0, 0.053, 0.0)),
        material=dark_steel,
        name="right_ear",
    )
    lamp.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.0, -0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="left_pin_shank",
    )
    lamp.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, -0.063, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="left_pin_cap",
    )
    lamp.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.0, 0.058, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="right_pin_shank",
    )
    lamp.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.0, 0.063, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="right_pin_cap",
    )
    lamp.visual(
        Box((0.020, 0.054, 0.003)),
        origin=Origin(xyz=(-0.024, 0.0, 0.021)),
        material=dark_steel,
        name="rear_fin_upper",
    )
    lamp.visual(
        Box((0.018, 0.050, 0.003)),
        origin=Origin(xyz=(-0.031, 0.0, 0.011)),
        material=dark_steel,
        name="rear_fin_lower",
    )
    lamp.inertial = Inertial.from_geometry(
        Box((0.102, 0.112, 0.082)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.8),
    )
    model.articulation(
        "yoke_to_lamp",
        ArticulationType.REVOLUTE,
        parent=head,
        child=lamp,
        origin=Origin(xyz=(0.182, 0.0, 0.122)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.6, lower=-0.70, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    base = object_model.get_part("base")
    head = object_model.get_part("swivel_head")
    lamp = object_model.get_part("lamp")
    swivel = object_model.get_articulation("base_to_swivel")
    tilt = object_model.get_articulation("yoke_to_lamp")

    base_plate = base.get_visual("base_plate")
    bearing_collar = base.get_visual("bearing_collar")
    swivel_flange = head.get_visual("swivel_flange")
    column_shaft = head.get_visual("column_shaft")
    column_head = head.get_visual("column_head")
    t_bracket_web = head.get_visual("t_bracket_web")
    arm_tube = head.get_visual("arm_tube")
    yoke_crown = head.get_visual("yoke_crown")
    left_yoke = head.get_visual("left_yoke_plate")
    right_yoke = head.get_visual("right_yoke_plate")
    housing_shell = lamp.get_visual("housing_shell")
    lens = lamp.get_visual("lens")
    left_ear = lamp.get_visual("left_ear")
    right_ear = lamp.get_visual("right_ear")
    left_pin_shank = lamp.get_visual("left_pin_shank")
    left_pin_cap = lamp.get_visual("left_pin_cap")
    right_pin_shank = lamp.get_visual("right_pin_shank")
    right_pin_cap = lamp.get_visual("right_pin_cap")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.045)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(head, base, elem_a=swivel_flange, elem_b=bearing_collar)
    ctx.expect_overlap(head, base, axes="xy", min_overlap=0.02, elem_a=column_shaft, elem_b=bearing_collar)
    ctx.expect_contact(head, head, elem_a=column_head, elem_b=t_bracket_web)
    ctx.expect_contact(head, head, elem_a=t_bracket_web, elem_b=arm_tube)
    ctx.expect_contact(head, head, elem_a=arm_tube, elem_b=yoke_crown)
    ctx.expect_gap(
        head,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=swivel_flange,
        negative_elem=bearing_collar,
    )
    ctx.expect_gap(
        lamp,
        head,
        axis="x",
        min_gap=0.05,
        positive_elem=lens,
        negative_elem=column_shaft,
    )
    ctx.expect_overlap(lamp, head, axes="xz", min_overlap=0.010, elem_a=left_ear, elem_b=left_yoke)
    ctx.expect_overlap(lamp, head, axes="xz", min_overlap=0.010, elem_a=right_ear, elem_b=right_yoke)
    ctx.expect_gap(
        lamp,
        head,
        axis="y",
        min_gap=0.0,
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=left_ear,
        negative_elem=left_yoke,
    )
    ctx.expect_gap(
        head,
        lamp,
        axis="y",
        min_gap=0.0,
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem=right_yoke,
        negative_elem=right_ear,
    )
    ctx.expect_contact(lamp, head, elem_a=left_pin_cap, elem_b=left_yoke)
    ctx.expect_contact(lamp, head, elem_a=right_pin_cap, elem_b=right_yoke)
    ctx.expect_contact(lamp, head, elem_a=left_pin_shank, elem_b=left_yoke)
    ctx.expect_contact(lamp, head, elem_a=right_pin_shank, elem_b=right_yoke)
    ctx.expect_gap(
        lamp,
        base,
        axis="z",
        min_gap=0.07,
        positive_elem=housing_shell,
        negative_elem=base_plate,
    )

    with ctx.pose({tilt: 0.35}):
        ctx.expect_overlap(lamp, head, axes="xz", min_overlap=0.010, elem_a=left_ear, elem_b=left_yoke)
        ctx.expect_overlap(lamp, head, axes="xz", min_overlap=0.010, elem_a=right_ear, elem_b=right_yoke)
        ctx.expect_contact(lamp, head, elem_a=left_pin_cap, elem_b=left_yoke)
        ctx.expect_contact(lamp, head, elem_a=right_pin_cap, elem_b=right_yoke)
        ctx.expect_contact(lamp, head, elem_a=left_pin_shank, elem_b=left_yoke)
        ctx.expect_contact(lamp, head, elem_a=right_pin_shank, elem_b=right_yoke)
        ctx.expect_gap(
            lamp,
            base,
            axis="z",
            min_gap=0.08,
            positive_elem=lens,
            negative_elem=base_plate,
        )

    with ctx.pose({tilt: -0.65}):
        ctx.expect_overlap(lamp, head, axes="xz", min_overlap=0.008, elem_a=left_ear, elem_b=left_yoke)
        ctx.expect_overlap(lamp, head, axes="xz", min_overlap=0.008, elem_a=right_ear, elem_b=right_yoke)
        ctx.expect_contact(lamp, head, elem_a=left_pin_cap, elem_b=left_yoke)
        ctx.expect_contact(lamp, head, elem_a=right_pin_cap, elem_b=right_yoke)
        ctx.expect_contact(lamp, head, elem_a=left_pin_shank, elem_b=left_yoke)
        ctx.expect_contact(lamp, head, elem_a=right_pin_shank, elem_b=right_yoke)
        ctx.expect_gap(
            lamp,
            base,
            axis="z",
            min_gap=0.04,
            positive_elem=lens,
            negative_elem=base_plate,
        )

    with ctx.pose({swivel: 1.15, tilt: 0.20}):
        ctx.expect_contact(head, base, elem_a=swivel_flange, elem_b=bearing_collar)
        ctx.expect_gap(
            lamp,
            head,
            axis="x",
            min_gap=0.045,
            positive_elem=lens,
            negative_elem=column_shaft,
        )
        ctx.expect_contact(lamp, head, elem_a=left_pin_cap, elem_b=left_yoke)
        ctx.expect_contact(lamp, head, elem_a=right_pin_cap, elem_b=right_yoke)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
