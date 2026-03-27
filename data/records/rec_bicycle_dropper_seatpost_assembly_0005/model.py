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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_outer_sleeve_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0158, 0.000),
            (0.0158, 0.280),
            (0.0166, 0.294),
            (0.0176, 0.306),
            (0.0176, 0.334),
        ],
        [
            (0.0139, 0.008),
            (0.0139, 0.278),
            (0.0146, 0.294),
            (0.0152, 0.306),
            (0.0152, 0.326),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )


def _build_seal_head_ring():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0178, 0.000),
            (0.0178, 0.012),
        ],
        [
            (0.0131, 0.001),
            (0.0131, 0.010),
        ],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def _build_cable_port_shell():
    return LatheGeometry.from_shell_profiles(
        [
            (0.0054, 0.000),
            (0.0054, 0.014),
        ],
        [
            (0.0035, 0.001),
            (0.0035, 0.013),
        ],
        segments=40,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dropper_seatpost", assets=ASSETS)

    outer_alloy = model.material("outer_alloy", rgba=(0.75, 0.78, 0.81, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.15, 0.16, 0.17, 1.0))
    clamp_black = model.material("clamp_black", rgba=(0.10, 0.11, 0.12, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        _save_mesh(_build_outer_sleeve_shell(), "outer_sleeve_shell.obj"),
        material=outer_alloy,
        name="sleeve_shell",
    )
    outer_sleeve.visual(
        Cylinder(radius=0.0178, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=dark_anodized,
        name="lower_collar",
    )
    outer_sleeve.visual(
        _save_mesh(_build_seal_head_ring(), "outer_sleeve_seal_head.obj"),
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        material=seal_rubber,
        name="seal_head",
    )
    outer_sleeve.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0178, length=0.334),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.167)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0128, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        material=dark_anodized,
        name="stanchion",
    )
    inner_post.visual(
        Cylinder(radius=0.0146, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=seal_rubber,
        name="guide_bushing",
    )
    inner_post.visual(
        Cylinder(radius=0.0138, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.263)),
        material=clamp_black,
        name="stop_collar",
    )
    inner_post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0135, length=0.260),
        mass=0.43,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
    )

    clamp_head = model.part("clamp_head")
    clamp_head.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=clamp_black,
        name="mast_socket",
    )
    clamp_head.visual(
        Cylinder(radius=0.0085, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=clamp_black,
        name="clamp_neck",
    )
    clamp_head.visual(
        Box((0.044, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=clamp_black,
        name="lower_cradle",
    )
    clamp_head.visual(
        Box((0.036, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=outer_alloy,
        name="upper_yoke",
    )
    clamp_head.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, 0.007, 0.029), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_anodized,
        name="front_bolt_barrel",
    )
    clamp_head.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, -0.007, 0.029), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_anodized,
        name="rear_bolt_barrel",
    )
    clamp_head.visual(
        Box((0.006, 0.010, 0.008)),
        origin=Origin(xyz=(-0.019, 0.0, 0.022)),
        material=clamp_black,
        name="port_boss",
    )
    clamp_head.inertial = Inertial.from_geometry(
        Box((0.044, 0.020, 0.026)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
    )

    actuation_port = model.part("actuation_port")
    actuation_port.visual(
        _save_mesh(_build_cable_port_shell(), "clamp_head_actuation_port.obj"),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=seal_rubber,
        name="port_shell",
    )
    actuation_port.inertial = Inertial.from_geometry(
        Box((0.014, 0.012, 0.012)),
        mass=0.02,
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
    )

    dropper = model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.25,
            lower=0.0,
            upper=0.150,
        ),
    )
    model.articulation(
        "inner_to_clamp_head",
        ArticulationType.FIXED,
        parent=inner_post,
        child=clamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
    )
    model.articulation(
        "head_to_actuation_port",
        ArticulationType.FIXED,
        parent=clamp_head,
        child=actuation_port,
        origin=Origin(xyz=(-0.029, 0.0, 0.022)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    outer_sleeve = object_model.get_part("outer_sleeve")
    inner_post = object_model.get_part("inner_post")
    clamp_head = object_model.get_part("clamp_head")
    actuation_port = object_model.get_part("actuation_port")
    dropper = object_model.get_articulation("outer_to_inner")

    sleeve_shell = outer_sleeve.get_visual("sleeve_shell")
    seal_head = outer_sleeve.get_visual("seal_head")
    stanchion = inner_post.get_visual("stanchion")
    guide_bushing = inner_post.get_visual("guide_bushing")
    stop_collar = inner_post.get_visual("stop_collar")
    mast_socket = clamp_head.get_visual("mast_socket")
    lower_cradle = clamp_head.get_visual("lower_cradle")
    port_boss = clamp_head.get_visual("port_boss")
    port_shell = actuation_port.get_visual("port_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        inner_post,
        outer_sleeve,
        reason="The dropper uses nested internal guide bushings that ride inside the alloy outer sleeve.",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(
        inner_post,
        outer_sleeve,
        axes="xy",
        inner_elem=stanchion,
        outer_elem=sleeve_shell,
        name="stanchion stays centered within alloy outer sleeve",
    )
    ctx.expect_within(
        inner_post,
        outer_sleeve,
        axes="xy",
        inner_elem=guide_bushing,
        outer_elem=sleeve_shell,
        name="internal guide bushing rides inside the outer sleeve bore",
    )
    ctx.expect_gap(
        inner_post,
        outer_sleeve,
        axis="z",
        positive_elem=stop_collar,
        negative_elem=seal_head,
        max_gap=0.001,
        max_penetration=0.0,
        name="retracted stop collar seats on the sleeve seal head",
    )
    ctx.expect_gap(
        clamp_head,
        inner_post,
        axis="z",
        positive_elem=mast_socket,
        negative_elem=stanchion,
        max_gap=0.001,
        max_penetration=0.0,
        name="saddle clamp head sits on top of the inner post",
    )
    ctx.expect_overlap(
        clamp_head,
        inner_post,
        axes="xy",
        elem_a=mast_socket,
        elem_b=stanchion,
        min_overlap=0.0003,
        name="clamp head remains aligned over the moving post",
    )
    ctx.expect_gap(
        clamp_head,
        actuation_port,
        axis="x",
        positive_elem=port_boss,
        negative_elem=port_shell,
        max_gap=0.001,
        max_penetration=0.0,
        name="cable actuation port is mounted into the side boss of the clamp head",
    )
    ctx.expect_overlap(
        clamp_head,
        actuation_port,
        axes="yz",
        elem_a=port_boss,
        elem_b=port_shell,
        min_overlap=0.00003,
        name="cable actuation port shares the clamp head footprint",
    )

    with ctx.pose({dropper: 0.150}):
        ctx.expect_within(
            inner_post,
            outer_sleeve,
            axes="xy",
            inner_elem=stanchion,
            outer_elem=sleeve_shell,
            name="extended stanchion remains coaxial with the outer sleeve",
        )
        ctx.expect_within(
            inner_post,
            outer_sleeve,
            axes="xy",
            inner_elem=guide_bushing,
            outer_elem=sleeve_shell,
            name="guide bushing remains captured in the sleeve during extension",
        )
        ctx.expect_gap(
            inner_post,
            outer_sleeve,
            axis="z",
            positive_elem=stop_collar,
            negative_elem=seal_head,
            min_gap=0.145,
            name="prismatic dropper travel opens a visible extension gap",
        )
        ctx.expect_gap(
            clamp_head,
            outer_sleeve,
            axis="z",
            positive_elem=lower_cradle,
            negative_elem=seal_head,
            min_gap=0.155,
            name="saddle clamp head rises well above the outer sleeve at full extension",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
