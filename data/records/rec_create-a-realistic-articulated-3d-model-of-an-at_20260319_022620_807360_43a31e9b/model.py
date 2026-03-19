from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from pathlib import Path
import numpy as np
from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Mesh,
    mesh_from_geometry,
    tube_from_spline_points,
    ExtrudeGeometry,
    rounded_rect_profile,
    LatheGeometry,
    Sphere,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="atv", assets=ASSETS)

    # Materials
    metal_dark = model.material("metal_dark", rgba=(0.2, 0.2, 0.2, 1.0))
    plastic_red = model.material("plastic_red", rgba=(0.8, 0.1, 0.1, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.1, 0.1, 0.1, 1.0))
    chrome = model.material("chrome", rgba=(0.8, 0.8, 0.8, 1.0))
    seat_material = model.material("seat_material", rgba=(0.15, 0.15, 0.15, 1.0))

    chassis = model.part("chassis")

    # Dimensions
    L = 1.6
    W_body = 0.42  # Slightly wider for connectivity
    wheel_radius = 0.25
    wheel_width = 0.22
    wheelbase = 1.1
    track = 0.9

    # 1. Main Frame
    rail_points_l = [(-L/2, -0.15, 0.17), (L/2, -0.15, 0.17)]
    rail_points_r = [(-L/2, 0.15, 0.17), (L/2, 0.15, 0.17)]
    chassis.visual(mesh_from_geometry(tube_from_spline_points(rail_points_l, radius=0.03), ASSETS.mesh_path("frame_rail_l.obj")), material=metal_dark)
    chassis.visual(mesh_from_geometry(tube_from_spline_points(rail_points_r, radius=0.03), ASSETS.mesh_path("frame_rail_r.obj")), material=metal_dark)

    # 2. Body
    # Main hull - lengthened to connect to axles (wheelbase/2 = 0.55)
    chassis.visual(Box((1.25, W_body, 0.4)), origin=Origin(xyz=(0.0, 0.0, 0.35)), material=plastic_red)
    # Fenders - slightly taller for better silhouette and connectivity
    chassis.visual(Box((0.45, 1.1, 0.08)), origin=Origin(xyz=(0.5, 0.0, 0.54)), material=plastic_red)
    chassis.visual(Box((0.65, 1.1, 0.08)), origin=Origin(xyz=(-0.4, 0.0, 0.54)), material=plastic_red)
    # Seat
    chassis.visual(Box((0.55, 0.3, 0.15)), origin=Origin(xyz=(-0.1, 0.0, 0.53)), material=seat_material)
    # Engine
    chassis.visual(Box((0.45, 0.35, 0.35)), origin=Origin(xyz=(0.1, 0.0, 0.2)), material=metal_dark)
    # Footrests - moved in for solid connectivity
    chassis.visual(Box((0.2, 0.4, 0.02)), origin=Origin(xyz=(0.1, 0.2, 0.17)), material=metal_dark)
    chassis.visual(Box((0.2, 0.4, 0.02)), origin=Origin(xyz=(0.1, -0.2, 0.17)), material=metal_dark)

    # Steering mount
    chassis.visual(Box((0.1, 0.1, 0.25)), origin=Origin(xyz=(0.4, 0.0, 0.48)), material=metal_dark)

    # Front Bumper
    chassis.visual(Box((0.1, 0.6, 0.3)), origin=Origin(xyz=(0.7, 0.0, 0.35)), material=metal_dark)

    # Axles / Suspension
    # Rear Axle - slightly longer to ensure contact with wheels
    chassis.visual(Cylinder(radius=0.04, length=track + 0.1), origin=Origin(xyz=(-wheelbase/2, 0, wheel_radius), rpy=(1.5708, 0, 0)), material=metal_dark)
    # Front suspension arms - extended to reach the hub joint (y=0.45)
    for y_sign in [1, -1]:
        chassis.visual(Box((0.1, 0.32, 0.04)), origin=Origin(xyz=(wheelbase/2, y_sign * 0.3, wheel_radius)), material=metal_dark)

    chassis.inertial = Inertial.from_geometry(Box((1.5, 0.8, 0.6)), mass=150.0, origin=Origin(xyz=(0.0, 0.0, 0.3)))

    # 3. Steering
    steering = model.part("steering")
    steering.visual(Cylinder(radius=0.02, length=0.4), origin=Origin(xyz=(0.0, 0.0, -0.2), rpy=(0, 0.4, 0)), material=metal_dark)
    hb_points = [(0.0, -0.3, 0.0), (0.0, -0.1, 0.05), (0.0, 0.1, 0.05), (0.0, 0.3, 0.0)]
    steering.visual(mesh_from_geometry(tube_from_spline_points(hb_points, radius=0.02), ASSETS.mesh_path("handlebars.obj")), material=chrome)
    steering.inertial = Inertial.from_geometry(Box((0.1, 0.6, 0.4)), mass=5.0)

    model.articulation(
        "chassis_to_steering",
        ArticulationType.REVOLUTE,
        parent="chassis",
        child="steering",
        origin=Origin(xyz=(0.4, 0.0, 0.6), rpy=(0, -0.4, 0)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=10, velocity=2, lower=-0.6, upper=0.6)
    )

    # 4. Wheels
    def add_wheel(name, x, y, z, parent="chassis"):
        wheel = model.part(name)
        wheel.visual(Cylinder(radius=wheel_radius, length=wheel_width), origin=Origin(rpy=(1.5708, 0, 0)), material=rubber_black)
        wheel.visual(Cylinder(radius=wheel_radius*0.6, length=wheel_width+0.01), origin=Origin(rpy=(1.5708, 0, 0)), material=chrome)
        wheel.inertial = Inertial.from_geometry(Cylinder(radius=wheel_radius, length=wheel_width), mass=10.0)
        model.articulation(f"{parent}_to_{name}", ArticulationType.CONTINUOUS, parent=parent, child=name, origin=Origin(xyz=(x, y, z)), axis=(0, 1, 0), motion_limits=MotionLimits(effort=50, velocity=20))

    add_wheel("rl_wheel", -wheelbase/2, track/2, z=wheel_radius)
    add_wheel("rr_wheel", -wheelbase/2, -track/2, z=wheel_radius)

    for side, y_sign in [("l", 1), ("r", -1)]:
        hub_name = f"front_{side}_hub"
        hub = model.part(hub_name)
        hub.visual(Sphere(radius=0.05), material=metal_dark)
        # Add a steering arm to the hub so its AABB center moves when rotated
        hub.visual(Box((0.1, 0.05, 0.05)), origin=Origin(xyz=(0.05, 0, 0)), material=metal_dark)
        hub.inertial = Inertial.from_geometry(Sphere(radius=0.05), mass=2.0)
        model.articulation(f"chassis_to_{hub_name}", ArticulationType.REVOLUTE, parent="chassis", child=hub_name, origin=Origin(xyz=(wheelbase/2, y_sign * track/2, wheel_radius)), axis=(0, 0, 1), motion_limits=MotionLimits(effort=10, velocity=2, lower=-0.5, upper=0.5))
        add_wheel(f"f{side}_wheel", 0, 0, z=0, parent=hub_name)

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check_articulation_origin_near_geometry(tol=0.01)
    ctx.check_part_geometry_connected(use="visual")

    ctx.allow_overlap("chassis", "steering", reason="steering mount")
    ctx.allow_overlap("front_l_hub", "fl_wheel", reason="hub mount")
    ctx.allow_overlap("front_r_hub", "fr_wheel", reason="hub mount")
    ctx.allow_overlap("chassis", "front_l_hub", reason="suspension mount")
    ctx.allow_overlap("chassis", "front_r_hub", reason="suspension mount")
    # Allow chassis-wheel overlap for A-arms inside rim
    ctx.allow_overlap("chassis", "fl_wheel", reason="A-arm inside rim")
    ctx.allow_overlap("chassis", "fr_wheel", reason="A-arm inside rim")

    ctx.check_no_overlaps(max_pose_samples=16, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_aabb_gap("rl_wheel", "rr_wheel", axis="y", min_gap=0.4)
    ctx.expect_aabb_gap("fl_wheel", "rl_wheel", axis="x", min_gap=0.5)

    # Steering motion check
    with ctx.pose(chassis_to_front_l_hub=0.4):
        # The hub AABB center should move in Y (CCW rotation of arm at +X should move center in +Y)
        ctx.expect_joint_motion_axis("chassis_to_front_l_hub", "front_l_hub", world_axis="y", direction="positive")

    return ctx.report()

object_model = build_object_model()
# >>> USER_CODE_END

object_model = build_object_model()
