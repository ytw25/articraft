from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
import cadquery as cq

from sdk_hybrid import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BODY_SIZE = 0.056
BODY_HALF = BODY_SIZE / 2.0
BODY_CHAMFER = 0.0032
WALL = 0.0042
TOP_SKIN = 0.007
TOP_PANEL_SIZE = 0.034
TOP_PANEL_T = 0.0022
TOP_SOCKET_WELL = 0.024

SWITCH_RECESS_D = 0.0016
SWITCH_PANEL_W = 0.044
SWITCH_PANEL_H = 0.037
TRACK_HEIGHT = 0.028
TRACK_CENTER_Z = 0.004
SLIDER_TRAVEL = 0.012
SLIDER_ORIGIN_Y = BODY_HALF - SWITCH_RECESS_D / 2.0
SLIDER_ORIGIN_Z = 0.010
SWITCH_XS = (-0.014, 0.0, 0.014)

USB_PANEL_W = 0.034
USB_PANEL_H = 0.024
USB_PANEL_T = 0.0016
USB_RECESS_D = 0.0018
USB_PANEL_Z = -0.001


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(
        shape,
        filename,
        assets=ASSETS,
        tolerance=0.0005,
        angular_tolerance=0.08,
    )


def _body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BODY_SIZE, BODY_SIZE, BODY_SIZE)
    body = body.edges().chamfer(BODY_CHAMFER)

    main_cavity = cq.Workplane("XY").box(
        BODY_SIZE - 2.0 * WALL,
        BODY_SIZE - 2.0 * WALL,
        BODY_SIZE - TOP_SKIN + 0.0008,
    ).translate((0.0, 0.0, -TOP_SKIN / 2.0 - 0.0004))
    body = body.cut(main_cavity)

    top_panel_pocket = cq.Workplane("XY").box(
        TOP_PANEL_SIZE + 0.0008,
        TOP_PANEL_SIZE + 0.0008,
        TOP_PANEL_T + 0.0004,
    ).translate((0.0, 0.0, BODY_HALF - (TOP_PANEL_T + 0.0004) / 2.0))
    body = body.cut(top_panel_pocket)

    socket_well = cq.Workplane("XY").box(
        TOP_SOCKET_WELL,
        TOP_SOCKET_WELL,
        0.0048,
    ).translate((0.0, 0.0, BODY_HALF - TOP_PANEL_T - 0.0024))
    body = body.cut(socket_well)

    switch_pocket = cq.Workplane("XY").box(
        SWITCH_PANEL_W,
        SWITCH_RECESS_D,
        SWITCH_PANEL_H,
    ).translate((0.0, BODY_HALF - SWITCH_RECESS_D / 2.0, 0.001))
    body = body.cut(switch_pocket)

    for x_pos, width in zip(SWITCH_XS, (0.0092, 0.0118, 0.0105)):
        track = cq.Workplane("XY").box(
            width + 0.0012,
            SWITCH_RECESS_D + 0.0024,
            TRACK_HEIGHT,
        ).translate((x_pos, BODY_HALF - (SWITCH_RECESS_D + 0.0024) / 2.0, TRACK_CENTER_Z))
        finger_relief = cq.Workplane("XY").box(
            width + 0.002,
            SWITCH_RECESS_D + 0.003,
            0.008,
        ).translate((x_pos, BODY_HALF - (SWITCH_RECESS_D + 0.003) / 2.0, -0.010))
        body = body.cut(track).cut(finger_relief)

    usb_pocket = cq.Workplane("XY").box(
        USB_PANEL_W,
        USB_RECESS_D,
        USB_PANEL_H,
    ).translate((0.0, -BODY_HALF + USB_RECESS_D / 2.0, USB_PANEL_Z))
    body = body.cut(usb_pocket)

    usb_back_cavity = cq.Workplane("XY").box(
        0.028,
        0.010,
        0.020,
    ).translate((0.0, -BODY_HALF + 0.005, USB_PANEL_Z - 0.001))
    body = body.cut(usb_back_cavity)

    return body


def _top_socket_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(TOP_PANEL_SIZE, TOP_PANEL_SIZE, TOP_PANEL_T)
    panel = panel.edges("|Z").fillet(0.00065)

    socket_dish = cq.Workplane("XY").box(
        0.026,
        0.026,
        0.0007,
    ).translate((0.0, 0.0, TOP_PANEL_T / 2.0 - 0.00035))
    panel = panel.cut(socket_dish)

    earth_slot = cq.Workplane("XY").box(0.0034, 0.0013, TOP_PANEL_T + 0.003)
    earth_slot = earth_slot.translate((0.0, 0.0075, 0.0))

    angled_left = cq.Workplane("XY").box(0.0013, 0.0068, TOP_PANEL_T + 0.003)
    angled_left = angled_left.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), -24.0)
    angled_left = angled_left.translate((-0.0054, 0.0016, 0.0))

    angled_right = cq.Workplane("XY").box(0.0013, 0.0068, TOP_PANEL_T + 0.003)
    angled_right = angled_right.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 24.0)
    angled_right = angled_right.translate((0.0054, 0.0016, 0.0))

    lower_left = cq.Workplane("XY").cylinder(TOP_PANEL_T + 0.003, 0.0015)
    lower_left = lower_left.translate((-0.0052, -0.0082, 0.0))
    lower_right = cq.Workplane("XY").cylinder(TOP_PANEL_T + 0.003, 0.0015)
    lower_right = lower_right.translate((0.0052, -0.0082, 0.0))

    straight_left = cq.Workplane("XY").box(0.0012, 0.0062, TOP_PANEL_T + 0.003)
    straight_left = straight_left.translate((-0.0078, -0.0015, 0.0))
    straight_right = cq.Workplane("XY").box(0.0012, 0.0062, TOP_PANEL_T + 0.003)
    straight_right = straight_right.translate((0.0078, -0.0015, 0.0))

    return (
        panel.cut(earth_slot)
        .cut(angled_left)
        .cut(angled_right)
        .cut(lower_left)
        .cut(lower_right)
        .cut(straight_left)
        .cut(straight_right)
    )


def _usb_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(USB_PANEL_W, USB_PANEL_T, USB_PANEL_H)
    panel = panel.edges("|Y").fillet(0.00035)

    for x_pos in (-0.0082, 0.0082):
        usb_c = cq.Workplane("XY").box(0.0035, USB_PANEL_T + 0.002, 0.0068)
        usb_c = usb_c.translate((x_pos, 0.0, 0.0065))
        panel = panel.cut(usb_c)

    for x_pos in (-0.0088, 0.0088):
        usb_a = cq.Workplane("XY").box(0.0078, USB_PANEL_T + 0.002, 0.0031)
        usb_a = usb_a.translate((x_pos, 0.0, -0.0036))
        panel = panel.cut(usb_a)

    for label, x_pos in (("C1 5V/3A", -0.0082), ("C2 5V/3A", 0.0082)):
        text = cq.Workplane("XZ").text(label, 0.00195, 0.00018, combine=False)
        text = text.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 180.0)
        text = text.translate((x_pos, -USB_PANEL_T / 2.0 + 0.00005, 0.011))
        panel = panel.cut(text)

    max_text = cq.Workplane("XZ").text("3.4A MAX", 0.00215, 0.00018, combine=False)
    max_text = max_text.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 180.0)
    max_text = max_text.translate((0.0, -USB_PANEL_T / 2.0 + 0.00005, -0.0112))
    panel = panel.cut(max_text)

    return panel


def _linkage_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(0.028, 0.003, 0.003)
    frame = frame.translate((0.0, 0.004, -0.017))

    for x_pos in SWITCH_XS:
        rail = cq.Workplane("XY").box(0.0025, 0.018, 0.024)
        rail = rail.translate((x_pos, 0.004, -0.004))
        frame = frame.union(rail)

    crossbar = cq.Workplane("XY").box(0.034, 0.004, 0.003)
    crossbar = crossbar.translate((0.0, -0.008, -0.023))
    frame = frame.union(crossbar)

    mount_bridge = cq.Workplane("XY").box(0.034, 0.003, 0.004)
    mount_bridge = mount_bridge.translate((0.0, 0.0212, 0.007))
    frame = frame.union(mount_bridge)

    for x_pos in SWITCH_XS:
        tab = cq.Workplane("XY").box(0.0045, 0.011, 0.003)
        tab = tab.translate((x_pos, 0.0155, 0.006))
        frame = frame.union(tab)
    return frame.translate((0.0, -0.0212, -0.007))


def _button_shape(label: str, width: float, text_size: float) -> cq.Workplane:
    button = cq.Workplane("XY").box(width, 0.0044, 0.0062)
    button = button.translate((0.0, 0.0018, 0.0))
    button = button.edges("|Z").fillet(0.0009)

    label_shape = cq.Workplane("XZ").text(label, text_size, 0.00024, combine=False)
    label_shape = label_shape.translate((0.0, 0.00395, -0.0003))
    return button.union(label_shape)


def _uk_mechanism_shape() -> cq.Workplane:
    rod = cq.Workplane("XY").box(0.0024, 0.022, 0.024).translate((0.0, -0.0095, -0.012))
    brace = cq.Workplane("XY").box(0.006, 0.003, 0.009).translate((0.0, -0.008, -0.015))
    carriage = cq.Workplane("XY").box(0.012, 0.008, 0.0034).translate((0.0, -0.009, -0.023))

    mech = rod.union(brace).union(carriage)
    for x_pos, y_pos, height in ((-0.0041, -0.0105, 0.008), (0.0041, -0.0105, 0.008), (0.0, -0.0056, 0.0095)):
        prong = cq.Workplane("XY").box(0.0017, 0.0023, height)
        prong = prong.translate((x_pos, y_pos, -0.028))
        mech = mech.union(prong)
    return mech


def _usa_aus_mechanism_shape() -> cq.Workplane:
    rod = cq.Workplane("XY").box(0.0024, 0.022, 0.024).translate((0.0, -0.0095, -0.012))
    slider = cq.Workplane("XY").box(0.010, 0.008, 0.0034).translate((0.0, -0.009, -0.023))
    mech = rod.union(slider)

    for x_pos, angle in ((-0.0036, -18.0), (0.0036, 18.0)):
        blade = cq.Workplane("XY").box(0.0015, 0.0042, 0.010)
        blade = blade.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        blade = blade.translate((x_pos, -0.0088, -0.028))
        mech = mech.union(blade)
    return mech


def _euro_mechanism_shape() -> cq.Workplane:
    rod = cq.Workplane("XY").box(0.0022, 0.022, 0.024).translate((0.0, -0.0095, -0.012))
    slider = cq.Workplane("XY").box(0.010, 0.008, 0.0034).translate((0.0, -0.009, -0.023))
    mech = rod.union(slider)

    for x_pos in (-0.0036, 0.0036):
        pin = cq.Workplane("XY").cylinder(0.010, 0.00115)
        pin = pin.translate((x_pos, -0.0088, -0.029))
        mech = mech.union(pin)
    return mech


def _vec3(value) -> tuple[float, float, float]:
    if hasattr(value, "toTuple"):
        return tuple(float(v) for v in value.toTuple())
    if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
        return (float(value.x), float(value.y), float(value.z))
    if isinstance(value, (tuple, list)) and len(value) == 3:
        return tuple(float(v) for v in value)
    raise TypeError(f"Unsupported vector format: {value!r}")


def _aabb_bounds(aabb) -> dict[str, float]:
    direct_names = ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax")
    if all(hasattr(aabb, name) for name in direct_names):
        return {name: float(getattr(aabb, name)) for name in direct_names}

    alt_names = ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")
    if all(hasattr(aabb, name) for name in alt_names):
        values = {name: float(getattr(aabb, name)) for name in alt_names}
        return {
            "xmin": values["min_x"],
            "xmax": values["max_x"],
            "ymin": values["min_y"],
            "ymax": values["max_y"],
            "zmin": values["min_z"],
            "zmax": values["max_z"],
        }

    if hasattr(aabb, "lower") and hasattr(aabb, "upper"):
        lower = _vec3(aabb.lower)
        upper = _vec3(aabb.upper)
        return {
            "xmin": lower[0],
            "xmax": upper[0],
            "ymin": lower[1],
            "ymax": upper[1],
            "zmin": lower[2],
            "zmax": upper[2],
        }

    if isinstance(aabb, dict):
        if all(name in aabb for name in direct_names):
            return {name: float(aabb[name]) for name in direct_names}

    if isinstance(aabb, (tuple, list)):
        if len(aabb) == 2:
            lower = _vec3(aabb[0])
            upper = _vec3(aabb[1])
            return {
                "xmin": lower[0],
                "xmax": upper[0],
                "ymin": lower[1],
                "ymax": upper[1],
                "zmin": lower[2],
                "zmax": upper[2],
            }
        if len(aabb) == 6:
            return {
                "xmin": float(aabb[0]),
                "xmax": float(aabb[1]),
                "ymin": float(aabb[2]),
                "ymax": float(aabb[3]),
                "zmin": float(aabb[4]),
                "zmax": float(aabb[5]),
            }

    raise TypeError(f"Unsupported AABB format: {aabb!r}")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="universal_travel_adapter", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.11, 0.11, 0.12, 1.0))
    thumb_black = model.material("thumb_black", rgba=(0.06, 0.06, 0.07, 1.0))
    metallic_grey = model.material("metallic_grey", rgba=(0.57, 0.59, 0.62, 1.0))
    plated_steel = model.material("plated_steel", rgba=(0.76, 0.77, 0.79, 1.0))
    led_blue = model.material("led_blue", rgba=(0.18, 0.56, 1.0, 0.9))

    body = model.part("body")
    body.visual(_mesh(_body_shape(), "body_shell.obj"), material=matte_black)
    body.inertial = Inertial.from_geometry(Box((BODY_SIZE, BODY_SIZE, BODY_SIZE)), mass=0.18)

    top_socket = model.part("top_socket")
    top_socket.visual(_mesh(_top_socket_shape(), "top_socket.obj"), material=metallic_grey)
    top_socket.inertial = Inertial.from_geometry(
        Box((TOP_PANEL_SIZE, TOP_PANEL_SIZE, TOP_PANEL_T)),
        mass=0.01,
    )

    usb_panel = model.part("usb_panel")
    usb_panel.visual(_mesh(_usb_panel_shape(), "usb_panel.obj"), material=metallic_grey)
    usb_panel.inertial = Inertial.from_geometry(
        Box((USB_PANEL_W, USB_PANEL_T, USB_PANEL_H)),
        mass=0.008,
    )

    linkage_frame = model.part("linkage_frame")
    linkage_frame.visual(_mesh(_linkage_frame_shape(), "linkage_frame.obj"), material=metallic_grey)
    linkage_frame.inertial = Inertial.from_geometry(
        Box((0.036, 0.02, 0.028)),
        mass=0.01,
    )

    led_bar = model.part("led_bar")
    led_bar.visual(Box((0.014, 0.0008, 0.0018)), material=led_blue)
    led_bar.inertial = Inertial.from_geometry(Box((0.014, 0.0008, 0.0018)), mass=0.001)

    uk_cartridge = model.part("uk_cartridge")
    uk_cartridge.visual(_mesh(_button_shape("UK", 0.0092, 0.0034), "uk_button.obj"), material=thumb_black)
    uk_cartridge.visual(_mesh(_uk_mechanism_shape(), "uk_mechanism.obj"), material=plated_steel)
    uk_cartridge.inertial = Inertial.from_geometry(Box((0.013, 0.027, 0.038)), mass=0.012)

    usa_aus_cartridge = model.part("usa_aus_cartridge")
    usa_aus_cartridge.visual(
        _mesh(_button_shape("USA/AUS", 0.012, 0.00185), "usa_aus_button.obj"),
        material=thumb_black,
    )
    usa_aus_cartridge.visual(_mesh(_usa_aus_mechanism_shape(), "usa_aus_mechanism.obj"), material=plated_steel)
    usa_aus_cartridge.inertial = Inertial.from_geometry(Box((0.014, 0.027, 0.038)), mass=0.012)

    euro_cartridge = model.part("euro_cartridge")
    euro_cartridge.visual(
        _mesh(_button_shape("EURO", 0.0105, 0.00235), "euro_button.obj"),
        material=thumb_black,
    )
    euro_cartridge.visual(_mesh(_euro_mechanism_shape(), "euro_mechanism.obj"), material=plated_steel)
    euro_cartridge.inertial = Inertial.from_geometry(Box((0.013, 0.027, 0.038)), mass=0.012)

    model.articulation(
        "body_to_top_socket",
        ArticulationType.FIXED,
        parent="body",
        child="top_socket",
        origin=Origin(xyz=(0.0, 0.0, BODY_HALF - TOP_PANEL_T / 2.0)),
    )
    model.articulation(
        "body_to_usb_panel",
        ArticulationType.FIXED,
        parent="body",
        child="usb_panel",
        origin=Origin(xyz=(0.0, -BODY_HALF + USB_PANEL_T / 2.0, USB_PANEL_Z)),
    )
    model.articulation(
        "body_to_linkage_frame",
        ArticulationType.FIXED,
        parent="body",
        child="linkage_frame",
        origin=Origin(xyz=(0.0, 0.0212, 0.007)),
    )
    model.articulation(
        "body_to_led_bar",
        ArticulationType.FIXED,
        parent="body",
        child="led_bar",
        origin=Origin(xyz=(0.0, -BODY_HALF + 0.0004, -0.0092)),
    )

    motion = MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=SLIDER_TRAVEL)
    model.articulation(
        "uk_deploy",
        ArticulationType.PRISMATIC,
        parent="body",
        child="uk_cartridge",
        origin=Origin(xyz=(SWITCH_XS[0], SLIDER_ORIGIN_Y, SLIDER_ORIGIN_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=motion,
    )
    model.articulation(
        "usa_aus_deploy",
        ArticulationType.PRISMATIC,
        parent="body",
        child="usa_aus_cartridge",
        origin=Origin(xyz=(SWITCH_XS[1], SLIDER_ORIGIN_Y, SLIDER_ORIGIN_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=SLIDER_TRAVEL),
    )
    model.articulation(
        "euro_deploy",
        ArticulationType.PRISMATIC,
        parent="body",
        child="euro_cartridge",
        origin=Origin(xyz=(SWITCH_XS[2], SLIDER_ORIGIN_Y, SLIDER_ORIGIN_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=SLIDER_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE, geometry_source="collision")
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_connected(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("top_socket", "body", axes="xy", min_overlap=0.025)
    ctx.expect_aabb_overlap("usb_panel", "body", axes="xz", min_overlap=0.019)
    ctx.expect_aabb_overlap("led_bar", "usb_panel", axes="xz", min_overlap=0.0012)

    for joint_name, part_name in (
        ("uk_deploy", "uk_cartridge"),
        ("usa_aus_deploy", "usa_aus_cartridge"),
        ("euro_deploy", "euro_cartridge"),
    ):
        ctx.expect_joint_motion_axis(joint_name, part_name, world_axis="z", direction="negative", min_delta=0.004)

    body_bounds = _aabb_bounds(ctx.part_world_aabb("body", use="visual"))
    body_x = body_bounds["xmax"] - body_bounds["xmin"]
    body_y = body_bounds["ymax"] - body_bounds["ymin"]
    body_z = body_bounds["zmax"] - body_bounds["zmin"]
    if not (0.05 <= body_x <= 0.062 and 0.05 <= body_y <= 0.062 and 0.05 <= body_z <= 0.062):
        raise AssertionError("Adapter body should read as a compact cube.")

    top_bounds = _aabb_bounds(ctx.part_world_aabb("top_socket", use="visual"))
    if top_bounds["zmax"] < body_bounds["zmax"] - 0.003:
        raise AssertionError("Universal AC socket panel should sit on the top face.")
    if top_bounds["zmin"] < body_bounds["zmax"] - 0.008:
        raise AssertionError("Universal AC socket panel should read as a shallow top insert, not a buried block.")

    usb_bounds = _aabb_bounds(ctx.part_world_aabb("usb_panel", use="visual"))
    if usb_bounds["ymin"] > body_bounds["ymin"] + 0.0015:
        raise AssertionError("USB charging panel should be mounted on a side face.")
    if usb_bounds["ymax"] > body_bounds["ymin"] + 0.0045:
        raise AssertionError("USB charging panel recess should stay shallow rather than sinking deep into the body.")

    led_bounds = _aabb_bounds(ctx.part_world_aabb("led_bar", use="visual"))
    if led_bounds["ymin"] > usb_bounds["ymin"] + 0.0004:
        raise AssertionError("LED indicator bar should sit near the outer face of the USB panel.")
    if led_bounds["ymax"] > usb_bounds["ymax"] + 0.0002:
        raise AssertionError("LED indicator bar should remain seated within the USB panel recess.")
    if led_bounds["xmax"] - led_bounds["xmin"] < 0.012:
        raise AssertionError("LED indicator should read as a horizontal light bar, not a point indicator.")

    linkage_bounds = _aabb_bounds(ctx.part_world_aabb("linkage_frame", use="visual"))
    if linkage_bounds["zmin"] > body_bounds["zmin"] + 0.01:
        raise AssertionError("Visible internal linkage structure should hang into the lower cavity.")
    if linkage_bounds["ymax"] < body_bounds["ymax"] - 0.015:
        raise AssertionError("Internal linkage frame should visibly reach toward the side slider tracks.")

    slider_centers_x = []
    for part_name in ("uk_cartridge", "usa_aus_cartridge", "euro_cartridge"):
        bounds = _aabb_bounds(ctx.part_world_aabb(part_name, use="visual"))
        slider_centers_x.append((bounds["xmin"] + bounds["xmax"]) / 2.0)
        if bounds["ymax"] < body_bounds["ymax"] + 0.0005:
            raise AssertionError(f"{part_name} should present a thumb switch proud of the side wall.")
        if bounds["zmin"] < body_bounds["zmin"] - 0.0015:
            raise AssertionError(f"{part_name} should start mostly retracted in the body.")

    if max(slider_centers_x) - min(slider_centers_x) < 0.02:
        raise AssertionError("Three distinct side slider tracks should span the switch face.")

    with ctx.pose(uk_deploy=SLIDER_TRAVEL):
        bounds = _aabb_bounds(ctx.part_world_aabb("uk_cartridge", use="visual"))
        if bounds["zmin"] > body_bounds["zmin"] - 0.004:
            raise AssertionError("UK cartridge should deploy male prongs below the bottom face.")

    with ctx.pose(usa_aus_deploy=SLIDER_TRAVEL):
        bounds = _aabb_bounds(ctx.part_world_aabb("usa_aus_cartridge", use="visual"))
        if bounds["zmin"] > body_bounds["zmin"] - 0.004:
            raise AssertionError("USA/AUS cartridge should deploy male prongs below the bottom face.")

    with ctx.pose(euro_deploy=SLIDER_TRAVEL):
        bounds = _aabb_bounds(ctx.part_world_aabb("euro_cartridge", use="visual"))
        if bounds["zmin"] > body_bounds["zmin"] - 0.004:
            raise AssertionError("EURO cartridge should deploy male prongs below the bottom face.")

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
