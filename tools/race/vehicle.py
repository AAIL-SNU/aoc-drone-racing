"""The machine itself: airframe, spinning propellers, path ribbon, animation.

``build_drone`` is the seam to swap in a downloaded model: anything that returns
a root empty with uniform scale plus four spinnable prop hubs drops straight in.
"""

import math
from pathlib import Path
import re

import bpy
from mathutils import Matrix, Vector

from .scene import add_box, glowing, metal, solid

HOVER_THRUST_N = 0.5 * 9.81  # nominal mass x g; the props idle here
DRONE_COLORS = ((0.14, 0.45, 0.55), (0.80, 0.33, 0.18),
                (0.36, 0.60, 0.33), (0.55, 0.40, 0.70))


def build_drone(name, color, scale):
    """A 5-inch FPV racer at RotorPy's Hummingbird scale: 0.17 m arms, 0.10 m props.

    The root is an empty carrying only a uniform scale. Parenting the parts to a
    body cube instead would multiply them by that cube's half-extents, which are
    not uniform, and collapse the rest of the airframe.

    Returns the root and the four propeller hubs, which spin independently.
    """
    root = bpy.data.objects.new(name, None)
    root.empty_display_size = 0.12
    root.scale = (scale, scale, scale)
    bpy.context.scene.collection.objects.link(root)

    def attach(child, parent=root):
        child.parent = parent
        child.matrix_parent_inverse = Matrix.Identity(4)
        return child

    carbon = solid(f"{name}_carbon", (0.050, 0.052, 0.062), roughness=0.34)
    aluminium = metal(f"{name}_alu", (0.62, 0.63, 0.66), roughness=0.22)
    livery = solid(f"{name}_livery", color, roughness=0.25)
    blade = solid(f"{name}_blade", (0.58, 0.60, 0.66), roughness=0.35)
    led = glowing(f"{name}_led", color, strength=1.6)

    # Stacked carbon plates with the electronics sandwiched between them.
    attach(add_box(f"{name}_plate_bottom", (-0.062, -0.028, -0.006), (0.062, 0.028, -0.0025), carbon))
    attach(add_box(f"{name}_plate_top", (-0.050, -0.026, 0.0235), (0.050, 0.026, 0.0265), carbon))
    # A lit strip along the spine; from a chase camera this is what picks the
    # machine out against a dark floor.
    attach(add_box(f"{name}_spine", (-0.044, -0.005, 0.0485), (0.026, 0.005, 0.0505),
                   glowing(f"{name}_spine", color, strength=2.4)))
    attach(add_box(f"{name}_stack", (-0.020, -0.020, -0.002), (0.020, 0.020, 0.022), livery))
    attach(add_box(f"{name}_battery", (-0.040, -0.018, 0.0265), (0.030, 0.018, 0.0485), livery))

    # Camera pod up front; this is what an FPV feed would actually look through.
    attach(add_box(f"{name}_pod", (0.030, -0.014, 0.018), (0.054, 0.014, 0.042), carbon))
    attach(add_box(f"{name}_lens", (0.054, -0.009, 0.023), (0.060, 0.009, 0.037),
                   glowing(f"{name}_lens", (0.08, 0.10, 0.16), strength=0.5)))

    span = 0.17
    offset = span / math.sqrt(2.0)
    props = []
    for index, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        yaw = math.atan2(sy, sx)
        # Arm, motor mount, bell motor, then the prop hub the blades hang off.
        attach(add_box(f"{name}_arm_{index}", (0.020, -0.009, -0.0035), (span, 0.009, 0.0035),
                       carbon, yaw=yaw))
        attach(add_box(f"{name}_led_{index}", (0.055, -0.006, -0.0075), (span - 0.012, 0.006, -0.004),
                       led, yaw=yaw))
        bpy.ops.mesh.primitive_cylinder_add(radius=0.0145, depth=0.020, vertices=20)
        motor = bpy.context.object
        motor.name = f"{name}_motor_{index}"
        motor.data.materials.append(aluminium)
        attach(motor)
        motor.location = (sx * offset, sy * offset, 0.0135)

        hub = bpy.data.objects.new(f"{name}_prop_{index}", None)
        hub.empty_display_size = 0.02
        bpy.context.scene.collection.objects.link(hub)
        attach(hub)
        hub.location = (sx * offset, sy * offset, 0.0265)
        hub.rotation_mode = "XYZ"
        for side in range(2):
            attach(add_box(f"{name}_blade_{index}_{side}",
                           (0.010, -0.0135, -0.0012), (0.098, 0.0135, 0.0012),
                           blade, yaw=side * math.pi), hub)
        props.append(hub)

    return root, props


def animate_props(props, samples, fps):
    """Spin the blades fast enough that motion blur smears them into real discs.

    Rate follows the logged thrust, so the props visibly spool up into a climb and
    ease off in a dive. Adjacent rotors turn opposite ways, as they must. 900 rad/s
    at hover is roughly 8600 RPM, which is where a 0.5 kg machine on 5-inch props
    actually sits, and it is fast enough that the shutter never catches a blade.
    """
    angle = 0.0
    directions = (1.0, -1.0, -1.0, 1.0)
    for index, sample in enumerate(samples):
        action = sample.get("action")
        thrust = action["thrust_N"] if action else HOVER_THRUST_N
        angle += 900.0 * math.sqrt(max(thrust, 0.0) / HOVER_THRUST_N + 0.08) / fps
        for direction, hub in zip(directions, props):
            hub.rotation_euler = (0.0, 0.0, direction * angle)
            hub.keyframe_insert("rotation_euler", frame=index + 1)
    for hub in props:
        for curve in hub.animation_data.action.fcurves:
            for point in curve.keyframe_points:
                point.interpolation = "LINEAR"


def animate(drone, samples):
    """One keyframe per sample: the log's 50 Hz is the render's 50 fps."""
    drone.rotation_mode = "QUATERNION"
    previous = None
    for index, sample in enumerate(samples):
        x, y, z, w = sample["quaternion_xyzw"]
        quaternion = Vector((w, x, y, z))
        # Consecutive log quaternions may flip sign; keep the short way around.
        if previous is not None and quaternion.dot(previous) < 0.0:
            quaternion = -quaternion
        previous = quaternion
        drone.location = sample["position_m"]
        drone.rotation_quaternion = quaternion
        drone.keyframe_insert("location", frame=index + 1)
        drone.keyframe_insert("rotation_quaternion", frame=index + 1)
    for curve in drone.animation_data.action.fcurves:
        for point in curve.keyframe_points:
            point.interpolation = "LINEAR"


def build_trail(name, samples, color):
    """A tube along the flown path that grows with the vehicle."""
    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.bevel_depth = 0.013
    curve.bevel_resolution = 0
    spline = curve.splines.new("POLY")
    spline.points.add(len(samples) - 1)
    for point, sample in zip(spline.points, samples):
        point.co = (*sample["position_m"], 1.0)
    # RESOLUTION mapping walks the poly spline one control point at a time, and we
    # have exactly one control point per frame, so the tip tracks the vehicle.
    curve.bevel_factor_mapping_end = "RESOLUTION"
    trail = bpy.data.objects.new(name, curve)
    trail.data.materials.append(glowing(name, color, strength=1.3))
    bpy.context.scene.collection.objects.link(trail)
    for index in range(len(samples)):
        curve.bevel_factor_end = index / max(len(samples) - 1, 1)
        curve.keyframe_insert("bevel_factor_end", frame=index + 1)
    for curve_data in curve.animation_data.action.fcurves:
        for point in curve_data.keyframe_points:
            point.interpolation = "LINEAR"
    return trail


CONTACT_RADIUS_M = 0.40      # the patch at touchdown
CONTACT_SPREAD = 0.11        # how much wider it gets per metre of altitude
CONTACT_DARKNESS = 0.75      # opacity directly under the vehicle
CONTACT_FADE_M = 5.0         # altitude at which it has faded to 1/e


def build_contact_shadow(name, samples):
    """A soft dark patch under the vehicle, tracking it along the ground.

    Not a sun shadow. The HDRI this renders under is a sunrise whose sun sits a
    tenth of a degree above the horizon, so the real shadow is tens of metres
    long and horizontal - it is never under the vehicle, and a second sun hung
    somewhere more convenient would light the course from a direction the sky
    visibly contradicts. What *is* under the vehicle is the sky it blocks: the
    ground there sees less of a very bright dome. That is what this draws, and
    it is why it sits directly below rather than off to one side.

    It widens and fades with altitude, the way an occlusion patch does.
    """
    bpy.ops.mesh.primitive_plane_add(size=2.0)
    patch = bpy.context.object
    patch.name = name
    patch.visible_shadow = False
    patch.data.materials.append(_contact_material(name))

    for index, sample in enumerate(samples):
        x, y, z = sample["position_m"]
        height = max(z, 0.0)
        radius = CONTACT_RADIUS_M + CONTACT_SPREAD * height
        patch.location = (x, y, 0.02)
        patch.scale = (radius, radius, 1.0)
        patch.color = (1.0, 1.0, 1.0,
                       CONTACT_DARKNESS * math.exp(-height / CONTACT_FADE_M))
        frame = index + 1
        patch.keyframe_insert("location", frame=frame)
        patch.keyframe_insert("scale", frame=frame)
        patch.keyframe_insert("color", frame=frame)
    for fcurve in patch.animation_data.action.fcurves:
        for point in fcurve.keyframe_points:
            point.interpolation = "LINEAR"
    return patch


def _contact_material(name):
    """Black, transparent, softest at the rim. Opacity comes from object colour."""
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    tree = material.node_tree
    bsdf = tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (0.0, 0.0, 0.0, 1.0)
    bsdf.inputs["Roughness"].default_value = 1.0

    coordinates = tree.nodes.new("ShaderNodeTexCoord")
    centre = tree.nodes.new("ShaderNodeMapping")
    # Generated runs 0..1 over the plane; scale first, then shift, so the centre
    # of the patch lands on the centre of the gradient.
    centre.inputs["Location"].default_value = (-1.0, -1.0, 0.0)
    centre.inputs["Scale"].default_value = (2.0, 2.0, 0.0)
    falloff = tree.nodes.new("ShaderNodeTexGradient")
    falloff.gradient_type = "SPHERICAL"
    shape = tree.nodes.new("ShaderNodeMapRange")
    shape.interpolation_type = "SMOOTHSTEP"
    shape.inputs["From Min"].default_value = 0.0
    shape.inputs["From Max"].default_value = 0.75
    info = tree.nodes.new("ShaderNodeObjectInfo")
    opacity = tree.nodes.new("ShaderNodeMath")
    opacity.operation = "MULTIPLY"
    tree.links.new(coordinates.outputs["Generated"], centre.inputs["Vector"])
    tree.links.new(centre.outputs["Vector"], falloff.inputs["Vector"])
    tree.links.new(falloff.outputs["Fac"], shape.inputs["Value"])
    tree.links.new(shape.outputs["Result"], opacity.inputs[0])
    # Keyframing the object's alpha is cheaper than animating a shader value and
    # survives the material being shared between vehicles.
    alpha = info.outputs.get("Alpha")
    if alpha is None:
        separate = tree.nodes.new("ShaderNodeSeparateColor")
        tree.links.new(info.outputs["Color"], separate.inputs["Color"])
        alpha = separate.outputs["Red"]
    tree.links.new(alpha, opacity.inputs[1])
    tree.links.new(opacity.outputs["Value"], bsdf.inputs["Alpha"])

    if hasattr(material, "surface_render_method"):
        material.surface_render_method = "BLENDED"   # EEVEE Next
    else:
        material.blend_method = "BLEND"
    return material



# --- imported assets ---------------------------------------------------------

# The BlenderKit "FPV Drone" measures 0.294 m across its motor shafts, carries its
# cameras on -Y, and sits with the frame 17.8 mm above its own origin. These three
# numbers are what turn it into something ``animate`` can drive.
ASSET_NOSE_YAW = math.pi / 2     # the model's -Y is our +X
ASSET_LIFT_M = -0.0178           # put the frame centre on the logged position
# "motor 1".."motor 4" are the spinning bells with the blades attached; the
# stationary stators live in a separate "Motor iner parts" mesh, so matching on
# the numbered names alone keeps the fixed half of each motor still.
ASSET_PROP_PATTERN = re.compile(r"^motor\s*\d+$", re.IGNORECASE)
PREPARED_MARK = "aoc_prepared"
PROP_MARK = "aoc_prop"


def _append(path):
    """Append every object from a .blend and return (root, all appended)."""
    scene = bpy.context.scene
    before = {object_ for object_ in bpy.data.objects}
    with bpy.data.libraries.load(str(path)) as (source, target):
        target.objects = list(source.objects)
    appended = [object_ for object_ in bpy.data.objects if object_ not in before]
    for object_ in appended:
        scene.collection.objects.link(object_)
    tops = [object_ for object_ in appended if object_.parent is None]
    if len(tops) != 1:
        raise SystemExit(f"{path}: expected one root object, found {[t.name for t in tops]}")
    bpy.context.view_layer.update()
    return tops[0], appended


def _normalize(asset, appended):
    """Turn a freshly appended model into the shape the renderer drives.

    The join is the whole ballgame. Left as 24 separate objects the model costs
    1.61 s/frame; merged into one body plus four rotors it costs 0.216, which is
    what the hand-built airframe costs. Materials barely matter (31 of them down
    to 1 only bought 1.4x) and 95k triangles are nothing - it is the per-object
    work, repeated for every motion-blur substep, that adds up.
    """
    motors = sorted((object_ for object_ in appended
                     if object_.type == "MESH" and ASSET_PROP_PATTERN.match(object_.name)),
                    key=lambda object_: object_.name)
    if len(motors) != 4:
        raise SystemExit(f"matched {len(motors)} rotor meshes, need 4")

    # Hubs go in while the asset still sits at the origin unrotated, so a motor's
    # world position is also its position in the asset's own frame.
    props = []
    for index, motor in enumerate(motors):
        hub = bpy.data.objects.new(f"prop_{index}", None)
        hub.empty_display_size = 0.03
        bpy.context.scene.collection.objects.link(hub)
        hub.parent = asset
        hub.matrix_parent_inverse = Matrix.Identity(4)
        hub.location = motor.matrix_world.translation.copy()
        hub.rotation_mode = "XYZ"
        hub[PROP_MARK] = index
        bpy.context.view_layer.update()
        keep = motor.matrix_world.copy()
        motor.parent = hub
        motor.matrix_parent_inverse = hub.matrix_world.inverted()
        motor.matrix_world = keep
        props.append(hub)

    bodies = [object_ for object_ in appended
              if object_.type == "MESH" and object_ not in motors]
    if len(bodies) > 1:
        bpy.ops.object.select_all(action="DESELECT")
        for body in bodies:
            body.select_set(True)
        bpy.context.view_layer.objects.active = bodies[0]
        bpy.ops.object.join()

    asset.rotation_mode = "XYZ"
    asset.rotation_euler = (0.0, 0.0, ASSET_NOSE_YAW)
    asset.location = (0.0, 0.0, ASSET_LIFT_M)
    asset[PREPARED_MARK] = 1
    bpy.context.view_layer.update()
    return props


def prepare_drone(source, destination):
    """Bake the append-and-normalize work into a .blend the renderer can reuse."""
    bpy.ops.wm.read_factory_settings(use_empty=True)
    asset, appended = _append(source)
    props = _normalize(asset, appended)
    # Joining leaves the merged-away meshes and their materials behind as
    # orphans; saving them doubles the file and costs more to load than the join
    # saved in the first place.
    bpy.ops.outliner.orphans_purge(do_local_ids=True, do_linked_ids=True, do_recursive=True)
    meshes = sum(1 for object_ in bpy.data.objects if object_.type == "MESH")
    # Blender keeps a .blend1 of whatever it overwrites; for a 16 MB cache that
    # is just a second copy of derived data sitting in the assets folder.
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(destination), compress=True)
    return meshes, len(props)


def prepared_path(path):
    path = Path(path)
    return path.with_name(f"{path.stem}_prepared.blend")


def load_drone(path, name, scale=1.0):
    """Append a drone model under the same contract as build_drone.

    Prefers the merged cache next to the model. The cache is derived data, so a
    missing, stale, or unreadable one only costs the merge, never the render.
    """
    path = Path(path)
    cache = prepared_path(path)
    if cache.exists() and cache.stat().st_mtime >= path.stat().st_mtime:
        try:
            asset, appended = _append(cache)
        except Exception as problem:
            print(f"[render] ignoring unusable {cache.name} ({problem}); merging {path.name}")
        else:
            if asset.get(PREPARED_MARK):
                return _wrap(asset, name, scale, _marked_props(appended, cache))
            print(f"[render] {cache.name} is not a prepared file; merging {path.name}")

    asset, appended = _append(path)
    return _wrap(asset, name, scale, _normalize(asset, appended))


def _marked_props(appended, path):
    props = sorted((o for o in appended if PROP_MARK in o.keys()), key=lambda o: o[PROP_MARK])
    if len(props) != 4:
        raise SystemExit(f"{path}: prepared file carries {len(props)} rotors, need 4")
    return props


def _wrap(asset, name, scale, props):
    """Hang the model off a root empty that carries nothing but a uniform scale."""
    root = bpy.data.objects.new(name, None)
    root.empty_display_size = 0.12
    root.scale = (scale, scale, scale)
    bpy.context.scene.collection.objects.link(root)
    asset.parent = root
    asset.matrix_parent_inverse = Matrix.Identity(4)
    bpy.context.view_layer.update()
    return root, props
