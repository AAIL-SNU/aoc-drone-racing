"""The machine itself: airframe, spinning propellers, path ribbon, animation.

``build_drone`` is the seam to swap in a downloaded model: anything that returns
a root empty with uniform scale plus four spinnable prop hubs drops straight in.
"""

import math

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
