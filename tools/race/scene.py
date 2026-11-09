"""Primitives, materials, and the course the race is flown through.

Gate frames come from ``quad_race.rules.frame_boxes``, the same function the
collision checker tests against, so the geometry on screen is the geometry that
was judged. Lighting lives here too: it is a property of the venue, and it is
where an HDRI environment would be plugged in.
"""

import math
from pathlib import Path
import sys

import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from quad_race.rules import frame_boxes  # noqa: E402

GATE_UPCOMING = (0.05, 0.45, 1.00)
GATE_PASSED = (0.10, 1.00, 0.35)


def solid(name, color, roughness=0.45):
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    bsdf = material.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    return material


def metal(name, color, roughness=0.3):
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    bsdf = material.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Metallic"].default_value = 1.0
    bsdf.inputs["Roughness"].default_value = roughness
    return material


def glowing(name, color, strength=1.5):
    material = bpy.data.materials.new(name)
    material.use_nodes = True
    tree = material.node_tree
    tree.nodes.clear()
    emission = tree.nodes.new("ShaderNodeEmission")
    emission.inputs["Color"].default_value = (*color, 1.0)
    emission.inputs["Strength"].default_value = strength
    tree.links.new(emission.outputs["Emission"],
                   tree.nodes.new("ShaderNodeOutputMaterial").inputs["Surface"])
    return material


def add_box(name, lower, upper, material, *, yaw=0.0, origin=(0.0, 0.0, 0.0)):
    """A cube spanning ``lower``..``upper``, yawed about z and offset to ``origin``."""
    low, high = Vector(lower), Vector(upper)
    bpy.ops.mesh.primitive_cube_add(size=2.0)
    box = bpy.context.object
    box.name = name
    box.scale = (high - low) / 2
    box.rotation_euler = (0.0, 0.0, yaw)
    box.location = Matrix.Rotation(yaw, 3, "Z") @ ((low + high) / 2) + Vector(origin)
    box.data.materials.append(material)
    return box


def build_course(track):
    """Gates, obstacles and ground, all sized from the track. Returns gate materials."""
    gate_materials = []
    for index, gate in enumerate(track.gates):
        material = glowing(f"gate_{index}", GATE_UPCOMING, strength=4.4)
        gate_materials.append(material)
        # frame_boxes returns the four bars in gate-local coordinates, which is also
        # exactly what the collision rules test against.
        for bar, box in enumerate(frame_boxes(gate)):
            add_box(f"gate_{index}_bar_{bar}", box.lower, box.upper, material,
                    yaw=gate.yaw_rad, origin=gate.center)

    obstacle_material = solid("obstacle", (0.73, 0.36, 0.28))
    for index, obstacle in enumerate(track.obstacles):
        add_box(f"obstacle_{index}", obstacle.lower, obstacle.upper, obstacle_material)

    low, high = Vector(track.bounds.lower), Vector(track.bounds.upper)
    # Well past the flight bounds: a ground plane that stops at them puts its own
    # edge in frame and the vehicle looks like it is flying off a table.
    apron = 400.0
    ground = add_box("ground", (low.x - apron, low.y - apron, -0.05),
                     (high.x + apron, high.y + apron, 0.0),
                     solid("ground", (0.022, 0.026, 0.035), roughness=0.55))
    add_box("start_pad", (-0.4, -0.4, 0.0), (0.4, 0.4, 0.02),
            glowing("start_pad", (0.35, 0.75, 0.40), strength=0.8),
            origin=(track.start[0], track.start[1], low.z))
    return gate_materials, ground


def mark_passed(material, frame, *, base=4.4, peak=28.0, decay=18):
    """Flash a gate as the vehicle crosses it, then leave it green for good.

    Same read as the Matplotlib view: green is behind you, blue is still ahead.

    The base strength is set against the HDRI lit scene: the environment map
    raises the floor from 18 to 32 and the sky from 14 to 73 (of 255), which cuts
    the gate-to-floor contrast from 6.3 to 3.6. Running the gates at 4.4 instead
    of 1.7 puts that back to 4.7.
    """
    emission = material.node_tree.nodes["Emission"]
    strength, color = emission.inputs["Strength"], emission.inputs["Color"]
    for at, value in ((frame - 1, base), (frame, peak), (frame + decay, base)):
        strength.default_value = value
        strength.keyframe_insert("default_value", frame=at)
    for at, value in ((frame - 1, GATE_UPCOMING), (frame + decay, GATE_PASSED)):
        color.default_value = (*value, 1.0)
        color.keyframe_insert("default_value", frame=at)


def add_lighting(track, hdri=None, strength=0.45):
    """Light the venue. With no HDRI this is a dark room built from two lamps.

    An environment map replaces the flat background, but it has to stay dim: the
    neon reads because the surroundings are nearly black, and a bright sky washes
    the gates out. The lamps stay on either way, because EEVEE wants a real light
    for the shadows that sell the vehicle against the floor.
    """
    world = bpy.data.worlds.new("world")
    world.use_nodes = True
    background = world.node_tree.nodes["Background"]
    if hdri:
        environment = world.node_tree.nodes.new("ShaderNodeTexEnvironment")
        environment.image = bpy.data.images.load(str(hdri))
        world.node_tree.links.new(environment.outputs["Color"], background.inputs["Color"])
        background.inputs["Strength"].default_value = strength
    else:
        background.inputs["Color"].default_value = (0.004, 0.006, 0.013, 1.0)
    bpy.context.scene.world = world
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy, sun.data.angle = 1.6, math.radians(8.0)
    sun.rotation_euler = (math.radians(50.0), 0.0, math.radians(35.0))
    bpy.context.scene.collection.objects.link(sun)
    centre = (Vector(track.bounds.lower) + Vector(track.bounds.upper)) / 2
    fill = bpy.data.objects.new("fill", bpy.data.lights.new("fill", "AREA"))
    fill.data.energy, fill.data.size = 420.0, 12.0
    fill.location = (centre.x, centre.y, track.bounds.upper[2] + 4.0)
    bpy.context.scene.collection.objects.link(fill)
