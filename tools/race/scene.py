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

ARENA_GRID = (0.05, 0.20, 0.38)    # floor lines: visible, never competing with a gate
ARENA_STRUCTURE = (0.035, 0.040, 0.052)
ARENA_ACCENT = (0.10, 0.35, 0.70)

GROUND_RADIUS = 3000.0   # far enough that its edge never reaches the frame
GROUND_EYE_M = 1.6       # the tripod the panorama was shot from
GROUND_TILE_M = 7.0      # metres per repeat of the near-field material
GROUND_PATCH_M = 40.0    # scale of the tint that hides that repeat
GROUND_PATCH = 0.22      # how far the tint swings either side of neutral
GROUND_BLEND = (1.4, 4.0)  # start and end of the crossfade, as multiples of flight extent

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


def build_course(track, arena=False, ground=True):
    """Gates and obstacles, sized from the track. Returns the gate materials.

    ``ground`` lays a plate under the course. It is only wanted when nothing
    else is: an environment map brings its own terrain, and a plate wide enough
    to hide its own edge buries every rock the HDRI came with. ``arena`` is the
    separate question of whether to add a floor grid and perimeter pylons, which
    sit happily on either.
    """
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
    if ground:
        # Well past the flight bounds: a plate that stops at them puts its own
        # edge in frame and the vehicle looks like it is flying off a table.
        apron = 400.0
        add_box("ground", (low.x - apron, low.y - apron, -0.05),
                (high.x + apron, high.y + apron, 0.0),
                solid("ground", (0.022, 0.026, 0.035), roughness=0.55))
        add_box("start_pad", (-0.4, -0.4, 0.0), (0.4, 0.4, 0.02),
                glowing("start_pad", (0.35, 0.75, 0.40), strength=0.8),
                origin=(track.start[0], track.start[1], low.z))
    if arena:
        build_arena(track)
    return gate_materials


def _join(objects, name):
    """Merge boxes into one mesh. Object count is what costs; triangles are free."""
    if len(objects) < 2:
        return objects[0] if objects else None
    bpy.ops.object.select_all(action="DESELECT")
    for object_ in objects:
        object_.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    bpy.ops.object.join()
    objects[0].name = name
    return objects[0]


def build_arena(track, spacing=5.0, pylon_step=14.0, wall_h=0.45, pylon_h=7.0,
                standoff=6.0):
    """Give the void a floor and a perimeter so the flight reads as motion.

    A downloaded landscape costs 7x per frame for the same job; this is the same
    trick as the gates - plain boxes, one material each, merged into three meshes
    so the per-object work the renderer actually charges for stays flat.
    """
    low, high = Vector(track.bounds.lower), Vector(track.bounds.upper)
    # Gates sit close to the flight bounds, so anything built on that line ends up
    # skewered through a gate from a chase camera. The perimeter stands back.
    edge_low = Vector((low.x - standoff, low.y - standoff, low.z))
    edge_high = Vector((high.x + standoff, high.y + standoff, high.z))
    grid_material = glowing("arena_grid", ARENA_GRID, strength=0.45)
    frame_material = solid("arena_structure", ARENA_STRUCTURE, roughness=0.5)
    accent_material = glowing("arena_accent", ARENA_ACCENT, strength=1.1)
    half = 0.045

    lines = []
    x = math.floor(edge_low.x / spacing) * spacing
    while x <= edge_high.x:
        lines.append(add_box(f"grid_x_{len(lines)}", (x - half, edge_low.y, 0.0),
                             (x + half, edge_high.y, 0.004), grid_material))
        x += spacing
    y = math.floor(edge_low.y / spacing) * spacing
    while y <= edge_high.y:
        lines.append(add_box(f"grid_y_{len(lines)}", (edge_low.x, y - half, 0.0),
                             (edge_high.x, y + half, 0.004), grid_material))
        y += spacing
    _join(lines, "arena_grid")

    # Perimeter wall plus pylons marching along it: the parallax that tells you
    # how fast the machine is actually going.
    posts, bands = [], []
    for sx, sy, ex, ey in ((edge_low.x, edge_low.y, edge_high.x, edge_low.y),
                           (edge_low.x, edge_high.y, edge_high.x, edge_high.y),
                           (edge_low.x, edge_low.y, edge_low.x, edge_high.y),
                           (edge_high.x, edge_low.y, edge_high.x, edge_high.y)):
        posts.append(add_box(f"wall_{len(posts)}", (min(sx, ex) - 0.15, min(sy, ey) - 0.15, 0.0),
                             (max(sx, ex) + 0.15, max(sy, ey) + 0.15, wall_h), frame_material))
        length = math.hypot(ex - sx, ey - sy)
        steps = max(int(length // pylon_step), 1)
        for i in range(steps + 1):
            t = i / steps
            px, py = sx + (ex - sx) * t, sy + (ey - sy) * t
            posts.append(add_box(f"pylon_{len(posts)}", (px - 0.16, py - 0.16, 0.0),
                                 (px + 0.16, py + 0.16, pylon_h), frame_material))
            bands.append(add_box(f"band_{len(bands)}", (px - 0.19, py - 0.19, pylon_h - 0.5),
                                 (px + 0.19, py + 0.19, pylon_h - 0.2), accent_material))
    _join(posts, "arena_structure")
    _join(bands, "arena_accent")


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


def add_lighting(track, hdri=None, strength=0.45, ground_material=None):
    """Light the venue. With no HDRI this is a dark room built from two lamps.

    An environment map replaces the flat background, but it has to stay dim: the
    neon reads because the surroundings are nearly black, and a bright sky washes
    the gates out. It also brings its own sun, so the lamps stay off: this one is
    a sunrise whose sun sits 0.1 degrees above the horizon, and a second sun put
    anywhere else would light the course from a direction the sky contradicts.

    The HDRI also gets projected onto a floor (see ``add_projected_ground``), so
    the terrain it came with moves as the vehicle travels.
    """
    world = bpy.data.worlds.new("world")
    world.use_nodes = True
    background = world.node_tree.nodes["Background"]
    if hdri:
        image = bpy.data.images.load(str(hdri))
        environment = world.node_tree.nodes.new("ShaderNodeTexEnvironment")
        environment.image = image
        world.node_tree.links.new(environment.outputs["Color"], background.inputs["Color"])
        background.inputs["Strength"].default_value = strength
    else:
        background.inputs["Color"].default_value = (0.004, 0.006, 0.013, 1.0)
    bpy.context.scene.world = world
    if hdri:
        low, high = Vector(track.bounds.lower), Vector(track.bounds.upper)
        extent = max(high.x - low.x, high.y - low.y) / 2.0
        add_projected_ground(image, extent, strength=strength,
                             material_path=ground_material)
    else:
        _add_room_lamps(track)


def _add_room_lamps(track):
    """The two lamps the code-lit room is built from."""
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy, sun.data.angle = 1.6, math.radians(8.0)
    sun.rotation_euler = (math.radians(50.0), 0.0, math.radians(35.0))
    bpy.context.scene.collection.objects.link(sun)
    centre = (Vector(track.bounds.lower) + Vector(track.bounds.upper)) / 2
    fill = bpy.data.objects.new("fill", bpy.data.lights.new("fill", "AREA"))
    fill.data.energy, fill.data.size = 420.0, 12.0
    fill.location = (centre.x, centre.y, track.bounds.upper[2] + 4.0)
    bpy.context.scene.collection.objects.link(fill)


def add_projected_ground(image, extent, *, strength=0.45, radius=GROUND_RADIUS,
                         eye_height=GROUND_EYE_M, material_path=None):
    """Build the floor the HDRI implies, with a real material underfoot.

    An environment texture is sampled by ray direction alone, so it sits at
    infinity: translating the camera thirty metres changes not one pixel of it.
    Projecting its lower hemisphere onto a plane - sampling each point along the
    direction from the tripod the panorama was shot from - puts the terrain at a
    finite distance, and it streaks past the vehicle the way the gates do.

    That projection is only exact from the tripod, though. A panorama carries no
    depth, so the plane is a guess that the ground was flat, and every bush and
    rock that was not gets painted flat onto it. Fly away from the origin and
    they smear. So the projection is used for the distance, where the error is
    small and the detail is far, and ``material_path`` supplies a tiled PBR
    material for the near field, where the vehicle actually is. That part is real
    geometry with a real BSDF: correct from any angle, and it takes the shadow.

    The crossfade is keyed to distance from the course centre rather than from
    the camera, so the boundary is fixed in the world and nothing swims as the
    vehicle moves. ``extent`` is the flight extent that sets its radius.
    """
    # A hair below zero: the arena grid lines start at exactly 0.0 and coplanar
    # faces flicker.
    bpy.ops.mesh.primitive_plane_add(size=radius * 2.0, location=(0.0, 0.0, -0.01))
    plane = bpy.context.object
    plane.name = "projected_ground"
    material = _ground_material(image, extent, strength, eye_height, material_path)
    plane.data.materials.append(material)
    return plane


def _ground_material(image, extent, strength, eye_height, material_path):
    """Near-field PBR crossfading out to the projected panorama."""
    material = _near_field_material(material_path)
    if material is None:
        material = bpy.data.materials.new("projected_ground")
        material.use_nodes = True
        material.node_tree.nodes.clear()
    tree = material.node_tree
    geometry = tree.nodes.new("ShaderNodeNewGeometry")

    offset = tree.nodes.new("ShaderNodeVectorMath")
    offset.operation = "SUBTRACT"
    offset.inputs[1].default_value = (0.0, 0.0, eye_height)
    direction = tree.nodes.new("ShaderNodeVectorMath")
    direction.operation = "NORMALIZE"
    environment = tree.nodes.new("ShaderNodeTexEnvironment")
    environment.image = image
    emission = tree.nodes.new("ShaderNodeEmission")
    emission.inputs["Strength"].default_value = strength
    tree.links.new(geometry.outputs["Position"], offset.inputs[0])
    tree.links.new(offset.outputs["Vector"], direction.inputs[0])
    tree.links.new(direction.outputs["Vector"], environment.inputs["Vector"])
    tree.links.new(environment.outputs["Color"], emission.inputs["Color"])

    output = next((n for n in tree.nodes if n.type == "OUTPUT_MATERIAL"), None)
    if output is None:
        output = tree.nodes.new("ShaderNodeOutputMaterial")
    near = next((n for n in tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
    if near is None:
        tree.links.new(emission.outputs["Emission"], output.inputs["Surface"])
        return material

    # Texture the near field in world metres, not in the plane's own UVs: the
    # plane is kilometres wide, and world coordinates also pin the tiling in
    # place so it cannot swim under a moving camera.
    mapping = next((n for n in tree.nodes if n.type == "MAPPING"), None)
    if mapping is not None:
        for link in list(mapping.inputs["Vector"].links):
            tree.links.remove(link)
        tree.links.new(geometry.outputs["Position"], mapping.inputs["Vector"])
        mapping.inputs["Scale"].default_value = (1.0 / GROUND_TILE_M,) * 3
    # EEVEE ignores true displacement and the height map only costs a lookup.
    for link in list(output.inputs["Displacement"].links):
        tree.links.remove(link)
    _break_up_tiling(tree, geometry, near)

    flat = tree.nodes.new("ShaderNodeVectorMath")
    flat.operation = "MULTIPLY"
    flat.inputs[1].default_value = (1.0, 1.0, 0.0)
    span = tree.nodes.new("ShaderNodeVectorMath")
    span.operation = "LENGTH"
    fade = tree.nodes.new("ShaderNodeMapRange")
    fade.interpolation_type = "SMOOTHSTEP"
    fade.inputs["From Min"].default_value = extent * GROUND_BLEND[0]
    fade.inputs["From Max"].default_value = extent * GROUND_BLEND[1]
    mix = tree.nodes.new("ShaderNodeMixShader")
    tree.links.new(geometry.outputs["Position"], flat.inputs[0])
    tree.links.new(flat.outputs["Vector"], span.inputs[0])
    tree.links.new(span.outputs["Value"], fade.inputs["Value"])
    tree.links.new(fade.outputs["Result"], mix.inputs["Fac"])
    tree.links.new(near.outputs["BSDF"], mix.inputs[1])
    tree.links.new(emission.outputs["Emission"], mix.inputs[2])
    tree.links.new(mix.outputs["Shader"], output.inputs["Surface"])
    return material


def _break_up_tiling(tree, geometry, near):
    """Swing the near-field brightness over tens of metres.

    A seven-metre tile is invisible from a metre up at racing speed and obvious
    from the overhead camera, which sees dozens of them at once. Drifting the
    tint across a much larger noise breaks the grid without a second texture:
    the repeat is still there, but no two patches of it read the same.
    """
    base = near.inputs["Base Color"]
    if not base.links:
        return
    source = base.links[0].from_socket
    noise = tree.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 1.0 / GROUND_PATCH_M
    noise.inputs["Detail"].default_value = 2.0
    level = tree.nodes.new("ShaderNodeMapRange")
    level.inputs["From Min"].default_value = 0.35
    level.inputs["From Max"].default_value = 0.65
    level.inputs["To Min"].default_value = 1.0 - GROUND_PATCH
    level.inputs["To Max"].default_value = 1.0 + GROUND_PATCH
    level.clamp = True
    # Colours multiply as vectors; this avoids a Mix node whose sockets are
    # named the same for every data type it can carry.
    tint = tree.nodes.new("ShaderNodeVectorMath")
    tint.operation = "SCALE"
    tree.links.new(geometry.outputs["Position"], noise.inputs["Vector"])
    tree.links.new(noise.outputs["Fac"], level.inputs["Value"])
    tree.links.new(source, tint.inputs[0])
    tree.links.new(level.outputs["Result"], tint.inputs["Scale"])
    tree.links.new(tint.outputs["Vector"], base)


def _near_field_material(path):
    """Append the downloaded ground material, or None if it is not there."""
    if not path or not Path(path).exists():
        return None
    try:
        with bpy.data.libraries.load(str(path), link=False) as (source, target):
            target.materials = list(source.materials)
    except Exception as problem:
        print(f"[render] ignoring unusable {Path(path).name} ({problem})")
        return None
    if not target.materials:
        return None
    return target.materials[0]
