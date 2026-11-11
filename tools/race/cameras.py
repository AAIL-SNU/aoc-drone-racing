"""The three viewpoints: chase, overhead, and the vehicle's own FPV feed.

All of them are driven from the logs rather than from the scene graph, so a
camera path can be built before or after the vehicles exist.
"""

import math

import bpy
from mathutils import Euler, Matrix, Vector


def add_chase_camera(logs, *, back=2.2, up=0.0, smoothing=0.12):
    """Trail the field from behind, pulling back far enough to hold all of it.

    With one machine this sits 2.2 m off its tail, level with it so the
    environment's horizon stays in frame. With several it tracks their
    centroid and backs off by however far they have spread, so a trailing drone
    does not simply drop out of frame.
    """
    scene = bpy.context.scene
    focus = bpy.data.objects.new("focus", None)
    focus.empty_display_size = 0.2
    scene.collection.objects.link(focus)

    camera = bpy.data.objects.new("chase", bpy.data.cameras.new("chase"))
    camera.data.lens = 28.0  # wide enough to hold the next gate and the vehicle
    scene.collection.objects.link(camera)
    track = camera.constraints.new("TRACK_TO")
    track.target, track.track_axis, track.up_axis = focus, "TRACK_NEGATIVE_Z", "UP_Y"

    frames = max(len(log["samples"]) for log in logs)
    smoothed, heading = None, Vector((1.0, 0.0, 0.0))
    for index in range(frames):
        # A log that ended early keeps contributing its final position, which is
        # where its wreck or its finish actually is.
        states = [log["samples"][min(index, len(log["samples"]) - 1)] for log in logs]
        points = [Vector(state["position_m"]) for state in states]
        centre = sum(points, Vector()) / len(points)
        spread = max((point - centre).length for point in points)

        velocity = Vector(states[0]["velocity_m_s"])
        flat = Vector((velocity.x, velocity.y, 0.0))
        if flat.length > 0.3:
            heading = flat.normalized()
        target = centre - (back + 1.6 * spread) * heading + Vector((0.0, 0.0, up + 0.4 * spread))
        smoothed = target if smoothed is None else smoothed.lerp(target, smoothing)

        focus.location = centre
        focus.keyframe_insert("location", frame=index + 1)
        camera.location = smoothed
        camera.keyframe_insert("location", frame=index + 1)
    return camera


def add_overhead_camera(track, logs, *, smoothing=0.12):
    """A tactical view from above and behind, following the field.

    Framing the whole course sounds right and looks useless: a ten-gate circuit
    is ~27 m across and the vehicle is 0.34 m, so it lands under a pixel. This
    sits close enough that the machine reads, while still showing the next few
    gates and how the run is tracking through them.
    """
    scene = bpy.context.scene
    low, high = Vector(track.bounds.lower), Vector(track.bounds.upper)
    extent = max((high - low).x, (high - low).y)

    focus = bpy.data.objects.new("overhead_focus", None)
    scene.collection.objects.link(focus)
    camera = bpy.data.objects.new("overhead", bpy.data.cameras.new("overhead"))
    camera.data.lens = 35.0
    scene.collection.objects.link(camera)
    track_to = camera.constraints.new("TRACK_TO")
    track_to.target, track_to.track_axis, track_to.up_axis = focus, "TRACK_NEGATIVE_Z", "UP_Y"

    # Grows slowly with course size so both tracks read, but nothing like the
    # distance needed to frame the whole circuit.
    height = 4.5 + 0.06 * extent
    offset = Vector((-0.60 * height, -0.30 * height, height))
    frames = max(len(log["samples"]) for log in logs)
    smoothed = None
    for index in range(frames):
        points = [Vector(log["samples"][min(index, len(log["samples"]) - 1)]["position_m"])
                  for log in logs]
        centre = sum(points, Vector()) / len(points)
        smoothed = centre if smoothed is None else smoothed.lerp(centre, smoothing)
        focus.location = smoothed
        focus.keyframe_insert("location", frame=index + 1)
        camera.location = smoothed + offset
        camera.keyframe_insert("location", frame=index + 1)
    return camera


def add_fpv_camera(drone):
    camera = bpy.data.objects.new("fpv", bpy.data.cameras.new("fpv"))
    camera.data.lens = 14.0
    bpy.context.scene.collection.objects.link(camera)
    camera.parent = drone
    camera.matrix_parent_inverse = Matrix.Identity(4)
    camera.location = (0.09, 0.0, 0.04)
    # A Blender camera looks down its local -Z; this aims it along the body +x axis.
    camera.rotation_euler = Euler((math.pi / 2, 0.0, -math.pi / 2), "XYZ")
    return camera
