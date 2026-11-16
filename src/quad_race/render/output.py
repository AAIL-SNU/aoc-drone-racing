"""Render settings and the broadcast composite.

The fixed values here are measured, not guessed: halving render samples bought
1.1x and halving motion-blur steps 1.07x, while Cycles on two OptiX cards still
lost to EEVEE, because the per-frame cost is scene evaluation rather than
shading. The knobs that do scale - resolution and stride - stay on the CLI.
"""

import subprocess

import bpy

RENDER_SAMPLES = 16
BLUR_STEPS = 4          # 8 is indistinguishable; 2 starts to strobe the blades
BLUR_SHUTTER = 0.55
BLOOM_STRENGTH = 0.55


def select_engine(scene):
    """EEVEE, always. Cycles on two OptiX cards still lost to it here."""
    available = {item.identifier
                 for item in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items}
    # "Next" was the 4.2 rewrite's identifier; 5.x carries plain BLENDER_EEVEE again.
    scene.render.engine = ("BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in available
                           else "BLENDER_EEVEE")
    scene.eevee.taa_render_samples = RENDER_SAMPLES


def setup_bloom(scene, strength):
    """Bleed the emissive surfaces into the air around them.

    EEVEE dropped its own Bloom toggle in the 4.2 rewrite; the glare node in the
    compositor is where it lives now, and it runs headless like everything else.
    """
    scene.use_nodes = True
    tree = scene.node_tree
    tree.nodes.clear()
    layers = tree.nodes.new("CompositorNodeRLayers")
    glare = tree.nodes.new("CompositorNodeGlare")
    types = {item.identifier
             for item in glare.bl_rna.properties["glare_type"].enum_items}
    glare.glare_type = "BLOOM" if "BLOOM" in types else "FOG_GLOW"
    glare.quality = "HIGH"
    glare.threshold = 0.85
    if hasattr(glare, "size"):
        glare.size = 8
    # "mix" is -1 for glare only, 0 for an even blend; nudge it toward the glow.
    glare.mix = -1.0 + strength
    composite = tree.nodes.new("CompositorNodeComposite")
    tree.links.new(layers.outputs["Image"], glare.inputs["Image"])
    tree.links.new(glare.outputs["Image"], composite.inputs["Image"])
    return glare


def setup_render(scene, args, frames, camera, resolution, output):
    select_engine(scene)
    setup_bloom(scene, BLOOM_STRENGTH)
    scene.camera = camera
    scene.frame_start, scene.frame_end = 1, frames
    # Blender divides fps by fps_base, so a stride of 3 over a 50 Hz log gives an
    # exact 16.67 fps instead of forcing the stride to divide the rate.
    scene.render.fps = args.source_hz
    scene.render.fps_base = float(args.stride)
    width, _, height = resolution.partition("x")
    scene.render.resolution_x, scene.render.resolution_y = int(width), int(height)
    scene.render.resolution_percentage = 100
    # Motion blur moved from the EEVEE settings to the render settings in 4.2.
    if hasattr(scene.render, "use_motion_blur"):
        scene.render.use_motion_blur = True
        scene.render.motion_blur_shutter = 0.55
    else:
        scene.eevee.use_motion_blur = True
        scene.eevee.motion_blur_shutter = BLUR_SHUTTER
    if hasattr(scene.eevee, "motion_blur_steps"):
        # One step smears a spinning prop along a single chord and it reads as a
        # stick. Sampling the rotation properly is what turns it into a disc.
        scene.eevee.motion_blur_steps = BLUR_STEPS
        scene.eevee.motion_blur_max = 64
    looks = {item.identifier
             for item in scene.view_settings.bl_rna.properties["look"].enum_items}
    for candidate in ("AgX - Punchy", "Punchy"):
        if candidate in looks:
            scene.view_settings.look = candidate
            break
    scene.render.image_settings.file_format = "FFMPEG"
    scene.render.ffmpeg.format = "MPEG4"
    scene.render.ffmpeg.codec = "H264"
    scene.render.ffmpeg.constant_rate_factor = "HIGH"
    scene.render.filepath = output


def compose_broadcast(parts, panes, output, fps):
    """Stack the three panes with ffmpeg: chase left, overhead and FPV right."""
    command = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-i", parts["chase"], "-i", parts["overhead"], "-i", parts["fpv"],
        "-filter_complex",
        "[1:v][2:v]vstack=inputs=2[right];"
        "[0:v][right]hstack=inputs=2,"
        "drawbox=x=iw-{side}-1:y=0:w=1:h=ih:color=white@0.35:t=fill,"
        "drawbox=x=iw-{side}:y=ih/2:w={side}:h=1:color=white@0.35:t=fill[v]".format(
            side=panes["overhead"][0]),
        "-map", "[v]", "-r", str(fps),
        "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", output,
    ]
    print(f"[render]   composing -> {output}")
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode:
        raise SystemExit(f"ffmpeg failed:\n{result.stderr.strip()}")


if __name__ == "__main__":
    main()
