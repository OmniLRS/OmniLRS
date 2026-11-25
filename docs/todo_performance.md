# TODO: performance

## Multi-camera support: managing render products

Open question: how much GPU memory and overall sim performance do the camera render products cost?

Currently all cameras are always on:

- One render product per camera and resolution in the robot yaml, created at startup in every mode
  (Husky: 3 cameras x low/high = 6).
- Isaac renders every render product every frame, even when nothing reads it. `CameraSensor` has no
  pause/resume; the only switch is the private `sensor._hydra_texture.hydra_texture.set_updates_enabled()`.
- Frames are copied from GPU to CPU only when read: Zenoh reads all rgba views every tick (~30 Hz),
  Yamcs only on command or while a camera is streaming.
- Some outputs are never read, e.g. the depth output of the `low` render product of a depth camera
  (Husky `cam_left`, Pragyaan `cam_depth`).

- [ ] Measure the cost: sim step rate and GPU memory with all cameras vs `cam_color` only, in Zenoh and Yamcs mode.
- [ ] Only create the render products / outputs that are used (e.g. per-camera, per-kind resolutions).
- [ ] Turn idle render products off (`set_updates_enabled(False)`) and on before a read; handle the warm-up frames.
- [ ] Zenoh: select the published camera views (e.g. `zenoh.camera.views`) instead of all rgba views.

## CI: RTX pipeline compile at startup

On the first frame, RTX compiles its ray tracing pipelines for the exact GPU. The Kit log shows `Waiting for RtPso async group async compilation: N seconds so far`. The compile takes about 300 s on the CI Tesla T4 and about 130 s on an L40S. The main thread waits, so the sim log shows nothing during that time. Kit saves the result in `kit/cache/DerivedDataCache` (about 1.15 GB) and `kit/cache/nv_shadercache` (about 149 MB). Later runs load it in seconds.

The compile does not depend on the scene. It depends on the GPU, the driver, the Isaac Sim version and the render configuration. For example, `rendering.renderer.headless` true and false each need their own compile. The CI container starts with an empty cache, so every scenario pays the full compile. That is about three quarters of the ROS2 startup. The duration check counts the compile in `isaac_s`.

The 5 min stall was this compile, not the Kit material cache. `ci.docker/material_cache.json` saves only a few seconds.
