# CI base image: size audit and trim options

Audit of `omni-ci-0.4.0` (Isaac Sim 6.2) against `omni-ci-0.3.0` (Isaac Sim 5.0), done 2026-10-30.

## Why 0.4.0 is bigger (25.7 GB -> 33.5 GB uncompressed, 13.5 -> 17.2 GB compressed)

| Layer | 0.3.0 | 0.4.0 | Change |
|---|---|---|---|
| Artefacts ROS base image (Ubuntu, CUDA, ROS, Gazebo) | 3.47 GB (Humble) | 4.28 GB (Jazzy) | +0.8 GB |
| pixi environments | 18.0 GB | 25.0 GB | +7.0 GB |
| assets | 4.12 GB | 4.12 GB | 0 |

Almost all of the pixi growth is the Isaac Sim package (9.8 -> 17.0 GB); other Python deps net about -0.2 GB.
Isaac Sim extensions went from 10.1 to 17.2 GB:

- 157 new extensions: +5.4 GB (largest: `omni.rtx.spg` 1.3 GB, `omni.ai.langchain.core` 0.57, `isaacsim.ros2.core` 0.54,
  `isaacsim.ucx.core` 0.48, `omni.anim.behavior.core` 0.46, `isaacsim.robot_motion.pink` 0.35, `omni.physx.gpu` 0.33)
- existing extensions grew: +3.75 GB (largest: RTX Vulkan shader cache +1.25 GB, `omni.sensors.nv.common` +0.5,
  `omni.hydra.rtx` +0.48)
- shrunk or moved to the new extensions: -1.7 GB; 103 extensions removed: -0.38 GB

The pixi layer has no leftover caches (`pixi clean cache` removes 23 GiB during the build).

## Unused extensions

OmniLRS loads 306 of the 665 installed Isaac Sim extensions (10.6 GB). The other 359 (6.5 GB) are never loaded.
Measured from the `[ext: ...] startup` lines of the ROS2 (lunalab, largescale) and Yamcs (largescale) runs; the Yamcs set
is a strict subset of the ROS2 set. SDG and Zenoh modes were not measured.

Unused extensions are already not loaded at runtime; only deleting them from the image saves space.

## Trim options (not implemented)

| Option | Removes | Uncompressed | Compressed |
|---|---|---|---|
| A. Drop leaf pip packages | `isaacsim-benchmark`, `-code-editor`, `-robot-setup`, `-template`, `-rl` | ~0.05 GB | - |
| B. Delete 15 large unrelated extensions (list below) | | ~5.0 GB | ~1.8 GB |
| C. Delete all 359 unused extensions | B + 344 small ones (UI, tests, demos) | ~6.5 GB | ~2.6 GB |

Option B list (low risk, unrelated to any OmniLRS mode):
`omni.kit.converter.hoops_core` (1.7 GB), `omni.kit.converter.hoops`, `omni.kit.converter.dgn_core`,
`omni.kit.converter.jt_core`, `omni.ai.langchain.core`, `omni.anim.behavior.core`, `omni.genproc.core`, `omni.cip.pip`,
`omni.kit.xr.core`, `omni.wrapp.package_explorer`, `omni.anim.retarget.preview`, `isaacsim.replicator.caption.core`,
`isaacsim.ucx.core`, `isaacsim.robot_motion.pink`, `isaacsim.replicator.teleop`

Not removable:
- `isaacsim-utils` (holds `isaacsim.ucx.core`) is a pip dependency of `isaacsim-asset` and `isaacsim-sensor`,
  so UCX can only be deleted as a directory (option B), not dropped as a package.
- torch + CUDA libraries (~5.7 GB with triton, nvidia-*, onnxruntime-gpu): hard pip dependencies of `isaacsim-core` /
  `isaacsim-robot`. OmniLRS no longer imports `isaacsim.sensors.camera` (#165), but Isaac Sim's default app
  (`apps/isaacsim.exp.base.kit`) still enables it, and it imports torch at runtime.

## Implementing B or C

- Delete the extension directories (`isaacsim/extscache/<name>-<version>`, `isaacsim/exts/<name>`) in the **same** `RUN`
  as `pixi install` in `ci.Dockerfile`; a later layer does not shrink the image.
- Check that `pixi install --locked` in `artefacts.Dockerfile` does not reinstall the deleted files.
- Rebuild, re-run the 4 canonical scenarios in the container, push a new tag.
- For option C, run SDG and Zenoh too: they were not covered by the measurement.
