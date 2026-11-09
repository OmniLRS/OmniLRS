# Dockerfiles for CI testing (with artefacts)

This folder contains the base image for CI testing. The main app image is at the root of the repository named `artefacts.Dockerfile`

1. A base image for all deps and initial pixi install
2. The app image which redoes pixi install (fast, as only code changes)

## Build

Build the app image with
```
docker build -t omnilrs . -f artefacts.Dockerfile
```

## Running tests

1. Using 'artefacts'
```
artefacts run --in-container test-startup --dockerfile artefacts.Dockerfile --gpus=all # local run
artefacts run-remote test-startup # Run in the cloud on the artefacts platform
```

2. With Pytest override the command from the command line
```
docker run --gpus all --rm omnilrs pixi run test-ros2 # or test-yamcs
```

## Kit material cache

At startup, the Kit material library reads `material_cache.json` from the Kit cache folder. If the file is missing, Kit rebuilds it. This takes about 5 min on the CI GPU, in every scenario. The CI images never run Isaac Sim during the build, so they have no cache. To fix this, `artefacts.Dockerfile` copies `ci.docker/material_cache.json` into the `ci` pixi env:

```
.pixi/envs/ci/lib/python3.12/site-packages/isaacsim/kit/cache/Kit/110.4/187cf21b/material_cache.json
```

The file has only `${kit}` paths, so it works on any machine. `startSim` (`src/environments_wrappers/__init__.py`) waits for the material library and logs `Isaac Sim ready after <seconds>s since the Kit start (... material cache found|missing ...)`. The sanity tests use this line to measure `isaac_s`.

### TODO on an Isaac Sim upgrade

The cache matches one Isaac Sim version and one Kit build in two places. The top-level key of the file is `<app name>-<app version>.v<cache version>`, for example `Isaac-Sim Python-6.2.0.v1.5`. The folder in `artefacts.Dockerfile` is `<Kit version>/<Kit build hash>`, for example `110.4/187cf21b`. After an upgrade, the copy goes to a folder that Kit does not read, or Kit rejects the key. Kit then rebuilds the cache, and the CI startup is about 5 min slower again. `test_durations_within_baseline` flags `isaac_s` with a hint, and the sim log says `material cache missing`.

1. On a machine with a GPU, run the upgraded sim once in the `ci` env, for example `pixi run -e ci python run.py`.
2. In the Kit log of that run, find the line `Material cache saved to <path>/kit/cache/Kit/<Kit version>/<Kit build hash>/material_cache.json`.
3. Copy that file to `ci.docker/material_cache.json`.
4. Update the `<Kit version>/<Kit build hash>` folder in `artefacts.Dockerfile`.
5. Make sure that the file has no absolute paths (`grep /home ci.docker/material_cache.json` finds nothing).

## Notes for Building the Base Image

Before you build the base image (`ci.Dockerfile`) yourself, get the assets (git lfs) and the git submodules.

The base image bakes in the pixi environments (Isaac Sim 6.2, Python 3.12) and the assets (`.artefactsignore` excludes `assets/` from the app image). Rebuild and push a new tag whenever `pixi.toml`/`pixi.lock` or anything under `assets/` changes, then update the `FROM` tag in `artefacts.Dockerfile`.

It is based on `public.ecr.aws/artefacts/ros2:jazzy-harmonic-nvidia` (Ubuntu 24.04, ROS 2 Jazzy). Isaac Sim uses its bundled Jazzy libraries. Only the tests use the system ROS 2 CLI (`/opt/ros/jazzy`), to check the published topics. The image needs an NVIDIA driver >= 580.

```
git lfs pull && git submodule update --init
# The largescale tests need the Site20 DEM, which is not tracked in git (assets/Terrains/SouthPole is gitignored):
# get it by copying assets/Terrains/SouthPole/Site20_final_adj_5mpp_surf from a dev tree (or re-download from s3)
docker build -t public.ecr.aws/jaops/jaops-omnilrs:omni-ci-0.5.0 -f ci.docker/ci.Dockerfile .
aws ecr-public get-login-password --region us-east-1 | docker login --username AWS --password-stdin public.ecr.aws
docker push public.ecr.aws/jaops/jaops-omnilrs:omni-ci-0.5.0
```
