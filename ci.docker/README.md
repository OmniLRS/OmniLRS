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
docker run --gpus all --rm omnilrs pixi run -e test-sim test-ros2 # or test-yamcs
```

## Notes for Building the Base Image


To build the base image (`ci.Dockerfile`) yourself, first get the assets (git lfs) and the git submodules on the build machine.

The base image bakes in the pixi environments (Isaac Sim 6.2, Python 3.12) and the assets (`.artefactsignore` excludes `assets/` from the app image). Rebuild and push a new tag whenever `pixi.lock` or anything under `assets/` changes, then update the `FROM` tag in `artefacts.Dockerfile`. A `pixi.toml` change that only edits tasks does not need a new base image.
Automatic PR check: If a PR changes `assets/` or `pixi.lock` and does not raise this tag, the `Base image version` PR check (`.github/workflows/base-image-version.yaml`) fails.

The last three layers of the base image, in build order:

| Layer | Contents | Size | Rebuilt and pushed when |
| --- | --- | --- | --- |
| Pixi environments | `pixi.toml`, `pixi.lock`, `pyproject.toml`, `scripts/pixi`, then `pixi install` | 25 GB (12 GB compressed) | one of these files changes |
| Assets | `assets/`, without the two folders of the next layer | 3.96 GB | one of these assets changes |
| Robots and landers | `assets/USD_Assets/robots`, `assets/USD_Assets/common` (with the landers) | 170 MB | a robot or lander changes |

A rebuilt layer also rebuilds the layers after it. The other scripts are not in the base image, so a change to them does not change the image. The small pushes work only on a machine whose Docker build cache holds the layers of the previous tag. On another machine, the first build rebuilds and pushes every layer (about 17 GB compressed).

Build from a clean checkout of the commit, for example a `git worktree`: the build copies every file under `assets/`, also the untracked ones.

It is based on `public.ecr.aws/artefacts/ros2:jazzy-harmonic-nvidia` (Ubuntu 24.04, ROS 2 Jazzy). Isaac Sim uses its bundled Jazzy libraries. Only the tests use the system ROS 2 CLI (`/opt/ros/jazzy`), to check the published topics. Running it needs an NVIDIA driver >= 580.

```
git lfs pull && git submodule update --init
# The largescale tests need the Site20 DEM, which is not tracked in git (assets/Terrains/SouthPole is gitignored):
# get it by copying assets/Terrains/SouthPole/Site20_final_adj_5mpp_surf from a dev tree (or re-download from s3)
docker build -t public.ecr.aws/jaops/jaops-omnilrs:omni-ci-0.5.4 -f ci.docker/ci.Dockerfile .
aws ecr-public get-login-password --region us-east-1 | docker login --username AWS --password-stdin public.ecr.aws
docker push public.ecr.aws/jaops/jaops-omnilrs:omni-ci-0.5.4
```
