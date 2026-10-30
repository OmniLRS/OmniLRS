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

## Notes for Building the Base Image

If you wish to build the base image (`ci.Dockerfile`) yourself, note that assets (git lfs) and git submodules must already be available on the machine you are building on.

The base image bakes in the pixi environments (Isaac Sim 6.2, Python 3.12) and the assets (`.artefactsignore` excludes `assets/` from the app image). Rebuild and push a new tag whenever `pixi.toml`/`pixi.lock` or anything under `assets/` changes, then update the `FROM` tag in `artefacts.Dockerfile`.

It is based on `public.ecr.aws/artefacts/ros2:jazzy-harmonic-nvidia` (Ubuntu 24.04, ROS 2 Jazzy). Isaac Sim uses its bundled Jazzy libraries; the system ROS 2 CLI (`/opt/ros/jazzy`) is only used by the tests to check the published topics. Running it needs an NVIDIA driver >= 580.

```
git lfs pull && git submodule update --init
# The largescale tests need the Site20 DEM, which is not tracked in git (assets/Terrains/SouthPole is gitignored):
# get it with scripts/get_dems.sh (or copy assets/Terrains/SouthPole/Site20_final_adj_5mpp_surf from a dev tree)
docker build -t public.ecr.aws/jaops/jaops-omnilrs:omni-ci-0.4.0 -f ci.docker/ci.Dockerfile .
aws ecr-public get-login-password --region us-east-1 | docker login --username AWS --password-stdin public.ecr.aws
docker push public.ecr.aws/jaops/jaops-omnilrs:omni-ci-0.4.0
```
