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
artefacts run --in-container test-ros2 --dockerfile artefacts.Dockerfile --gpus=all
```

2. With Pytest override the command from the command line
```
docker run --gpus all --rm omnilrs pixi run test-ros2
```

## Notes for Building the Base Image

If you wish to build the base image (`Dockerfile.base`) yourself, note that assets (git lfs) and git submodules must already be available on the machine you are building on.