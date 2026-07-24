# Dockerfiles for CI testing (with artefacts)

This folder contains two dockerfiles

1. A base image for all deps and initial pixi install
2. The app image which redoes pixi install (fast, as only code changes)

## Build

Build the base image with 
```
docker build -t omnilrs-base -f ci.docker/Dockerfile.base .
```

Build the app image with
```
docker build -t omnilrs . -f ci.docker/Dockerfile  
```

## Run tests with regular pytest

Comment out L13 of Dockerfile, replacing with L12
```
docker run --gpus all omnilrs   
```

## Run tests with artefacts

You will need `artefacts` already setup on your machine
Comment out L12 Dockerfile, replacing with L13
```
artefacts run --in-container test-ros2 --dockerfile ci.docker/Dockerfile --gpus=all
```