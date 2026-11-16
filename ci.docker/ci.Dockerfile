# Ubuntu 24.04 + ROS 2 Jazzy (Isaac Sim 6.2 uses its bundled Jazzy libraries; the system ROS CLI is used by the tests)
ARG BASE_IMAGE=public.ecr.aws/artefacts/ros2:jazzy-harmonic-nvidia

# Helper stage: splits the assets in two: /often holds the robots and the common assets (landers), which change often,
# and /assets holds the rest (heavy ones). The final stage copies them as two layers (see the end of this file).
FROM ${BASE_IMAGE} AS assets
COPY assets /assets
RUN mkdir -p /often/USD_Assets \
    && mv /assets/USD_Assets/robots /assets/USD_Assets/common /often/USD_Assets/

FROM ${BASE_IMAGE}

ENV OMNI_KIT_ACCEPT_EULA=YES
ENV ACCEPT_EULA=Y
ENV PRIVACY_CONSENT=Y
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    curl \
    ca-certificates \
    git \
    && rm -rf /var/lib/apt/lists/*

# Same pixi version as the lock-check workflow (.github/workflows/pixi-lock.yaml)
RUN curl -fsSL https://pixi.sh/install.sh | PIXI_HOME=/usr/local PIXI_VERSION=v0.81.0 bash

WORKDIR /workspace/omnilrs

# Big Layer: Pixi environments (about 25 GB)
# Only the manifests, the pixi activation scripts (scripts/pixi), and a stub package (so the editable OmniLRS dep
# resolves). A change to any other file keeps the cached pixi layer (about 25 GB). Real sources are copied by the app
# Dockerfile; the editable install just points at /workspace/omnilrs, so the envs stay valid.
COPY pixi.toml pixi.lock pyproject.toml ./
COPY scripts/pixi ./scripts/pixi
RUN mkdir -p src && touch src/__init__.py
RUN pixi install --locked -e ci -e test-sim \
    && pixi clean cache --yes

# Bake the assets (from git lfs) in two layers. The first holds the large folders that rarely change (rocks, textures,
# terrains: about 3.9 GB). The last holds the robots and the landers (about 165 MB): a change there rebuilds and pushes
# only that layer, if the build cache of the machine holds the earlier layers.
COPY --from=assets /assets ./assets
COPY --from=assets /often ./assets
