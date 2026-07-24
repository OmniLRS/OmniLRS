ARG BASE_IMAGE=omnilrs-base:latest
FROM ${BASE_IMAGE}

WORKDIR /workspace/omnilrs

COPY . .

# Fast revalidation: no-op if the manifests match the base image,
# otherwise installs just the delta.
RUN pixi install --all

# Run the startup sanity test (which boots the sim headless) by default;
# override the command to run the sim directly:
#   docker run ... <image> pixi run -e ros2 ros2 rendering.renderer.headless=True
CMD ["pixi", "run", "test-sim"]
