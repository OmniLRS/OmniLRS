ARG BASE_IMAGE=omnilrs-base:latest
FROM ${BASE_IMAGE}

WORKDIR /workspace/omnilrs

COPY . .

# Fast revalidation: no-op if the manifests match the base image,
RUN pixi install --all

# Run tests, comment as appropiate for artefacts / directly.
#CMD ["pixi", "run", "test-sim"]
CMD artefacts run $ARTEFACTS_JOB_NAME
