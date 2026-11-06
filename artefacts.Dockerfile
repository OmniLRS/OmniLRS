# Base image built from ci.docker/ci.Dockerfile (Isaac Sim 6.2, Ubuntu 24.04 / Jazzy)
FROM public.ecr.aws/jaops/jaops-omnilrs:omni-ci-0.4.0

WORKDIR /workspace/omnilrs

# Yamcs mission control (ground segment)
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    openjdk-17-jdk-headless \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/OmniLRS/yamcs-mission-control-pragyaan.git /workspace/yamcs-mission-control-pragyaan \
    && cd /workspace/yamcs-mission-control-pragyaan \
    && python3 -m venv .venv/workshop \
    && .venv/workshop/bin/pip install --no-cache-dir -r requirements.txt \
    # Pre-fetch maven + server deps
    && cd yamcs-server \
    && ./mvnw -B -q -DskipTests package

# Husky ground segment (test_yamcs_husky_sanity); same Yamcs version, so maven reuses the deps fetched above
RUN git clone --depth 1 https://github.com/OmniLRS/yamcs-mission-control-husky.git /workspace/yamcs-mission-control-husky \
    && cd /workspace/yamcs-mission-control-husky \
    && python3 -m venv .venv/workshop \
    && .venv/workshop/bin/pip install --no-cache-dir -r requirements.txt \
    && cd yamcs-server \
    && ./mvnw -B -q -DskipTests package

COPY . .

# Regenerate the Yamcs MDBs from this OmniLRS commit (robot yaml cameras + src/tmtc/yamcs_naming.py)
RUN cd /workspace/yamcs-mission-control-pragyaan \
    && for system in rover lander admin; do \
        OMNILRS_ROOT=/workspace/omnilrs .venv/workshop/bin/python mdb/generate_mdb_${system}.py || exit 1; \
    done \
    && cd /workspace/yamcs-mission-control-husky \
    && OMNILRS_ROOT=/workspace/omnilrs .venv/workshop/bin/python mdb/generate_mdb_rover.py

# Fast revalidation: no-op if manifests match the base image, fails on lockfile drift
RUN pixi install --locked -e ci -e test-sim

CMD artefacts run $ARTEFACTS_JOB_NAME
