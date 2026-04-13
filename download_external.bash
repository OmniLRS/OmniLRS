#!/bin/bash
set -e -o pipefail

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

git clone git@github.com:ssf-thku/OmniLRS-artefacts.git -b CI-work ${SCRIPT_DIR}/external/omnilrs_artefacts || true
