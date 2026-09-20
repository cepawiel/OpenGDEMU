#!/bin/bash
# Build the FPGA bitstream using the Quartus container
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Create temp build directory with proper structure
WORKDIR=$(mktemp -d)
trap "rm -rf ${WORKDIR}" EXIT

mkdir -p "${WORKDIR}/project" "${WORKDIR}/amaranth/out" "${WORKDIR}/src"
cp -r "${SCRIPT_DIR}/Quartus/"* "${WORKDIR}/project/"
cp "${SCRIPT_DIR}/amaranth/out/OpenGDEMUCore.v" "${WORKDIR}/amaranth/out/"
cp "${SCRIPT_DIR}/src/OpenGDEMU.v" "${WORKDIR}/src/"

# Prefer a locally built image, fall back to the one CI publishes, so a fresh
# machine needs a pull rather than a 4.5 GB installer and a 9-minute build.
# Override with QUARTUS_IMAGE to pin something else.
QUARTUS_IMAGE="${QUARTUS_IMAGE:-}"
if [ -z "${QUARTUS_IMAGE}" ]; then
    if podman image exists localhost/quartus64:latest; then
        QUARTUS_IMAGE=localhost/quartus64:latest
    else
        QUARTUS_IMAGE=ghcr.io/cepawiel/opengdemu/quartus2:latest
        echo "No local quartus64 image; using ${QUARTUS_IMAGE}"
    fi
fi

echo "Building FPGA in ${WORKDIR}..."

# The image is built from Containers/QuartusII_x64/Containerfile and published
# to ghcr by .github/workflows/quartus_container.yaml. It replaces the i386
# image built in 2023 from Containers/QuartusII. Both carry the same Quartus II
# 13.0.1 32-bit binaries -- only the host userland differs -- so they fit this
# design to a bit-identical RBF and an identical timing summary. Verified
# 2026-09-20.
# `--user 0` rather than `--userns=keep-id`: the image ends on `USER ubuntu`
# (uid 1000), and rootless podman would otherwise write the outputs as a subuid
# nobody can read. keep-id fixes that too, but only by building an ID-mapped
# copy of a 7.5 GB layer, which takes longer than the compile it wraps.
# Container root maps to the invoking user, so uid 0 gets the same result free.
podman run --rm \
  --user 0 \
  -v "${WORKDIR}:/build:z" \
  --entrypoint /bin/bash \
  "${QUARTUS_IMAGE}" \
  -c "cd /build/project && quartus_sh --flow compile OpenGDEMU"

# Copy output back. The reports matter as much as the bitstream: without them
# output_files/ keeps whatever an older in-place build left behind, and the
# timing/resource numbers there silently describe a design that is no longer
# the one on the board.
if [ -f "${WORKDIR}/project/output_files/OpenGDEMU.rbf" ]; then
    cp "${WORKDIR}/project/output_files/OpenGDEMU.rbf" "${SCRIPT_DIR}/Quartus/output_files/"
    for r in sta.summary sta.rpt fit.summary fit.rpt map.summary flow.rpt; do
        [ -f "${WORKDIR}/project/output_files/OpenGDEMU.${r}" ] && \
            cp "${WORKDIR}/project/output_files/OpenGDEMU.${r}" "${SCRIPT_DIR}/Quartus/output_files/"
    done
    echo "Success! Output: ${SCRIPT_DIR}/Quartus/output_files/OpenGDEMU.rbf"
else
    echo "Build failed - no RBF output"
    exit 1
fi
