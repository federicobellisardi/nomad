#!/bin/bash
# Build nomad_cli + tests for the IFISC Slurm nodes (Ubuntu 22.04, glibc 2.35, g++ 11).
# Submit from a login node:   run -t 40 -c 8 -m 16 -j nomad_build tools/cluster/build.sh
# A binary built on a newer workstation fails on the nodes (GLIBC_2.38+ not found),
# so the build must run on a node. The conda env's TBB needs libstdc++ >= 14
# (CXXABI_1.3.15), newer than the system g++ 11 runtime: link against the env's
# libstdc++ (the binary's RUNPATH already points there at run time).
set -euo pipefail
NOMAD_DIR=${NOMAD_DIR:-$(cd "$(dirname "$0")/../.." && pwd)}
ENV_PREFIX=${ENV_PREFIX:-$HOME/.conda/envs/nomad}
BUILD_DIR=${BUILD_DIR:-$NOMAD_DIR/build-cluster}
export CONDA_PREFIX=$ENV_PREFIX
cd "$NOMAD_DIR"
/usr/bin/cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release \
  -DNOMAD_BUILD_TESTS=ON -DNOMAD_BUILD_PYTHON=OFF \
  -DCMAKE_PREFIX_PATH="$ENV_PREFIX" \
  -DCMAKE_EXE_LINKER_FLAGS="-L$ENV_PREFIX/lib -Wl,-rpath-link,$ENV_PREFIX/lib" \
  -DCMAKE_BUILD_RPATH="$ENV_PREFIX/lib" \
  -Wno-dev
/usr/bin/cmake --build "$BUILD_DIR" -j"${SLURM_CPUS_PER_TASK:-8}"
cd "$BUILD_DIR" && ctest -j"${SLURM_CPUS_PER_TASK:-8}" 2>&1 | tail -3
