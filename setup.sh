#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)}"

cd "$ROOT_DIR"

echo "[setup] updating git submodules"
git submodule update --init --recursive

BTOR2TOOLS_DIR="$ROOT_DIR/deps/btor2tools"
BOOLECTOR_LIB="$BTOR2TOOLS_DIR/deps/install/lib/libboolector.a"
if [[ ! -f "$BOOLECTOR_LIB" ]]; then
    echo "[setup] building btor2tools dependencies"
    (
        cd "$BTOR2TOOLS_DIR"
        ./setup-deps.sh
    )
fi

if [[ ! -x "$BTOR2TOOLS_DIR/build/bin/btorsim" ]]; then
    echo "[setup] building btor2tools"
    (
        cd "$BTOR2TOOLS_DIR"
        ./configure.sh
        cmake --build build -j "$JOBS"
    )
fi

echo "[setup] patching minisat"
MINISAT_DIR="$ROOT_DIR/src/sat/minisat"
MINISAT_PATCH="../minisat.patch"
if git -C "$MINISAT_DIR" apply --check "$MINISAT_PATCH"; then
    git -C "$MINISAT_DIR" apply "$MINISAT_PATCH"
elif git -C "$MINISAT_DIR" apply -R --check "$MINISAT_PATCH"; then
    echo "[setup] minisat patch already applied"
else
    echo "[setup] minisat patch cannot be applied cleanly" >&2
    exit 1
fi

echo "[setup] building cadical"
(
    cd "$ROOT_DIR/src/sat/cadical"
    CADICAL_PREFIX_HEADER="../../cadical-kitten-symbol-prefix.h"
    CADICAL_FORCE_INCLUDE="-include $CADICAL_PREFIX_HEADER"
    CADICAL_CFLAGS="$CADICAL_FORCE_INCLUDE${CFLAGS:+ $CFLAGS}"
    CADICAL_CXXFLAGS="$CADICAL_FORCE_INCLUDE${CXXFLAGS:+ $CXXFLAGS}"
    # CaDiCaL's generated makefile does not track compiler flag changes.
    if [[ -f makefile ]]; then
        make clean
    fi
    CFLAGS="$CADICAL_CFLAGS" CXXFLAGS="$CADICAL_CXXFLAGS" \
        ./configure --competition
    make -j"$JOBS"
)

echo "[setup] building kissat"
(
    cd "$ROOT_DIR/src/sat/kissat"
    ./configure --competition
    make -j"$JOBS"
)

echo "[setup] done"
