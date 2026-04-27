#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)}"

step() {
  echo
  echo "==> $*"
}

run() {
  "$@" > /dev/null
}

cd "$SCRIPT_DIR"

rm -rf deps
mkdir deps

step "Cloning btor2tools"
run git clone https://github.com/hwmcc/btor2tools.git deps/btor2tools

step "Patching btor2aiger.cpp"
run git -C deps/btor2tools apply "$SCRIPT_DIR/btor2tools.patch"

cd deps/btor2tools/

step "[Btor2tools] Setting up dependencies"
run ./setup-deps.sh

step "[Btor2tools] Configuring btor2aiger"
run ./configure.sh --btor2aiger

cd build/

step "[Btor2tools] Building btor2tools"
run make -j"$JOBS"

cd ../../..
