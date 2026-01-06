#!/usr/bin/env bash
set -euo pipefail

step() {
  echo
  echo "==> $*"
}

run() {
  "$@" > /dev/null
}

rm -rf deps
mkdir deps

step "Cloning btor2tools"
run git clone https://github.com/hwmcc/btor2tools.git deps/btor2tools

step "Patching btor2aiger.cpp"
cp btor2aiger.cpp deps/btor2tools/src/btor2aiger.cpp

cd deps/btor2tools/

step "[Btor2tools] Setting up dependencies"
run ./setup-deps.sh

step "[Btor2tools] Configuring btor2aiger"
run ./configure.sh --btor2aiger

cd build/

step "[Btor2tools] Building btor2tools"
run make -j"$(nproc)"

cd ../../..
