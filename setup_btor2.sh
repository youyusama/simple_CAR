#!/bin/bash

set -euo pipefail

rm -rf deps
mkdir deps

#Setup btor2aiger
git clone https://github.com/hwmcc/btor2tools.git deps/btor2tools
cp btor2aiger.cpp deps/btor2tools/src/btor2aiger.cpp

cd deps/btor2tools/
./setup-deps.sh
./configure.sh --btor2aiger

cd build/
make

cd ../../..