#!/bin/bash
# Build the solver: one translation unit, about 15 s. Run from the repository root.
set -e
cd "$(dirname "$0")"
g++ -std=c++20 -O3 -I HOG2/gui/STUB src/main.cpp -o rcpsp "$@"
echo "built ./rcpsp"
