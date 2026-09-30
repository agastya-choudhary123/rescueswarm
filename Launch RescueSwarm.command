#!/bin/zsh
set -e

PROJECT_DIR="${0:A:h}"
cd "$PROJECT_DIR"

if [[ ! -x build/rescueswarm ]]; then
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
  cmake --build build -j "$(sysctl -n hw.logicalcpu)"
fi

exec ./build/rescueswarm --scenario "$PROJECT_DIR/scenarios/city.json" --seed 42
