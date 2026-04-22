#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

jobs="$(getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target \
    smart_cart_ui \
    frame_codec_tests \
    domain_tests \
    integration_tests \
    -j"${jobs}"
ctest --test-dir build --output-on-failure
