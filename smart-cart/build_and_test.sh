#!/usr/bin/env bash
set -e

rm -rf build/build_qt
cmake --build build -j$(nproc) --target smart_cart_ui


