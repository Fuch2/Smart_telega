#!/usr/bin/env bash
set -e
cd /Users/fuch/Document/Smart_Telega/smart-cart && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --target smart_cart_ui -j$(nproc)



