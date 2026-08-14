#!/usr/bin/env bash
set -euo pipefail

NATIVE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${NATIVE_DIR}/build"
SOURCE="${NATIVE_DIR}/target_overlay.cpp"
OUTPUT="${BUILD_DIR}/libgstbdtargetoverlay.so"

mkdir -p "${BUILD_DIR}"
g++ -std=c++17 -O3 -DNDEBUG -fPIC -shared \
    -I/usr/include/hailo/tappas \
    "${SOURCE}" -o "${OUTPUT}" \
    $(pkg-config --cflags --libs gstreamer-video-1.0 gsthailometa)

echo "Native overlay hazır: ${OUTPUT}"
