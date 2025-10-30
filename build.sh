#!/usr/bin/env bash

set -euo pipefail

# Simple build helper for CMake
# Usage:
#   ./build.sh                       # Release build
#   ./build.sh debug                 # Debug build
#   ./build.sh clean                 # Remove build and bin
#   ./build.sh --clean [release|debug]  # Clean then fresh build

CLEAN_FIRST=0

if [[ "${1:-}" == "--clean" ]]; then
  CLEAN_FIRST=1
  shift || true
fi

BUILD_TYPE=${1:-release}

case "$BUILD_TYPE" in
  clean)
    rm -rf build bin
    echo "Cleaned build/ and bin/"
    exit 0
    ;;
  debug|Debug)
    CMAKE_BUILD_TYPE=Debug
    ;;
  release|Release)
    CMAKE_BUILD_TYPE=Release
    ;;
  *)
    echo "Unknown option: $BUILD_TYPE"
    echo "Usage: ./build.sh [release|debug|clean|--clean [release|debug]]"
    exit 1
    ;;
esac

if [[ "$CLEAN_FIRST" == "1" ]]; then
  rm -rf build bin
  echo "Cleaned build/ and bin/"
fi

cmake -S . -B build -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
cmake --build build --parallel

echo "Build complete. Binaries in ./build" 


