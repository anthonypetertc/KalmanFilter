#!/usr/bin/env bash
set -e  # exit on first error

# 1. Clean previous build artifacts
rm -rf build

# 2. Bootstrap vcpkg once per clone
if [ ! -d "./vcpkg" ]; then
  git clone https://github.com/microsoft/vcpkg.git
  ./vcpkg/bootstrap-vcpkg.sh
fi

# 3. Configure (this creates build/ automatically)
cmake -S . -B build \
      -DCMAKE_TOOLCHAIN_FILE="$PWD/vcpkg/scripts/buildsystems/vcpkg.cmake" \
      -DVCPKG_TARGET_TRIPLET=x64-linux      # adjust triplet as needed

# 4. Build
cmake --build build --parallel

echo "✅ Build complete."
