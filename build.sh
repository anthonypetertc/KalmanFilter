#!/usr/bin/bash
set -e  # Exit if any command fails

echo "Cleaning build directory..."
rm -rf build

#Pull vcpkg if doesn't already exist.
if [ ! -d "./vcpkg" ]; then
  git clone https://github.com/microsoft/vcpkg.git
  ./vcpkg/bootstrap-vcpkg.sh
fi

echo "Creating build directory..."
cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Debug \
      -DCMAKE_CXX_FLAGS="-g3 -O0 -fno-omit-frame-pointer -fno-inline" \
      -DCMAKE_TOOLCHAIN_FILE="$PWD/vcpkg/scripts/buildsystems/vcpkg.cmake" \
      -DVCPKG_TARGET_TRIPLET=x64-linux      # adjust triplet as needed

# 4. Build
cmake --build build --parallel

echo "✅ Build complete."
