#!/bin/bash

set -e  # Exit if any command fails

echo "Cleaning build directory..."
rm -rf build

echo "Creating build directory..."
mkdir build
cd build

echo "Running CMake..."
cmake ..

echo "Compiling..."
make

echo "✅ Build complete."

cd ..
