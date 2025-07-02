#!/bin/bash

set -e  # Exit if any command fails

cd build/tests
ctest
cd ..
