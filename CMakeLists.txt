cmake_minimum_required(VERSION 3.15)
project(KalmanFilter)

set(CMAKE_CXX_STANDARD 17)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add include directory
include_directories(include)
include_directories("/opt/homebrew/opt/eigen/include/eigen3")

# Add subdirectories
add_subdirectory(src)
add_subdirectory(tests)
