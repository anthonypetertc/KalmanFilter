cmake_minimum_required(VERSION 3.15)
project(KalmanFilter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# --- Fetch Eigen ---
include(FetchContent)

FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0  
)

FetchContent_MakeAvailable(Eigen)

# --- Include directories ---
include_directories(include)
# include_directories("/opt/homebrew/opt/eigen/include/eigen3")

# --- Subdirectories ---
add_subdirectory(src)
add_subdirectory(tests)
