
add_executable(example example_usage.cpp)
target_link_libraries(example PRIVATE kalman_filter Matplot++::matplot)
