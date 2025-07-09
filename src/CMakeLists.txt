add_library(kalman_filter kalman_step.cpp kalman_filter.cpp)
target_link_libraries(kalman_filter PUBLIC Eigen3::Eigen)

add_executable(kalman_filter_main main.cpp)
target_link_libraries(kalman_filter_main PRIVATE kalman_filter)
