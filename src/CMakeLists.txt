add_library(kalman_filter example.cpp kalman_step.cpp)
target_link_libraries(kalman_filter PUBLIC Eigen3::Eigen)

add_executable(kalman_filter_main main.cpp)
target_link_libraries(kalman_filter_main PRIVATE kalman_filter)
