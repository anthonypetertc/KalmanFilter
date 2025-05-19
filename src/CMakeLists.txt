add_library(kalman_filter example.cpp kf.cpp)
add_executable(kalman_filter_main main.cpp)
target_link_libraries(kalman_filter_main kalman_filter)