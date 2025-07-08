enable_testing()

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

add_executable(unit_tests test_example.cpp)
target_link_libraries(unit_tests Boost::unit_test_framework kalman_filter)

add_executable(kalman_step_tests test_kalman_step.cpp)
target_link_libraries(kalman_step_tests Boost::unit_test_framework Eigen3::Eigen kalman_filter)

add_executable(kalman_filter_tests test_kalman_filter.cpp)
target_link_libraries(kalman_filter_tests Boost::unit_test_framework Eigen3::Eigen kalman_filter)

add_test(NAME MyUnitTests COMMAND unit_tests)
add_test(NAME KalmanStepTests COMMAND kalman_step_tests)
add_test(NAME KalmanFilterTests COMMAND kalman_filter_tests)
