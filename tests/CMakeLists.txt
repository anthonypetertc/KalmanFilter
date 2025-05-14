enable_testing()

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

add_executable(unit_tests test_example.cpp)
target_link_libraries(unit_tests Boost::unit_test_framework kalman_filter)

add_test(NAME MyUnitTests COMMAND unit_tests)