if(GTest_FOUND)
    add_executable(perception-unit-tests unit/gtest_suite.cpp)
    target_link_libraries(perception-unit-tests PRIVATE perception::core perception_warnings
                                                        GTest::gtest_main)
else()
    message(STATUS "GoogleTest not found: using the dependency-free host test harness")
    add_executable(
        perception-unit-tests
        unit/test_main.cpp
        unit/bounded_queue_test.cpp
        unit/config_test.cpp
        unit/camera_health_test.cpp)
    target_link_libraries(perception-unit-tests PRIVATE perception::core perception_warnings)
endif()
add_test(NAME perception-unit-tests COMMAND perception-unit-tests)
