cmake_minimum_required(VERSION 3.2)

find_package(GTest REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(test_rotations test_rotations.cpp ../src/rotations.h)
target_include_directories(test_rotations PUBLIC "../src")
target_link_libraries(test_rotations ${CMAKE_THREAD_LIBS_INIT} GTest::gtest GTest::gtest_main ${OpenCV_LIBS})
add_test(NAME test_rotations COMMAND test_rotations)
