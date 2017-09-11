cmake_minimum_required(VERSION 2.8)

project(aruco_tracker)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -Ofast")

if (${CMAKE_BUILD_TYPE} EQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
endif()

add_executable(aruco_tracker
    src/common.cpp
    src/common.h
    src/traces.cpp
    src/traces.h
    src/jsonxx.cpp
    src/jsonxx.h
    src/imgdump.cpp
    src/imgdump.h
    src/image_saver.cpp
    src/image_saver.h
    src/aruco_detector.cpp
    src/aruco_detector.h
    src/cv_helpers.cpp
    src/cv_helpers.h
    src/cv_parallel.h
    src/tracker.cpp
    # src/camera_basler1300_200uc.cpp
    src/camera_fake.cpp
    src/camera.h
    src/transforms.h
)

target_link_libraries(aruco_tracker ${OpenCV_LIBS})
target_link_libraries(aruco_tracker ${CMAKE_THREAD_LIBS_INIT})

execute_process(COMMAND /opt/pylon5/bin/pylon-config --cflags OUTPUT_VARIABLE PYLON_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /opt/pylon5/bin/pylon-config --libs OUTPUT_VARIABLE PYLON_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
    COMMAND /opt/pylon5/bin/pylon-config --libdir
    OUTPUT_VARIABLE PYLON_LIBDIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libpylonbase.so)
target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libpylonutility.so)
target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libGenApi_gcc_v3_0_Basler_pylon_v5_0.so)
target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libGCBase_gcc_v3_0_Basler_pylon_v5_0.so)
add_definitions("${PYLON_CFLAGS}")
