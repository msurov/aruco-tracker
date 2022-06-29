cmake_minimum_required(VERSION 3.2)

project(aruco_tracker CXX)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(jsoncpp REQUIRED)

if (MSVC)
    add_compile_definitions(_USE_MATH_DEFINES)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W2")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

add_subdirectory(3rd_party/cppmisc)
add_subdirectory(3rd_party/cppmath)
add_subdirectory(3rd_party/networking)

option(WITH_BASLER_CAMERA "Basler camera support" OFF)
option(WITH_FAKE_CAMERA "Fake camera (read photos from foler) support" ON)

set(CAMERA_SRC
    src/camera.h
    src/camera.cpp
)
set(CAMERA_DEPS ${OpenCV_LIBS})

if (WITH_FAKE_CAMERA)
    set(CAMERA_SRC ${CAMERA_SRC} src/camera_fake.cpp)
    add_definitions(-DBUILD_FAKE_CAMERA)
endif()

if (WITH_BASLER_CAMERA)
    find_package(pylon REQUIRED)
    add_definitions(BUILD_BASLER_CAMERA)
    set(CAMERA_SRC ${CAMERA_SRC} src/camera_basler_usb.cpp)
    set(CAMERA_DEPS ${CAMERA_DEPS} pylon)
    add_definitions(-DBUILD_BASLER_CAMERA)
endif()

add_executable(recorder 
    ${CAMERA_SRC}
    src/recorder.cpp
)
target_link_libraries(recorder PUBLIC cppmisc ${CAMERA_DEPS})

add_executable(aruco_tracker
    src/aruco_detector.h
    src/aruco_detector.cpp
    src/edges_fitting.h
    src/pnp_4pts.h
    src/pnp_4pts.cpp
    src/imgdump.cpp
    src/imgdump.h
    src/publisher.h
    src/publisher.cpp
    src/publisher_file.cpp
    src/publisher_jsontcp.cpp
    src/aruco_tracker.cpp
    ${CAMERA_SRC}
)
target_link_libraries(aruco_tracker PUBLIC
    cppmisc
    cppmath
    networking
    ${CMAKE_THREAD_LIBS_INIT}
    ${CAMERA_DEPS}
)
