cmake_minimum_required(VERSION 2.8)

project(aruco_tracker)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS_RELEASE "-Ofast -march=native -mtune=native")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

FIND_PACKAGE(jsoncpp REQUIRED)
INCLUDE_DIRECTORIES("/usr/include/jsoncpp")
SET(JSONCPP_LIBS jsoncpp)

include_directories(${CMAKE_SOURCE_DIR}/src)


# 
# Pylon
# 
execute_process(COMMAND /opt/pylon5/bin/pylon-config --cflags OUTPUT_VARIABLE PYLON_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /opt/pylon5/bin/pylon-config --libs OUTPUT_VARIABLE PYLON_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
    COMMAND /opt/pylon5/bin/pylon-config --libdir
    OUTPUT_VARIABLE PYLON_LIBDIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(PYLON_LIBS 
    ${PYLON_LIBDIR}/libpylonbase.so
    ${PYLON_LIBDIR}/libpylonutility.so
    ${PYLON_LIBDIR}/libGenApi_gcc_v3_0_Basler_pylon_v5_0.so
    ${PYLON_LIBDIR}/libGCBase_gcc_v3_0_Basler_pylon_v5_0.so
)
# target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libpylonbase.so)
# target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libpylonutility.so)
# target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libGenApi_gcc_v3_0_Basler_pylon_v5_0.so)
# target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libGCBase_gcc_v3_0_Basler_pylon_v5_0.so)
add_definitions("${PYLON_CFLAGS}")


# 
# misc
# 
ADD_LIBRARY(misc STATIC
	src/misc/traces.h
	src/misc/traces.cpp
	src/misc/threads.h
	src/misc/json.h
	src/misc/filters.h
	src/misc/splines.h
	src/misc/signals.h
	src/misc/signals.cpp
	src/misc/time.h
	src/misc/formatting.h
	src/misc/files.h
)
TARGET_LINK_LIBRARIES(misc ${CMAKE_THREAD_LIBS_INIT} ${JSONCPP_LIBRARIES} ${JSONCPP_LIBS})


# 
# math
# 
# ADD_LIBRARY(mmath STATIC
#     src/mmath/transforms.h
#     src/mmath/stdalg.h
# )
# TARGET_LINK_LIBRARIES(mmath ${OpenCV_LIBS})


# 
# cameras
# 
ADD_LIBRARY(cameras STATIC
    src/camera/camera.h
    # src/camera/camera_basler1300_200uc.cpp
    # src/camera/camera_fake.cpp
)
target_link_libraries(cameras PYLON_LIBS)
TARGET_LINK_LIBRARIES(cameras misc ${OpenCV_LIBS})


# 
# aruco
# 
ADD_LIBRARY(aruco STATIC
    src/aruco/aruco_detector.h
    src/aruco/aruco_detector.cpp
)
TARGET_LINK_LIBRARIES(aruco misc ${JSONCPP_LIBS})


# 
# tests
# 
add_executable(test_refine
    src/tests/test_refine.cpp
)
target_link_libraries(test_refine misc aruco ${OpenCV_LIBS})


# include_directories("${PROJECT_SOURCE_DIR}/src")

# target_link_libraries(aruco_tracker ${OpenCV_LIBS})
# target_link_libraries(aruco_tracker ${CMAKE_THREAD_LIBS_INIT})

# execute_process(COMMAND /opt/pylon5/bin/pylon-config --cflags OUTPUT_VARIABLE PYLON_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
# execute_process(COMMAND /opt/pylon5/bin/pylon-config --libs OUTPUT_VARIABLE PYLON_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
# execute_process(
#     COMMAND /opt/pylon5/bin/pylon-config --libdir
#     OUTPUT_VARIABLE PYLON_LIBDIR
#     OUTPUT_STRIP_TRAILING_WHITESPACE
# )
# target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libpylonbase.so)
# target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libpylonutility.so)
# target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libGenApi_gcc_v3_0_Basler_pylon_v5_0.so)
# target_link_libraries(aruco_tracker ${PYLON_LIBDIR}/libGCBase_gcc_v3_0_Basler_pylon_v5_0.so)
# add_definitions("${PYLON_CFLAGS}")


# add_executable(test_local_autocontrast
#     tests/test_local_autocontrast.cpp
#     src/common.cpp
#     src/common.h
#     src/traces.cpp
#     src/traces.h
#     src/jsonxx.cpp
#     src/jsonxx.h
#     src/imgdump.cpp
#     src/imgdump.h
#     src/image_saver.cpp
#     src/image_saver.h
#     src/aruco_detector.cpp
#     src/aruco_detector.h
#     src/cv_helpers.cpp
#     src/cv_helpers.h
#     src/cv_parallel.h
#     src/camera_fake.cpp
#     src/camera.h
#     src/transforms.h
# )
# target_link_libraries(test_local_autocontrast ${OpenCV_LIBS})
# target_link_libraries(test_local_autocontrast ${CMAKE_THREAD_LIBS_INIT})
