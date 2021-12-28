cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)
message("Current Cmake version is : " ${CMAKE_VERSION})

project(my_Intelligent_driving)

########################
# set compile options  #
########################
#delete the cache to rebuild after changing the options
option(BUILD_WITH_OPENCV "Build with OpenCV2 for Image related Processing" ON)
option(BUILD_WITH_TEST "Build with TEST" ON)
option(BUILD_WITH_VISUAL "BUILD with visual" OFF)
message("Build with TEST: " ${BUILD_WITH_TEST})

set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -pthread")

########################
# find libs & include  #
########################
#OpenCV3
if (BUILD_WITH_OPENCV)
    find_package(OpenCV 3 REQUIRED)
    include_directories(${OpenCV_INCLUDE_DIRS})
    message("OPENCV [OK]: " ${OpenCV_INCLUDE_DIRS})
    add_definitions(-DOPENCV_ON)
endif (BUILD_WITH_OPENCV)

########################
#      link libs       #
########################
if (BUILD_WITH_OPENCV)
    #link opencv (optional)
    set(DEP_LIBS ${DEP_LIBS} ${OpenCV_LIBS})
endif (BUILD_WITH_OPENCV)

########################
#   add executables    #
########################
#my_ID

if (BUILD_WITH_TEST STREQUAL "ON")
    add_library(my_lib src/serial/Serial.cpp)
    add_executable(my_Intelligent_driving src/test_main.cpp)
else()
    add_library(my_lib 
    src/drive/drive.cpp
    src/extract/f_extract.cpp
    src/serial/Serial.cpp
    src/elude/elude.cpp
    src//Initial/Initial.cpp
    src/extract/f_extract.cpp
    src/Frame/Frame.cpp
    src/ORBextractor/ORBextractor.cpp)
    add_executable(my_Intelligent_driving src/Intel_dri_main.cpp)
endif ()
target_link_libraries(my_Intelligent_driving ${DEP_LIBS})
target_link_libraries(my_Intelligent_driving my_lib)