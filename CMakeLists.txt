cmake_minimum_required(VERSION 3.12)
project(simucpp)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE release)
set(ZHNMAT_DIR D:/cppcode/zhnmat)
message(STATUS "project name: ${CMAKE_PROJECT_NAME}")
message(STATUS "project build type: ${CMAKE_BUILD_TYPE}")

option(USE_ZHNMAT "If you can use zhnmat." OFF)

set(SIMUCPP_SOURCES
    ${PROJECT_SOURCE_DIR}/src/unitmodules.cpp
    ${PROJECT_SOURCE_DIR}/src/packmodules.cpp
    ${PROJECT_SOURCE_DIR}/src/simulator.cpp
)

add_library(${CMAKE_PROJECT_NAME} STATIC ${SIMUCPP_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC "${PROJECT_SOURCE_DIR}/inc")
if(USE_ZHNMAT)
    message(STATUS "zhnmat dir: ${ZHNMAT_DIR}")
    add_definitions(-DUSE_ZHNMAT)
    target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC ${ZHNMAT_DIR}/build/libzhnmat.a)
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC ${ZHNMAT_DIR})
endif()
