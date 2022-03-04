cmake_minimum_required(VERSION 3.12)
project(simucpp VERSION 1.7.3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE release)
set(ZHNMAT_DIR D:/cppcode/zhnmat)
configure_file(config.h.in config.h)
message(STATUS "project name: ${CMAKE_PROJECT_NAME}")
message(STATUS "project version: ${PROJECT_VERSION}")
message(STATUS "project build type: ${CMAKE_BUILD_TYPE}")

option(USE_ZHNMAT "If you can use zhnmat." OFF)

set(SIMUCPP_SOURCES
    ${PROJECT_SOURCE_DIR}/src/unitmodules.cpp
    ${PROJECT_SOURCE_DIR}/src/packmodules.cpp
    ${PROJECT_SOURCE_DIR}/src/simulator.cpp
)

add_library(${CMAKE_PROJECT_NAME} STATIC ${SIMUCPP_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "${PROJECT_SOURCE_DIR}/inc"
    "${PROJECT_BINARY_DIR}"
)
if(USE_ZHNMAT)
    message(STATUS "zhnmat dir: ${ZHNMAT_DIR}")
    add_definitions(-DUSE_ZHNMAT)
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE ${ZHNMAT_DIR}/build/libzhnmat.a)
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${ZHNMAT_DIR})
endif()
