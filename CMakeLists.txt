# Set the directory of dependent libraries
set(ZHNMAT_DIR D:/cppcode/zhnmat)  # directory of zhnmat
set(MPLT_DIR D:/cppcode/matplotlibcpp)  # directory of matplotlib-cpp
set(PYTHON_HOME D:/python)  # directory of python

# If you don't wan't to use the cooresponding dependent libraries,
#  change the following options to "OFF"
option(USE_ZHNMAT "If you can use zhnmat." ON)
option(USE_MPLT "If you can use matplotlib-cpp." ON)

cmake_minimum_required(VERSION 3.12)
project(simucpp VERSION 1.7.12)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE release)
configure_file(config.h.in config.h)

message(STATUS "project name: ${CMAKE_PROJECT_NAME}")
message(STATUS "project version: ${PROJECT_VERSION}")
message(STATUS "project build type: ${CMAKE_BUILD_TYPE}")

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
if(USE_MPLT)
    message(STATUS "matplotlib-cpp dir: ${MPLT_DIR}")
    message(STATUS "python dir: ${PYTHON_HOME}")
    add_definitions(-DUSE_MPLT)
    target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
        D:/python/libs/_tkinter.lib
        D:/python/libs/python3.lib
        D:/python/libs/python39.lib
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${MPLT_DIR}
        D:/python/include
        D:/python/Lib/site-packages/numpy/core/include
    )
endif()
