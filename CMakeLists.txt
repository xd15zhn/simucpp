cmake_minimum_required(VERSION 3.12)
project(simucpp VERSION 2.0.14)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE release)

message(STATUS "project name: ${CMAKE_PROJECT_NAME}")
message(STATUS "project version: ${PROJECT_VERSION}")
message(STATUS "project build type: ${CMAKE_BUILD_TYPE}")

set(SIMUCPP_SOURCES
    ${PROJECT_SOURCE_DIR}/src/unitmodules.cpp
    ${PROJECT_SOURCE_DIR}/src/packmodules.cpp
    ${PROJECT_SOURCE_DIR}/src/matmodules.cpp
    ${PROJECT_SOURCE_DIR}/src/simulator.cpp
    ${PROJECT_SOURCE_DIR}/src/connector.cpp
    ${PROJECT_SOURCE_DIR}/src/debugs.cpp
)

if (WIN32)
    MESSAGE(STATUS "System is WINDOWS.")
    set(CMAKE_INSTALL_LIBDIR ${CMAKE_INSTALL_PREFIX}/lib)
    set(CMAKE_INSTALL_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
elseif (UNIX)
    MESSAGE(STATUS "System is UNIX.")
    add_compile_options(-Wno-psabi)
    include(GNUInstallDirs)
endif ()
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "INSTALL_CONFIGDIR: ${INSTALL_CONFIGDIR}")
message(STATUS "CMAKE_INSTALL_INCLUDEDIR: ${CMAKE_INSTALL_INCLUDEDIR}")

option(USE_ZHNMAT "Dependent library zhnmat, used for matrix modules." ON)
option(USE_MPLT "Dependent library matplotlibcpp, used to plot waves." ON)
option(USE_TRACELOG "Dependent library tracelog, used to print logs." ON)
option(SUPPORT_DEBUG "Print more informations about simulators and modules." ON)

add_library(${CMAKE_PROJECT_NAME} STATIC ${SIMUCPP_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/inc>
)
if(USE_TRACELOG)
    message(STATUS "Use dependent library tracelog.")
    find_package(tracelog REQUIRED)
    message(STATUS "tracelog_VERSION: ${tracelog_VERSION}")
    message(STATUS "tracelog_DIR: ${tracelog_DIR}")
    message(STATUS "tracelog_LIBS: ${tracelog_LIBS}")
    message(STATUS "tracelog_INCLUDE_DIRS: ${tracelog_INCLUDE_DIRS}")
    add_definitions(-DUSE_TRACELOG)
    target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC ${tracelog_LIBS})
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${tracelog_INCLUDE_DIRS}>)
endif()
if(USE_ZHNMAT)
    message(STATUS "Use dependent library zhnmat.")
    find_package(zhnmat REQUIRED)
    message(STATUS "zhnmat_VERSION: ${zhnmat_VERSION}")
    message(STATUS "zhnmat_DIR: ${zhnmat_DIR}")
    message(STATUS "zhnmat_LIBS: ${zhnmat_LIBS}")
    message(STATUS "zhnmat_INCLUDE_DIRS: ${zhnmat_INCLUDE_DIRS}")
    add_definitions(-DUSE_ZHNMAT)
    target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC ${zhnmat_LIBS})
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${zhnmat_INCLUDE_DIRS}>)
endif()
if(USE_MPLT)
    message(STATUS "Use dependent library matplotlibcpp.")
    find_package(matplotlibcpp REQUIRED)
    message(STATUS "matplotlibcpp_VERSION: ${matplotlibcpp_VERSION}")
    message(STATUS "matplotlibcpp_DIR: ${matplotlibcpp_DIR}")
    message(STATUS "matplotlibcpp_LIBS: ${matplotlibcpp_LIBS}")
    message(STATUS "matplotlibcpp_INCLUDE_DIRS: ${matplotlibcpp_INCLUDE_DIRS}")
    add_definitions(-DUSE_MPLT)
    target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC ${matplotlibcpp_LIBS})
    target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${matplotlibcpp_INCLUDE_DIRS}>)
endif()
if (SUPPORT_DEBUG)
    MESSAGE(STATUS "Support simulator debug functions.")
    add_definitions(-DSUPPORT_DEBUG)
endif ()

include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)
install(TARGETS ${PROJECT_NAME}
    EXPORT install_targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/
)
install(EXPORT install_targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${INSTALL_CONFIGDIR}
)
install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/baseclass.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/matmodules.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/packmodules.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/simucpp.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/simulator.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/unitmodules.hpp
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)
