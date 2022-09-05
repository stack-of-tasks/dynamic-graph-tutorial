#
# Copyright 2010, 2018-2020 CNRS Authors: Florent Lamiraux, Guilhem Saurel
#

cmake_minimum_required(VERSION 3.1)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME dynamic-graph-tutorial)
set(PROJECT_DESCRIPTION "Tutorial for dynamic-graph")
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
option(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)

# Project configuration
if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
set(CUSTOM_HEADER_DIR "dynamic-graph/tutorial")

# JRL-cmakemodule setup
include(cmake/base.cmake)
include(cmake/boost.cmake)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

# Project dependencies
add_project_dependency(dynamic-graph REQUIRED)
if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
endif(BUILD_TESTING)

if(BUILD_PYTHON_INTERFACE)
  add_project_dependency(dynamic-graph-python 4.0.6 REQUIRED)
  include(cmake/python.cmake) # TODO: overwriting eigenpy/python.cmake
endif(BUILD_PYTHON_INTERFACE)

if(Boost_VERSION GREATER 107299 OR Boost_VERSION_MACRO GREATER 107299)
  # Silence a warning about a deprecated use of boost bind by boost >= 1.73
  # without dropping support for boost < 1.73
  add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
endif()

# Main Library
set(${PROJECT_NAME}_HEADERS include/${CUSTOM_HEADER_DIR}/inverted-pendulum.hh
                            include/${CUSTOM_HEADER_DIR}/feedback-controller.hh)

set(${PROJECT_NAME}_SOURCES src/inverted-pendulum.cpp
                            src/feedback-controller.cpp)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC dynamic-graph::dynamic-graph)

if(SUFFIX_SO_VERSION)
  set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
endif(SUFFIX_SO_VERSION)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)

add_subdirectory(src)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif(BUILD_TESTING)

if(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
endif(NOT INSTALL_PYTHON_INTERFACE_ONLY)
