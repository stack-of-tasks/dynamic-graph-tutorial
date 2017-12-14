#
#  Copyright 2010 CNRS
#  Author: Florent Lamiraux
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET (CXX_DISABLE_WERROR ON)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME dynamic-graph-tutorial)
project(${PROJECT_NAME})

SET(PROJECT_VERSION 1.0)
SET(PROJECT_DESCRIPTION "Tutorial for dynamic-graph")
SET(PROJECT_URL "http://github.com/jrl-umi3218/dynamic-graph-tutorial")

SET(${PROJECT_NAME}_HEADERS
    include/dynamic-graph/tutorial/inverted-pendulum.hh
    include/dynamic-graph/tutorial/feedback-controller.hh
)
#
# Look for dependencies
#

SETUP_PROJECT()

#
#  Documentation dependency to package dynamic-graph
#

ADD_REQUIRED_DEPENDENCY("dynamic-graph")

#
#  Dependency to package dynamic-graph-python
#
ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(doc)

SETUP_PROJECT_FINALIZE()
