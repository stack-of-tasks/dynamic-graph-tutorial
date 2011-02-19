# Copyright 2010, Florent Lamiraux, CNRS
#
# This file is part of sot-core.
# sot-core is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-core is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph-tutorial. If not, see <http://www.gnu.org/licenses/>.

# Libraries DEFINITIONS and INCLUDES

INCLUDE(../cmake/python.cmake)

SET(LIBRARY_NAME ${PROJECT_NAME})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

# provide path to dependency libraries
LINK_DIRECTORIES(${DYNAMIC_GRAPH_PYTHON_LIBRARY_DIRS})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  inverted-pendulum.cc
  feedback-controller.cc
  )

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

SET_TARGET_PROPERTIES(${PROJECT_NAME_LIB}
  PROPERTIES
  COMPILE_FLAGS ${${PROJECT_NAME}_src_CXXFLAGS}
  LINK_FLAGS ${${PROJECT_NAME}_src_LDFLAGS}
)

# Add dynamic-graph-python compilation flags and link to dependency libraries
ADD_DEFINITIONS(${DYNAMIC_GRAPH_PYTHON_CFLAGS})
IF(UNIX)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} ${DYNAMIC_GRAPH_PYTHON_LIBRARIES})
ENDIF(UNIX)

IF(WIN32)
foreach(dlink ${DYNAMIC_GRAPH_PYTHON_LDFLAGS})
	SET ( ${PROJECT_NAME}_src_LDFLAGS "${${PROJECT_NAME}_src_LDFLAGS}  ${dlink}")
endforeach(dlink)
ENDIF(WIN32)

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  LINK_FLAGS "${${PROJECT_NAME}_src_LDFLAGS}"
)

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

SET(NEW_ENTITY_CLASS
  InvertedPendulum
  FeedbackController
  )

DYNAMIC_GRAPH_PYTHON_MODULE("tutorial" ${LIBRARY_NAME} wrap)
