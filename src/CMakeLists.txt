# Copyright 2010, 2018, Florent Lamiraux, CNRS
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


SET(${PROJECT_NAME}_SOURCES
  inverted-pendulum
  feedback-controller
  )

SET(${PY_NAME}_PYTHON
  simu.py
  )

ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} dynamic-graph)

IF(BUILD_PYTHON_INTERFACE)
  PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} dynamic-graph-python)
  DYNAMIC_GRAPH_PYTHON_MODULE("tutorial" ${PROJECT_NAME} wrap)
  INSTALL(FILES ${${PY_NAME}_PYTHON} DESTINATION "${PYTHON_SITELIB}/dynamic_graph/tutorial/")
ENDIF(BUILD_PYTHON_INTERFACE)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib)
