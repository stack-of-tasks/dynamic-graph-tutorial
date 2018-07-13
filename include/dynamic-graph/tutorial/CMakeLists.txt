SET(${PROJECT_NAME}_HEADERS
  inverted-pendulum.hh
  feedback-controller.hh
  )

INSTALL(FILES
  ${${PROJECT_NAME}_HEADERS}
  DESTINATION include/dynamic-graph/tutorial/
  )
