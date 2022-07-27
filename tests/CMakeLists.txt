add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

add_unit_test(pendulum pendulum.cpp)
target_link_libraries(pendulum PRIVATE ${PROJECT_NAME}
                                       Boost::unit_test_framework)

add_unit_test(controller controller.cpp)
target_link_libraries(controller PRIVATE ${PROJECT_NAME}
                                         Boost::unit_test_framework)

if(BUILD_PYTHON_INTERFACE)
  add_python_unit_test(simu "tests/simu.py" src)
endif(BUILD_PYTHON_INTERFACE)
