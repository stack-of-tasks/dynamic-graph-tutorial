if(BUILD_PYTHON_INTERFACE)
  python_install_on_site("dynamic_graph/tutorial" simu.py)
  dynamic_graph_python_module(
    "tutorial" ${PROJECT_NAME} tutorial-wrap MODULE_HEADER
    "${CMAKE_CURRENT_SOURCE_DIR}/tutorial-python.h")
endif(BUILD_PYTHON_INTERFACE)
