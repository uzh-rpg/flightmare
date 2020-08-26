cmake_minimum_required(VERSION 3.0.0)

project(pybind11-download)

include(ExternalProject)
ExternalProject_Add(pybind11
  GIT_REPOSITORY    https://github.com/pybind/pybind11.git
  GIT_TAG           master 
  SOURCE_DIR        "${PROJECT_SOURCE_DIR}/externals/pybind11-src"
  BINARY_DIR        "${PROJECT_SOURCE_DIR}/externals/pybind11-bin"
  CONFIGURE_COMMAND ""
  CMAKE_ARGS        ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
  UPDATE_DISCONNECTED ON
)