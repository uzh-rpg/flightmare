cmake_minimum_required(VERSION 3.0.0)

project(googletest-download NONE)

include(ExternalProject)
ExternalProject_Add(googletest
  GIT_REPOSITORY    https://github.com/google/googletest.git
  GIT_TAG           master
  SOURCE_DIR        "${PROJECT_SOURCE_DIR}/externals/googletest-src"
  BINARY_DIR        "${PROJECT_SOURCE_DIR}/externals/googletest-build"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
  UPDATE_DISCONNECTED ON
)