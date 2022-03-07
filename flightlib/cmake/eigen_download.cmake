cmake_minimum_required(VERSION 3.0.0)

project(eigen-external)

include(ExternalProject)
ExternalProject_Add(eigen
  GIT_REPOSITORY    https://gitlab.com/libeigen/eigen.git
  GIT_TAG           3.3.4 
  SOURCE_DIR        "${PROJECT_SOURCE_DIR}/externals/eigen/eigen3"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
  UPDATE_DISCONNECTED ON
)