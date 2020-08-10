cmake_minimum_required(VERSION 3.0)

project(agilib-download NONE)

include(ExternalProject)

ExternalProject_Add(agilib
  GIT_REPOSITORY    git@github.com:uzh-rpg/agilicious.git
  GIT_TAG           master
  SOURCE_DIR        "${PROJECT_SOURCE_DIR}/externals/agilib-src"
  BINARY_DIR        "${PROJECT_SOURCE_DIR}/externals/agilib-bin"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
  UPDATE_DISCONNECTED ON
) 
