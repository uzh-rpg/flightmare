cmake_minimum_required(VERSION 3.0)

project(dodgelib-download NONE)

include(ExternalProject)

ExternalProject_Add(dodgelib
  GIT_REPOSITORY    git@github.com:uzh-rpg/agilicious.git
  GIT_TAG           master
  SOURCE_DIR        "${PROJECT_SOURCE_DIR}/externals/dodgelib-src"
  BINARY_DIR        "${PROJECT_SOURCE_DIR}/externals/dodgelib-bin"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
  UPDATE_DISCONNECTED ON
) 
