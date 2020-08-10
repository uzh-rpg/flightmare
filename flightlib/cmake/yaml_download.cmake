cmake_minimum_required(VERSION 3.0.0)

project(yaml-download)

include(ExternalProject)
ExternalProject_Add(yaml
  GIT_REPOSITORY    https://github.com/jbeder/yaml-cpp.git
  GIT_TAG           master
  SOURCE_DIR        "${PROJECT_SOURCE_DIR}/externals/yaml-src"
  BINARY_DIR        "${PROJECT_SOURCE_DIR}/externals/yaml-bin"
  CONFIGURE_COMMAND ""
  CMAKE_ARGS        "-DBUILD_TESTING=OFF -DYAML_CPP_INSTALL=OFF"
  CMAKE_CACHE_ARGS  -DBUILD_TESTING:BOOL=OFF -DYAML_CPP_INSTALLC:BOOL=ON
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
  UPDATE_DISCONNECTED ON
)