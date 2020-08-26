# Download and unpack pybind11 at configure time
message(STATUS "Getting Pybind11...")

# set(PYBIND11_PYTHON_VERSION 3.6)
set(PYBIND11_PYTHON_VERSION ${PYTHON_VERSION_STRING})   

configure_file(
  cmake/pybind11_download.cmake
  ${PROJECT_SOURCE_DIR}/externals/pybind11-download/CMakeLists.txt)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . 
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/pybind11-download
  OUTPUT_QUIET)
if(result)
  message(FATAL_ERROR "Cmake Step for Pybind11 failed: ${result}")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/pybind11-download
  OUTPUT_QUIET)
if(result)
  message(FATAL_ERROR "Build step for eigen failed: ${result}")
endif()

message(STATUS "Pybind11 downloaded!")

set(PYBIND11_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include)
add_subdirectory(${PROJECT_SOURCE_DIR}/externals/pybind11-src
                 EXCLUDE_FROM_ALL)

include_directories(SYSTEM "${PROJECT_SOURCE_DIR}/externals/pybind11-src/include")