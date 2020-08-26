# Download and unpack eigen at configure time
message(STATUS "Getting yaml-cpp...")

configure_file(
  cmake/yaml_download.cmake
  ${PROJECT_SOURCE_DIR}/externals/yaml-download/CMakeLists.txt)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . 
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/yaml-download
  OUTPUT_QUIET
  ERROR_QUIET)
if(result)
  message(FATAL_ERROR "CMake step for yaml-cpp failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/yaml-download
  OUTPUT_QUIET
  ERROR_QUIET)
if(result)
  message(FATAL_ERROR "Build step for yaml failed: ${result}")
endif()

message(STATUS "Yaml downloaded!")

add_subdirectory(${PROJECT_SOURCE_DIR}/externals/yaml-src
                 ${PROJECT_SOURCE_DIR}/externals/yaml-build
                 EXCLUDE_FROM_ALL)
target_compile_options(yaml-cpp PUBLIC -fPIC -w)

include_directories(SYSTEM "${PROJECT_SOURCE_DIR}/externals/yaml-src/include")
link_directories("${PROJECT_SOURCE_DIR}/externals/yaml-build")