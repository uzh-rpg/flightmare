# Download and unpack agilicious at configure time
message(STATUS "Getting RPG agilicious...")

configure_file(
  cmake/agilib_download.cmake
  ${PROJECT_SOURCE_DIR}/externals/agilib-download/CMakeLists.txt)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/agilib-download/
  OUTPUT_QUIET
  ERROR_QUIET)
if(result)
  message(FATAL_ERROR "CMake step for agilib-cpp failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/externals/agilib-download
  OUTPUT_QUIET
  ERROR_QUIET)
if(result)
  message(FATAL_ERROR "Build step for agilib failed: ${result}")
endif()

message(STATUS "Agilicious downloaded!")

add_subdirectory(${PROJECT_SOURCE_DIR}/externals/agilib-src/agilib
                 ${PROJECT_SOURCE_DIR}/externals/agilib-build
                 EXCLUDE_FROM_ALL)
target_compile_options(agilib PRIVATE -w)

include_directories(SYSTEM "${PROJECT_SOURCE_DIR}/externals/agilib-src/agilib/include")
link_directories("${PROJECT_SOURCE_DIR}/externals/agilib-build")