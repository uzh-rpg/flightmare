cmake_minimum_required(VERSION 3.0)

project(flightlib VERSION 0.1.0)

message(STATUS "======================  !Flightmare!  ======================")

################################################################################
# Options
################################################################################
option(BUILD_TESTS "Building the tests" ON)
option(BUILD_UNITY_BRIDGE_TESTS "Building the Unity Bridge tests" ON)
option(BUILD_BENCH "Building the benchmark." OFF)
option(ENABLE_FAST "Build with optimizations for speed" ON)
option(ENABLE_BLAS "Build using BLAS and LAPACK libraries" OFF)
option(ENABLE_PARALLEL "Build using openmp parallelization" ON)
option(EIGEN_FROM_SYSTTEM "Use the system-provided Eigen" OFF)
set(
  EIGEN_ALTERNATIVE "" CACHE STRING
  "Path to alternative Eigen, autodownload if blank"
)

################################################################################
# Finding Dependencies
################################################################################

message(STATUS "======> Setup Dependencies")
if(EIGEN_FROM_SYSTTEM)
  find_package(Eigen3 3.3.4 QUIET)
  if(EIGEN3_FOUND)
    message(STATUS "Using system provided Eigen.")
    message(${EIGEN3_INCLUDE_DIR})
  else()
    message(STATUS "No sufficient Eigen version (3.3.9) found.")
    message(STATUS "Restoring to download Eigen sources.")
    include(cmake/eigen.cmake)
  endif()
elseif(EIGEN_ALTERNATIVE STREQUAL "")
  include(cmake/eigen.cmake)
else()
  set(EIGEN3_INCLUDE_DIR ${EIGEN_ALTERNATIVE})
endif()

message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")

# Including dependencies 
find_package(yaml-cpp)
if  (NOT yaml-cpp_FOUND)
  message(STATUS "YAML not found, downloading...")
  include(cmake/yaml.cmake)
endif  ()

# Including dependencies 
find_package(OpenCV REQUIRED)
message("Opencv Version ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(OpenMP REQUIRED)

if(ENABLE_BLAS)
  set(BLA_VENDOR "Generic")
  find_package(BLAS REQUIRED)
  if(BLAS_FOUND)
    message(STATUS "Found BLAS: ${BLAS_LIBRARIES}")
  else()
    message(ERROR "Could not enable BLAS because BLAS was not found")
  endif()
  find_package(LAPACK REQUIRED)
  if(LAPACK_FOUND)
    message(STATUS "Found Lapack: ${LAPACK_LIBRARIES}")
  else()
    message(ERROR "Could not enable LAPACK because LAPACK was not found")
  endif()
endif()

# Check for ccache
if(NOT DEFINED CATKIN_DEVEL_PREFIX)
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
  else()
    message(INFO "Build time could be improved with ccache!")
    message(INFO "    sudo apt install ccache")
  endif()
endif()

################################################################################
# Setup Compilation
################################################################################
message(STATUS "======> Setup Compilation")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_definitions(-fno-finite-math-only)
add_definitions(-Werror -Wno-int-in-bool-context)
add_definitions(-Wall)

include_directories(include)
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=1048576)
include_directories(${EIGEN3_INCLUDE_DIR} "tests")
include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${Torch_INCLUDE_DIRS})

# Set default build type
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

# Add c++ flags 
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -fPIC -Wall -DNDEBUG  -fopenmp")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS} -fPIC -Wall -DNDEBUG  -fopenmp")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -fPIC -Wall -g -fopenmp")
set(CMAKE_CXX_STANDARD 17)

# Architectural flags
if("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
  message(STATUS "Using ARMv7 optimized flags!")
  set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
  message(STATUS "Using ARM aarch64 optimized flags!")
  set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else()
  set(CMAKE_CXX_ARCH_FLAGS " -march=native")
endif()

# Optimized flags
if(ENABLE_FAST)
  message(STATUS "Enabling fast optimization flags!")
  set(CMAKE_CXX_FAST_FLAGS " -Ofast")
else()
  set(CMAKE_CXX_FAST_FLAGS " -O0")
endif()


# BLAS Flags
if(BLAS_FOUND AND LAPACK_FOUND)
  message(STATUS "Enabling BLAS and LAPACK")
  set(CMAKE_CXX_BLAS_FLAGS " -DEIGEN_USE_BLAS -DEIGEN_USE_LAPACK -DEIGEN_USE_LAPACKE")
else()
  set(CMAKE_CXX_BLAS_FLAGS "")
endif()

# Summarize Flags
set(CMAKE_CXX_FLAGS_RELEASE
  "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FAST_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} ${CMAKE_CXX_PAR_FLAGS}")
string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
message(STATUS "The activated CXX RELEASE configuration is:\n ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "The activated CXX DEBUG configuration is:\n ${CMAKE_CXX_FLAGS_DEBUG}")

################################################################################
# Specify Build Resources
################################################################################
message(STATUS "======>  Setup Build  ")

# Create file lists for flightlib source
file(GLOB_RECURSE FLIGHTLIB_SOURCES
  src/bridges/*.cpp
  src/dynamics/*.cpp
  src/objects/*.cpp
  src/sensors/*.cpp
  src/envs/*.cpp
  src/common/*.cpp
  src/controller/*.cpp
)

# Create file lists for flightlib tests
file(GLOB_RECURSE FLIGHTLIB_TEST_SOURCES
  tests/dynamics/*.cpp
  tests/objects/*.cpp
  tests/sensors/*.cpp  
  tests/envs/*.cpp
  tests/common/*.cpp
)

# Create file lists for flightlib_gym source 
file(GLOB_RECURSE FLIGHTLIB_GYM_SOURCES
  src/wrapper/*.cpp 
)

# Create file lists for flightlib_gym tests
file(GLOB_RECURSE FLIGHTLIB_GYM_TEST_SOURCES
  tests/wrapper/*.cpp
)

# Create file lists for flightlib_unity_bridge tests
file(GLOB_RECURSE FLIGHTLIB_UNITY_BRIDGE_TEST_SOURCES
  tests/bridges/*.cpp
)

################################################################################
# Optional Catkin Build
################################################################################

if(DEFINED CATKIN_DEVEL_PREFIX)
  message(STATUS "======>  Building with -- catkin -- ")
  include(cmake/catkin.cmake)
  return()
else()
  include(cmake/pybind11.cmake)
endif()

################################################################################
# Setup Build
################################################################################

# Setup Testing and Benchmark
if(BUILD_TESTS OR BUILD_BENCH)
  include(cmake/gtest.cmake)
enable_testing()
endif()

if(NOT FLIGHTLIB_SOURCES)
	set(LIBRARY_NAME)
else()
  # flightlib
  add_library(${PROJECT_NAME} ${FLIGHTLIB_SOURCES})
  target_link_libraries(${PROJECT_NAME} PRIVATE
    ${OpenCV_LIBRARIES}
    yaml-cpp
    zmq
    zmqpp)
  set(LIBRARY_NAME ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
endif()

if(FLIGHTLIB_GYM_SOURCES)
  # flightlib_gym (python3 binding with Pybind11)
  pybind11_add_module(flightgym MODULE 
    ${FLIGHTLIB_GYM_SOURCES})
  if(EIGEN3_FOUND)
    target_include_directories(flightgym PRIVATE
      # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
      ${PYBIND11_INCLUDE_DIR}
      ${PROJECT_SOURCE_DIR}/include
      ${EIGEN3_INCLUDE_DIR})  
  else()
    target_include_directories(flightgym PRIVATE
      # ${PROJECT_SOURCE_DIR}/externals/pybind11-src/include
      ${PYBIND11_INCLUDE_DIR}
      ${PROJECT_SOURCE_DIR}/externals/eigen/eigen3 #pybind11 use #include <Eigen/Core>, however, flightmare use #include <eigen3/Eigen/Core>
      ${PROJECT_SOURCE_DIR}/include)  
  endif()
  message("${LIBRARY_NAME}")
  target_link_libraries(flightgym PRIVATE ${LIBRARY_NAME})
endif()

if(ENABLE_BLAS AND BLAS_FOUND AND LAPACK_FOUND)
  message(STATUS "Linking standard BLAS ${BLAS_LIBRARIES}")
  target_link_libraries(${LIBRARY_NAME}
    ${BLAS_LIBRARIES}
    ${LAPACK_LIBRARIES}
    ${LAPACKE_LIBRARIES}
  )
endif()

# Build tests for flightlib gym wrapper
if(BUILD_TESTS AND FLIGHTLIB_GYM_TEST_SOURCES)
  add_executable(test_gym ${FLIGHTLIB_GYM_TEST_SOURCES})
  target_link_libraries(test_gym
    ${LIBRARY_NAME}
    gtest
    gtest_main)
add_test(test_gym test_gym)
endif()

# # Build tests for flightlib
if(BUILD_TESTS AND FLIGHTLIB_TEST_SOURCES)
  add_executable(test_lib ${FLIGHTLIB_TEST_SOURCES})
  target_link_libraries(test_lib PUBLIC
    ${LIBRARY_NAME}
    gtest
    gtest_main)
add_test(test_lib test_lib)
endif()

# Build tests for flightlib unity bridge
if(BUILD_UNITY_BRIDGE_TESTS AND FLIGHTLIB_UNITY_BRIDGE_TEST_SOURCES)
  add_executable(test_unity_bridge ${FLIGHTLIB_UNITY_BRIDGE_TEST_SOURCES})
  target_link_libraries(test_unity_bridge PUBLIC
    ${LIBRARY_NAME}
    gtest
    gtest_main)
add_test(test_unity_bridge test_unity_bridge)
endif()
