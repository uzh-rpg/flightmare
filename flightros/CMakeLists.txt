project(flightros)

cmake_minimum_required(VERSION 3.0.0)

find_package(catkin_simple REQUIRED)

find_package(OpenCV REQUIRED)

catkin_simple()

catkin_package(
    LIBRARIES
    CATKIN_DEPENDS
    )

# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architcture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    set(CMAKE_CXX_ARCH_FLAGS " -march=native")
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")
add_compile_options(-O3)

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -g")

include_directories(include)

cs_add_library(ros_utils
    src/ros_utils.cpp
)

cs_add_library(rosbag_writer
src/rosbag_writer.cpp
)

cs_add_executable(testing
src/test/testing.cpp)

target_link_libraries(testing
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    stdc++fs
    rosbag_writer
    ros_utils
    zmq
    zmqpp

)

cs_add_executable(record
src/record.cpp)

target_link_libraries(record
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    stdc++fs
    rosbag_writer
    ros_utils
    zmq
    zmqpp
)

# Finish
cs_install()
cs_export()