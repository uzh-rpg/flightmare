project(flightros)

cmake_minimum_required(VERSION 3.0.0)

find_package(catkin_simple REQUIRED)

find_package(OpenCV REQUIRED)

option(BUILD_SAMPLES "Building the samples" OFF)

catkin_simple()

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

#pilot

cs_add_library(flight_pilot
    src/pilot/flight_pilot.cpp
)

target_link_libraries(flight_pilot
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  stdc++fs
)

cs_add_executable(flight_pilot_node
   src/pilot/flight_pilot_node.cpp
)

target_link_libraries(flight_pilot_node
  flight_pilot
  ${OpenCV_LIBRARIES}
  stdc++fs
)

# motion_planning

if(BUILD_SAMPLES)

    # find all open3D stuff
    message("Searching open3D in /usr/local/lib/cmake/")
    find_package(Open3D HINTS /usr/local/lib/cmake/)
    list(APPEND Open3D_LIBRARIES dl)

    find_package(OpenMP)
    if (OpenMP_CXX_FOUND)
        message("Found OpenMP ${OpenMP_CXX_FOUND}  ${OpenMP_VERSION} ${OpenMP_CXX_VERSION_MAJOR} ${Open3D_VERSION} OpenMP::OpenMP_CXX")
        get_cmake_property(_variableNames VARIABLES)
        list (SORT _variableNames)
        foreach (_variableName ${_variableNames})
            message(STATUS "${_variableName}=${${_variableName}}")
        endforeach()
    endif ()

    # Open3D
    if (Open3D_FOUND)
        message("Found Open3D ${Open3D_VERSION}")
        # link_directories must be before add_executable
        link_directories(${Open3D_LIBRARY_DIRS})
    else ()
        message("Open3D not found")
    endif ()

    find_package(ompl REQUIRED)

    link_directories(/usr/local/lib)
    include_directories(${OMPL_INCLUDE_DIRS})

    cs_add_executable(motion_planning
        src/motion_planning/motion_planning.cpp
    )

    target_include_directories(motion_planning PUBLIC ${Open3D_INCLUDE_DIRS})

    target_link_libraries(motion_planning
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    stdc++fs
    ompl
    ${Open3D_LIBRARIES}
    OpenMP::OpenMP_CXX
    zmq
    zmqpp
    )

endif()

# racing

if(BUILD_SAMPLES)

    catkin_package(
    LIBRARIES
    CATKIN_DEPENDS
    )

    cs_add_executable(racing
        src/racing/racing.cpp
    )

    target_link_libraries(racing
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    stdc++fs
    zmq
    zmqpp
    )

endif()

# camera

if(BUILD_SAMPLES)

    cs_add_executable(camera
        src/camera/camera.cpp
    )

    target_link_libraries(camera
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    stdc++fs
    zmq
    zmqpp
    )

endif()

# Finish
cs_install()
cs_export()
