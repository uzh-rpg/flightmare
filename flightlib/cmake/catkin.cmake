# Setup catkin simple
find_package(catkin_simple REQUIRED)

catkin_simple()

add_definitions(-std=c++17)

# Library and Executables
cs_add_library(${PROJECT_NAME} ${FLIGHTLIB_SOURCES})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${BLAS_LIBRARIES}
  ${LAPACK_LIBRARIES}
  ${LAPACKE_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
  zmq
  zmqpp
)

# Build tests
if(BUILD_TESTS)
  catkin_add_gtest(flightlib_tests ${FLIGHTLIB_TEST_SOURCES})
  target_link_libraries(flightlib_tests ${PROJECT_NAME} gtest gtest_main)
endif()

# Finish catkin simple
cs_install()
cs_export()
