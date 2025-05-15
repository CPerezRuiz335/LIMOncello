cmake_minimum_required(VERSION 3.5)
project(limoncello)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Build configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# General dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

# Disable testing in manif subdirectory to avoid GTest dependency issues
set(BUILD_TESTING OFF CACHE BOOL "Disable testing for manif library" FORCE)

# External libraries
add_subdirectory(external/manif)

# Main executable
add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Define libraries
target_link_libraries(${PROJECT_NAME}
  manif
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TBB_IMPORTED_TARGETS}
  Eigen3::Eigen
  ${BLAS_LIBRARIES}
  ${LAPACK_LIBRARIES}
)

# ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pcl_conversions
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
)

# Installation
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
