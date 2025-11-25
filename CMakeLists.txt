cmake_minimum_required(VERSION 3.5)
project(limoncello)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING
      "Build type (Release or Debug)" FORCE)
endif()

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Global flags (you might later move BLAS/LAPACK to target_link_libraries)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -pthread -fexceptions -O3")

# Other dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# Livox support
find_package(livox_interfaces REQUIRED)
find_package(livox_ros_driver REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Header-only manif library (vendored in external/manif)
add_library(manif INTERFACE)
target_include_directories(manif INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/manif/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/manif/external/tl>
  $<INSTALL_INTERFACE:include>
)

# Include your own headers
include_directories(
  include
)

# Create executable
add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    rclcpp
    pcl_conversions
    sensor_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_eigen
    livox_interfaces
	  livox_ros_driver
	  livox_ros_driver2
)

# Link the executable with libraries
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${Boost_LIBRARIES}
    ${TBB_IMPORTED_TARGETS}
    manif
    m blas lapack 
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY external/manif/include/
  DESTINATION include
)

ament_package()
