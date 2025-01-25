cmake_minimum_required(VERSION 3.0.2)
project(limoncello VERSION 0.0.1)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (Release or Debug)" FORCE)
endif()

# Set C++ standard and compiler flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -pthread -fexceptions -O3 -lm -lblas -llapack")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

# Find Boost and TBB
find_package(Boost REQUIRED COMPONENTS system)
find_package(TBB REQUIRED)


# Include Boost directories
include_directories(${Boost_INCLUDE_DIRS})

# Find Eigen3 and PCL
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

# Find ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  nav_msgs
  tf2
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Catkin package configuration
catkin_package(
  CATKIN_DEPENDS
    roscpp
    pcl_ros
    sensor_msgs
    nav_msgs
    tf2
  DEPENDS PCL
  INCLUDE_DIRS include
)

# Manif
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/external/manif)

# Link libraries
set(LINK_LIBRARIES
  PRIVATE manif
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TBB_IMPORTED_TARGETS}
  ${OpenMP_LIBS}
)

# Create executable
add_executable(${PROJECT_NAME} src/main.cpp)

# Link the executable with libraries
target_link_libraries(${PROJECT_NAME}
  ${LINK_LIBRARIES}
)
