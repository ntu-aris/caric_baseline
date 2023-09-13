cmake_minimum_required(VERSION 3.0.2)
project(caric_baseline)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mav_msgs
  sensor_msgs
  trajectory_msgs
  nav_msgs
  tf

  #PCL
  caric_mission#for communication
  pcl_conversions
  pcl_ros  
  trajectory_msgs
)
find_package(OpenMP REQUIRED)
# find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})


###################################
## catkin specific configuration ##
###################################
catkin_package(
  # caric_mission
#  INCLUDE_DIRS include
#  LIBRARIES caric_baseline
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${caric_mission_INCLUDE_DIRS}
)

add_executable(baseline_planner src/baseline_planner.cpp)
add_executable(gcs_task src/general_task_initial.cpp)


target_link_libraries(baseline_planner ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
target_link_libraries(baseline_planner ${OCTOMAP_LIBRARIES})

target_link_libraries(gcs_task ${catkin_LIBRARIES} ${OpenMP_CXX_FLAGS})
target_link_libraries(gcs_task ${OCTOMAP_LIBRARIES})