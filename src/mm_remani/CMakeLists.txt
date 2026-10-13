cmake_minimum_required(VERSION 3.0.2)
project(mm_remani)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  livox_ros_driver2
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

find_package(PCL REQUIRED COMPONENTS common filters kdtree segmentation)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_obstacle_tracker_core
  CATKIN_DEPENDS geometry_msgs livox_ros_driver2 pcl_conversions pcl_ros
                 roscpp rospy sensor_msgs std_msgs tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_library(dynamic_obstacle_tracker_core
  src/dynamic_obstacle_tracker_core.cpp)
add_dependencies(dynamic_obstacle_tracker_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_obstacle_tracker_core ${catkin_LIBRARIES})

add_executable(dynamic_obstacle_tracker_node
  src/dynamic_obstacle_tracker_node.cpp)
add_dependencies(dynamic_obstacle_tracker_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_obstacle_tracker_node
  dynamic_obstacle_tracker_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(livox_custom_to_pcl src/livox_custom_to_pcl.cpp)
add_dependencies(livox_custom_to_pcl ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_custom_to_pcl ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_dynamic_obstacle_tracker_core
    test/test_dynamic_obstacle_tracker_core.cpp)
  if(TARGET test_dynamic_obstacle_tracker_core)
    target_link_libraries(test_dynamic_obstacle_tracker_core
      dynamic_obstacle_tracker_core
      ${catkin_LIBRARIES})
  endif()
endif()
