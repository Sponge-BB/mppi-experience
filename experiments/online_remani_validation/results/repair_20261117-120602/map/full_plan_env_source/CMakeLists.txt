cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  tf2_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library( plan_env 
    src/dynamic_point_buffer.cpp
    src/esdf_worker.cpp
    src/grid_map.cpp 
    src/raycast.cpp
    )
target_link_libraries( plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )  

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_dynamic_point_buffer
    test/test_dynamic_point_buffer.cpp)
  if(TARGET test_dynamic_point_buffer)
    target_link_libraries(test_dynamic_point_buffer
      plan_env
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_grid_map_parallel
    test/test_grid_map_parallel.cpp)
  if(TARGET test_grid_map_parallel)
    target_link_libraries(test_grid_map_parallel
      plan_env
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_esdf_worker
    test/test_esdf_worker.cpp)
  if(TARGET test_esdf_worker)
    target_link_libraries(test_esdf_worker
      plan_env
      ${catkin_LIBRARIES})
  endif()
endif()
