cmake_minimum_required(VERSION 3.0.2)
project(mm_controller)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
  quadrotor_msgs
  mm_utils
  traj_utils
)

catkin_package()

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  # ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library( controller_utils
  src/controller_utils.cpp
  src/mm_controller_fsm.cpp
)

target_link_libraries( controller_utils
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable( mm_controller_node
  src/mm_controller_node.cpp
)

target_link_libraries( mm_controller_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  controller_utils
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_execution_capacity test/test_execution_capacity.cpp)
  if(TARGET test_execution_capacity)
    target_link_libraries(test_execution_capacity ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_feedback_source test/test_feedback_source.cpp)
  if(TARGET test_feedback_source)
    target_link_libraries(test_feedback_source ${catkin_LIBRARIES})
  endif()
endif()
