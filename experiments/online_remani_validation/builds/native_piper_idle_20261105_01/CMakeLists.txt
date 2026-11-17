cmake_minimum_required(VERSION 3.10)
project(native_piper_idle_baseline)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs sensor_msgs
  nav_msgs geometry_msgs control_msgs visualization_msgs quadrotor_msgs
  cv_bridge tf2_ros pcl_conversions)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ompl REQUIRED)
set(P ${CMAKE_CURRENT_SOURCE_DIR}/source/mm_remani_planner)
foreach(part plan_manage plan_env mm_config path_searching traj_opt traj_utils)
  include_directories(${P}/${part}/include)
endforeach()
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/mm_controller/include
  ${CMAKE_CURRENT_SOURCE_DIR}/source/Utils/mm_utils/include)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(native_env STATIC ${P}/plan_env/src/grid_map.cpp
  ${P}/plan_env/src/raycast.cpp)
target_link_libraries(native_env ${catkin_LIBRARIES} ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES})
add_library(native_model STATIC ${P}/mm_config/src/mm_config.cpp)
target_link_libraries(native_model native_env)
add_library(native_search STATIC ${P}/path_searching/src/kino_astar.cpp
  ${P}/path_searching/src/sample_mani_RRT.cpp ${P}/path_searching/src/rrt.cpp)
target_link_libraries(native_search native_model ${OMPL_LIBRARIES})
add_library(native_opt STATIC ${P}/traj_opt/src/poly_traj_optimizer.cpp)
target_link_libraries(native_opt native_search)
add_executable(remani_planner_node ${P}/plan_manage/src/remani_planner_node.cpp
  ${P}/plan_manage/src/remani_replan_fsm.cpp
  ${P}/plan_manage/src/planner_manager.cpp
  ${P}/plan_manage/src/planning_visualization.cpp)
target_link_libraries(remani_planner_node native_opt)
add_executable(mm_controller_node
  ${CMAKE_CURRENT_SOURCE_DIR}/source/mm_controller/src/mm_controller_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/source/mm_controller/src/mm_controller_fsm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/source/mm_controller/src/controller_utils.cpp)
target_link_libraries(mm_controller_node ${catkin_LIBRARIES})
