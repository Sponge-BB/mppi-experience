cmake_minimum_required(VERSION 3.0.2)
project(mm_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_ros
  robot_state_publisher
)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/mm_controller.py
  scripts/cmd_vel_to_model_twist.py
  scripts/odom_openloop_publisher.py
  scripts/odom_to_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
