cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

find_package(catkin REQUIRED COMPONENTS
  python_qt_binding
  mission_manager
  rospy
  rqt_gui
  std_msgs
  std_srvs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

