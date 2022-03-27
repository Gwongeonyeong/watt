cmake_minimum_required(VERSION 2.8.3)
project(gwonbot_control)

find_package(catkin REQUIRED
  gazebo_ros
  std_msgs
  actionlib_msgs
)

add_action_files(
  DIRECTORY action
  FILES
  Moving.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)


catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
