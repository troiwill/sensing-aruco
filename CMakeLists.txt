cmake_minimum_required(VERSION 2.8.3)
project(realsense2_description)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  rospy
  std_msgs
)

# RealSense description
catkin_package(
    )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/img_getter.py scripts/aruco_pose.py scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Install files
install(DIRECTORY launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
