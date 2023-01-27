cmake_minimum_required(VERSION 3.0.0)
project(sense_aruco_ros)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    sensor_msgs
    rospy
)

catkin_package()

catkin_install_python(PROGRAMS
    nodes/aruco_detector_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY nodes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
