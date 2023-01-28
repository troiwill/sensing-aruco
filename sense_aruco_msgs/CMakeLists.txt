cmake_minimum_required(VERSION 3.0.0)
project(sense_aruco_msgs)

find_package(catkin REQUIRED COMPONENTS
   std_msgs
   geometry_msgs
   message_generation
   sense_aruco_ros
)

add_message_files(
    FILES
    Marker.msg
    MarkerArray.msg
    MarkerPose.msg
    MarkerPoseArray.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)
