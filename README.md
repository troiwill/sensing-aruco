# Sensing Aruco

A ROS package that contains aruco markers and a Python module for detecting aruco markers.

## Installation

Perform the following steps to install the sense-aruco repository.
1) Clone the sensing-aruco repository.
```
cd ${HOME}/repos
git clone https://github.com/troiwill/sensing-aruco
```

2) Create links to the packages.
```
cd ${HOME}/catkin_ws/src
ln -s ${HOME}/repos/sensing-aruco/aruco_marker_description aruco_marker_description
ln -s ${HOME}/repos/sensing-aruco/sense_aruco sense_aruco
```

3) Build the workspaces.
```
cd ..
catkin build
```

## Usage
- Launch the realsense camera in Gazebo & interface it with ROS using `roslaunch sensing-aruco view_d435_model_rviz_gazebo.launch`
- change directory with `cd ~/catkin_ws/src/sensing-aruco/scripts`
- Subscribe to the color & depth topics of realsense & publish the estimated pose by running `rosrun sense_aruco aruco_pose_estimator.py parampath:=$(find sense_aruco)/calib/ost.yaml marker_len:=0.5`
