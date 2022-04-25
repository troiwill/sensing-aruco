# Sensing Aruco

A ROS package that contains aruco markers and a Python module for detecting aruco markers.

## Prerequisites

Uninstall opencv-python if it is installed.
```
python2 -m pip uninstall opencv-python
```

Run the following commands to install the prerequisites.
```
sudo apt-get install ros-melodic-geometry-msgs ros-melodic-cv-bridge ros-melodic-std-msgs
python2 -m pip install opencv-contrib-python==3.4.8.29 scipy==1.2.0 --user
```

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

3) Make the scripts executable.
```
chmod +x ${HOME}/repos/sensing-aruco/sense_aruco/scripts/*.py
```

4) Build the workspaces.
```
cd ..
catkin build
```

## Usage

1) Launch the RealSense camera in Gazebo & interface it with ROS using:
```
roslaunch sensing-aruco view_d435_model_rviz_gazebo.launch
```

2) Run the following node, which publishes a marker's pose.
```
rosrun sense_aruco aruco_pose_estimator.py parampath:=${HOME}/repos/sensing-aruco/sense_aruco/calib/ost.yaml marker_len:=0.1
```
