# Sensing Aruco

A ROS package that contains aruco markers and a Python module for detecting aruco markers.

## Python 3 and ROS Noetic Prerequisites

The following lines provide the Python prerequisites for Python 3 and ROS Noetic.

1) Uninstall opencv-python if it is installed. Having opencv-python and opencv-contrib-python will conflict with each other.
```
# Use the following for Python 3.
python3 -m pip uninstall opencv-python
```

2) Install ROS geometry msgs, std msgs, and cv-bridge packages.
```
# Use the following for ROS Noetic.
sudo apt-get install ros-noetic-geometry-msgs ros-noetic-std-msgs ros-noetic-cv-bridge
```

3) Install the following Python packages.
```
# Use the following for Python 3.
python3 -m pip install opencv-contrib-python>=4.6.0.66 scipy>=1.9.0 --user
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
ln -s ${HOME}/repos/sensing-aruco sensing-aruco
```

3) Make the scripts executable.
```
chmod +x ${HOME}/repos/sensing-aruco/sense_aruco_ros/nodes/aruco_detector_node
```

4) Build the workspaces.
```
cd ..
catkin_make
```

5) Source the catkin workspace.
```
source devel/setup.bash
```

## Run Demo

Run Gazebo, export the model paths, and run the demo launch file.
```
# In one terminal tab, run:
roslaunch gazebo_ros empty_world.launch


# In a second terminal tab, run:
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:${HOME}/repos/sensing-aruco/aruco_marker_description/models:${HOME}/repos/sensing-aruco/sense_aruco_simulations/models"

roslaunch sense_aruco_simulations spawn_demo_cam_marker.launch
```
