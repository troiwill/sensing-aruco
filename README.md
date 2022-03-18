# sensing-aruco
A library for sensing and working with ArUco markers in simulated and physical environments.

## Libraries required:
- python==3.8.8
- opencv==4.5.4
- opencv-contrib-python==4.5.4.60
- pyyaml
- rospkg
- plotly
- Alternatively, you can install a new conda env using the provided file: `conda env create -f environment.yml`


## Instructions to run:
- Go to the src folder of your catkin workspace (assuming you have it at: `~/catkin_ws/src`) with `cd ~/catkin_ws/src`
- Clone this repo.
- Copy the camera folder to your gazebo models directory at `~/.gazebo/models`.
- Go to the catkin workspace `cd ~/catkin_ws`
- build the package with: `catkin_make sensing-aruco`
- Source the setup file with `. ~/catkin_ws/devel/setup.bash`
- Launch the gazebo environment with `roslaunch sensing-aruco aruco.launch`
- In a new terminal, source the bash file again & go to the scripts folder: `cd ~/catkin_ws/src/sensing-aruco/scripts`
- Run the following script to create a ROS node which subscribes to the topic on which camera publishes images & runs the aruco estimation script: `python3 img_getter.py`
- In a new terminal, source the bash file again & go to the scripts folder: `cd ~/catkin_ws/src/sensing-aruco/scripts`
- Run the following script to change the camera's pose: `python3 cam_pose_changer.py`
- After the experiment is done, the measurement & ground truth data gets collected into estimates.txt & gt.txt
- Run `python3 plot.py` to view the measurement error vs ground truth comparision.
