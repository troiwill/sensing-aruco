# Estimating aruco pose & depth using Realsense D435 camera
- This repo was created using [this](https://github.com/issaiass/realsense2_description) repo.
- The steps involved in the development are jotted in `dev_steps.md`.

## Usage
- Clone this repo into the `src` folder of your catkin workspace (eg: `~/catkin_ws/src`).
- Move the `aruco_marker` folder to your defaut gazebo models directory (eg: `~/.gazebo/models/`)
- Build the package from the root of your workspace using `catkin_make`
- Source the devel space using `. ~/catkin_ws/devel/setup.bash`
- Launch the realsense camera in Gazebo & interface it with ROS using `roslaunch sensing-aruco view_d435_model_rviz_gazebo.launch`
- change directory with `cd ~/catkin_ws/src/sensing-aruco/scripts`
- Subscribe to the color & depth topics of realsense & publish the estimated pose by running `python3 img_getter.py --arucomarker_side=0.5`

## Problems Faced
- TimeSynchronizer can't be used if the all the topics involved don't publish at the exact same time. So, used ApproximateTimeSynchronizer instead.
