# Estimating aruco pose & depth using Realsense D435 camera
- This repo was created using [this](https://github.com/issaiass/realsense2_description) repo.
- The steps involved in the development are jotted in `dev_steps.md`.

## Usage
- Clone this repo into the `src` folder of your catkin workspace (eg: `~/catkin_ws/src`).
- Build the package from the root of your workspace using `catkin_make`
- Source the devel space using `. ~/catkin_ws/devel/setup.bash`
- Launch the realsense camera in Gazebo & interface it with ROS using `roslaunch realsense2_description view_d435_model_rviz_gazebo.launch`
- change directory with `cd ~/catkin_ws/src/realsense2_description/scripts`
- Subscribe to the color & depth topics of realsense & publish the estimated pose by running `python3 img_getter.py`

## Problems Faced
- TimeSynchronizer can't be used if the all the topics involved don't publish at the exact same time. So, used ApproximateTimeSynchronizer instead.
