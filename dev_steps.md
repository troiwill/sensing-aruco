# Steps followed to create this repo:
- Clone Pal robotics's [realsense_gazebo_plugin](https://github.com/pal-robotics/realsense_gazebo_plugin) repo
- Clone issaiass's [realsense2_description](https://github.com/issaiass/realsense2_description) repo
- Make the `view_d435_model_rviz_gazebo.launch` file to include spawning a world (with aruco plane in it) by modifying the `gazebo.launch` file
- Calibrate realsense camaera to get its `calibration.yaml` file
- Edit the generated yaml file to the format of the yaml file present in this repo.

## Note:
- The aruco marker involved is applied as texture on top of a plane.
- The model file of the aruco marker is provided as `aruco_marker`
