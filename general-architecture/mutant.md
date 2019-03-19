# Mutant

## Launching Mutant
1. Make sure you've correctly namespaced `Mutant` in your `.bashrc` file.
1. Once you've been able to ssh into mutant, run `bu` (bringup) on the robot. This bringup runs both the usual turtlebot
bringup as well as the kinect (camera) bringup.
1. Now you can run files as they were on a turtlebot3 (eg. teleop). If you want to run SLAM, you should create a map as
on a turtlebot3, but for the navigation, clone the cr_ros_2 package, and run `roslaunch cr_ros_2 mutant_navigation.launch map_file:=$HOME/map.yaml` and run the map server as usual as well.

## Viewing camera input
1. To view a color video feed from the kinect camera, run `rosrun image_view image_view image:=/mutant/camera/rgb/image_color`.
1. To view the depth feed from the camera, run `rosrun image_view image_view image:=mutant/camera/depth/image`.
  * For the depth camera, parts that appear darker are closer while parts that are lighter are farther away.
