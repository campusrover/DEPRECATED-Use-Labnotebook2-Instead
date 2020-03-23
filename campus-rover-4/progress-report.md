# Where the Campus Rover 4 Project concluded

Due to novel Coronavirus COVID-19, the efforts of the CR4 team in spring 2020 was cut short. In about two months, we were able to complete quite a bit but unable to get the new rover to a stable and usable state for future students. This document lists what was finished, and elaborates on what remains to be done.

## What Was Done

* The biggest endeavor of this project was to use the `Diff_Drive_Controller` to produce stable and reliable motor movement (in tandem with a PID loop on the Tivac board) and odometry.
* the embedded ROS node on Tivac publishes IMU and Sonar topics, but their data is unreliable.
* a basic urdf was constructed
* basic launch files created for robot bringup
* depth camera and lidar fusion as a point cloud

## WHat needs to be completed

* PID gains need to be more finely tuned. Refer to motor.h for the defines of the gains
* IMU should be calibrated more finely. Perhaps DMP should be used?
* `Diff_Drive` published NAN in odoms by default. perhaps some of the arrays in hw_interface.cpp need to be initialized with 0's?
* attach a more reliable lidar and prove that SLAM works with diff_drive
* finish tuning navigation params
* create a more deatiled robot model and urdf (perhaps usinf xacro) in a CAD or other modeling softawre (blender?)
* build a sinple system that can detect when the battery is at low levels and indicate to the user in some way (a beeping noise, a topic, a light, turning off the motors, etc)

We hope that this is a substantial base with a clear direction to move forward with, once the time is right. When resuming work, please go to the `hardware_interface` branch of `rover_4_core`, which is the latest branch.
