---
title: Interbotix Pincher X100 Arm
date: mar-2023
author: 
type: faq
status: new
---
# arm setup

## Links

* [Arm Details](https://www.trossenrobotics.com/pincherx-100-robot-arm.aspx)
* [Quickstart Guide](https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_interface/quickstart.html)
* [Hardware setup](https://www.youtube.com/watch?v=tkDbmWAyHYw)
    - only needed to install the grippers
        - the screws would not go through initially - had to go from the other side with the screw first to ensure that the holes were completely open and the screw could be secured and then installed the grippers as the instructions said
    - plug in power supply first and then plug usb into computer and then plug microUSB into arm
* [ROS Installation Guide](https://www.youtube.com/watch?v=kZx2tNVfQAQ)
* [Troubleshooting](https://www.trossenrobotics.com/docs/interbotix_xsarms/troubleshooting/index.html)
* [DYNAMIXEL Software](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
    * after installing software and plugging arm in, scan for the arm in the dynamixel software to check that everything is working properly:
        * in [options](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#scan-dynamixel), select baudrates 57600 and 1000000
        * if any link is in 57600, then change to 1000000
    * make sure to disconnect before running ros
    * NOTE: you may not actually need to do this, but it may be good to do the first time you try to connect the arm to your computer to make sure everything is running correctly. 
* [Arm Control](https://www.trossenrobotics.com/docs/interbotix_xsarms/ros_packages/arm_control.html)
    * contains command line configs for launch file
* [Python-ROS Interface](https://www.trossenrobotics.com/docs/interbotix_xsarms/python_ros_interface/index.html)
    * contains details on methods that control the arm using their methods - can find basic overview at the bottom of this file. 

****************************Installation:**************************** 

On Intel/AMD based processor: 

```bash
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d noetic
```

**Basic Commands:** 

- move the arm manually:
    - `roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=px100`
- disable torque:
    - `rosservice call /px100/torque_enable "{cmd_type: 'group', name: 'all', enable: false}"`
- re-enable torque to hold a pose:
    - `rosservice call /px100/torque_enable "{cmd_type: 'group', name: 'all', enable: true}"`
- run with moveit:
    - `roslaunch interbotix_xsarm_moveit xsarm_moveit.launch robot_model:=px100 use_actual:=true dof:=4`
    - `roslaunch interbotix_xsarm_moveit xsarm_moveit.launch robot_model:=px100 use_gazebo:=true dof:=4`
- run using ROS-PYTHON API:
    - `roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=px100 use_sim:=true`
    - `roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=px100 use_actual:=true`
- play with joints:
    - `roslaunch interbotix_xsarm_descriptions xsarm_description.launch robot_model:=px100 use_joint_pub_gui:=true`
- publish static transforms between two frames:
    - `rosrun tf static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period(milliseconds)`

# Interbotix Python-ROS Interface

-   `arm.set_ee_pose_components()`
    
    -   sets an absolute position relative to the base of the frame
        -   ee_gripper_link frame with respect to the base_link frame
-   `arm.set_single_joint_position()`
    
    -   move the specified joint
    -   usually used for the waist to turn the robot
-   `arm.set_ee_cartesian_trajectory()`
    
    -   move the end effector the specified value in each direction relative to the current position
    -   for a 4dof arm, the y and yaw values cannot be set through this
-   `arm.go_to_sleep_position()`
    
    -   return the arm to the sleep position
-   `arm.go_to_home_position()`
    
    -   return the arm to the home position
-   `gripper.open()`
    
    -   open the gripper
-   `gripper.close()`
    
    -   close the gripper
-   `arm.set_trajectory_time()`
    
    -   **moving_time** - duration in seconds it should take for all joints in the arm to complete one move.
    -   **accel_time** - duration in seconds it should take for all joints in the arm to accelerate/decelerate to/from max speed.
