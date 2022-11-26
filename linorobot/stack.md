# Intro to LinoRobot

# Intro

Examine [Linorobot](www.linorobot.org)  again. You will see very detailed instructions for building a robot, both the hardware and the software. In our world, **bullet**, **platform**, **cat** are all fully compliant Linorobot robots. 

# Base hardware stack

* For the SBC we use either a Rasperry Pi 3B+ or a Raspberry Pi 4
    * Lidar is connected via USB to the Raspberry Pi
    * The Microcontroller is connected via USB to the Raspberry Pi
* For the microcontroller we use either a Teensy 3.2 or a Teensy 4.x (check this)
    * The motor controller is connected to the Teensy via
    * The IMU is connected to the Teensy via I2C bus
## SBC Software

The SBC is running Ubuntu 20.04 and ROS 1.0. It is a standard install which we get from the Linorobot installation. Of course our own ROS code and scripts are then added to it. Certain standard Linorobot Nodes are launched.
### Standard Linorobot Nodes


## Microcontroller Software

The Teensy code is provided by Linorobot. We have tweaked it in small ways. See [How To](linorobot/howto.md) for information on rebuilding it and installing the software. This software has the following jobs:

1. Read the encoders to determine the apparent speed and direction of the robot
1. Subscribe to cmd_vel to determine the desired speed and direction
1. Use a PID controller to drive the motors to meet the desired speed and direction
1. Publish the actual speed and direction as computed by the encoders as `ODOM_RAW`
1. Read the IMU data (via the I2C bus) and publish it as `IMU_RAW`
1. Read other I2C sensors and actuators (coming soon!)


