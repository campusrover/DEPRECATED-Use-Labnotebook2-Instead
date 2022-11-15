# Intro to LinoRobot

## Base software stack

Examine [Linorobot](www.linorobot.org)  again. You will see very detailed instructions for building a robot, both the hardware and the software. In our world, **bullet**, **platform**, **cat** are all fully compliant Linorobot robots. 

## Base hardware stack

* For the SBC we use either a Rasperry Pi 3B+ or a Raspberry Pi 4
* For the microcontroller we use either a Teensy 3.2 or a Teensy 4.x (check this)

### SBC

The SBC is running Ubuntu 20.04 and ROS 1.0. It is a standard install which we get from the Linorobot

### Microcontroller

The Teensy code is provided by Linorobot. We have tweaked it in small ways. When tweaking it you need to re-load it, as follows:

