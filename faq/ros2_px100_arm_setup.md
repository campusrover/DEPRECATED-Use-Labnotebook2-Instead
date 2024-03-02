---
title: PX-100 Arm ROS2 Setup 
author: James Lee
description: How do I set up the PX-100 arm to work with ROS2? 
status: new 
date: march-2024
---

# Question

How do I set up the PX-100 arm to work with ROS2?

# Answer

## Introduction

The PX-100 arm currently supports ROS2 Galactic on Ubuntu Linux 20.04,
or ROS2 Humble on Ubuntu Linux 22.04. This is unlikely to change in the
future, as Trossen Robotics seems to have stopped working further on
the PX-100.

This guide assumes you will be using ROS2 Humble on Ubuntu Linux 22.04.
If you need to, it wouldn't be too hard to adapt the instructions here
for a setup on ROS2 Galactic.

## Install Ubuntu 

Follow [these official
instructions](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwiIh_vevtaEAxXvFVkFHWt6CKUQFnoECBEQAQ&url=https%3A%2F%2Fubuntu.com%2Ftutorials%2Finstall-ubuntu-desktop&usg=AOvVaw3xXGnpwpoUCqklBF6Ot0MF&opi=89978449),
or others you might find on the web, to install Ubuntu Linux 22.04 on
your computer. You might meet with a few hiccups along the way, e.g.,
about Ubuntu being incompatible with RST, etc. When you do, don't
panic, and search for the simplest solution to the problem.

## Check for Hardware Compatibility

Plug in the arm to a usb port of your computer. After waiting for a
bit, run the `lsusb` command to see the USB devices connected to your
computer. If you see something like:

```bash
Bus 001 Device 003: ID 0403:6014 Future Technology Devices International, Ltd FT232H Single HS USB-UART/FIFO IC
```

as a line in your output, your computer is probably compatible with the
arm. 

## Install ROS2 Humble, `colcon`, and `rosdep` 

First, install ROS2 Humble. The [official
guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
is good. The only recommendation I'd make is to add `source
/opt/ros/humble/setup.bash` to your `.bashrc` file (if you're using a
BASH shell. To see whether you are, run `echo $SHELL`. If the output is
`/bin/bash`, you're using BASH).

Second, install colcon, ROS2's build tool.

```bash
sudo apt install python3-colcon-common-extensions
```  

Third, install rosdep. Run:

```bash
sudo apt install python3-rosdep
```

Then execute

```bash
sudo rosdep init
rosdep update
```

## Install Interbotix's Software for ROS2

Finally, it's time to install software for the PX-100 arm. Execute:
 
```bash
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d humble
```

If you're using galactic, replace 'humble' with 'galactic' in the last
line.

After installation, follow these [Installation
Checks](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html#installation-checks)
from Trossen Robotics, to confirm that your installation was
successful.

## Checking that Things Work as Expected

Execute the following line:

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100
```

This should launch an RVIZ window displaying a graphical representation
of your arm. Don't panic if you don't get the arm on the first try, and
there are white blocks where components of the robot should be. Try
closing rviz, and running the command again. 

If this still doesn't work, check to see if the arm is in [the sleeping
position](https://www.trossenrobotics.com/Shared/Images/Product/PincherX-100-Robot-Arm/Img0289.jpg).
If it isn't, unplug the arm, and _gently_ move it into the sleeping
position.

Even if the arm is in a sleeping position, unplug, and then replug the
arm.

Run the command above again. It should have worked this time. If it
still doesn't, there was probably something wrong with the installation
process. Or the hardware of the arm is incompatible with your computer
for mysterious reasons.

If rviz does work, execute the following command:

```bash
ros2 service call /px100/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: false}"
```

This command disables the mechanisms on the robot that keep its joints
fixed in their positions. This means that you should be able to move
the robot's joints manually. Try this, _GENTLY_, and see the graphical
representation of the robot on RVIZ move in tandem.

After you're satisfied with your experiment, place the robot back into
its sleeping position, and quit the two ros2 processes you started.

## Next Steps

The documentation for Interbotix's ROS2 API is very lacking. If you're
curious about something, try searching our lab notes first to see if
you can find what you're looking for. 

Otherwise, dig into the source code, installed under the
`interbotix_ws` directory. The directory's path is likely
`~/interbotix_ws`. Don't be afraid to do this! It's what you'll have to
do anyway if you join a company or work on a substantial open source
project. Don't be afraid to modify the source code either, if you feel
it's not working as it should! The developers are only human, and may
have made mistakes.

