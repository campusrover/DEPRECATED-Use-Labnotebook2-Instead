# USB Camera Setup

### Author: Ken Kirio

This guide will show how to set up an external camera by connecting it to a computer running ROS. This guide assumes your computer is running ROS on Ubuntu natively, not via the VNC, in order to access the computer's USB port. (The lab has computers with Ubuntu preinstalled if you need one.)

1. Installation
	- Install the ros package usb_cam: `sudo apt install ros-noetic-usb-cam`
	- Install guvcview for the setup: `sudo apt install guvcview`

2. Edit the launch file, `usb_cam-test.launch`
	- Find the location of the file inside the usb_cam package: `roscd usb_cam`
	- Set `video_device` parameter to the port of your camera 
		- Check which ports are in use: `ls /dev` and look for video0, video1, etc.
		- Check the output of each port: `guvcview /dev/<port>`
		- If you unplug the camera between uses or restart your computer, the camera may be on a different port. Check every time!

3. Run the node! `roslaunch usb_cam usb_cam-test.launch`

