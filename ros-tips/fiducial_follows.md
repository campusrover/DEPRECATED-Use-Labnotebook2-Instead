# Working with Fiducials

**UNDER DEVELOMENT**

This document will not be explaining fiducials or localization. It is meant to help you get the software up and running on your robot. We are working here purely with ROS1 based robots, running on a Raspberry Pi with a Rasberry Pi Camera. Also we are working purely with Aruco Fiducials and the Ubiquity aruco_detect library. There are many other variations which are going to be ignored.

## Building Blocks

There are several new components which come into play in order to use fiducials 

1. Camera - Which converts light into a published topic with an image
1. Fiducial "signs" - Which you print out and place within view of the robot
1. `aruco_detect` package - which analyzes the images and locates fiducials in them, publishing a tf for the relative position between the camera and the fiducial
1. `fiducial_slam` package - which collects information about multiple fiducials that can be placed around it.
## Packages necessary

### [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node) - Main Camera Node

This package, by Ubiquity Robotics enables the raspberry Pi camera. It should be installed on your robot already. Raspi cameras have quite a lot of configuration parameters and setting them up as well as possible requires 'calibration'.

### [aruco_detect](http://wiki.ros.org/aruco_detect) - Reconition software

This package is also from Ubituity contains the components that go into recognizing the fiducials and working with them.

## Fiducials

Fiducials as you know are black and white images. There is software that recognizes them and more than that, are able to compute precisely the relative positions between the camera and the fiducials. 

### Fiducial Dictionaries

All fiducials are not created equally. Simply because it looks like a black and white image doesn't mean that it will work. The algorithm depends on the images coming from a standardized set of possibiltities. An individual Aruco fiducial comes from one of a small set of predefined Dictionaries and has a specific index therein. You need to know what you are dealing with.  Therefore you must make sure the fiducials you are using match the software

### Print out some fiducials

Use this web site: [Aruco Markers Generator](https://chev.me/arucogen/) and print out one or more fiducials. Pick "Original Aruco Dictionary" which corresponds to "16" in the code. If you print more than one give them different IDs. Later on that ID will come through in the code to allow you to tell one from the other. Tape the fiducial within sight of the camera on your robot.


## Running the software`

### Enable the camera

First you have to enable the camera. **On the robot (`onboard`)** run the following, either by itself or as part of another launch file. 

`roslaunch raspicam_node camerav2_410x308_30fps.launch`

You can view the image of the camera within Rviz by subscribing to `/raspicam_node/image` or `rqt_image`. If the image happens to be upside down then. If the image is upsdide down get help to change the VFlip default variable.

#### Other launch options

`roslaunch raspicam_node camerav2_1280x960_10fps.launch` # for a comppressed image, a good default
`roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true` # for an uncompressed image

#### Topics

You will see that when you enable the camera it will begin publishing on one of several topics, for example:

`raspicam_node/camera/compressed`

Note that the last part of the topic (as you would see it in `rostopic list`) is not actually used when you subscribe to it. In fact the topic you subscribe to is `raspicam_node/camera` and the `/compressed` is used to designate that the data is a compressed image. This is confusing and unusual.

## Detect Fiducials

### Basic

Make sure first that your camera is pointed at a Fiducial that you printed earlier. Now run (on your vnc)

 `roslaunch aruco_detect aruco_detect.launch vis_msgs:=false dictionary:=16 fiducial_len:=0.10`

`

If detect sees the tag and identifies it you should see a large number of new topics (`rostopic list`). One that you can check is `/fiducial_images/. View it with rqt_image or rviz. If it is working and the fiducial is in view, you will see an colored outline around the fiducial. aruco_detect does have numerous parameters that in the future you can look at tweaking.

### Transforms

When you have just one fiducial things are simpler. `aruco_detect` will create a tf between that fiducial and the robot itself. In theory that means that as the robot moves, as long as it can see the fiducial, you can use the TF to determin the robot's location relative to the tf.

You can see this by running rviz, looking at the tf tree. When you display the tf tree you will see the tf and the robot. Depending on which of the two you make the rviz "fixed frame".
## Reference Links
[Ubiquity Overview of Fiducials](https://learn.ubiquityrobotics.com/fiducials)
[Fiducial Slam Project Report](https://campus-rover.gitbook.io/lab-notebook/gen4-reports/fiducialslam)


