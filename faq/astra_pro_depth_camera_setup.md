# Astra Pro Depth Camera Setup
by Veronika Belkina

This is a guide to installing the Astra Pro Depth Camera onto a robot and the various problems and workarounds that were experienced along the way. 

---
## Setup
To start, try to follow the instructions given on the [Astra github].

If this goes well, then you're pretty much all set and should skip down to the **usb_cam** section.
If this doesn't go well, then keep reading to see if any of the errors that you received can be resolved here. 

---
## Possible errors and ways to solve them
- make sure to run ```sudo apt update``` on the robot 
- if you are getting an error that mentions a lock when you are installing dependencies, try to reboot the robot: ```sudo reboot now```

If you have tried to install the dependencies using this: 
```
sudo apt install ros-$ROS_DISTRO-rgbd-launch ros-$ROS_DISTRO-libuvc ros-$ROS_DISTRO-libuvc-camera ros-$ROS_DISTRO-libuvc-ros
```
and this is not working, then here is an alternative to try: 
```
sudo apt install libuvc
sudo apt install ros-*-rgbd-launch
```
If this is successful, then within ~/catkin_ws/src, ```git clone https://github.com/orbbec/ros_astra_camera``` and run the following commands: 

```
roscd astra_camera
chmod 777 /scripts/create_udev_rules
./scripts/create_udev_rules
```

After this, run ```catkin_make``` on the robot. This might freeze on you. Restart the robot and run ```catkin_make -j1``` to run on a single core. This will be slower, but will finish. 

If you are getting a *publishing checksum error*, try to update the firmware on the robot using commands from [TB3_image] or run these commands, line by line:
```
export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger_noetic# or waffle_noetic if you have a waffle tb3
rm -rf ./opencr_update.tar.bz2
wgethttps://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2
tar -xvf opencr_update.tar.bz2
cd ./opencr_update
./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
```

At this point, hopefully, all the errors have been resolved and you are all set with the main astra_camera package installation. 

There is one more step that needs to be done. 

---
## usb_cam
The Astra Pro camera doesn't have an RGB camera that's integrated with OpenNI2. Instead, it has a regular Video4Linux webcam. This means that from ROS's point of view, there are two completely separate devices.To resolve this, you can install another package onto the robot called usb_cam following these [instructions]:
```
cd ~/catkin_ws/src
git clone https://github.com/bosch-ros-pkg/usb_cam.git
cd ..
catkin_make
source ~/catkin-ws/devel/setup.bash
```

Test it by running ```rosrun usb_cam usb_cam_node``` 

Afterwards, when you need to run the depth camera and need the rgb stream as well, you will need to run the following instructions onboard the robot: 
```
bringup 
roslaunch astra_camera astra.launch
rosrun usb_cam usb_cam_node
```

If this is something that you will be needing often, then it might be worth it to add the usb_cam node into the astra launch file as you do need to ssh onto the robot for each instruction. The usb_cam_node publishes to the topic ```/usb_cam/image_raw```. You can check ```rostopic list``` to see which one suits your needs. 

If you want to just explore the depth camera part of the camera, then just run the astra.launch file. 

Then there will be a view topics that the camera publishes: 

```jsx
/camera/depth/camera_info
/camera/depth/image_raw
/camera/depth/points
/camera/depth_registered/points
/camera/extrinsic/depth_to_color
/camera/ir/camera_info
/camera/ir/image_raw
/camera/reset_device
```

If you open rviz, click add, go to the by topic tab and open the PointCloud2 topic under /camera/depth/points: 

![image](https://user-images.githubusercontent.com/72238100/206863676-13025ab6-ee4b-4cc2-a4b1-73c6caa2ef5c.png)


If you’re working with just the camera, you might need to fix the frame and can just pick any random one for now other than map. A point cloud of what’s in front of you should appear: 

![image](https://user-images.githubusercontent.com/72238100/206863696-29f3c418-b8c7-4742-a44d-aa7b232c6e9b.png)

If you're looking through rqt, then you might see something like this: 

![image](https://user-images.githubusercontent.com/72238100/206863725-499e6bdc-5f7c-4786-94e6-a72a7d501d7d.png)


From there, you could use colour detection, object detection, or whatever other detector you want to get the pixels of your object and connect them with the pixels in the point cloud. It should output a distance from the camera to the object. However, I can’t speak for how reliable it is. It can’t see objects right in front of it - for example when I tried to wave my hand in front of it, it wouldn’t detect it until it was around 40 or so cm away.

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)
   [TB3_image]: <https://github.com/campusrover/TB3_image> 
   [Astra github]: <https://github.com/orbbec/ros_astra_camera>
   [instructions]: <https://answers.ros.org/question/197651/how-to-install-a-driver-like-usb_cam/?answer=197656#post-id-197656>

   [PlDb]: <https://github.com/joemccann/dillinger/tree/master/plugins/dropbox/README.md>
   [PlGh]: <https://github.com/joemccann/dillinger/tree/master/plugins/github/README.md>
   [PlGd]: <https://github.com/joemccann/dillinger/tree/master/plugins/googledrive/README.md>
   [PlOd]: <https://github.com/joemccann/dillinger/tree/master/plugins/onedrive/README.md>
   [PlMe]: <https://github.com/joemccann/dillinger/tree/master/plugins/medium/README.md>
   [PlGa]: <https://github.com/RahulHP/dillinger/blob/master/plugins/googleanalytics/README.md>
