# Using the Raspberry Pi camera V2

## setup:
[This Robotis emanual page](http://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/#raspberry-pi-camera) describes how to setup the Raspberry Pi camera to be used with Turtlebot3.

Here is a streamlined guide to quickly get a raspi camera working with a TB3. This entire process may take a few minutes - worst case, if you have to fix apt-get errors, upwards of 30 minutes. 

1. `sudo raspi-config` in the TB3's terminal. Navigate to option 3: Interfacing options. The first option in the sub-menu is camera - select it, then select yes when prompet to enable camera interfacing. Then, navigate to finish and reboot the robot for the change to take effect.
2. do a `sudo apt-get update` and `sudo apt-get upgrade` to make sure there are no errors. If update throws a missing pubkey error, then record the pubkey and use this command: `sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys <PUBKEY>` where <PUBKEY> is the pubkey that you recorded. once the pubkey is added, update & upgrade. If there are no errors, continue.
3. run the following two commands to add Ubiquity Robotic's repos to apt
  ```
  $ sudo sh -c 'echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list'
  $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8
  ```
4. update & upgrade again.
5. `sudo apt-get install ros-kinetic-raspicam-node`
6. catkin make
7. if catkin_make fails due to missing diagnostics, install this: `sudo apt-get install ros-kinetic-diagnostic-updater`

## How to launch
`roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch` will launch the camera alone at a resolution of 640x480.
Alternatively, you can also use `roslaunch raspicam_node camerav1_1280x720.launch` to launch at a higher resolution.
To include in a custom launch file, consider using a command like this:

```
<node pkg="raspicam_node" type="raspicam_node" name="raspicam_node" output="screen">
  <param name="camera_info_url" value="package://turtlebot3_bringup/camera_info/turtlebot3_rpicamera.yaml"/>
  <param name="width" value="640"/>
  <param name="height" value="480"/>
  <param name="framerate" value="10"/>
  <param name="enable_raw" value="true"/>
  <param name="camera_frame_id" value="camera"/>
</node>
```

## handy parameters
The following parameters can be edited in a launch file that launches the Raspi cam to alter its performance:
* `enable_raw` : allows the camera to publish a topic `~/image`, of topic type `sensor_msgs/Image` if set to `true`. If not true, only `~/image/compressed` will publish (which publishes a topic type `sensor_msgs/CompressedImage`).
* `height` and `width` : change the resolution of the image.
* `framerate` : changes the rate at which the camera publishes images (maximum 90 fps). Max FPS is also affected by the resolution (higher resolution -> lower max fps)

## useful commands:
* `rqt_image_view` : opens a gui where you can select an image topic currently being published and view it from your remote PC.
* `rosrun rqt_reconfigure rqt_reconfigure` : opens a gui which can edit various raspi settings, such as vertical/ horizontal flipping, image stabilization, and other sliders for various settings
