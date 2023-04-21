# Leap Motion: Alternative approach (successful one)  

## Intro

As a very last minute and spontaneous approach, we decided to use a Leap Motion device. Leap Motion uses an Orion SDK, two IR camerad and three infared LEDs. This is able to generate a roughly hemispherical area where the motions are tracked.

![motion axies](../images/leap.png)

![leap laptop setup](../images/setup.png)

It has a smaller observation area dn higher resolution of the device that differentiates the product from using a Kinect (which is more of whole body tracking in a large space). This localized apparatus makes it easier to just look for a hand and track those movements.

The set up is relatively simple and just involved downloading for the appropriate OS. In this case, Linux (x86 for a 32 bit Ubuntu system).  

## Steps to downloading Leap Motion and getting it started

### Link if needed

- [here](https://www.leapmotion.com/setup/desktop/linux/)

1. download the SDK from [this link](https://www.leapmotion.com/setup/linux); you can extract this package and you will find two DEB files that can be installed on Ubuntu.
2. Open Terminal on the extracted location and install the DEB file using the following command (for 64-bit PCs):

    ```sh
        sudo dpkg -install Leap-*-x64.deb
    ```

    **If you are installing it on a 32-bit PC, you can use the following command:**

    ```sh
        sudo dpkg -install Leap-*-x86.deb
    ```

3. plug in leap motion and type dmesg in terminal to see if it is detected

4. clone ros drivers:

    ```sh
        git clone https://github.com/ros-drivers/leap_motion
    ```

5. edit .bashrc:

    ```sh
        export LEAP_SDK=$LEAP_SDK:$HOME/LeapSDK
        export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x64
    ```

6. save bashrc and restart terminal then run:

    ``` sh
        sudo cp $LeapSDK/lib/x86/libLeap.so /usr/local/lib
        sudo ldconfig
        catkin_make install --pkg leap_motion
    ```

7. to test run:

    ``` sh
        sudo leapd
        roslaunch leap_motion sensor_sender.launch
        rostopic list
    ```

Once having Leap Motion installed, we were able to simulate it on RViz. We decided to program our own motion controls based on angular and linear parameters (looking at directional and normal vectors that leap motion senses):

![image](../images/code.png)

This is what the Leap Motion sees (the raw info):

![rviz hand](../images/leapmotionss.png)

![terminal screenshot](../images/leapmotionss2.png)

In the second image above, the x y and z parameters indicate where the leap motion detects a hand (pictured in the first photo)

**This is how the hand gestures looked relative to the robot's motion:**

### Stationary

![hand](../images/stat.png)

### Forward

![hand](../images/forward.jpg)

![rviz](../images/forward2.png)

![rviz](../images/forward3.png)

### Backward

![hand](../images/backwards.png)

![rviz](../images/backwards2.png)

![rviz](../images/backwards3.png)

### Left Rotation

![hand](../images/left.png)

![rviz](../images/left2.png)

![rviz](../images/left3.png)

### Right Rotation

![hand](../images/right.png)

![rviz](../images/right2.png)

![rviz](../images/right3.png)

## RQT Graph

![node graph](../images/leapmotionss3.png)

## **Conclusion**

So, we got the Leap Motion to successfully work and are able to have the robot follow our two designated motion. We could have done many more if we had discovered this solution earlier. One important thing to note is that at this moment we are not able to mount the Leap Motion onto the physical robot as LeapMotion is not supported by the Raspberry Pi (amd64). If we are able to obtain an Atomic Pi, this project should be able to be furthered explored. Leap Motion is a very powerful and accurate piece of technology that was much easier to work with than the Kinect, but I advise still exploring both options.
