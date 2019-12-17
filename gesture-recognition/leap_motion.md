# Leap Motion: Alternative approach (semi-successful one)  

### **Intro** 
 
 As a very last minute and spontaneous approach, we decided to use a Leap Motion device. Leap Motion uses an Orion SDK, two IR camerad and three infared LEDs. This is able to generate a roughly hemispherical area where the motions are tracked.
 

 <img src="../images/leap.png" alt="Your image title" width="400"/>
 
 
 <img src="../images/setup.png" alt="Your image title" width="400"/>
 

 
 It has a smaller observation area dn higher resolution of the device that differentiates the product from using a Kinect (which is more of whole body tracking in a large space). This localized apparatus makes it easier to just look for a hand and track those movements. 
 
 The set up is relatively simple and just involved downloading for the appropriate OS. In this case, Linux (x86 for a 32 bit Ubuntu system).  
 
 ### **Steps to downloading Leap Motion and getting it started:**
 
 1. download the SDK from https://www.leapmotion.com/setup/linux; you can extract this package and you will find two DEB files that can be installed on Ubuntu.
2. Open Terminal on the extracted location and install the DEB file using the following command (for 64-bit PCs):

 	**$ sudo dpkg -install Leap-*-x64.deb** 
 	
 	**If you are installing it on a 32-bit PC, you can use the following command:**
 	
	**sudo dpkg -install Leap-*-x86.deb**
	
3. plug in leap motion and type dmesg in terminal to see if it is detected

4. clone ros drivers:   

    **$ git clone https://github.com/ros-drivers/leap_motion**

5. edit .bashrc:

    **export LEAP_SDK=$LEAP_SDK:$HOME/LeapSDK**
    
    **export PYTHONPATH=$PYTHONPATH:$HOME/LeapSDK/lib:$HOME/LeapSDK/lib/x64**
6. save bashrc and restart terminal then run:

    **sudo cp $LeapSDK/lib/x86/libLeap.so /usr/local/lib**
    
    **sudo ldconfig**
    
    **catkin_make install --pkg leap_motion**

7. to test run:
 
    **sudo leapd**
 
    **roslaunch leap_motion sensor_sender.launch**
       
    **rostopic list**
    
Once having Leap Motion installed, we were able to simulate it on RViz. We decided to program our own motion controls based on angular and linear parameters (looking at directional and normal vectors that leap motion senses): 

![](labnotebook/images/code.png)

This is what the Leap Motion sees (the raw info):

<img src="../images/leapmotionss.png" alt="Your image title" width="800" height="500"/>

<img src="../images/leapmotionss2.png" alt="Your image title" width="800" height="500"/>

In the second image above, the x y and z parameters indicate where the leap motion detects a hand (pictured in the first photo)




**This is how the hand gestures looked relative to the robot's motion:** 

### **Stationary** 
<img src="../images/stat.png" alt="Your image title" width="400"/>

### **Forward** 

<img src="..images/forward.jpg" alt="Your image title" width="350" height="500"/>

<img src="..images/forward2.png" alt="Your image title" width="355" height="500"/>

<img src="..images/forward3.png" alt="Your image title" width="350" height="500"/>


### **Backward** 

<img src="..images/backwards.png" alt="Your image title" width="350" height="500"/>

<img src="../images/backwards2.png" alt="Your image title" width="359" height="500"/>

<img src="..images/backwards3.png" alt="Your image title" width="360" height="500"/>

### **Left Rotation** 

<img src="../images/left.png" alt="Your image title" width="350" height="500"/>

<img src="../images/left2.png" alt="Your image title" width="360" height="500"/>

<img src="../images/left3.png" alt="Your image title" width="370" height="500"/>

### **Right Rotation**

<img src="../images/right.png" alt="Your image title" width="340" height="500"/>

<img src="../images/right2.png" alt="Your image title" width="375" height="500"/>

<img src="../images/right3.png" alt="Your image title" width="350" height="500"/>






