# Kinect: libfreenect2 

#### **Intro**

In Kinect.md, the previous generations dicussed the prospects and limitations of using a Kinect camera. We attempted to use the new Kinect camera v2, which was released in 2014. 

![figure1](../images/5.png) 

Thus, we used the libfreenect2 package to download all the appropiate files to get the raw image output on our Windows. The following link includes instructions on how to install it all properly onto a Linux OS. 

https://github.com/OpenKinect/libfreenect2

### **Issues**

We ran into a lot of issues whilst trying to install the drivers, and it took about two weeks to even get the libfreenect2 drivers to work. The driver is able to support RGB image transfer, IR and depth image transfer, and registration of RGB and depth images. Here were some essential steps in debugging, and recommendations if you have the ideal hardware set up: 

- Even though it says optional, I say download OpenCL, under the "Other" option to correspond to Ubuntu 18.04+ 
- If your PC has a Nvidia GPU, even better, I think that's the main reason I got libfreenect to work on my laptop as I had a GPU that was powerful enough to support depth processing (which was one of the main issues) 
- Be sure to install CUDA for your Nvidia GPU 
- Install OpenNI2 if possible 
- Make sure you build in the right location 

Please look through this for common errors: 

 https://github.com/OpenKinect/libfreenect2/wiki/Troubleshooting 
 
 Although we got libfreenect2 to work and got the classifier model to locally work, we were unable to connect the two together. What this meant is that although we could use already saved PNGs that we found via a Kaggle database (that our pre-trained model used) and have the ML model process those gestures, we could not get the live, raw input of depth images from the kinect camera. We kept running into errors, especially an import error that could not read the freenect module. I think it is a solvable bug if there was time to explore it, so I also believe it should continued to be looked at. 
 
 However, also fair warning that it is difficult to mount on the campus rover, so I would just be aware of all the drawbacks with the kinect before choosing that as the primary hardware. 
 
 ### Database  
 
 https://www.kaggle.com/gti-upm/leapgestrecog/data 

### Machine Learning model 

https://github.com/filipefborba/HandRecognition/blob/master/project3/project3.ipynb

- What this model predicts: Predicted Thumb Down
Predicted Palm (H), Predicted L, Predicted Fist (H), Predicted Fist (V), Predicted Thumbs up, Predicted Index, Predicted OK, Predicted Palm (V), Predicted C

### GITHUB REPO 

https://github.com/campusrover/gesture_recognition
