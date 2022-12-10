# camera calibration

Sometimes, you might want to use object detection or use certain algorithms that require a camera such as VSLAM. These algorithms usually require a very good calibration of the camera to work properly. The calibration fixes things like distortion by determining the camera’s true parameters such as focal length, format size, principal point, and lens distortion. If you see lines that are curved but are supposed to be straight, then you should probably calibrate your camera. 

Usually this is done with some kind of checkerboard pattern. This can be a normal checkerboard or a Charuco/Aruco board which has some patterns that look like fiducials or QR codes on it to further help with calibration. In this tutorial, we’ll be using a 7x9 checkerboard with 20x20mm squares: [checkerboard pdf](https://www.mrpt.org/downloads/camera-calibration-checker-board_9x7.pdf). 

The most ideal way to do is to print the checkerboard on a large matte and sturdy piece of paper so that the checkerboard is completely flat and no reflections can be seen on it. However, it’s okay to just print it on a normal piece of paper as well and put it on a flat surface. Then, take at least ten photos with your camera from a variety of angles and positions so that the checkerboard is in all corners of the photos. Make sure the whole checkerboard is seen in each picture. Save those photos in an easy to find place and use the following to get your intrinsic calibration matrix. 

The code I used was this [opencv](https://learnopencv.com/camera-calibration-using-opencv/) calibration. It also has more notes and information about what the information you are getting is.

Step by step: 

- print out checkerboard pattern
- take at least 10 photos of the checkerboard at a variety of angles and positions (see image 1 for examples) and save in an easy to access place
- download/copy the opencv calibration code and run it after changing the folder path
- get the intrinsic matrix and distortion and enter it into whatever you need

![image](https://user-images.githubusercontent.com/72238100/206863487-40fa1b71-dce7-4278-a8e2-149ebdc284ec.png)

Image 1: some examples of having the checkerboard cover all corners of the image

Potential intrinsic matrix:

[[688.00030953   0.         307.66412893]

[  0.         689.47629485 274.053018  ]

[  0.           0.           1.        ]]

Pastable into python: 

fx, fy, cx, cy = (688.00030953, 689.47629485, 307.66412893, 274.053018)

Distortion coefficients: 

[9.39260444e-03, 4.90535732e-01, 1.48962564e-02, 4.68503188e-04,
-1.77954077e+00]
