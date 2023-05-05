# Project Report for Project Sample
* Names: Isaac Goldings (isaacgoldings@brandeis.edu), Jeremy Huey (jhuey@brandeis.edu), David Pollack  
* Instructor: Pito Salas (rpsalas@brandeis.edu) 
* Date: 05/03/2023
* Github repo: https://github.com/campusrover/COSI119-final-objectSorter

## Introduction
This Object Sorter robot uses qrcode/fiducial detection and color detection to find cans and then pick them up with a claw and drop them off at designated sorted positions. In this implementation, the robot is mobile and holds a claw for grabbing objects. The robot executes a loop of finding an item by color, grabbing it, searching for a dropoff fiducial for the correct color, moving to it, dropping it off, and then returning to its pickup fiducial/area. 
The project runs via the ros environment on linux, requires some publically available ros packages (mostly in python), and runs on a custom "Platform" robot with enough motor strength to hold a servo claw and camera. Ros is a distributed environment running multiple processes called nodes concurrently. To communicate between them, messages are published and subscribed to. The robot itself has a stack of processes that publish information about the robot's state, including its camera image, and subscribes to messages that tell it to move and open and close the claw. 
Our project is divided up into a main control node that also includes motion publishing, and two nodes involved with processing subscribed image messages and translating them into forward and angular error. These are then used as inputs to drive the robot in a certain direction. The claw is opened and closed when the color image of the soda can is a certain amount of pixel widths within the camera's image. 
The project runs in a while loop, with different states using control from the two different image platforms at different states until it is finished with a finished parameter, in the current case, when all 6 cans are sorted after decrementing.  

![image](https://github.com/campusrover/labnotebook/blob/master/images/Platform_Robot.jpg)

### Problem Statement including original objectives
Our plan for this project is to build a robot that will identify and sort colored objects in an arena.
These objects will be a uniform shape and size, possible small cardboard boxes, empty soda cans, or
another option to be determined. The robot will be set up in a square arena, with designated locations for
each color. Upon starting, The robot will identify an object, drive towards it, pick it up using the claw
attachment, and then navigate to the drop off location for that color and release it. It will repeat this until
all the objects have been moved into one of the designated locations.

### Relevant literature
Color detection robots and robotic arms are a large and known factor of industrial workflows. We wanted to incorporate the image detection packages and show its use in a miniature/home application. 

Color Detection : http://wiki.ros.org/opencv_apps
Aruco Fiducial Detection and explanation: https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html

## What was created

The project has been realized almost exactly to the specifications. We found that a bounding arena was not neccesary if we assumed that the travelling area was safe to the robot and the project. 
This implementation melds two detection camera vision algorithms and motion control via error feedback loop. 

The current version uses three cans of two different colors, sorts the 6 cans, and finishes. 
We found some limitations with the project, such as network traffic limiting the refresh information of the visual indicators. 

1. The first vision algorithm is to detect fiducials, which look like qr codes. The fiducials are printed to a known dimension so that the transformation of the image to what is recognized can be used to determine distance and pose-differential. The aruco_detect package subscribes to the robot's camera image. It then processes this image and returns an array of transforms/distances to each fiducial. The package can recognize the distinctness between fiducials and searches for the correct one neede in each state. The transform is then used to determine the amount of error between then robot and it. Then the robot will drive ion that direction, while the error is constantly updated with new processed images. (See Section: Problems that were Solved for issues that arose.)

2. The second vision algorithm is to detect color within the bounds of the camera image. The camera returns a color image, and the image is filtered via HSV values to seek a certain color (red or green). A new image is created with non-filtered pixels (all labelled one color) and the rest is masked black. A contour is drawn around the largest located area. Once the width of that contour is a large enough size in the image, the claw shuts. The processing thus can detect different objects, and the robot will go towards the object that is the clsoest due to its pixel count. (See Section: Problems that were Solved for issues that arose.)

## Technical Description

Our project is to get a platform robot with a claw mount to sort soda cans into two different areas based on color. The test area is set up with two fiducials on one side and one fiducial on the other, approximately 2 meters apart. The two fiducials are designated as the red and green drop-off points, and the one is the pickup point. Six cans are arranged in front of the pickup point with the only requirements being enough spacing so the robot can reverse out without knocking them over, and a direct path to a can of the color the robot is targeting (e.x. so it doesn’t have to drive through a red can to get to a green one). The robot starts in the middle of the area facing the pickup point. 

The robot’s behavior is defined by three states, “find item,” “deliver item,” and “return to start.” For “find item” the robot uses computer vision to identify the closest can of the target color, and drive towards it until it is in the claw. Then it closes the claw and switches state to “deliver item.” For this state, the robot will rotate in place until it finds the fiducial corresponding to the color of can it is holding, then drive towards that fiducial until it is 0.4 meters away. At that distance, the robot will release the can, and switch to “return to start.” For this last state, the robot will rotate until it finds the pickup fiducial, then drive towards it until it is 1 meter away, then switch back to “find item.” The robot will loop through these three states until it has picked up and delivered all the cans, which is tracked using a decrementing variable. 

## Guide to Codebase and algorithms
This project consists of three main nodes, each with their own file, main_control.py, contour_image_rec.py, and fiducial_recognition.py. contour_image_rec.py uses the raspicam feed to calculate the biggest object of the target color within the field of view, and publishes the angle necessary to turn to drive to that object. It does this by removing all pixels that aren’t the target color, then drawing a contour around the remaining shapes. It then selects the largest contour, and draws a rectangle around it. After that it calculates the middle of the rectangle and sends uses the difference between that and the center of the camera to calculate the twist message to publish.

Fiducial_recognition.py just processes the fiducials and publishes the transforms. main_control.py subscribes to both of the other nodes, and also contains the behavior control for the robot. This node is in charge of determining which state to be in, publishing twist values to cmd_vel based on the input from the other nodes, and shutting the robot down when the cans are all sorted.
Prod_v5.launch is our launch file, it starts the main_control node, the image recognition node, the fiducial recognition node, and the aruco_detect node that our fiducial recognition node relies on.


## Story of the project.

### How it unfolded, how the team worked together

### problems that were solved, pivots that had to be taken
1. Fidicial-based issues: 
Recognition only worked to a certain distance, after that, the image resolution is too low for certainty. To resolve this, we allowed the bounds of the project to remain within 3 meters. 

Network traffic (more other robots/people) on the network would introduce significant lag to sending images across the network from the robot to the controlling software (on vnc on a separate computer). This would cause things such as overturning based on an old image, or slow loading of the fiducial recognition software. This can be worked around by: a. increasing the bandwidth related to the project, b. buffering images, c. reducing the size or performing some calculations on the robot's Raspberry Pi chip (this is limited by the capacity of that small chip), d. changing and reducing speed parameters when network traffic is high manually (allowing less differential between slow images and current trajectory). 

Positions where the entirely of the fiducial is too close and gets cropped. Fiducial recognition requires the entire image to be on screen. This information can be processed however and saved, either as a location (which can be assessed via mapping) or as previous information of error. To combat this issue, we raised the positions of the fiducials so that they could be seen above the carried soda can and had the drop off point at 0.4m away from the fiducial. 

2. Color-based issues: 

3. Other/Physical issues:
The platform robot is a big chonker! It also has meaty claws. The robot being large causes it to take up more room and move farther. In the process it sometimes will knock over other features. To combat this, we added a reversing -0.05m to the turning motion when the robot begins to switch to seeking a dropoff fiducial. The meaty claws is one of the causes of requiring a larger robot with stronger motors. 

![image](https://github.com/campusrover/labnotebook/blob/master/images/lobster.jpg)
