# Project Report for Project Sample
* Team: Pito Salas (rpsalas@brandeis.edu) and Charlie Squires (charliesquires@gmail.com)
* Date: xxx
* Github repo: xxxx
`
## Introduction

### Problem Statement including original objectives

### Relevant literature

## What was created

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

### Your own assessment
