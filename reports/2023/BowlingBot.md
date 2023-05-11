---
title: Bowling Bot
author: Michael Jiang, Matthew Merovitz
status: new
date: may-2023
type: report
---
# BowlingBot (Spring 2023)

Team Members: Michael Jiang (michaeljiang@brandeis.edu) & Matthew Merovitz (mmerovitz@brandeis.edu)


## Background:
The project aimed to create a robot that could mimic the game of bowling by grabbing and rolling a ball towards a set of pins. The robot was built using a Platform bot with a pincer attachment. The ball is a lacrosse ball and the “pins” are either empty coffee cups or empty soda cans.

At first, we imagined that in order to achieve this task, we would need to create an alley out of blocks, map the space, and use RGB for ball detection. None of this ended up being how we ultimately decided to implement the project. Instead, we opted to use fiducial detection along with line following to implement ball pickup and bowling motion. The fiducial helps a lot with consistency and the line following allows the robot to be on track. 

## Objective:
The primary objective of the BowlingBot project was to develop a robot that could perform the task of bowling, including locating the ball, grabbing it, traveling to the bowling alley, and knocking down the pins with the ball. The project aimed to use a claw attachment on the Platform bot to grab and roll the ball towards the pins. The robot was designed to use fiducial markers for ball detection and orientation, and line detection for pin alignment.

## Algorithms:
The BowlingBot project utilized several algorithms to achieve its objectives. These included:

1. Fiducial Detection: The robot was designed to use fiducial markers to detect the location and orientation of the ball. The fiducial markers used were ArUco markers, which are widely used in robotics for marker-based detection and localization. In addition, we used odometry to ensure precision, allowing us to consistently pick up the ball. 

![IMG-1403](https://user-images.githubusercontent.com/55816618/236976236-ec01f3f1-6a72-4b8f-be1a-cbf5f50c88e8.jpg)

2. Line Detection: The robot utilized line detection techniques to align itself with the pins. This involved using a camera mounted on the robot to detect the yellow line on the bowling alley and align itself accordingly. This also uses OpenCV.

![IMG-1405](https://user-images.githubusercontent.com/55816618/236976252-3e4381c5-6e8e-425a-999c-cd21f2c02653.jpg)

3. Pincer Attachment: The BowlingBot utilized a pincer attachment to pick up the ball and roll it towards the pins. 

## Description of the Code:

The code is centered around a logic node that controls the logic of the entire bowling game. It relies on classes in fiducialbowl.py, pincer.py, bowlingmotion.py. 

Fiducialbowl.py: This class has two modes. Travel mode is for traveling to the ball in order to pick it up and then traveling back to the bowling alley. Bowling mode is for ensuring centering of the robot along the line with the assistance of the yellow line. 

Pincer.py: Controls the opening and closing of the pincer attachment on the platform bot. 

Bowlingmotion.py: This node turns until the yellow line is seen, centers the robot along the line, and then performs the quick motion of moving forwards and releasing the ball. 


## Blockers and Roadblocks:
During our time developing the BowlingBot, we encountered several obstacles and challenges that we were forced to overcome and adjust for. Initially, the crowded lab hours slowed down our camera and sensors, peaking at about a minute delay between the camera feed and what was happening in real time. We decided collectively to instead come during office hours when it was less crowded in order to avoid the severe network traffic, which allowed us to test the camera properly. 

Another roadblock was finding an appropriate ball, which held up our testing of the bowling motion. We knew we needed a ball that was heavy enough to knock over pins, but small enough to fit inside the pincer. We initially bought a lacrosse ball to test with, but it was too grippy and ended up catching on the ground and rolling under the robot, forcing us to pivot to using a field hockey ball, which was the same size and weight but crucially was free from the grippiness of the lacrosse ball.

Another challenge we had to adapt to was the actual bowling motion. We initially proposed that the robot should implement concepts from the line follower so that it could adjust its course while rolling down the alley. However, we discovered that the robot’s bowling motion was too quick for it to reliably adjust itself in such a short time. To resolve this, we pivoted away from adjusting in-motion and instead implemented aligning itself as best as possible before it moves forward at all.

## Story of the Project:
The BowlingBot project was inspired by the team's love for robotics and bowling. The team was intrigued by the idea of creating a robot that could mimic the game of bowling and we spent lots of time researching and developing the project.

The project was challenging and required us to utilize several complex algorithms and techniques to achieve our objectives. We faced several obstacles, including issues with ball detection and alignment, and had to continuously iterate and refine our design to overcome these challenges.

Despite the challenges, the team was able to successfully develop a functional BowlingBot that could perform the task of bowling. We are excited to showcase our project to our peers and hold a demonstration where participants can compete against the robot.

Overall, the BowlingBot project was a success, and the team learned valuable lessons about robotics, problem-solving, and combining several class concepts into a more complex final product.


