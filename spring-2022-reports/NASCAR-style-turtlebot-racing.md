# NASCAR TurtleBot Racing Series

## Introduction

### Problem statement, including original objectives

As old as the field of robotics itself, robot competitions serve as a means of testing and assessing the hardware, software, in-between relationships, and efficiencies of robots. Our group was inspired by previous robot-racing related projects that navigated a robot through a track in an optimal time, as well as the challenge of coordinating multiple robots. Subsequently, our project aims to simulate a NASCAR-style race in order to determine and test the competitive racing capabilities of TurtleBots.
Our project’s original objectives were to limit-test the movement capabilities of TurtleBots, implement a system that would allow for the collaboration of multiple robots, and present it in an entertaining and interactive manner. We managed to accomplish most of our goals, having encountered several problems along the way, which are described below. As of the writing of this report, the current iteration of our project resembles a NASCAR time-trial qualifier, in which a TurtleBot moves at varying speed levels depending on user input through a gui controller, which publishes speed data to our ‘driver’ node. For our driver node, we’ve adapted the prrexamples OpenCV algorithm in order to ‘drive’ a TurtleBot along a track, explained below, which works in conjunction with a basic PID controller in order to safely navigate the track.

### Relevant literature

[Resizing an image w/ openCV](https://www.tutorialkart.com/opencv/python/opencv-python-resize-image/).   
[openCV resizing interpolation](https://medium.com/@wenrudong/what-is-opencvs-inter-area-actually-doing-282a626a09b3).

## What was created (biggest section)

### Technical descriptions, illustrations

Below is the general setup of each robot’s “node tree.” These are all running on one roscore; the names of these nodes are different per robot. Green nodes are the ones belonging to the project, whereas blue nodes represent nodes initialized by the Turtlebot. The interconnecting lines are topics depicting publisher/subscriber relationships between different nodes.
The “driver” in the robot is the Line Follower node. This is what considers the many variables in the world (the track, pit wall (controller), car status, and more) and turns that into on-track movement. To support this functionality, there are some nodes and functions which help the driver.

![node structure](https://media.discordapp.net/attachments/640359081688956928/971202293175320636/Untitled.png)

#### Image processor

This is a function inside the line follower node which we felt was better to inline. Its workings are explained in exhausting detail in the next section. The main bit is that it produces an image with a dot on it. If the dot is on the left, the driver knows the center of the track is on the left.

#### Lap Counter

This node is another subscriber to the camera. It notifies a car whenever it has crossed the finish line. This is accomplished, again, with the masking function of OpenCV. It is very simple but obviously necessary. This also required us to put a bit of tape with a distinct color (green in our case) to use as a finish line.

#### Overtake

For a robot to pass another on track, it has to drive around it. The first and final design we are going to use for this involves LiDAR. A robot knows when one is close enough to pass when it sees the scanner of another robot close enough in front of it. It then knows it can go back when it sees the passed robot far enough behind it, and also sees that the track to its side is clear.

#### GUI

The GUI serves as the driver’s controller, allowing the driver to accelerate or decelerate the car, while also showing information about current speed and current heat levels. In combination with the GUI, we also provide the driver with a window showing the Turtlebot’s view from the raspicam, in order to simulate an actual driving experience.

#### The track

Track construction was made specifically with the game design in mind. The track is made of two circles of tape: blue and tan in an oval-sh configuration. In the qualifying race, the cars drive on the tan line. In the actual race, the cars primarily drive on the blue line, which is faster. Cars will switch to the outer tan track to pass a slower one in the blue line, however. The intent behind some of the track’s odd angles is to create certain areas of the track where passing can be easier or harder. This rewards strategy, timing, and knowledge of the game’s passing mechanics.
In qualifying, the tan track has turns that, under the right conditions, can cause the robot to lose track of the line. This is used as a feature in the game, as the players must balance engine power output with staying on track. If a careless player sends the car into the corner too quickly, it will skid off of the track, leaving the car without a qualifying time.

![The track](https://i.imgur.com/y8lb6K5.jpg)

### Discussion of interesting algorithms, modules, techniques

#### Line Follower Algorithm

The core of our robot navigation relies on prrexamples’ OpenCV line follower algorithm, which analyzes a compressed image file to detect specific HSV color values, then computes a centroid representing the midpoint of the detected color values. The algorithm first subscribes to the robot’s raspberry pi camera, accepting compressed images over raw images in order to save processing power, then passes the image through the openCV’s bridge function, converting the ROS image to a CV2 image manipulatable by other package functions. In order to further save on process power, we resize the image using cv2.resize(), which changes the resolution of the image, reducing the number of pixels needing to be passed through the masking function. Regarding the resize() function, we recommend using cv2.INTER_AREA in order to shrink the image, and cv2.LINEAR_AREA or cv2.CUBIC_AREA in order to enlarge the image. Note that while using the cv2.CUBIC_AREA interpolation method will result in a higher quality image, it is also computationally more complex. For our project’s purposes, we used the cv2.INTER_AREA interpolation method. The algorithm then accepts a lower and upper bound of HSV color values to filter out of the CV2 image, called ‘masking’. The image then has all of its pixels removed except for the ones which fall within the specified range of color values. Then, a dot is drawn on the “centroid” of the remaining mass of pixels, and the robot attempts to steer towards it. To make the centroid more accurate and useful, the top portion of the image is stripped away before the centroid is drawn. This also means that if the color of surroundings happen to fall within the specified range of color values, the robot won’t take it into account when calculating the centroid (which is desirable).

#### Modularity Technique

Modular programming was a critical part of our program to determine which parts of our driver program were computationally intensive. The original line follower node processed the images inside the camera callback function. This arrangement causes the computer to be processed tens of times per second, which is obviously bad. The compression also would process the full size image. We tried remedying this by shrinking the already compressed image down to just 5% of its original size before doing the masking work. Unfortunately, both of these did not appear to have an effect on the problem.

#### Interacting with ros using a tkinter gui

We were able to construct a tkinter gui that interacts with other nodes. We accomplished this by initializing the gui as its own node, then using rostopics to pass user input data between the gui and other nodes.

### Execution

Executing our project is twofold; first, run roslaunch nascar nascar.launch. Make sure the parameters of the launch file correspond to the name of the TurtleBot. Subsequently, open a separate terminal window and cd into the project folder, then run python tkinter_gui.py.

## Story of the project.

Our original project was an entirely different concept, and was meant to primarily focus on creating a system that allowed for multi-robot (2-4+) collaboration. We originally intended to create a “synchronized robot dancing” software, which was imagined as a set of 2, 3 or 4 robots performing a series of movements and patterns of varying complexity. Due to the project lacking an end goal and purpose, we decided to shift our focus onto our idea of a NASCAR-style racing game involving two robots. Rather than focusing on designing a system and finding a way to apply it, we instead began with a clear end goal/visualization in mind and worked our way towards it.

While working on our subsequent NASCAR-style multi-robot racing game, two major problems stuck out: one, finding suitable color ranges for the masking algorithm; two, computational complexity when using multiple robots. Regarding the first, we found that our mask color ranges needed to be changed depending heavily on lighting conditions, which changed based on time of day, camera exposure, and other hard-to-control factors. We resolved this by moving our track to a controlled environment, then calibrated our color range by driving the TurtleBot around the track. Whenever the TurtleBot stopped, we passed whatever image the raspicam currently saw through an image editor to determine HSV ranges, then multiplied the values to correspond with openCV’s HSV values (whereas GIMP HSV values range from H: 0-360, S: 0-100, and V: 0-100, openCV’s range from H: 0-180, S: 0-255, and V: 0-255).
The cause of our second problem remains unknown, but after some experimentation we’ve narrowed down the cause to relating to computational power and complexity. When running our driver on multiple robots, we noticed conflicting cmd_vels being published resulting from slowdowns in our image processing function, realizing that adding additional robots would exponentially increase the computational load of our algorithm. We attempted to resolve this by running the collection of nodes associated with each robot on different computers, with both sets of nodes associated with the same roscore.

Overall, our group felt that we accomplished some of the goals we originally set for ourselves. Among the original objectives, which were to limit-test the movement capabilities of TurtleBots, implement a system that would allow for the collaboration of multiple robots, and present it in an entertaining and interactive manner, we felt we successfully achieved the first and last goal; in essence, our project was originally conceived as a means of entertainment. However, we were also satisfied with how we approached the second goal, as throughout the process we attempted many ways to construct a working multi-robot collaboration system; in the context of our project, we feel that our overtake algorithm is a suitable contribution to roboticists who would consider racing multiple robots.

