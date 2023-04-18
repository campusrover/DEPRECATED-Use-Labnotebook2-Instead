# not-play-catch
Veronika Belkina
Robotics Independent Study
Fall 2022

## Links
- [not-play-catch github](https://github.com/campusrover/not-play-catch])
- [arm_control github](https://github.com/campusrover/arm_control)
- [arm documentation](https://github.com/campusrover/labnotebook/blob/master/faq/InterbotixPincherX100.md)
- [camera calibration](https://github.com/campusrover/labnotebook/blob/master/faq/camera_calibration.md)
- [apriltag setup](https://github.com/campusrover/labnotebook/blob/master/faq/apriltags_setup.md)
- [video](https://drive.google.com/file/d/1layYan32hGv9hBpsms_8D5jIO6dBdrxE/view?usp=share_link)

## Introduction

**Objectives**

-   Document the Interbotix X100 arm and create a program that would send commands to the arm through messages and help explore the capabilities and limitations of the arm.
-   Create a program that would push a ball towards a target and also to guess if a ball is under a goal and push it out.
-   The original purpose of this project was to create an arm that could play catch with the user. This would have involved catching a ball that was rolling towards it, and also pushing the ball towards a target.

## What was created

### Technical Description
#### `arm-control`

-   arm-control is a basic command line program that allows the user to send commands to the arm through a ROS message. There are two nodes: send_command.py and arm_control.py. The arm must be turned on, connected to the computer, and the ROS launch files launched.
-   send_command.py allows for a user to input a command (from a limited pool of commands) such as moving the arm to a point, opening or closing the gripper, moving to the sleep position, etc. There are some basic error checks in place such as whether the command is correct or whether the inputs are the correct type. The user can enter one command at a time and see what happens. The node will then publish the proper message out. There are 8 message types for the various commands to simplify parsing through and figuring out what is happening. When the user decides to exit the program, the sleep message will be sent out to put the arm in the sleep position and exit out of the send_command program.
-   Arm_control subscribes to the same messages that the send_command node publishes. It connects to the arm and initiates in a pose. From there, it will wait for a message. When it receives a message of a certain type, it will execute that command, and then stay in that position until another command comes.

#### `not-play-catch`

![image](https://user-images.githubusercontent.com/72238100/207402825-887121e1-4d26-47d8-9cfe-33e1c386a240.png)


This is the setup for play-catch. There is a board that is taped off to indicate the boundaries of the camera. The robot is placed in the center of the bottom half and its base-link is treated as the origin (world). There is a ball that is covered in green tape for colour detection, and Apriltags that are placed on top of goals. The camera is situated around 80 cm above the board to get a full view of the board.

**Camera and Computer Vision**

_Camera.py_

This node does the main masking and filtering of the images for colour detection. It then publishes a variety of messages out with the various image types after processing. It also allows for dynamic reconfiguration so that it is easier to find the colour that you need.

*Ball_distance.py*

![image](https://user-images.githubusercontent.com/72238100/207402903-b0f39dcc-f608-4fcd-8522-a1975fbca6af.png)

This node finds the green ball and determines its coordinates on the x-y plane. It publishes those coordinates and publishes an Image message that displays the scene with a box around the ball and the coordinates in centimeters in the corner.

**Sending Commands**

*Pickup.py*

The pickup node calculates the position of the ball and picks it up. It first turns to the angle in the direction of the ball and then moves in the x-z direction to pickup the ball. It then turns to the target Apriltag and puts the ball down 15cm away in front of the Apriltag. The calculations for the angles are down in the same way – determining the atan of the y/x coordinates. The steps are all done at trajectory time of 2.0 sec.

*Push.py*

The push node pushes the ball towards the target. It starts from the sleep position and then turns back to Apriltag target. Since it knows that the ball is already in position, it can push the ball with confidence after closing its gripper. The steps are all done at a trajectory time 0.9 sec to give extra impact to the ball and let it roll.

*Guess.py*

The guess node is a similar idea to the previous nodes, however, there is some random chance involved. The arm guesses between the two Apriltags, and then tries to push the ball out from under it. If a ball rolls out and is seen by the camera, then the arm will do a celebration dance. If there is no ball, then it will wait for the next turn. For this node, I had to make some adjustments to the way I publish the coordinates of the ball because when the ball wasn’t present, the colour detector was still detecting some colour and saying that there was a ball. So I adjusted the detector to only work when the detected box is greater than 30 pixels, otherwise, the node will publish (0,0,0). Since the ball is (x, y, 0.1) when it is being seen, there shouldn’t be a chance for an error with the ball being at (0,0,0). It would be (0,0,0,1) instead.

**Miscellaneous**

*Main.py*

The node that initiates the game with the user and the one that is run in the roslaunch file. From here, the other nodes are run. It also keeps track of the score.

*Ball_to_transform.py*

This node creates the transform for the ball using the coordinates that are published from ball_distance.py. It uses pose.yaml file to see what the name of the frame is, the parent frame, and the message topic the Pose is being published to.

**Guide on how to use the code written**

-   ``roslaunch arm_control command_center.launch``

-   If you’re working with the actual arm, ensure that it is turned on and plugged into the computer before starting. Otherwise, if you’re working in simulation, then uncomment ``<arg name=use_sim value=true />`` line in the launch file.
-   If you want to just run this file and send commands to the arm, then you can also run: ``rosrun arm_control send_command.py``

-   ``roslaunch playcatch play-catch.launch``

**Tables**

`arm_control`

|Nodes | Description|
|------|------------|
|send_command.py| Receives command line input from user and sends it to arm_control as a message|
|arm_control.py| Receives command line input from user and sends it to arm_control as a message|

`not-play-catch`

|Nodes | Description|
|------|------------|
|main.py| Brings everything together into a game|
|pickup.py| Locates the ball, determines if it can pick it up, and sends a series of commands to the arm to pick and place the ball in the correct angle for the Apriltag.|
|push.py| Pushes the ball towards the Apriltag using a series of commands.|
|guess.py| Guesses a box and pushes the ball out to see if it is there or not. If it is, then it does a celebratory dance. If it’s not, then it will wait for the next round sadly.|
|ball_distance.py| Calculates the location of the ball on the x-y plane from the camera and publishes a message with an image of the ball with a box around it and the coordinates of the ball.|
|ball_to_transform.py| Creates a transform for the ball.|
|camera.py| Does all the masking and filtering and colour detection to make the computer vision work properly.|



**Messages**

|Message |Type| Description|
|--------|----|------------|
|/arm_control/point| Pose| Pose message which tells the arm where to go|
|/arm_control/pose| Pose| Pose message which sets the initial pose of the arm|
|/arm_control/home| Bool| Sends the arm to the home position|
|/arm_control/sleep| Bool| Sends the arm to the sleep position|
|/arm_control/gripper| String| Opens or closes the gripper|
|/arm_control/exit| Bool| Exits the program and sends the arm to the sleep position|
|/arm_control/time| Float32| Sets the trajectory time for movement execution|
|/arm_control/celebrate| Bool| Tells the arm to celebrate!|

**Other Files/Libraries**

| Name | Description|
|------|------------|
|apriltag library| A library that allows for users to easily make use of Apriltags and get their transforms.|
|catch_settings.yaml| Apriltags settings to specify the family of tags and other parameters.|
|catch_tags.yaml| The yaml file for specifying information about the Apriltags.|
|pose.yaml| The yaml file for specifying information about the ball.|
|cvconfig.cfg| The dynamic reconfiguration file for colour detection and dynamically changing the colour that is being detected.|

## Story of the project

The idea for this project started a while ago, during the time when we were selecting what project to do in COSI119. I had wanted to play catch with a robot arm then, however, we ended up deciding on a different project at that time. But the idea still lived on inside me, and when Pito bought the PX100 over the summer, I was really excited to give the project a try.

The beginning was a little unorganized and a little difficult to get familiar with the arm because I had to stay home due to personal reasons for a month. I worked primarily on the computer vision aspects of the project at that time, setting up the Apriltag library and developing the colour detection for the ball and publishing a transform for it.

When I was able to finally return to the lab, it still felt hard to get familiar with the arm’s capabilities and limitations, however, after talking to Pito about it, I ended up developing the arm_control program to send commands and see how the ROS-Python API works more properly. This really helped push the project forward and from there, things felt like they went smoother. I was able to finish developing the pickup and pushing nodes. However, this program also allowed for me to see that the catching part of the project was not really feasible with the way things were. The commands each took several seconds to process, there had to be time between sending messages otherwise they might be missed, and it took two steps to move the arm in the x, y, z directions. It was a bit of a letdown, but not the end of the world.

As an additional part of the project, I was going to do a pick and place node, but it didn’t seem very exciting to be honest. So, I was inspired by the FIFA World Cup to create a game for the robot to play which developed into the guessing game that you see in the video.

This project had a lot of skills that I needed to learn. A big part of it was breaking down the task into smaller more manageable tasks to keep myself motivated and see consistent progress in the project. Since this project was more self-dictated than other projects that I have worked on, it was a challenge to do sometimes, but I’m happy with the results that I have achieved.

As I was writing this report, I had an insight that I felt would have made the program run much faster. It is a little sad that I hadn’t thought of it earlier which is simply to add a queue into the program to act as a buffer that holds commands as they come in. However, I wouldn’t have had to time to implement in a couple of days because it would require to rework the whole structure of the arm_control node. When I thought of the idea, I really didn’t understand how I didn’t think of it earlier, but I suppose that is what happens when you are in the middle of a project and trying to get things to work. Since I had written the arm_control program for testing and discovery and not for efficiency. But then I used it since it was convenient to use, but I didn’t consider that it was not really an optimized program. So next semester, I would like to improve the arm_control program and see if the arm runs the not-play-catch project faster.

Either way, I really enjoyed developing this project and seeing it unfold over time, even if it’s not exactly what I had aspired to do in the beginning. Hopefully, other people will also enjoy creating exciting projects with the arm in the future!

(I didn’t really want to change the name of the project, so I just put a not in front of it…)

