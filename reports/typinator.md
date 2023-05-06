# Typinator.md
## Cosi 119A - Autonomous Robotics
### Team: Elliot Siegel (elliotsiegel@brandeis.edu) and Kirsten Tapalla (ktapalla@brandeis.edu)
### Date: May 2023
### Github repo: https://github.com/campusrover/typinator2023
## Introduction
What is a better use of a robot than to do your homework for you? With the surging use of ChatGPT among struggling, immoral college students, we decided to make cheating on homework assignments even easier. Typinator is a robot that uses computer vision and control of a robotic arm to recognize a keyboard, input a prompt to ChatGPT, and type out the response.

### Problem Statement

<b>Original Problem Statement</b>
> Demonstrate the robotic arm by having it type its own code. The arm will hold an object to press keys with. The robot will hopefully write code quickly enough so that people can watch it write in real time. The robot could also type input to chatgpt, and have a separate program read the output to the robot. It will then cmd+tab into a document and type the answer. Both of these demonstrations will show the arm “doing homework,” which I think is a silly butchallenging way to demonstrate the robot’s capabilities.

<b>Learning Objective</b><br>

The goal of this project is to demonstrate the capability of the robotic arm in conjunction with computer vision. While the end result of the project is not actually useful in a real world setting, the techniques used to produce the end result have a wide variety of applications. Some of the challenges we overcame in computer vision include detecting contours on images with a lot of undesired lines and glare, reducing noise in contoured images, and transforming image coordinates to real world coordinates. Some of the challenges we overcame in using a robotic arm include moving to very specific positions and translating cartesian coordinates to polar coordinates since the arm moves by rotating. We also worked extensively with ROS.

<b>Original Goals</b>
- Text Publishing
  - Publish large chunks of text with ROS
  - Clean text into individual characters that can be typed
  - Send and receive data from ChatGPT
- Computer Vision
  - Recognize keys on a keyboard
  - Detect letters on a keyboard (this was not used in the final project due to inconsistency)
  - Transform image coordinates to real world coordinates
- Arm Control
  - Move the arm to specific positions with high accuracy

## What was created
### Technical Description
<b>Project Structure</b>
![Project structure diagram](https://user-images.githubusercontent.com/62267188/236589001-e161beb5-63ed-4a94-a648-fa01d3452393.png "Project structure diagram")
<i>Project structure diagram</i>

The project works by having a central coordinating node connect arm motion, image recognition, keyboard input, and text input. All of these functionalities are achieved through ROS actions except for the connection to the keyboard input, which is done through a simple function call (coordinator --> keys_to_type_action_client). Our goal with designing the project in this way was to keep it as modular as possible. For example, the chatgpt_connection node could be replaced by another node that publishes some other text output to the <code>generated_text</code> ROS topic. A different arm_control node could be a server for the arm_control action if a different robotic arm requiring different code was used. Any of the nodes connected with actions or topics could be substituted, and we often used this to our advantage for debugging.

<b>Arm Motion</b>

<img width="366" alt="Screen Shot 2023-05-05 at 9 22 23 PM" src="https://user-images.githubusercontent.com/62267188/236590646-f3e0d91f-3ffa-45a7-b622-5871449c2e01.png"><i>Arm motion diagram</i>

As shown above, the arm moves with rotation and extension to hover above different keys. To achieve this movement from a flat image with (x,y) coordinates, we converted the coordinates of each key into polar coordinates. From (x,y) coordinates in meters, the desired angle of the arm is determined by $θ=atan(x/y)$. The desired extension from the base of the arm is found by $dist=y/cos(θ)$. We calculate this relative to the current extension of the arm so $Δdist=dist-currentDistance$.

<b>Keyboard Recognition</b>

![keyboard_img](https://user-images.githubusercontent.com/62267188/236589023-4044c059-9794-4980-80c2-0cb8967b3a74.png "Keyboard contouring example 1")<i>Keyboard contouring example 1</i> ![keyboard_img 2](https://user-images.githubusercontent.com/62267188/236589021-a9bdbf8b-21b8-473c-926e-7eda8a39d767.png "Keyboard contouring example 2")<i>Keyboard contouring example 2, with a more difficult keyboard to detect</i>

Keyboard recognition was achieved with OpenCv transformations and image contouring. The keys of many different keyboards are able to be recognized and boxes are drawn around them. We were able to achieve key recognition with relatively little noise.

The first transformation applied to an image is optional glare reduction with <code>cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))</code>. Then, the image is converted to greyscale with <code>cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)</code>. A threshold is then applied to the image. Different thresholds work for different images, so a variety of thresholds must be tried. One example that often works is <code>cv2.THRESH_OTSU | cv2.THRESH_BINARY_INV</code>. This applies an Otsu threshold followed by a binary inverse threshold. Finally, dilation and blur are applied to the image with customizable kernel sizes.

After the transformations are applied, OpenCV contouring is used with <code>contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)</code>. External contours (those with child contours and no parent contours) are ignored to reduce noise.

### Running the code

Before running the project, connect the arm and a usb webcam to a computer with ROS and the software for the Interbotix arm installed (this project will not work in the vnc). [More details on the arm software are provided here.](https://campus-rover.gitbook.io/lab-notebook/faq/interbotixpincherx100)

Launch the arm: <code>roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=px100 use_sim:=false</code>

<b>Calibration</b><br>

Follow these instructions if the keyboard you are using has not already been calibrated.

Position the usb camera in the center of the keyboard, facing directly down. The whole keyboard should be in frame. Ideally the keyboard will have minimal glare and it will be on a matte, uniform surface. The width of the image must be known, in meters. Alternatively, an image of the keyboard can be uploaded, cropped exactly to known dimensions (i.e. the dimensions of the keyboard).

Run <code>roslaunch typinator image_setup.launch</code> to calibrate the image filters for optimized key detection. A variety of preset filters will be applied to the image. Apply different combinations of the following parameters and determin which filter preset/parameter combination is the best. An optimal filter will show keys with large boxes around them, not have a lot of noise, and most importantly detect all of the keys. If there is a lot of noise, calibration will take a long time.

| Parameter | Default | Use |
|:---|:---|:---|
| image_file | False | Input the path to the image file you would like to use if not using the camera |
| reduce_glare | False | Set to True to apply glare-reduction to the image |
| blur | 1 | Apply blur to the image before finding contours. 1 results in no blur. |
| kernel | 1 | Apply a dilation kernel to the image. 1 has no effect (1 pixel kernel) |


Once the tranformation preset, reduce_glare, blur, and kernel have been selected, the arm can be calibrated and run. Run <code>roslaunch typinator typinator.launch</code> with the appropriate parameters:

| Parameter | Default | Use |
|---|---|---|
| calibrate | True | Set to false if loading the keyboard from a preset |
| keyboard_preset | "temp_preset.json" | In calibration mode, save the keyboard to this file. Load this keyboard file if calibrate=False. |
| chatgpt | "Generate random text, but not lorem ipsum" | Text input to chatgpt |
| img_width | 0 | MUST SET THIS VALUE DURING CALIBRATION: Set to the the real world width of the image in meters |
| img_height | img_width*aspect ratio | Set this to the real world height of the image if necessary |
| arm_offset | 0 | Set this to the distance from the bottom edge of the image to the middle of the base of the arm |
| image_file | False | Input the path to the image file you would like to use if not using the camera |
| reduce_glare | False | Set to True to apply glare-reduction to the image |
| blur | 1 | Apply blur to the image before finding contours. 1 results in no blur. |
| kernel | 1 | Apply a dilation kernel to the image. 1 has no effect (1 pixel kernel) |

The arm will press each key it detects and save its position! It will then type the output from ChatGPT! You can open a file you want it to type into, even while calibrating.

<b>Running from a preset</b></br>

If a specific keyboard has already been calibrated, position the arm appropriately and run <code>roslaunch typinator typinator.launch calibrate:=False keyboard_preset:="filename.json"</code>
Also set <code>image_file</code> if you want boxes to display current keys, and <code>chatgpt</code> if you want a custom input to chatgpt.

### Source Files

### keys_to_type_server.py
<b>*indiv_keys():* </b><br/>
This is the callback function of Key service. It processes the text that it’s passed as its goal, and separates it by key/letter so that only one will be typed at a time. It also takes into account capital letters and inserts ‘caps_lock’ before and after the letter is added to the result list, since a keyboard only types in lower case. Furthermore, it takes into account new lines and passes ‘Key.enter’ in place of logging ‘\n’.

### keys_to_type_client.py
This class subscribes to the 'generated_text' topic, which consists of the text generated by ChatGPT that the arm is meant to type.<br/>
<b>*cb_to_action_server():*</b><br/>
This is the function that sends/sets the goal for the action server, and waits for the action to completely finish before sending the result. It makes the ChatGPT message the goal of the action, and waits for the text/letter separation to complete before sending the list of individual letters of the the text as the result.<br/>
<b>*get_next_key():*</b><br/>
This function returns the first key in the deque, which is the letter that needs to be typed by the arm next. 

### key_pos.py
This class is used to convert key pixel positions to arm target x,y coordinates from an image of a keyboard. <br/><br/>
<b>*img_cb() and img_cb_saved():* </b><br/>
These functions use cv2 image processing to find the individual keys on a keyboard from an image. It sets some of the class variables, such as height and width, which is then later used to calculate the conversion rates used to turn the key pixel positions into meter positions. The img_cb() function is used when it's subscribing to, and processing images from, a camera topic. Then, the img_cb_saved() function is used when an already saved image of a keyboard is being used instead of one received from the camera. Both functions essentially have/use the same code. <br/><br/>
<b>*find_key_points():* </b><br/>
This function converts key pixel locations in the image to the physical x,y meter positions that can then be used to move the real arm so that it can hit the keys as accurately as possible on the actual keyboard.<br/><br/>
<b>*print_points():* </b><br/>
This function prints each calculated point that has been stored in the class. The points are where we need the robot arm to move to in order to press a key. <br/><br/>
<b>*practice_points():* </b><br/>
This function returns the entire list of points saved in the class, with the x and y meter values of the keys' positions on the physical keyboard. <br/><br/>
<b>*key_pos_action and action_response():* </b><br/>
This is the KeyPos action and its callback function. The purpose of the action is to send the physical positions of the each key. It send real x, y, and z coordinates along with image x, image y, image height, and image width coordinates.

### chat_gpt.py
This node connects to ChatGPT through an API key to generate a message/string for the arm to type onto the keyboard. It then publishes the text to the 'generated_text' topic to be used by other nodes.<br/>

### arm_control.py
This file contains the sever for the 'arm_control' action, and it connects to the robotic arm. The MoveArm class acts as the link between the coordinates that the server receives and the motion of the arm.<br>
- <b>*calculate_distance()*</b> converts cartesian coordinates to polar coordinates.<br>
- <b>*set_xy_position(x, y)*</b> moves the arm to the specified cartensian x,y coordinates over the keyboard<br>
- <b>*press_point(z)*</b> moves the arm down to and back up from the specified depth coordinate<br>
- <b>*move_then_press(x,y,z)*</b> calls set_xy_position(x,y) followed by pres_point(z)<br>
- <b>*move_arm(msg)*</b> is an action callback function that calls for an instance of the MoveArm class to move the arm to the specified coordinates. It returns "True" when done.

### coordinator.py
This file connects the other ROS nodes together. It runs the "coordinator" node. In "calibrate" mode, the <b>*main()*</b> uses the Coordinator class to get the image of the keyboard and find the positions of individual keys. Since these keys are not laballed, it then has the arm press each found contour. After each press, it gets the last pressed key with <b>*coordinator.get_key_press()*</b>. This uses the KeyPress action to get the stored last key press. If a new key was pressed, it is mapped to the location that the arm went to. After the arm has gone to every found position, it saves the key to position mappings in a .json file.

In "type" mode, the <b>*main()*</b> function loads one of the stored key mappings into a python dictionary. It gets the next letter to type one at a time using the <b>*get_next_key()*</b> method of a KeysActionClient. It uses the dictionary to find the location of the key, and then it send the arm to that location.

### key_boxes.py
This file contains the FindKeys class. This class is used to find the positions of keyboard keys.<br>
- <b>*transform_image(img, threshold, blur, dilate)*</b> applies a transformation to an image of a keyboard with the specified threshold, blur kernel, and dilation kernel.<br>
- <b>*contour_image(img)*</b> applies image contouring and noise reduction to find the keys. It returns the contours as instances of the Key class.

### keys.py
This file contains the Key class that stores the data of each individual key found from the FindKeys class. <b>*center()*</b> returns the center of a key.

### key_pub.py
This file listens for keyboard input with pynput. The KeyListener class keeps track of the most recent key release and acts as the server for the KeyPress action. When called, it returns the most recent key that was pressed.

### Nodes

| Node | File |
|---|---|
| coordinator | coordinator.py |
| arm_control | arm_control.py |
| keys_to_type_action_client | keys_to_type_client.py |
| keys_to_type_action_server | keys_to_type_server.py |
| chatgpt_connection | chat_gpt.py |
| key_pub | key_pub.py |
| key_pos_action | key_pos.py |

### Actions

| Action | Type | Data |
|---|---|---|
| arm_control | ArmControl | float32 x<br>float32 y<br>float32 z<br>---<br>string success<br>---<br>string status |
| key_pos_action | KeyPos | string mode<br>---<br>float32[] positions<br>int32[] img_positions<br>---<br>bool status |
| key_press | KeyPress | string mode<br>bool print<br>---<br>string key<br>---<br>bool status |
| keys_to_type_action | Keys | string full_text<br>---<br>string[] keys<br>---<br>bool status |

### Topics

| Topic | Type |
|---|---|
| /generated_text | String |

## Story of the project
### How it unfolded, how the team worked together

We started the project with high hopes for the capabilities of the arm and image recognition software. We planned to have the arm hold a pen and press keys on the keyboard. We hoped that the keys would be recognized through letter detection with machine learning.

Initially, we mainly set up the Interbotix arm software in our vnc’s, tested the movement of the arm, and began working on keyboard recognition. Our preliminary tests with detecting the keys on the keyboard were promising. Letter detection for the letter keys was promising, however, it was not working for the special keys or keys with more than one symbol on them.

We decided that to get the main functionality of the project working, we would divide the work between ourselves. 

Kirsten worked on connecting to ChatGPT, publishing the response from ChatGPT as characters on the keyboard, and transforming pixel coordinates in an image into real world coordinates in meters. We discovered that ROS nodes that perform actions will block for too long to receive improperly timed callbacks, so in order to avoid issues with this, Kirsten worked on converting letter publishing ROS topics into ROS actions. By designing our code this way, we were able to have a central ROS node that called ROS actions and got responses sequentially.

Elliot worked on moving the robotic arm so that it could move to specific cartesian coordinates and then press down. He also worked on refining the keyboard recognition and designing the central node to bring the project together. After many attempts at reducing noise with statistical methods when finding the outlines of keys on a keyboard (with OpenCV contouring), he discovered that the contour hierarchy was a much more effective way of reducing noise. With contour hierarchy and number of customizable image transformations applied to the images, he was able to consistently find the keys on keyboards.

After reexamining letter detection with machine learning, we decided that it was not consistent enough to work for the desired outcome of our project. We realized we could “calibrate” the arm on the keyboard by pressing each key that was found through OpenCV contouring and listening for keystrokes. In theory, this could provide 100% accuracy.

We encountered many difficulties when putting our pieces of the project together and running it on the real arm. As can be expected, a few small quirks with ROS and a few small bugs caused us a lot of difficulty. For example, when we first ran the project as a whole, it started but nothing happened. We eventually found that our text publishing node had a global variable declared in the wrong place and was returning <code>None</code> instead of the letters it received from ChatGPT.

One major issue we encountered is that the arm motor turns itself off any time it thinks it is in one position but is actually in a different position. This usually only happened after around 10 key presses, so to fix it we have the arm return to its sleep position every 6 keystrokes. Another major issue is that the arm holds down on the keys for too long. We solved this problem by turning off key repeats in the accessibility settings of the computer controlling the arm. We found that the arm has some limitations with its accuracy which affects its typing accuracy, since the keys are such small targets.

### Our assessment

We were mostly successful in achieving our original project goals. We are able to recognize a keyboard, train the arm on that keyborad, and type an output from ChatGPT. We were not successful in using letter recognition to press found keys in real time, however, with our method of calibrating the arm on a specific keyboard, the arm was able press any key on almost any keyboard. If we manually input arm positions for each key on a specific keyboard we would likely get better typing accuracy, but this would not be in line with our initial goal. Our use of image contouring to find keys and transformations to convert pixels to real coordinates makes the project applicable to any keybaord without manual input.
