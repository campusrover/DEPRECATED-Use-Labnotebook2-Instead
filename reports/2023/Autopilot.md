---
title: Autopilot
author: Zihao Liu, Junhao Wang, Long Yi
date: may-2023
status: new
type: report
---

## **Project Report**

### **Team members: **Zihao Liu, Junhao Wang, Long Yi**

### **Github Repo**: [https://github.com/campusrover/autopilot](https://github.com/campusrover/autopilot)

### **Date**: May 2023

## **<span style="text-decoration:underline;">Introduction</span>**

The idea of the autopilot robot was simple – it’s a mobile robot augmented with computer vision, but it’s a fun project to work on for people wishing to learn and combine both robotics and machine learning techniques in one project. Evenmore, one can see our project as miniature of many real world robots or products: robot vacuum (using camera), delivery robot and of course, real autopilot cars; so this is also an educative project that can familiarize one with the techniques used, or at least gives a taste of the problems to be solved, for these robots/products.



### **Problem statement (original objectives)**
* Able to follow a route
* Able to detect traffic signs
* Able to behave accordingly upon traffic signs

### **Relevant literature**

[Thesis Traffic and Road Sign Recognition](ahttps://www.diva-portal.org/smash/get/diva2:523372/fulltext01.pdf)


**<span style="text-decoration:underline;">What was created</span>**



1. **Technical descriptions, illustrations**

There are five types and two groups of traffic signs that our robot can detect and react to. Here are descriptions of each of them and the rules associated with them upon the robot’s detection.

Traffic light (**See Fig1**)

  1. Red light: stops movement and restarts in two seconds.

  2. Orange light: slows down

  3.  Green light: speeds up

Turn signs (**See Fig2**)

  4. Left turn: turn left and merge into the lane

  5. Right turn: turn right and merge into the lane

* All turns are 90 degrees. The robot must go straight and stay in the lane if not signaled to turn.



![Screenshot 2023-05-05 at 7 59 42 PM](https://user-images.githubusercontent.com/59838570/236586656-b4029994-56bf-413c-abac-3ab3e5c1f63d.jpg)

    **Fig 1. Traffic light examples**


  ![Screenshot 2023-05-05 at 7 59 59 PM](https://user-images.githubusercontent.com/59838570/236586678-4429edc3-21a0-4e2c-b61d-67260f675b20.jpg)

    **Fig 2a. Initial turn sign examples**

![Screenshot 2023-05-05 at 8 00 13 PM](https://user-images.githubusercontent.com/59838570/236586703-2e710d8a-2018-4a4a-a8fd-9d353ddab318.jpg)


    **Fig 2b. Revised turn sign examples**



2. **Discussion of interesting algorithms, modules, techniques**

The meat of this project is computer vision. We soon realized that different strategy can be used the two groups of traffic signs as they have different types of differentiable attributes (color for traffic lights and shape for turning signs).Therefore, we used contour detection for traffic lights, and we set up a size and circularity threshold to filter noise out noise. Turning signs classification turned out to be much harder than we initially expected in the sense that the prediction results seem to be sometimes unstable, stochastic, and confusing (could be because of whatever nuance factors that differ in the training data we found online and the testing data (what the robot sees in lab).

We tried SVM, Sift, and CNN, and we even found a web API for traffic signs models that we integrated into our project – none of them quite work well.  

Therefore we decided to pivot to another more straightforward approach: we replaced our original realistic traffic signs (Fig2a) with simple arrows (Fig2b), and then what we are able to do is that:



* Detect the blue contour and crop the image from robot’s camera to a smaller size 
* Extract the edges and vertices of the arrow, find the tip and centroid of it
* Classify the arrow’s direction based on the relative position of the tip to the centroid

Two more things worth mentioning on lane following:

1. Our robot follows a route encompassed by two lines (one green and one red) instead of following one single line. Normally the robot uses contour detection to see the two lines and then computes the midpoint of them. But the problem comes when the robot turns – in this case, the robot can only see one colored line. The technique we use is to only follow that line and add an offset so that the robot turns back the the middle of the route (the direction of the offset is based on the color of that line)
2. How we achieve turning when line following is constantly looking for a midpoint of a route. What we did is that we published three topics that are named “/detect/lane/right_midpoint”, “/detect/lane/midpoint”, “/detect/lane/left_midpoint”. In this case there are multiple routes (upon an intersection), the robot will choose to turn based on one of midpoint position, based on the presence turning sign. By default, the robot will move according to the “/detect/lane/midpoint” topic.
3. **Guide on how to use the code written**

    Run “roslaunch autopilot autopilot.launch”, which starts the four nodes used in our project.

4. **Clear description and tables of source files, nodes, messages, actions**

<table>
  <tr>
   <td>
Node
   </td>
   <td>Messages
   </td>
   <td>Self defined?
   </td>
   <td>Usage
   </td>
  </tr>
  <tr>
   <td>Cmd_vel
   </td>
   <td>Twist
   </td>
   <td>NO
   </td>
   <td>Publish movement
   </td>
  </tr>
  <tr>
   <td>/raspicam_node/image/compressed
   </td>
   <td>CompressedImage
   </td>
   <td>NO
   </td>
   <td>Receive camera data
   </td>
  </tr>
  <tr>
   <td>/detect/lane/midpoint
   </td>
   <td>MidpointMsg
<p>
[int32 x
<p>
int32 y
<p>
int32 mul]
   </td>
   <td>YES
   </td>
   <td>Midpoint of a lane
   </td>
  </tr>
  <tr>
   <td>/detect/lane/right_midpoint
   </td>
   <td>MidpointMsg
   </td>
   <td>YES
   </td>
   <td>Midpoint of a left lane (when there’s a branch or cross)
   </td>
  </tr>
  <tr>
   <td>/detect/lane/left_midpoint
   </td>
   <td>MidpointMsg
   </td>
   <td>YES
   </td>
   <td>Midpoint of a right lane (when there’s a branch or cross)
   </td>
  </tr>
  <tr>
   <td>/detect/sign
   </td>
   <td>SignMsg
<p>
[string direction
<p>
float32 width
<p>
float32 height]
   </td>
   <td>YES
   </td>
   <td>Detection of the 
   </td>
  </tr>
  <tr>
   <td>/detect/light
   </td>
   <td>LightMsg
<p>
[string color
<p>
float32 size]
   </td>
   <td>YES
   </td>
   <td>Detection of a traffic light
   </td>
  </tr>
</table>


**Table 1. Nodes and Messages description**

**<span style="text-decoration:underline;">Story of the project</span>**



1. **How it unfolded, how the team worked together**

At the very beginning of our project, we set these three milestones for our work:


    **Milestone-1:** Be able to detect and recognize the signs in digital formats (i.e. jpg or png) downloaded online. No ROS work needs to be done at this stage.


    **Milestone-2:** Complete all the fundamental functionalities, and be able to achieve what’s described in a simulation environment.


    **Milestone-3:** Be able to carry out the tasks on a real robot in a clean environment that’s free of noises from other objects. We will use blocks to build the routes so that laser data can be used to make turns. We will print out images of traffic signs on A4 white paper and stick them onto the blocks.

And we have been pretty much following this plan, but as one can tell, the workload for each phase is not evenly distributed. 

Milestone 1 is something we’ve already done in assignment, but here we use two lines to define a route instead of just one single line, and the complexity is explained above. Traffic light classification was pretty straightforward in the hindsight. We simply used color masking and contour detection. To avoid confusing the robot between red green lines that define the route, and the red green traffic signs, we set up a circularity and area threshold. We spent the most time on milestone 3. We experimented with various methods and kept failing until we decided to simply our traffic signs – more details are explained in section 8.  Our team worked together smoothly. We communicated effectively and frequently. As there are three of us, we divided the work into lane detection and following, traffic light classification, and sign classification and each of us works on a part to maximize productivity. After each functionality is initially built, we worked closely to integrate them and debug together.



2. **problems that were solved, pivots that had to be taken**

The first problem or headache we encountered is that we use a lot of colors for object detection in this project, so we had to spend a great amount of time tuning HSV values of our color-of-interest, as slight changes in background lighting/color mess up with it.

The more challenging part is that, as alluded to above, turning signs classification is more difficult than we initially expected. We tried various model architectures, including CNN, SVM, SIFT, and linear layers. No matter what model we choose, the difficulty comes from the inherent instability of machine learning models. Just to illustrate it: we found a trained model online that was trained on 10000 traffic signs images, for 300 epochs, and reached a very low loss. It is able to classify various traffic signs – stop sign, rotary, speed limit, you name it. But sometimes, it just failed on left turn and right turn classification (when the image is clear and background is removed), and we didn’t figure out a great solution on remediating this. So instead we piloted away from using machine learning models and replaced curved traffic signs (Fig 2a) with horizontal arrows (Fig 2b). The detail of the algorithm is discussed in section 4.



3. **Your own assessment**

We think that this has been a fun and fruitful experience working on this project. From this semester’s work, we’ve all had a better appreciation of the various quirky problems in robotics – we had to basically re-pick all the parameters when we migrated from gazebo world the the real lab, and also we literally had to re-fine-tune the hsv values everytime we switched to a new robot because of the minute difference in background lighting and each robot’s camera. So if one thing we learned from this class, it’s real Robots Don't Drive Straight. But more fortunately, we’ve learned a lot more than this single lesson, and we’re able to use our problem solving and engineering skills to find work-arounds for all those quirky problems we encountered. Therefore, in summary we learned something from every failure and frustration, and after having been through all of these we’re very proud of what we achieved this semester. 
