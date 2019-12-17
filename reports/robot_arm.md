Final Project Arm Interfacing
===============

Jacob Smith COSI 119a  Fall 2019, Brandeis University

Github Repository: [Arm Interface Project](https://github.com/jsmith2021Brandeis/Robot-Arm-Interface)



1. **Introduction**

   1. **Problem statement, including original objectives**

      "The goal of this project will be to implement a robotics arm to the campus rover. This entails setting up the required low level hardware in arduino
      including rotation sensors. Then, We need to implement respond to ROS nodes such as
      twist messages (see robotics Arm interfacing sources). Two extension projects could be
      to program the arm to push an elevator button or pick up a package (see robot
      navigating elevator and robot picking up object).
      This is an interesting project because it will contribute to the existing campus
      rover project, especially by creating a framework to easily add components to it. In
      addition, it will allow us to test the concept of interfacing by seeing if a user not familiar
      with the arm can control it with ROS. Finally, this project will allow us to learn the
      low-level details of ROS. If successful, this project will establish a framework for other
      contributors to add components to the campus rover using ROS best practices.
      Background
      	I have researched existing tutorials and implementations of interfacing arduino
      components to ROS. Getting a motor to move with a ros command seems to be fairly
      well documented, but the challenge seems to be properly implementing abstracted
      interfaces. In other words, I can tell the servo to move, but I don’t know how to tell it to
      respond to specified velocity messages.	

      ​	The project should be assessed on: how elegant the ROS- robot arm interface is,
      how well a student can replicate the interfacing given our documentation, and whether
      the arm can be used in the campus rover project. [3]

   1. **Relevant literature**
      1. Sources included in github readmes, I treid to read 
      2. I researched academic approaches to robotics arm evaluation, and I found one
         source on arm path planning and following functionality benchmarks.1
         It specifies
         requirements for robot manipulators such as workspace, dexterity, robot motion,
         accuracy, and payload[1]. These metrics would be a useful
         framework to evaluate the current arm.
         Metrics for user Experience
         I found a book about user experience evaluation, and I can use an Issue based
         metrics to evaluate the quality of my software interfacing. One goal of the arm
         interfacing project is to provide a demonstration where the arm is easy to use even by
         someone who doesn’t understand it, and observing user confusion would be a way to
         measure the success of that [2].

2. **What was created (****biggest section****)**

   1. **Technical descriptions, illustrations**

   Technical Information on Arm
   Each servo can rotate 180 degrees, has feedback control, and a 3 wire interface. They
   cannot return their angle to the computer.
   The arm can rotate in Y axis, the elbow and shoulder move in X axis, and the base
   handles rotation in the axis. There is a previously defined reverse kinematics scheme
   for the arm, which calculates wrist angle with respect to ground, shoulder to wrist
   distance, and shoulder angle.

   

   ![Consensed form of Arm as Mounted](images\Robot Arm\ArmCondensed.jpg)

   | Goal                | ROS Command        | Arduino Command                       | Inputs other than Name | Outputs  | Comment                 |
   | ------------------- | ------------------ | ------------------------------------- | ---------------------- | -------- | ----------------------- |
   | Get Distance        | N/A                | Automatic                             |                        | Distance | Type Output Sensor      |
   | Set Arm Coordinates | ARM x y base power | set_arm(100, 200, 90, 0  servoSpeed); | 4 coordinates, speed   |          | Type 4 Coordinates      |
   | Set Manipulator     | MANIP setting      | manipulator(true =open/false =code);  | open or close          |          | Type Binary Manipulator |

   ![Running Through ROS Node](images\Robot Arm\ROSandBash.gif)

   Generic Electronics Interface

   ​	I wrote ArduinoV6, which uses a serial interface to control Arm position, manipulator, and and return the distance. 

   Talk about subscribing script, Arduino script,  

   1. Discussion of interesting algorithms, modules, techniques**

3. **Story of the project.**

   1. **How it unfolded, how the team worked together**

   2. #### Getting Arm to Move

      #### Getting Arduino to talk to Rasberry Pi

      #### **Getting Rasberry Pi to Talk to Arduino**

      #### **Workflow improvements**			

      ```
      	Mounting Servo arm, bash scripts
      ```

      #### **Two Way Communication** with one letter

      ### Nov 1-7: Control multiple parts of arm

      ### Nov 15-22

      (Feedback from testing arm with TA Took two minutes, 8 out of ten usability.

      

      Add Ultrasonic Sensor

      note on Charly and Luis, Chris

   3. **problems that were solved, pivots that had to be taken**

   4. ![Remounting of arm on Robot](images\Robot Arm\ArmMount.gif)

   5. ![Plot showing that servo cannot tell if the motor is still moving](images\Robot Arm\MotionChart.JPG)

      Knowing if servo is moving or can stop Trying to get Servo to tell my if its moving. I set the servo to run for 2 seconds at degree increments, and print whether it was moving. There would  be a spike at 40 and 160 if this method actually could tell if the servo was moving. Plotting the number of times the servo moved by different degrees shows that there is no correlation between where the servo is jammed and what the isMoving function returns. This agrees with what Charly said about the servos having no feedback, but I thought a function named isMoving would return whether the servo is moving. In summary, the servo functions I thought would work don't really.

   6. **Your own assessment**


Behavior Trees Integration

Facial Arm Integration

 I made sure necessary files from my project are imported, and catkin make passes.I removed subscriber node, mafe publisher node generic, making director of mutantStartup generic.  I also worked with Luis to make the arm_commander python node which reads the current face ros topic and publishes an arm command. The challenge is converting a data stream of current face into a set of commands for the arm that give the arm enough time to move. The current idea is to only set a new command every 4 seconds, which is clunky.  Future work: Show that Luis and my program work together and make a demo video.

 Luis is writing a node to clean up face recognition, then we will write a timer to request an arm motion every 4 seconds. I'm going to make sure the current repo can move the arm. 14:28: I'm getting usb cutting out after arm receives a few commands.

wait for robot to turn on in runRos, making initializeserial a method in topic subscriber (to allow method to be called again if usb isn't plugged in, lead to scope errors), made initiSerial wrapper method for modularity, reordered topic subscription and publication so a callback won't happen until serial is ready.

 Luis wrote the filtering nmode and tells the hand to open if it sees a face. Now we are trying to connect the two projects by having the hand move when a face is seen. I get the arm started up, then Luis starts up the camera and publishes the arm command topic

I got the arm moving, the usb port was wrong. I am talking to TA about a better USb port selection system than guessing

Luis and I got the arm waving when a face is seen, see our video below



**Appendix A: Links and Sources**

Sources

[1] Vemula, Bhanoday. Evaluation of Robot Structures. 2015
https://www.diva-portal.org/smash/get/diva2:859697/FULLTEXT02.pdf Accessed
November 19 2019
[2] Tullis Thomas and Albert, William Chapter 5 of Measuring the User Experience, 2nd
Edition 2013

[3] My original Project Proposal

[4] My Revised Project proposal

Links

(Please see github repository README files for sources)

[Arm Interface Project](https://github.com/jsmith2021Brandeis/Robot-Arm-Interface)

[Facial Recognition and and Arm Interfacing](https://github.com/jsmith2021Brandeis/FacialArm)

[Behavior Trees and Arm Interfacing](https://github.com/jsmith2021Brandeis/robot_behavior_trees)

Videos

**Main Project Video**

[![I show the last sprint from the arm interfacing project](http://img.youtube.com/vi/xTT16x3kTqU/0.jpg)](http://www.youtube.com/watch?v=xTT16x3kTqU "Arm Interfacing Demonstration")

**Behavior Tree Integration Video**

[![Chris and I demonstrate how behavior trees can be connected to the arm](http://img.youtube.com/vi/ZJRn2t_dehc/0.jpg)](http://www.youtube.com/watch?v=ZJRn2t_dehc "Behavior Tree Arm Interfacing")

**Facial Recognition Video**

 [![We explain our project and show that the arm moves when the robot sees a person's face](http://img.youtube.com/vi/wVTJThKsIWs/0.jpg) ](http://www.youtube.com/watch?v=wVTJThKsIWs "Arm Interface with Facial Recognition")





