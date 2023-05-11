---
title: Guard Robot
author: Liulu Yue, Rongzi Xie, and Karen Mai
date: may-2023
status: new
type: report
---
<img width="189129387" alt="Screen Shot 2023-05-05 at 2 04 40 AM" src="https://user-images.githubusercontent.com/89604161/236387225-d58a367c-4a97-4f7c-b819-a044ff157efd.png">

<h1 align=”center”> 🤖🤖 Guard Robot (by Liulu Yue, Rongzi Xie, and Karen Mai) 🤖🤖🤖 </h1> 

<img width="301923" alt="Screen Shot 2023-05-05 at 9 08 15 AM" src="https://user-images.githubusercontent.com/89604161/236465896-5bd5f586-b3f5-465c-9f20-6f35d838da74.png">

<p>
For our team's COSI119a Autonomous Robotics Term Project, we wanted to tackle a challenge that was not too easy or not hard. All of our members have only recently learned ROS, so to create a fully demo-able project in 8 weeks sounded impossible. We started off having a few ideas: walking robot that follows an owner, robot that plays tag, and a pet robot that accompanies one. There were a lot of blockers for those ideas from limitation of what we know, tech equipments, time constraints, and mentor's concern that it will not be able to. We had to address issues like: 

In the end, we decided to work on a robot that guards an object and stop an intruder from invading a specified region. We pivoted from having one robot to having multiple robots so that they are boxing out the object that we are protecting. We tested a lot of design decisions where we wanted to see which method would give us the desired result with the most simple, least error prone, and high perfomring result.  

There were a lot of learnings throughout this project, and while even during the last days we were working to deploy code and continue to test as there is so much that we want to add.  
</p>


Video demo: https://drive.google.com/drive/folders/1FiVtw_fQFKpZq-bJSLRGotYXPQ-ROGjI?usp=sharing

<img width="1234546576" alt="Screen Shot 2023-05-05 at 9 10 19 AM" src="https://user-images.githubusercontent.com/89604161/236466349-af0ddfdf-cf64-44f7-a5fe-d863c0cf6812.png">

<p>
As we will be approaching this project with an agile scrum methodology, we decided to take the time in developing our user stories, so that at the end of the week during our sprint review we know if we are keeping us on track with our project. After taking a review of the assignment due dates for other assignments of the class, we thought that having our own group based deadlines on Friday allows us to make these assignments achievable on time. Below is a brief outline of our weekly goals.
</p>

Here is a ⏰ timeline ⏳ , we thought were were going to be able to follow:
- March 3: Finish the Movie Script That Explains Story of Waht we want to do and Proposal Submission
- March 10: Figure out if tf can be done in gazebo
- March 17: Run multiple real robots
- March 24: Detect the object
- March 31: Aligning to be protecting it - circle or square like formation
- April 7: Detecting if something is coming at it
- April 14: Making the robot move to protect at the direction that the person is coming at it from 
- April 21: Testing
- April 28: Testing
- May 3: Stop writing code and work on creating a poster
- May 10 Continuing to prepare for the presentation

Here is a timeline of what we actually did: 
- March 3: Finished all Drafting and Proposal - Submitted to get Feedback and Confirmation That it is Good To Go (Karen, LiuLu, Rongzi)
- March 10: Figure out if tf can be done in gazebo (Rongzi), Creating Github Repo (Karen), Drawing Diagrams (Liulu), Explore Gazebo Worlds - Get Assests and Understand Structure (Karen)
- March 17: Run multiple real robots on Gazebo (Karen), Created multi robot gazebo launch files (Karen), Wrote Code on Patroling Line (Liulu), Create box world (Rongzi), Make Behavior Diagram (Karen)
- March 24: Continue to write code on patroling the line for multirobot (Liulu), Explored fiducials to integrate (Karen), Made better Gazebo world (Rongzi)
- March 31: Aligning to be protecting it - circle or square like formation
- April 7: Detecting if something is coming at it
- April 14: Making the robot move to protect at the direction that the person is coming at it from 
- April 21: Testing
- April 28: Testing
- May 3: Stop writing code and work on creating a poster
- May 10 Continuing to prepare for the presentation


Before            |  After
:-------------------------:|:-------------------------:
<img width="866" alt="Screen Shot 2023-05-05 at 2 09 55 AM" src="https://user-images.githubusercontent.com/89604161/236387884-c02bf777-ca33-4b76-8e44-d82aee0286e1.png"> | <img width="876" alt="Screen Shot 2023-05-05 at 2 11 27 AM" src="https://user-images.githubusercontent.com/89604161/236388088-48d3e45f-4730-4905-a425-b9b0c724456b.png">

Here is another breakdown of the timeline that we actually followed: 
![7131683301088_ pic](https://user-images.githubusercontent.com/89604161/236645060-07185b9e-8c21-450b-be6f-c79ff4f87719.jpg)



<img width="1245" alt="Screen Shot 2023-05-05 at 9 11 38 AM" src="https://user-images.githubusercontent.com/89604161/236466638-6ff7e061-705e-4a79-927e-84642f39c9ef.png">


<h3 align="left-center ">
Preliminary ❓ Questions ❓ Answered 
</h3> 

<p>
To address blockers, we had to start asking ourselves questions and addressing issues like: 
</p>

  1. What are the major risky parts of your idea so you can test those out first?
  2. What robot(s) will you want to use?
  3. What props (blocks, walls, lights) will it need? 
  4. What extra capabilities, sensors, will your robot need? How you have verified that this is possibler?
  5. And many other questions.


Here was the original plan of what we wanted to display: 
Seeing object they want to protect             |  Going towards the object and boxing it out
:-------------------------:|:-------------------------:
<img width="869" alt="Screen Shot 2023-05-05 at 2 07 59 AM" src="https://user-images.githubusercontent.com/89604161/236387641-64ae5517-e94b-458e-96f5-45cf3b1a56e1.png">  |  <img width="871" alt="Screen Shot 2023-05-05 at 2 08 23 AM" src="https://user-images.githubusercontent.com/89604161/236387687-3db63abd-7a1f-4b99-bf9b-e0eb7919179b.png">

Blocking the intruder when one robot detects it           |  Notifies robot to go over to intruder
:-------------------------:|:-------------------------:
<img width="867" alt="Screen Shot 2023-05-05 at 2 08 38 AM" src="https://user-images.githubusercontent.com/89604161/236387702-5f45fd65-3292-486a-a432-63d087863e19.png"> | <img width="860" alt="Screen Shot 2023-05-05 at 2 08 55 AM" src="https://user-images.githubusercontent.com/89604161/236387738-02f57a87-d7fc-4e17-86da-de20153979b9.png">

Though things do not often go to plan as we did not realize that step 2 to step 3 was going to be so much harder as there was all these edge cases and blockers that we discovered (concurrency, network latency, camera specs). We ended up really only focusing on the patrolling and block stages. 


<img width="14325490" alt="Screen Shot 2023-05-05 at 9 13 14 AM" src="https://user-images.githubusercontent.com/89604161/236467016-b3e1da86-f765-4b75-abd1-05ae222b173a.png">

There are many states to our project. There is even states withhin states that need to be broken down. Whenever we were off to coding, we had to make sure that we were going back to this table to check off all the marks about control of logic. We were more interested in getting all the states working, and so we did not use any of the packages of state management. Another reason why we did not proceed with using those state pacakges was because we needed to have multiple files as one python file represented one node so we were not able to be running multiple robot through one file. 

Here are the states that we were planning to have. 
<img width="23435" alt="Screen Shot 2023-05-06 at 4 38 57 PM" src="https://user-images.githubusercontent.com/89604161/236645734-4a008c68-3fe7-4316-8e93-ed145562813f.png">

Finding State: Trigger:
Lidar detection/Fiducial Detection/Opencv: Recognizing the target object, consider sticking a fiducial to the target object, and that will help the robot find the target object. The state is entered when Lidar finds nothing but the target object with a fiducial.

Patrolling State: Trigger: 
Lidar detection: When there is nothing detected in the right and left side of the robot, the robot will keep patrolling and receive callbacks from the subscriber.

Signal State: Trigger: 
Lidar detection: Finding intruders, which means the lidar detect something around and will recognize it as the intruder
New topic: If_intruder: Contain an array that formed by four boolean values corresponds to each robot’s publisher of this topic or the position of another robot. The robot will publish to a specific index of this array with the boolean value of the result of their lidar detection. If there is nothing, the robot will send False, and if it can get into the Signal State, it will send it’s position and wait for the other robot’s reaction. 

Form Line State: 
Trigger: The message from topic  if_intruder:
When a robot receives a position message, it will start to go in that direction by recognizing the index of the position message in the array, and they’ll go to that corresponding colored line.
Behavior Tree: A behavior will be created that can help the robot realign in a colored line.

Blocking State: 
Trigger: tf transformation:All the robots have the information that there is an intruder.
All the robot will go to the direction where the intruder is and realign and try to block the intruder

Notify State: 
Trigger: Lidar Detection: If the lidar keeps detecting the intruder and the intruder doesn’t want to leave for a long time, the robot will keep blocking the intruder without leaving.
If the intruder leaves at some time, the robot will get back to the blocking state and use lidar to detect and make sure there is no intruder here and turn back to the finding state

In the end, these were the only states that we actually tackled:

<img width="324567" alt="Screen Shot 2023-05-06 at 4 39 34 PM" src="https://user-images.githubusercontent.com/89604161/236645754-55c52bb0-bd7e-4759-940f-7db62529cef5.png">

<img width="891283" alt="Screen Shot 2023-05-05 at 9 14 15 AM" src="https://user-images.githubusercontent.com/89604161/236467210-25347e43-1d0e-406f-afaa-81e3bd5d835e.png">


Get correct color for line following in the lab
Line follower may work well and easy to be done in gazebo because the color is preset and you don't need to consider real life effect. However, if you ever try this in the lab, you'll find that many factors will influence the result of your color.

Real Life Influencer:
1. Light: the color of the tage can reflect in different angles and rate at different time of a day depend on the weather condition at that day. 
2. Shadow: The shadow on the tape can cause error of color recognization
3. type of the tape: The paper tage is very unstable for line following, the color of such tape will not be recognized correctly. The duct tape can solve most problem since the color that be recognized by the camera will not be influenced much by the light and weather that day. Also it is tougher and easy to clean compare to other tape.
4. color in other object: In the real life, there are not only the lines you put on the floor but also other objects. Sometimes robots will love the color on the floor since it is kind of a bright white color and is easy to be included in the range. The size of range of color is a trade off. If the range is too small, then the color recognization will not be that stable, but if it is too big, robot will recognize other color too.
if you are running multiple robots, it might be a good idea to use electric tape to cover the red wire in the battery and robot to avoid recognizing robot as red line.

OpenCV and HSV color:
Opencv use hsv to recognize color, but it use different scale than normal.
Here is a comparison of scale:
normal use  H: 0-360, S: 0-100, V: 0-100
Opencv use  H: 0-179, S: 0-255, V: 0-255

So if we use color pick we find online, we may need to rescale it to opencv's scale.

Here is a chart that talks about how we run the real robots live with those commands. On the right most column that is where we have all of the colors for each line that the robot home base should be: 

<img width="2345" alt="Screen Shot 2023-05-06 at 4 50 52 PM" src="https://user-images.githubusercontent.com/89604161/236646131-69a00729-192c-40c0-8c67-39302d8ea4a7.png">

<img width="234546" alt="Screen Shot 2023-05-05 at 9 14 42 AM" src="https://user-images.githubusercontent.com/89604161/236467313-4e6ecde6-ea64-45af-92e0-b5d12fba5a1d.png">

If you are interested in launching on the real turtlebot3, you are going to have to ssh into it and then once you have that ssh then you will be able to all bringup on it. There is more detail about this in other FAQs that can be searched up. When you are running multirobots, be aware that it can be a quite bit slow because of concurrency issues. 

These 3 files are needed to run multiple robots on Gazebo. In the object.launch that is what you will be running roslaunch. Within the robots you need to spawn multiple one_robot and give the position and naming of it. 

<img width="190" alt="Screen Shot 2023-05-06 at 4 55 10 PM" src="https://user-images.githubusercontent.com/89604161/236646236-c7dd85ab-3c2b-4901-b6a6-58fdc1937613.png">

Within the object.launch of line 5, it spawns an empty world. Then when you have launched it you want to throw in the guard_world which is the one with the multiple different colors and an object to project in the middle. Then you want to include the file of robots.launch because that is going to be spawning the robots. 

<img width="575" alt="Screen Shot 2023-05-06 at 4 55 45 PM" src="https://user-images.githubusercontent.com/89604161/236646252-7c480a02-de15-4329-925f-4496ce140233.png">


For each robot, tell it to spawn. We need to say that it takes in a robot name and the init_pose. And then we would specify what node that it uses.

<img width="628" alt="Screen Shot 2023-05-06 at 4 56 08 PM" src="https://user-images.githubusercontent.com/89604161/236646266-d71ddf41-e15e-4b14-94af-cab2416c06e1.png">


Within the robots.launch, we are going to have it spawn with specified position and name. 
<img width="576" alt="Screen Shot 2023-05-06 at 4 56 43 PM" src="https://user-images.githubusercontent.com/89604161/236646283-87e8b496-dd4c-4199-93e5-c729a0fe95ff.png">



<img width="2465735" alt="Screen Shot 2023-05-05 at 9 15 16 AM" src="https://user-images.githubusercontent.com/89604161/236467419-9112d8db-08a1-473c-903f-05a96d491371.png">


We had it run for more than one minute and it started sensing each other as the object and trying to stay away from it, so it was not exactly ideal. It would start going further and further say from each other. The perimeter started getting bigger and better which is a problem as we do not know when the intruder will come. |  Pros of this approach is that it will not hit the object. It is good enough in which it stays within the perimeter. It may become a problem if multiple run this because we would need to find a way to edit the auro detect so that all four runs that. 
:-------------------------:|:-------------------------:
<img width="12313" alt="Screen Shot 2023-05-06 at 5 06 33 PM" src="https://user-images.githubusercontent.com/89604161/236646577-a2169b96-8778-4191-954a-80eeb6f3e5f8.png"> | <img width="123123" alt="Screen Shot 2023-05-05 at 2 00 16 AM" src="https://user-images.githubusercontent.com/89604161/236386744-9d3376ba-ccf9-47d9-aa38-146439e3abc5.png">

Line follower is the best algorithm we found to deal with patrolling around the object after tried out different strategies and compare the performance. It allows stable protection of the object: the robot strictly follows the designed path indicated by line and would not run away from the object.   |  The simulation of patrolling environment in gazebo should work better than real world environment since the color can be set to pure green, yellow, red and blue and there is no shadow or reflection that can cause error on color recognition 
:-------------------------:|:-------------------------:
<img width="123123" alt="Screen Shot 2023-05-05 at 2 01 36 AM" src="https://user-images.githubusercontent.com/89604161/236386897-4cd7db0a-76ce-4aaf-807e-4d2b731fed24.png"> | <img width="123123" alt="Screen Shot 2023-05-05 at 2 01 08 AM" src="https://user-images.githubusercontent.com/89604161/236386843-1e1213ea-7d07-4572-9e42-21d70f66b5e5.png">



<img width="235465" alt="Screen Shot 2023-05-05 at 9 15 51 AM" src="https://user-images.githubusercontent.com/89604161/236467533-66784370-ce6c-409d-b3ca-cbd7d5053dd6.png">
Basic Idea:

Guard object has a Twist() which will control the velocity of the robot. It also has a subscriber that subscribe to scan topic to receive LaserScan message and process the message in scan_cb() which is the callback function of the scan topic. The ranges field of the LaserScan message gives us an array with 360 elements each indicating the distance from the robot to obstacle at that specific angle. The callback function takes the ranges field of the LaserScan message, split into seven regions unevenly as shown below：


The minimum data of each region is stored in a dictionary. Then change_state function is called inside the scan_cb, which will check the region dictionary and update the intruder state as needed.


Work with real robot:

- Dealing with signal noise

The laser scan on real robot is not always stable. Noisy data is highly likely to occur which can influence our intruder detection a lot. To avoid the influence of scan noise, we processed the raw ranges data to get a new ranges list which each element is the average of itself and 4 nearby elements in original ranges data. This data smoothing step can help us get a more reliable sensor data.

- Sensor on real robot

The sensor of each robot is build differently. In gazebo, if nothing is scanned at that angle, inf is shown for the corresponding element in the ranges. However, some real-life robot have different LaserScan data. Nothing detected within the scan range will give a 0 in ranges data. To make our code work correctly on both Gazebo and real robot, please follow our comment in the code. There are only two lines of code that need to be changed. 

Also, the laserScan data for Rafael has different number of elements. So the region division based on index is a little different for Rafael. We have different Python file for each robot, so this part is taken care of in line_F4.py which is only for Rafael.

<img width="1245" alt="Screen Shot 2023-05-05 at 9 20 33 AM" src="https://user-images.githubusercontent.com/89604161/236468664-10761f2c-95e1-4046-827b-c442a57fad98.png">
A new message is created for our robots to communicate about intruder detection. The message contains four boolean values each associated with one of our guardbot. When a guardbot detects the intruder, a new message will be created with that associated boolean value set to True, and the new message will be published to a topic called see_intruder. Each robot also has a subscriber that subscribes to this topic and a callback function that will check the passed-in message and get the information about which robot is seeing the intruder.  

The CMakeLists.txt and package.xml are also modified to recognize this newly created message.

We had to watch a lot of videos to figure out how to do this. We made an msg file which stored the type of data that we will hold which are boolean values on if the robot sees an intruder. We had to edit the cmake file and then had to edit the xml because we need to say that there is this new created message that the robots may communicate with and then have these structure look through the folder to see how it is created. 

We need our own messades and tonics for the rohots
to communicate. Here are the new messages:

- see intruder: the message contains four
std msgs/Boolean, each associated with a
specific robot. When an intruder is detected by
one robot. its' associated Boolean will be set to
True. Only one robot can be true.

- stop order: the message contains a list of
std_msgs/String which would record the name of
the robots in the stop order, and an std msas/Int8
which would record
the current index of the list
for the next stop or move.

Here is a chart talking about what we are interested in:
![7111683298265_ pic](https://user-images.githubusercontent.com/89604161/236646495-bc620be7-23f8-46ce-b37d-9dc20f7437bd.jpg)

Here are the links that we used quite a lot:
https://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv 
https://www.theconstructsim.com/solve-error-importerror-no-module-named-xxxx-msg-2/ 

<img width="2463547" alt="Screen Shot 2023-05-05 at 9 16 50 AM" src="https://user-images.githubusercontent.com/89604161/236467763-ea3e7515-7f1e-4810-a378-e259e5067946.png">

This project does not end here, there is a lot more that we can add. For an example here are a couple of other features that cam be added: 
- Having more sides so that there is faster converage
- Making it run faster and still be as accurate with its patrolling 
- Finding a better system so that it does not need the order ahead of time if possible 
- Try to make it so that the robots all patrolling around instead of it being on a single like
- Adding extra technologies where robots are connecting to like an Alexa can tell them to stop 
- Add a way to have it be stopped by human input and have it overrided

<img width="23546" alt="Screen Shot 2023-05-05 at 9 17 26 AM" src="https://user-images.githubusercontent.com/89604161/236467866-d5f71598-5fda-4141-a3c3-5e1d50989107.png">
We feel like as a team we suceeded a lot in this project. We had great communication and determination that makes us such good teammates. We did not know each other before doing this project and at the beginning of the week we were a bit hesitant. We 

👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻
