# Robot Waiter

#### Team: Ben Soli (bsoli@brandeis.edu) and Harry Zhuh (zhuh@brandeis.edu)

#### Date: 5/5/2022

#### Github repo: https://github.com/campusrover/waiter_bot

##### Introduction </br>
For this project, we wanted to create an automated catering experience. More and more, customer service jobs are being performed by chatbots and voicebots, so why not bring this level of automation to a more physical setting. We figured the perfect domain for this would be social events. Waiter bot serves as a voice activated full-service catering experience. The goal was to create a general purpose waiter robot, where all the robot needs is a map of a catering location, a menu, and coordinates of where to pick-up food and or drinks. 
###### Original Objectives: </br>
<li> Capable of speech to text transcription and intent understanding. </li>
<li> Has conversational AI. </li>
<li> Can switch between 3 states – Wander, Take_Order, and Execute – to achieve waiter-like behavior. </li>
<li> Able to localize itself and navigate to given locations on a map. <li>
Relevant literature: </br>
[The Alexa Skills Kit SDK for Python] (https://github.com/alexa/alexa-skills-kit-sdk-for-python.git) </br>
[actionlib SimpleActionClient class](http://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a4bce20a02332f8f96a5f087c87fae851) </br>

###### Core Functionality: 
This application was implemented as two primary nodes. One node serves as the user-interface, which is a flask implementation of an Alexa skill. This node is integrated with a ngrok subdomain. This node processes the speech and sends a voice response to Alexa while publishing necessary information for navigation. We created a custom message type called Order, which consists of three std_msg/String messages. This stores the food for the order, the drink for the order, and the name of the  On the backend, a navigation node processes these orders, and looks up the coordinates for where the food and drink items are located. We created a map of the space using SLAM with AMCL. The robot uses this map along with move-base to navigate to its goal and retrieve this item. While Alexa works great for taking orders, one of its flaws is that it cannot be programmed to speak unprompted. When the robot arrives to retrieve the item, the navigation node supplies the alexa-flask node with the item the robot needs. A worker can ask Alexa what the robot needs, and Alexa tells the worker what is needed. The user then tells Alexa that the robot has the item, and it proceeds to its next stop. Once the robot has collected all of the items, it returns to the location where it originally received the voice request. The navigation node then supplies the alexa-flask node with a message for the order it is delivering. The user can then ask Alexa if that is their order, and Alexa will read back the order to the user. After this, the order is deleted from the node, allowing the robot to take more orders. 

Due to the physical size constraints of the robot, it can only carry at most three orders at a time. The benefit of using the flask-ask module, is that the Alexa voice response can take this into account. If the robot has too many orders, it can tell the user to come back later. If the robot is currently in motion, and the user begins to make a user, a String message is sent from the alexa-flask node to the navigation node telling the robot to stop. Once Alexa has finished processing the order, the alexa-node sends another String message to the navigation node telling the robot to continue moving. If the robot has no orders to fulfill, it will roam around, making itself available to users. 

In terms of overall behavior, the WaiterBot alternates between wandering around the environment, taking orders and executing the orders. After initialization, it starts wandering around its environment while its order_log remains empty or while it’s not stopped by users. It randomly chooses from a list of coordinates from wander_location_list.py, turns it into a ```MoveBaseGoal ``` message using the ```goal_pose()``` function, and has actionlib’s ```SimpleActionClient``` send it using ```client.send_goal()```. And instead of calling ```client.wait_for_result()``` to wait till the navigation finishes, we enter a while loop with ```client.get_result() == None``` as its condition. 
This has 2 benefits – it makes the navigation interruptible by new user order and allows WaiterBot to end its navigation once it’s within a certain tuneable range of its destination. We noticed during our experimentation that while having ```SimpleActionClient``` send ```MoveBaseGoal``` works really well in gazebo simulation, it always results in WaiterBot hyper-correcting itself continuously when it arrives at its destination. This resulted in long wait times which greatly hampers the efficiency of WaiterBot’s service. Therefore, we set it up so that the robot cancels its ```MoveBaseGoal``` using ```client.cancel_goal()``` once it is within a certain radius of its goal coordinates. 
When a user uses the wake phrase “Alexa, tell WaiterBot (to do something)” in the proximity of the WaiterBot, the robot will interrupt its wandering and record an order. Once the order has been published as an ```Order``` message to the ```/orders``` topic, the robot will initiate an ```order_cb``` function, store the order on an order_log, and start processing the first order. Before it looks up the coordinates of the items, it will store an Odometry message from /odom topic for later use. It then looks at ```Order``` message’s data field food and look up its coordinates in ```loc_string.py```. After turning it into a ```MoveBaseGoal```, it uses ```client``` to send the goal. Once the Waiterbot arrives at its destination, it will broadcast a message, asking for the specific item it is there to collect. It will not move on to its next destination till it has received a String message from the ```order_picked_up``` topic. 
After it has collected all items, it uses the Odometry it stored at the beginning of the order and set and send another ```MoveBaseGoal```. Once it returns to its original location, to the user. If ```order_log``` is empty, it will being wandering again.

###### How to Use the Code:
```git clone  https://github.com/campusrover/waiter_bot.git``` </br>
 ```cm``` </br>
Download ngrok and authorize ngrok token </br>
```pip3 install flask-ask``` and ```sudo apt-install ros-noetic-navigation``` </br>
Have waiterbot skill on an Alexa device </br>
ssh into your robot </br>
In waiter_bot/src bash ngrok_launch.sh </br>
```roslaunch waiter_bot waiter_bot.launch``` </br>

###### Tables and Descriptions of nodes, files, and external packages:
| ROS Nodes   | Descriptions   |
|---|---|
|waiter_bot_navigation.py   | Master node. Has 3 functions Wander, Take_Order and Execute, which serve as states|
|waiter_bot_alexa_hook.py | Receives speech commands from the user, and sends them to the navigation node |

| Files   | Description |                                                             
|---|---|             
| bm.pgm | An image file of the lab basement created using SLAM of the gmapping package |
|bm.yaml | Contains map information such as origin, threshold, etc. related to bm.pgm |
|menu_constants.py | contains the food menu and the drink menu |                       
|loc_dictionary.py| Contains a dictionary mapping food and drink items to their coordinates|
|wander_location_list.py|Contains a list of coordinates for the robot’s wandering behavior|
|Order.msg| A custom message containing three String messages: food, drink and name|


|External Packages| Descriptions |
|---|---|
|Flask-ASK| A Python module simplfying the process of developing the backend for Alexa skills using a Flask endpoint|
|Ngrok | Creates a custom web address linked to the computer's local IP address for https transfers|




###### Story of the Project: </br>
Our original project intended to have at least two robots, one working as a robot waiter, the other working as a robot kitchen assistant. Initially, we planned to create our NLP system to handle speech and natural language understanding. This quickly became a problem, as we could only find a single dataset that contained food orders and its quality was questionable. At the same time, we attempted to use the Google Speech API to handle out speech-to-text and text-to-speech. We found a ROS package for this integration, but some dependencies were out of data and incompatible. Along with this, the firewalls the lab uses, made integration with this unlikely. We then attempted to use PocketSphinx which is an off-the-shelf automated speech recognition package which can run offline. Initial tests of this showed an incredibly high word-error-rate and sentence-error-rate, so we abandoned this quickly. Previous projects had success integrating Alexa with ROS nodes, so we decided to go in that direction. 

Building off of previous work using Flask and ngrok for Alexa integration was fairly simple and straightforward thanks to previous documentation in the lab notebook. Building a custom Alexa skill for our specific purpose was accomplished quickly using the Alexa Developer Console. However, previous methods did not allow a feature that we needed for our project, so we had to find a new approach to working with Alexa. At first, we tried to change the endpoint for the skill from an ngrok endpoint to a lambda.py provided by Amazon, but this was difficult to integrate with an ROS node. We returned to an approach that the previous projects had used, but this did not allow us to build Alexa responses that involved considering the robots state and asking the user for more information in a way that makes sense. For example, due to the physical size of the robot, it can probably never handle more than three orders. An order consists of a food, a drink, and name to uniquely identify the order. From the Alexa Developer Console, we could prompt the user for this information, then tell the user if the robot already had too many orders to take another one, but this felt like bad user-interface design. We needed a way to let the user know that the robot was too busy before Alexa went through the trouble of getting all this information. Then we found a module called Flask-ASK, which seamlessly integrated flask into the backend of an Alexa Skill. This module is perfect for Alexa ROS integration. On the Alexa Developer Console, all you need to do is define your intents and what phrases activate them. Then within your ROS, you can treat those intents like functions. Within those functions you can define voice responses based on the robot’s current state and also publish messages to other nodes, making the voice integration robust and seamless. 

Navigation-wise, we experimented with different ways of utilizing ```SimpleActionClient``` class of the ```actionlib``` package. As we have mentioned above, we originally used ```client.wait_for_result()``` so that WaiterBot can properly finish navigating. ```wait_for_result()```, however, is a black box by itself and prevented us from giving WaiterBot a more nuanced behavior, i.e. capable of interrupting its own navigation. So we looked into the documentation of ```actionlib``` and learnt about the ```get_result()``` function, which allowed us to tear away the abstraction of ```wait_for_result()``` and achieve what we want. 

It was surprising (though it should not be) that having a good map also proved to be a challenge. The first problem is creating a usable map using ```SLAM``` method of the ```Gmapping``` package only, which is not possible with any sizeable area. We searched for software to edit the pgm file directly and settled on GIMP. Having a easy-to-use graphical editor proved to be highly valuable since we are free to tweak any imperfections of a raw SLAM map and modify the image as the basement layout changes. 

Another challenge we had concerned the sizes of the maps and how that affect localization and navigation. At one point, we ended up making a map of half the basement since we wanted to incorporate WaiterBot into the demo day itself: WaiterBot would move freely among the guests as people chat and look at other robots and take orders if requested. The large map turned out to be a problem, however, since the majority of the space on the map is featureless. This affects two things: the amcl algorithm and the local map. To localize a robot with a map, ```AMCL``` starts with generating a normally distributed initalpose, which is a list of candidate poses. As the robot moves around, ```AMCL``` updates the probabilities of these candidate poses based on ```LaserScan``` messages and the map, elimating poses whose positions don't align their corresponding predictions. This way, ```AMCL``` is able to narrow down the list of poses that WaiterBot might be in and increase the accuracy of its localization. ```AMCL``` thus does not work well on a large and empty map since it has few useful features for predicting and deciding which poses are the most likely. We eventually settled down on having a smaller, more controlled map built with blocks.  

An issue unique to a waffle model like Mutant is that the four poles interfere with the lidar sensor. We originally tried to subcribe to ```/scan``` topic, modify the ```LaserScan``` messages, and publish it to a new topic called ```/scan_mod``` and have all the nodes which subscribe to ```/scan``` such as ```AMCL``` to subscribe to ```/scan_mod``` instead. This was difficult because of the level of abstraction we had been operating on by roslaunch ```Turtlebot3_navigation.launch``` and it was not convenient to track down every node which subscribe to ```/scan```. Instead, we went straight to the source and found a way to modify the ```LaserScan``` message itself so that its range_min definition is big enough so that it encompasses the four poles. We learnt that we need to modify the ```ld08_driver.cpp``` file on the TurtleBot itself and learnt how to access it through a vnc terminal. We then used ```nano``` editor to edit ```LaserScan``` range_min directly. 

###### Self Assessment 
Overall, we consider this project a success. Through this project, we were able to integrate conversational AI with a robotic application, learning more about Alexa skill development, mapping, localization, and navigation. While there are a few more things we wish we had time to develop, such as integrating a speaker to allow the robot to talk unprompted, or using a robotic arm to place food and drink items on the robot, this project served as a lesson in properly defining the scope of a project. We learned how to identify moments where we needed to pivot, and rapidly prototyped any new features we would need. In the beginning, we wanted to be evaluated on our performance with regards to integrating an NLP system with a robot, and we more than accomplished that. Our Alexa skill uses complex, multi-turn dialog, and our robotic application shows what may be possible for future automation. This project has been a combination of hardware programming, software engineering, machine-learning, and interaction design, all vital skills to be a proper roboticist. 

