# Work on progress
 
# Project Report for Project Sample
* Team: Chris Choi (mchoi@brandeis.edu), Lucian Fairbrother (lfairbrother@brandeis.edu),
Eyal Cohen(eyalcohen@brandeis.edu)
* Date: 3rd May 2022
* Github repo: https://github.com/campusrover/RoboTag
`
## Introduction
We wanted to create a dynamic project involving multiple robots. Since this project would be presented to people with a wide range of robotics knowledge, we wanted to create an intuitive project. We decided to recreate the game of tag using robots as most spectators know the rules, and so that each robot affects the other’s behavior.
 
The robots have one of two roles, cop and robber. The robbers flee from cops while avoiding obstacles, as the cops are in hot pursuit of them. When a cop is within a certain distance, and the cop catches the robber, their roles switch, and the new robber gets a 10-second head start to run away. If a spectating human feels left out of the fun, they can press a button to take control of their robot and chase the robber as the cop, or chase the cop as the robber.

 
### Problem Statement including original objectives

 
## What was created
Currently implemented two robots that can alternate between cop and robber, and user-controlled cop and robber. 
 
### Technical Description, illustrations
 ![IMG_0093](https://user-images.githubusercontent.com/92168798/166615484-2d3fd134-8995-4355-a80a-2a7a8d54f7e9.jpg)

### Discussion of interesting algorithms, modules, techniques
 COP ALGORITHYM-
    The cop algorithym was difficult to implement. The question of how to orient a cop towards moving coordinates was difficult for us to wrap our heads around. We first had to understand the pose variables. The pose orientation variable ranges from -3.14 to 3.14 and represents the angles a robot could be in, in radians. We eventually figured out a good compass algorithym, we used an if statement that calculated whether turning left or right was closer to the goal angle and then executed it. We had it go forward if the actual angle was within .2 radians of the goal angle
    
UDP-SOCKETS-
    We used UDP-sockets to send info accross our roscores. Because we had two roscores and we needed the robots to communicate their locations to each other we had them send their locations constantly over UDP sockets. We made a sender and receiver for each robot. The sender would subscribe to the AMCL_pose and then send out the message over the socket. The receiver would receive the message decode it, put it into a float64 list and publish it to the robot. 
    
STATE SWITCH-
    State switching in our game is hugely important, if the robots aren't localized properly and one thinks a tag has happened while the other doesn't they will get caught as the same state. To deal with this we used AMCL to increase the localization and decrease any error. We also set up the tag such that the robber would stop for ten seconds after it became the cop and not be able to tag the new robber during that period. There were a few reasons we did this. Firstly because we wanted the new robber to have a chance to get away before it would get tagged again. Otherwise the two robots could get into an infinite loop of state switching. We also set the distance for the robber tag to be further than the cop to recognize a tag. The robber recognizes a tag at .35 and the cop recognizes it at .3 the reason for this is because the robber stops after recognizing the tag and the cop will keep going until it recognizes the tag. This makes it very unlikely for only one robot to recognize a tag which would result in them getting stuck in the same state.
    
 
### Guide on how to use the code written
Every robot needs its own computer to run. 
1. On each computer clone the repository
2. Go into allinone.py and change one initialized state to robber, such that you have a cop and robber
3. go into tf_sender.py and change the ip address to the ip address of the other computer
4. go into receiver.py and change the ip address to your ip address
5. go into your vnc and run roslaunch robotag robo.launch

### Clear description and tables of source files, nodes, messages, actions and so on
 
 Robo.launch- The main launch file for our project
 
 NODES

  Allinone.py - main program node
  
  tf_sender.py - the socket sender node
  
  receiver.py - the socket receiver node
 
 OTHER FILES
 
  Map.yaml
  
  Map.pgm
  
  AMCL
  
### problems that were solved, pivots that had to be taken
After the ideation period of Robotag, we faced a lot of difficulties and had to make many pivots throughout the project. We spent lots of time on not only development but also designing new methods to overcome obstacles. We had multiple sleepless nights in the lab, and hours-long Zoom meetings to discuss how we design our project. In these talks, we made decisions to move from one stage to another, start over from scratch for modularity, and strategize navigational methods such as move_base, TF, or our ultimate homebrewed algorithms. 
 
First stage: Broadcast/Listener Pair Using TF.
We tried to let robots directly communicate with each other by using TF's broadcast/listener method. We initially turned to TF as it is a simple way to implement following between robots. However, this solution was overly-predictable and uninteresting. Solely relying on TF would limit our ability to avoid obstacles in crowded environments.
 
Second stage: Increase modularity
We decided to start over with a new control system with an emphasis on modularity. 
The new control system called ‘Control Tower’ is a node run on the computer that keeps a record of all robot’s roles and locations, then, orders where each robot has to go. Also, each robot ran the same code, that operates according to given stages. With this system, we would be able to switch roles freely and keep codes simple. 
 
Third stage: move_base
Although the Control Tower could properly listen to each robot, publishing commands to each robot was a grueling experience. For optimized maneuvering around a map with obstacles, we decided to use move_base. We successfully implemented this feature on one robot. Given coordinates, a robot could easily utilize move_base to reach that location. We were planning for this to be a cop’s method of tracking down the robber. We SLAMed a map of the second floor of our lab because it was mostly enclosed and has defining features allowing easier localizing. Using this map and AMCL, we could use move_base and move the robot wherever we wanted to. However, when it came time to run move_base on several robots, each robot’s AMCL cost map and move_base algorithm confused the opposing bot. Although in theory, this solution was the most favorable–due to its obstacle avoidance and accurate navigation–in practice, we were unable to figure out how to get move_base to work with multiple robots and needed to give up on AMCL and move_base sooner than we did.
 
 
Fourth stage: Using Odom
After move_base and AMCL failed us, we used the most primitive method of localization, Odometry. We started each robot from the same exact position and watched as one robot would go roughly in the direction of the other. After a minute, each robot’s idea of the coordinate plane was completely different and it was clear that we could not move forward with using odometry as our method of localization. Due to the limitation of each robot needing to run the program from the same location, and its glaring inaccuracies, we looked elsewhere for our localization needs.
 
Fifth stage: Better Localization
We missed the accuracy of AMCL. AMCL has fantastic localization capabilities combining both lidar and Odom and allows us to place a robot in any location on the map and know its present location, but we didn’t have time to wrestle with conflicting cost maps again, so we elected to run each robot on a separate computer. This would also allow future users to download our code and join the game with their robots as well! Once we had each robot running on its own computer, we created a strategy for robots to communicate between a UDP Socket, such that each one can receive messages about the others’ coordinates. This was an impromptu topic across multiple roscores!
 
From then on, our localization of each robot was fantastic and allowed us to focus on improving the cops’ tracking and the robber’s running away algorithm while effectively swapping their states according to their coordinate locations in their respective AMCLs.


