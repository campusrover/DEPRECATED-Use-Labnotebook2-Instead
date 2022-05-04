# Work on progress
 
# Project Report for Project Sample
* Team: Chris Choi (mchoi@brandeis.edu), Lucian Fairbrother (lfairbrother@brandeis.edu),
Eyal Cohen(eyalcohen@brandeis.edu)
* Date: 3rd May 2022
* Github repo: https://github.com/campusrover/RoboTag
`
## Introduction
We wanted to create a dynamic project involving multiple robots. Since this project would be presented to people with a wide range of robotics knowledge, we wanted to create an intuitive project. We decided to recreate the game of tag using robots as most people have played the game.
 
The robots have one of two roles, cop and robber. The robbers roam around until a cop approaches them. When a cop is inside a certain distance, the rubber starts running away from the cop. The cop always chances after the nearest rubber. When the cop catches the rubber, then the roles switch, and the rubber gets 5 seconds to run away. 
 
 
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
We faced a lot of difficulties and had to make many changes throughout the project. We spent lots of time on not only problem solving but also designing. We had multiple hours-long talks in person and on Zoom discussing how to design our project. In these talks, we made decisions to move from one stage to another, start over from scratch for modularity, and rather use move_base or TF. 
 
First stage: one node on each robot.
We tried to let robots directly talk to each other. We used TF to move robots around. The biggest problem was that robots could not switch roles between robber and cop. Also, each node had too many subscribers and publishers that send and receive information on coordinates and roles. 
 
Second stage: Increase modularity
We wanted the robots to switch roles when they make the tag. Also even though we were at the beginning of the project, the code ‘smelled’ bad. We decided to start over with a new control system with an emphasis on modularity. 
The new control system called ‘Control Tower’ is a node run on the computer that keeps a record of all robot’s roles and locations, then orders where each robot has to go. Also, each robot ran the same code, that operates according to given stages. With this system, we were able to switch roles freely and keep codes simple.  
 
Third stage: move_base
First, we were struggling with a method to send and receive coordinates between the control tower and robot. We had to either use one subscriber and publisher pair for each robot, or we had to use custom messages. While we were working on the control tower system, we made a change to how we are going to move robots around. For easier maneuvering around the map with obstacles, we decided to use move_base. We successfully implemented this feature. We made a map of the second floor of our lab because it was smaller, allowing easier localizing. Using this map, we could use move_base and move the robot where ever we wanted to. 
 
Fourth stage: back to TF
Implementing the move_base on one robot was done, but implementing it on multiple robots was a completely different issue. Until a week before the due date, we could not figure out how to run move_base on two robots. In the last week, we decided to go back to TF. We had the basics implemented, and communication between each robot was hard as we previously had a taste of it at the first stage. 
 
 
Fifth stage: TF + node on each computer
The solution we found is to run a node on a computer that deals with the coordinates, while the node on the robot focuses on moving around. This could solve the problem we encountered in the first stage. We also added more playability. The user can take over the control of either cop or rubber, and control the robot with Teleop. Or the user can also yield the control back to the computer. 
 
 


