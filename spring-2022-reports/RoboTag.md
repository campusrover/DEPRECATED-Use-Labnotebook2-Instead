# Work on progress
 
# Project Report for Project Sample
* Team: Chris Choi (mchoi@brandeis.edu), Lucian Fairbrother (lfairbrother@brandeis.edu),
Eyal Cohen(eyalcohen@brandeis.edu)
* Date: 3rd May 2022
* Github repo: https://github.com/campusrover/RoboTag
`
## Introduction
We wanted to create a dynamic project involving multiple robots. Since this project would be presented to people with a wide range of robotics knowledge, we wanted to create an intuitive project. We decided to recreate the game of tag using robots as most people have played the game.
 
The robots have one of two roles, cop and rubber. The robbers roam around until a cop approaches them. When a cop is inside a certain distance, the rubber starts running away from the cop. The cop always chances after the nearest rubber. When the cop catches the rubber, then the roles switch, and the rubber gets 5 seconds to run away. 
 
 
### Problem Statement including original objectives
 
### Relevant literature
The PRR examples and previous PAs were great resources for this project. The stage system was inspired by the wall follower PA, the human control mechanism is from the Teleop example, and TF is from the double TF PA. 
 
## What was created
Currently implemented two robots that can alternate between cop and rubber, and user-controlled cop and rubber. 
 
### Technical Description, illustrations
 
### Discussion of interesting algorithms, modules, techniques
 
### Guide on how to use the code written
Every robot needs its own computer to run. On each computer run $$$
### Clear description and tables of source files, nodes, messages, actions and so on
 
## Story of the project.
### How it unfolded, how the team worked together
### problems that were solved, pivots that had to be taken
We faced a lot of difficulties and had to make many changes throughout the project. We spent lots of time on not only problem solving but also designing. We had multiple hours-long talks in person and on Zoom discussing how to design our project. In these talks, we made decisions to move from one stage to another, start over from scratch for modularity, and rather use move_base or TF. 
 
First stage: one node on each robot.
We tried to let robots directly talk to each other. We used TF to move robots around. The biggest problem was that robots could not switch roles between rubber and cop. Also, each node had too many subscribers and publishers that send and receive information on coordinates and roles. 
 
Second stage: Increase modularity
We wanted the robots to switch roles when they make the tag. Also even though we were at the beginning of the project, the code ‘smelled’ bad. We decided to start over with a new control system with an emphasis on modularity. 
The new control system called ‘Control Tower’ is a node run on the computer that keeps a record of all robot’s roles and locations, then orders where each robot has to go. Also, each robot ran the same code, that operates according to given stages. With this system, we were able to switch roles freely and keep codes simple.  
 
Third stage: move_base
First, we were struggling with a method to send and receive coordinates between the control tower and robot. We had to either use one subscriber and publisher pair for each robot, or we had to use custom messages. While we were working on the control tower system, we made a change to how we are going to move robots around. For easier maneuvering around the map with obstacles, we decided to use move_base. We successfully implemented this feature. We made a map of the second floor of our lab because it was smaller, allowing easier localizing. Using this map, we could use move_base and move the robot where ever we wanted to. 
 
Fourth stage: back to TF
Implementing the move_base on one robot was done, but implementing it on multiple robots was a completely different issue. Until a week before the due date, we could not figure out how to run move_base on two robots. In the last week, we decided to go back to TF. We had the basics implemented, and communication between each robot was hard as we previously had a taste of it at the first stage. 
 
 
Fifth stage: TF + node on each computer
The solution we found is to run a node on a computer that deals with the coordinates, while the node on the robot focuses on moving around. This could solve the problem we encountered in the first stage. We also added more playability. The user can take over the control of either cop or rubber, and control the robot with Teleop. Or the user can also yield the control back to the computer. 
 
 
### Your own assessment

