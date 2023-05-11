---
title: Dynamaze
author: Aiden Dumas, Muthhukumar Malaiiyyappan
type: report
status: new
date: may-2023
---
# Dynamaze.md
## Spring 2023, Aiden Dumas and Muthhukumar Malaiiyyappan
### Github: https://github.com/campusrover/dynamaze

## Introduction

We started this project with a simple assumption. Solving a maze is an easy task. If you enter a maze and place your hand on one wall of the maze and traverse the maze without ever removing your hand you’ll reach the exit. What happens though if the maze is… dynamic? How would you solve a maze that shifts and changes? The problem is itself intractable. There is no guaranteed way to solve such a maze, but fundamentally we believe that a brute force algorithm is a poor solution to the problem and with the possibility of cycles arising in such a maze; sometimes not a solution at all. We believe that in a dynamic maze it is possible to come up with a solution that given an infinite amount of time, can solve the maze by using knowledge about the structure of the maze as a whole. 

This problem essentially is a simplification of a much more complicated problem in robotics: given a starting point and a known endpoint with an unknown location in an unknown environment, navigate from point A to point B with the added complication that the environment itself is not consistent. Think something comparable to a delivery robot being dropped off at an active construction site. There is no time to stop construction to allow the robot to scan and build a map of the site. There are going to be constantly moving vehicles and materials around the site. A worker tells the robot to go to the manager’s office and gives the robot some description to identify it. The robot must get from where it is to the office despite the scenario it is placed in. This is essentially the problem we are solving. 

## Objectives

- Problem: Develop an algorithm to autonomously navigate and solve a dynamically changing maze.
- Explore the maze and build a representation of it as a graph data structure with intersections as nodes and distances between them as edges
- Create a navigation system based on this graph representation to travel effectively around the maze, updating the graph as changes are detected in real-time. Essentially developing our own SLAM algorithm. (SLAM being an algorithm than automates navigation based on a known map)
- Efficiently search our graph for the exit to the maze its dynamic property making the graph unreliable

## Relevant Sources

- A Helpful Resource for Overall Maze Methodology: https://www.pololu.com/file/0J195/line-maze-algorithm.pdf
- Some Help With Tuning Your PID: https://www.crossco.com/resources/technical/how-to-tune-pid-loops/
- Lidar Methodology: https://github.com/ssscassio/ros-wall-follower-2-wheeled-robot/blob/master/report/Wall-following-algorithm-for-reactive%20autonomous-mobile-robot-with-laser-scanner-sensor.pdf
- Entity Used for Consulting: https://chat.openai.com

## Files & How to Use

Use: All that is needed to use the code is to clone the github repository and put your robot in a maze!

| File Name      | Purpose               |
|----------------|-----------------------|
| laserUtils.py  | Process Lidar Data    |
| pid_explore.py | Navigating Maze       |





## Methodology

From a birds eye view, the solution we came up with to solve a dynamic maze mirrors a very interesting problem in graph algorithms. With the assumption that the key to our solution lies in building an understanding of the structure of the maze as a whole, we build a map on the fly as we traverse the maze. We represent the maze as a graph data structure, labeling each intersection in the maze we reach as a node and each edge as the distance between these intersections in the maze. We create these nodes and edges as we encounter them in our exploration and delete nodes and edges as we come across changes in the maze. This problem can then be seen in actuality as a graph algorithms problem. Essentially we are running a graph search for a node with an unknown entry point into an unknown graph that itself is unreliable or rather, “live” and subject to modification as it is read. There are then 2 stages to solving this problem. The first is learning the graph for what it is as we explore, or rather create a full picture of the graph. Once we have explored everything and have not found the exit we know searching for the exit is now the problem of searching for change. The crux of solving this problem is characterizing the changes themselves. It is by understanding how changes take place that we devise our graph search from here to find these changes. For example, a graph whose changes occur randomly with equal probability of change at each node would have an efficient search solution of identifying the shortest non cyclical path that covers the most amount of nodes. The change behavior we used for our problem was that changes happen at certain nodes repeatedly. To solve the maze, we then use the previous solution of searching the most nodes as quickly as possible for changes if we fully explore the maze without identifying anything, then once a changing node is found we label it as such so we can check it again later, repeating these checks until new routes in the maze open up. 

As a note, some simplifications that we use for time’s sake:
Width of corridors in the maze are fixed to .5m
Turns with always be at right angles

We designed an intelligent maze-solving robot navigation system that employs a depth-first search (DFS) algorithm for effective graph traversal. Our solution is structured into two primary components: laserutils and pid_explore, which work together seamlessly to facilitate the robot's navigation and exploration within the maze.

The laserutils module is a collection of essential utility functions for processing laser scan data and assisting in navigation. It calculates direction averages, simplifies 90-degree vision frameworks, detects intersections, and implements a PID controller to maintain a straight path. Moreover, it incorporates functions for converting yaw values, managing node formatting, and handling graph data structures, ensuring a streamlined approach to processing and decision-making.

The pid_explore module serves as the core of the robot's navigation system. It subscribes to laser scan and odometry data and employs the laserutils functions for efficient data processing and informed decision-making. The robot operates in three well-defined states: "explore," "intersection," and "position," each contributing to a smooth and strategic navigation experience.

In the "explore" state, the robot uses the PID controller to navigate straight, unless it encounters an intersection, prompting a transition to the "intersection" state. Here, the robot intelligently labels the intersection, assesses the path's continuation, and determines its next move. If the robot is required to turn, it shifts to the "position" state, where it identifies the turn direction (left or right) and completes the maneuver before resuming its straight path.

Our navigation system employs a Graph class to store and manage the maze's structure, leveraging the power of the DFS algorithm to find the optimal path through the maze. The result is a sophisticated and efficient maze-solving robot that strategically navigates its environment, avoiding any semblance of brute force in its problem-solving approach.


## Story of Project
We built this project to address the challenges faced by robots in navigating unpredictable and dynamic environments, such as active construction sites or disaster zones. Traditional maze-solving algorithms, based on brute force and heuristics, are inadequate in such situations due to the constantly changing nature of the environment.

Our goal was to develop an intelligent navigation system that could adapt to these changes in real-time and continue to find optimal paths despite the uncertainties. By creating a program that can autonomously drive a robot through a changing maze, we aimed to advance robotic navigation capabilities and make it more efficient and useful in a wide range of applications.

With our innovative approach of "on the fly" map creation and navigation, we sought to enable robots to traverse dynamic spaces with minimal prior knowledge, making them more versatile and valuable in industries that require real-time adaptability. By simultaneously building and updating the map as the robot navigates, we ensured that the robot could rely on its current understanding of the environment to make informed decisions.

Ultimately, we built this project to push the boundaries of robotic navigation, opening up new possibilities for robots to operate in complex, ever-changing environments, and offering solutions to a variety of real-world challenges.

The project began our first week with a theoretical hashing out of how to approach the problem and what simplifications to make. We started by learning how to build simulations in Gazebo and implementing our Lidar methods. We then were able to create simple demos of an initial explore algorithm we created with the “explore”, “intersection”, “position” approach. Gazebo issues were encountered early on and were largely persistent throughout the course of the project with walls losing their physics upon world resets. At this point we built larger models in Gazebo to test with and implemented a PID algorithm to smooth the robot’s traversal. This PID would go through several versions as the project went on especially as we tuned navigation to be more precise as we adjusted our hall width from 1 meter down to .5. We then formulated and integrated our graph data structure and implemented a DFS algorithm to automatically find paths from node to node to navigate. We then added node labeling and cycle detection to our graph to recognize previously visited nodes. One of our group member’s github became permanently disconnected from our dev environment in the last stage of the process but all challenges are meant to be overcome. We finished by accounting for dynamic change in our algorithm and designed a demo. 


## Final Product
Despite having trouble in our Gazebo simulations we were surprised to see that our algorithm worked well when we moved our tests over to the physical robots.
PID especially, which had proven to be trouble in our testing in fact worked better in the real world than in simulations. There were a few corners cut in robustness (ie: with the simplifications agreed upon such as only right angle turns), but overall the final product is very adaptable within these constraints and accounts for a number of edge cases. Some of the potential improvements we would have liked to make is to expand past these simplifications of only having right-angled intersections and limited hall width. The prior would greatly change the way we would have to detect and encode intersections for the graph. A next algorithmic step could be having two robots traverse the maze from different points at the same time and sharing information between each other. This would be the equivalent of running two concurrent pointers in the graph algorithm. 
Overall, though an initially simple project on the surface, we believe that the significance of this problem as a graph algorithms problem not only makes it an important subject in robotics (maze traversal) but also an unexplored problem in the larger world of computer science; searching an unreliable/live modifying graph. 
