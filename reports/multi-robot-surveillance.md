# Multi-Robot Home Surveillance

## Table of Contents
- [Problem Statement](#Problem-Statement)
- [Challenges](#Challenges)
- [Video Demo](#Video-Demo)
- [Program Structure](#Program-Structure)
- [TF Tree](#TF-Tree)
- [RViz Interface](#RViz-Interface)
- [Leader Election Algorithm](#Leader-Election-Algorithm)
- [Resiliency and Node Failure Detection](#Resiliency-and-Node-Failure-Detection)
- [Limitations](#Limitations)
- [Applications](#Applications)
- [Team Members](#Team-Members)
- [Special Thanks](#Special-Thanks)

## Problem Statement

![home](images/home.png)

- The home above is divided into four security zones.
- We can think of them as a sequence, ordered by their importance: [A, B, C, D].
- If we have n live robots, we want the first n zones in the list to be patrolled by at least one robot.
  - n== 2 → patrolled(A, B); 
  - n == 3 → patrolled(A, B, C)
  - 1 <= n <= 4
- The goal is for this condition to be an invariant for our program, despite robot failures.
- Let robot<sub>z</sub> refer to the robot patrolling zone Z.
- Illustration:
  - n == 4 → patrolled(A, B, C, D)
  - robotA dies
  - n == 3 → patrolled(A, B, C)
  - robotB dies
  - n == 2 → patrolled(A, B)
  - robotA dies
  - n == 1 → patrolled(A)

## Challenges
- We need to:
  - map the home
    - SLAM (Simultaneous Localization and Mapping) with the gmapping package
  - have multiple robots patrol the security zone in parallel
    - AMCL (Adaptive Monte Carlo Localization) and the move_base package
    - One thread per robot with a SimpleActionClient sending move_base goals
  - have the system be resilient to robot failures, and adjust for the invariant appropriately
    - Rely on Raft’s Leader Election Algorithm (‘LEA’) to elect a leader robot.
    - The remaining robots will be followers.
    - The leader is responsible for maintaining the invariant among followers.
- More on system resiliency:
  - The leader always patrols zone A, which has the highest priority.
  - On election, the leader assigns the remaining zones to followers to maintain the invariant.
  - On leader failure: 
    - global interrupts issued to all patrol threads (all robots stop).
    - the LEA elects a new leader
  - On follower failure: 
    - local interrupts issued by leader to relevant patrol threads (some robots stop).
    - Example:
      - n==4 → patrol(A, B, C, D)
      - robotB dies
    - leader interrupts robotD
    - leader assigns robotD to zone B
    - n==3 → patrol(A, B, C)

## Video Demo
[Watch the System in Action!](https://drive.google.com/file/d/1UgOknVUCbNmY5NiatFo7d72_akHZG5z7/view?resourcekey)

## Program Structure
- Two points of entry:  real_main.launch and sim_main.launch.
- real_main.launch:
  - used for real-world surveillance of our home environment
  - runs the map_server with the home.yaml and home.png map files obtained by SLAM gmapping.
    - All robots will share this map_server
  - launches robots via the real_robots.launch file (to be shown in next slide)
  - launches rviz with the real_nav.rviz configuration
- sim_main.launch:
  - used for simulated surveillance of gazebo turtlebot3_stage_4.world
  - works similarly to real_main.launch except it uses a map of stage_4 and sim_robots.launch to run gazebo models of the robots via tailored urdf files.

## TF Tree
![TF Tree](images/tf-tree.png)

## RViz Interface
![RViz Interface](images/rviz.png)

## Leader Election Algorithm

- Based on Diego Ongaro et al. 2014, “In Search of an Understandable Consensus Algorithm”
- The full Raft algorithm uses a replicated state machine approach to maintain a set of nodes in the same state. Roughly:
  - Every node (process) starts in the same state, and keeps a log of the commands it is going to execute.
  - The algorithm elects a leader node, who ensures that the logs are properly replicated so that they are identical to each other.
- Raft’s LEA and log replication algorithm are fairly discrete. We will only use the LEA.

1. Background:
    1. Each of the robots is represented as an mp (member of parliament) node.
    2. mp_roba, mp_rafael, etc.
    3. Each node is in one of three states: follower, candidate, or leader.
    4. If a node is a leader, it starts a homage_request_thread that periodically publishes messages to the other nodes to maintain its status and prevent new elections.
    5. Every mp node has a term number, and the term numbers of mp nodes are exchanged every time they communicate via ROS topics.
    6. If an mp node’s term number is less than a received term number, it updates its term number to the received term number.
2. LEA:
    1. On startup, every mp node is in the follower state, and has:
        1. a term number; and
        2. a duration of time called the election timeout.
    2. When a follower node receives no homage request messages from a leader for the duration of its election timeout, it:
        1. increments its term number;
        2. becomes a candidate;
        3. votes for itself; and
        4. publishes vote request messages to all the other mp nodes in parallel.
    3. When a node receives a request vote message it will vote for the candidate iff:
        1. the term number of the candidate is at least as great as its own; and
        2. the node hasn’t already voted for another candidate.
    4. Once a candidate mp node receives a majority of votes it will:
        1. become a leader;
        2. send homage request messages to other nodes to declare its status and prevent new elections.
    5. The election timeout duration of each mp node is a random quantity within 2 to 3 seconds.
    6. This prevents split votes, as it’s likely that some follower will timeout first, acquire the necessary votes, and become the leader. 
    7. Still, in the event of a split vote, candidates will:
        1. reset their election timeout durations to a random quantity within the mentioned interval;
        2. wait for their election timeout durations to expire before starting a new election.

## Resiliency and Node Failure Detection
- The election timeout feature is part of what enables leader resiliency in my code. 
    - When a leader node is killed, one of the follower nodes time out and become the new leader.
- roscore keeps a list of currently active nodes. So:
    - followers, before transitioning to candidate state, update their list of live mp nodes by consulting roscore.
    - leaders do the same via a follower_death_handler thread, which runs concurrently with main thread and the homage_request thread, and maintains the invariant.

## Limitations
-  The mp nodes are all running on the remote VNC computer, which also runs the roscore.
    - So killing the mp node is just a simulation of robot failure, not true robot failure.
        - We’re just killing a process on the vnc computer, not the robot itself
    - We can remedy this by binding the mp node’s life to any other nodes that are crucial to our robot fulfilling its task. 
        - E.g., for our project the /roba/amcl and /roba/move_base nodes are critical.
        - So we can consult roscore periodically to see if any of these nodes die at any given point. And if they do, we can kill mp_roba.
        -  We can also define and run more fine-grained custom nodes that keep track of any functional status of the robot we’re interested in, and bind the life of those custom nodes to the mp nodes.
- Running each mp node on its corresponding robot is a bad idea!
    - Only if a robot gets totally destroyed, or the mp process fails, will the algorithm work.
    - Performance will take a hit: 
      - bandwidth must be consumed for messages between mp nodes
      - robot’s computational load will increase
- Our system has a single point of failure; the VNC computer running roscore.
    - So we have to keep the VNC safe from whatever hostile environment the robots are exposed to.
    - Maybe this can be remedied in ROS2, which apparently does not rely on a single roscore. 
- Our patrol algorithm is very brittle and non-adversarial
    - Depends on AMCL and move_base, which do not tolerate even slight changes to the environment, and which do not deal well with moving obstacles.
    - There are no consequences to disturbing the patrol of the robots, or the robots themselves (e.g. no alarm or ‘arrests’)

## Applications
-  The LEA is fairly modular, and the algorithm can be adapted to tasks other than patrolling.
    - It would work best for tasks which are clearly ordered by priority, and which are hazardous, or likely to cause robot failures.
    - Completely fanciful scenarios:
        - Drone strikes: Maybe certain military targets are ordered by priority, and drones can be assigned to them in a way that targets with a higher priority receive stubborn attention.
        - Planetary exploration: Perhaps some areas of a dangerous planet have a higher priority that need to be explored than others. Our system could be adapted so that the highest priority areas get patrolled first.

## Team Members
- James Lee (leejs8128@brandeis.edu)

## Special Thanks
- A special thanks to Adam Ring, the TA for the course, who helped with the project at crucial junctures.