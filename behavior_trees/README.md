# Behavior Tree Framework for Lab Robots

The goal behind the MRBT project was to create a efficient, modular, and user-friendly solution for programming complex behaviors into the Turtlebot3 robot stack. Our
solution came in the form of a popular method in the videogaming industry for programming behaviors into NPCs (Non-Playable Characters in videogames; think enemies in HALO or Bioshock).
With the use of behavior trees, we are now able to program complex behaviors using a simple tree definition in a JSON file, and we can partition a behavior into multiple different
subtrees which can then be used elsewhere or standalone.

## Key Components of the Behavior Tree

Any complex behavior tree can be broken down into a few key components which highlight the overall logical structure of how they operate.

### Blackboard
The Blackboard is the main storage component for data accessible to nodes in the behavior tree. It is stored as a Python dictionary and its reference is passed down to each node
in the behavior tree from the root node. The effect of this is that each node in the tree is able to change and share data through the Blackboard. Additionally, data that is sent from the sensors on the robot, including camera data, lidar data, etc, is accessable from each node in the tree.

### Nodes
A behavior tree is a tree-shaped data structure consisting of nodes, each of which contain a logical method which executes code. The behavor tree is evaluated recursively starting 
at the root node. Each node has the abilility to execute code which will either run a script, or execute all of its children. Each node will also return one of 3 outputs to its 
parent: "success", "failure", or "running". There are two main types of nodes: the control-flow (parent nodes) nodes and the leaf nodes.

#### Control-Flow Nodes

- Selector
  - The Selector executes its children sequentially from left to right. 
  - If one of its children returns either "success" or "running", it will halt execution of its children and it will return the result of the child it stopped on.
  - If all of its children return "failure", the Selector will also return "failure".
- Sequencer
  - The Sequencer executes its children sequentially from left to right.
  - The Sequencer will not halt execution of its children unless one of them returns "failure" or "running", in which case it will also return "failure" or "running".
  - If all children return "success" the Sequencer will return "success"
- Multitasker
  - The Multitasker runs all of its children concurrently, each in a separate thread.
  - The Multitasker will return "success" only if all of it's children return "success".
  - If any of its children return "running" but none return "failure", the Multitasker will return "running".
  - If any of its children return "failure", the Multitasker will return "failure".

#### Leaf Nodes

- Action Nodes
  - Action nodes send ROS topic messages from the behavior tree to the robot.
  - Often the type of message sent from an Action node is a cmd_vel message which encodes movement instructions for the robot.
- Update Nodes
  - Update nodes are designated for updating data in the blackboard.
  - Often times the types of data updates performed by Update nodes include preprocessing or processing of message data from the robot.
- Conditional Nodes
  - Conditional nodes will return either "success" or "failure", corresponding to the boolean values "true" and "false" respectively
  - Conditional nodes will access data in the blackboard and return one of the two values listed above based on if a particular condition is met within the data.

