# Nodes
A behavior tree is a tree-shaped data structure consisting of nodes, each of which contain a logical method which executes code. The behavor tree is evaluated recursively starting 
at the root node. Each node has the abilility to execute code which will either run a script, or execute all of its children. Each node will also return one of 3 outputs to its 
parent: "success", "failure", or "running". There are two main types of nodes: the control-flow (parent nodes) nodes and the leaf nodes.

## Control-Flow Nodes

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

## Leaf Nodes

- Action Nodes
  - Action nodes send ROS topic messages from the behavior tree to the robot.
  - Often the type of message sent from an Action node is a cmd_vel message which encodes movement instructions for the robot.
- Update Nodes
  - Update nodes are designated for updating data in the blackboard.
  - Often times the types of data updates performed by Update nodes include preprocessing or processing of message data from the robot.
- Conditional Nodes
  - Conditional nodes will return either "success" or "failure", corresponding to the boolean values "true" and "false" respectively
  - Conditional nodes will access data in the blackboard and return one of the two values listed above based on if a particular condition is met within the data.