
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