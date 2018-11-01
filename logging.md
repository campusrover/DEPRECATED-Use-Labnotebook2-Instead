@Alexander Feldman, feldmanay@gmail.com

# Logging in `ROS`

### What are `ROS` logs?
- Two different things called logs, data logs and normal (`rosout`) logs
- Data logs are collected with `rosbag` and are used to replay or simulate all messages which were sent.
- This is about the other kind of logs, which are primarily used for sending warnings, errors and debugging.
- Solves the problem of distributed information. Many nodes have information to share, which requires a robust logging infrastructure

### What do logs do?
1. Get printed to `stdout` or `stderr` (screen of terminal window running node)
1. Get sent to `rosout` which acts as unified log record
1. Get written a file on the computer running `roscore`. The logs are stored at `~/.ros/logs/`. The subdirectory `latest` can be used, or the command `$ roscd log` will point to the current log directory.

### Log implementation

`std_msgs/Log` type:
![log_msg](https://i.imgur.com/VQQogKf.png)

No publisher needs to be made to log in `rospy`. Instead, use the following functions:
![rospy tooling](https://i.imgur.com/H0ltU0e.png)

what goes where?
![table](https://i.imgur.com/9Bn53Oy.png)

A well formatted GUI exists to show all log messages and can be accessed with `$ rqt_console`:
![rqt_console](https://i.imgur.com/zzeorjS.png)

### More resources
- [My slides](https://docs.google.com/presentation/d/1WL0vn4XhEuDa36pvibgytF7ya8XHCRVbPJDk5l2LIhQ/edit?usp=sharing)
- [`rospy` logging overview](http://wiki.ros.org/rospy/Overview/Logging)
- Programming Robots with ROS ch 21, Quigley, Gerkey, Smart


### Message Node
* It subscribes to the /rosout topic. Every node brought up on the roscore can be seen in the /rosout. Therefore, 
  the Message Node, as a traffic cop, could communicate with any node through /rosout.
* To pass the message from a node to the Message Node, the code just needs one line of code, i.e. rospy.log(). The API can be   found in the Log Implementation section above. 
* User who wants to use Message Node only needs to put its name and what operations he needs to do inside the 
  callback function. In the example, if user wants to subscribe to the /talker_demo_node, he can just find it by 
  `msg.name == "/talker_demo_node"`.  Then, he can do some operation in it. 
  ![callback function](https://i.imgur.com/sAgGQjL.png)
  
