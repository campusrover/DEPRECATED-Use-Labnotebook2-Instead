## FAQ

### Accounts and passwords

* miniRover
  * hostname `gopigo3`
  * default account `pi`
  * default password raspberry
* Cloud or Docker Desktop
  * default password `dev@ros`
  * url: <unetid>.ros.campusrover.org:6080/vnc.html (desktop)
  * url: <unetid>.ros.campusrover.org:8080 (cloud vscode)

### Key Environment Variables

`ROS_MASTER_URI=http:/100.94.206.80:11311` (example!) should always be set to the computer where roscore is running. In our world, roscore runs on the robot itself. So we need to know the ip address of the robot. You think you know it, but wait.

`ROS_IP=100.94.206.80` (example) should be equal to the local ip address, a computer's own ip address. You think you know it, but wait.

As you know ROS requires that all nodes can talk to roscore. These two environment variables need to be defined on each computer that is involved, that is, both the robot and the remote computer. So you would have the following:

### IP Addresses

#### Robot

ROS_MASTER_URI = robot's own ip address
ROS_IP = robot's own ip address

##### Remote Computer 

ROS_MASTER_URI = robots ip address
ROS_IP = remote computer's own IP address

These IP addresses are on different networks and cannot access each other. So instead we've created what is called a "virtual private network" that connects them together. Both your robot and your cloud desktop have an *alternate* ip address which they can both see.
