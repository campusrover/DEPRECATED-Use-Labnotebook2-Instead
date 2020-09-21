
## Using the MiniRover

### Turning it on

It is important that you follow a rigid procedure when turning the robot on and off.

Assuming it is totally off:

1. Make sure the battery pack is correctly connected to the robot.

1. Switch on the battery pack

1. Click the micro button on the red board

![Button to reboot](button.jpg)

### Turning it off

1. From your ssh command line on the remote computer, type `sudo shutdown now`

1. Once the Raspberry Pi has stopped blinking you can turn off the power switch on the battery pack

#### Key Environment Variables

`ROS_MASTER_URI=http:/100.94.206.80:11311` (example!) should always be set to the computer where roscore is running. In our world, roscore runs on the robot itself. So we need to know the ip address of the robot. You think you know it, but wait.

`ROS_IP=100.94.206.80` (example) should be equal to the local ip address, a computer's own ip address. You think you know it, but wait.

As you know ROS requires that all nodes can talk to roscore. These two environment variables need to be defined on each computer that is involved, that is, both the robot and the remote computer. So you would have the following:

##### Robot

ROS_MASTER_URI = robot's own ip address
ROS_IP = robot's own ip address

##### Remote Computer 

ROS_MASTER_URI = robots ip address
ROS_IP = remote computer's own IP address

These IP addresses are on different networks and cannot access each other. So instead we've created what is called a "virtual private network" that connects them together. Both your robot and your cloud desktop have an *alternate* ip address which they can both see.
