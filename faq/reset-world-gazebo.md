# Resetting your Gazebo World in Python Files

## Author: Seho Kim

When you are making games or programs with python using ROS, you may need to reset your world at certain point in your code.
The easiest way to reset your Gazebo world would be ctrl+R. However, this may cause unwanted error such as killing one of your node or losing connection.

![ctrl+R Error](https://i.ibb.co/4RDfHwL/reset-world-error.jpg)

To prevent this, you can open up a new terminal and call service:

```
rosservice call /gazebo/reset_world
```

However, you may not want users to manually call service above each time you are required to reset.

##To Programmatically Reset Your Gazebo
Sometimes, your program should reset your Gazebo when certain conditions are met. In order to achieve this, implementing code below will do its job.

```
import rospy
from std_srvs.srv import Empty

rospy.wait_for_service('/gazebo/reset_world')
reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
reset_world()
```

### Empty

We import Empty from std_srvs.srv to send signal to correct services. Empty type is one of the most common service pattern for sending signal to ROS Node.
Empty service does not exchange any actual data between service and a client.

### rospy.wait_for_service('/gazebo/reset_world')

Line above allows code to wait until specific service is active. 

### reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

Here we declare reset_world as service definition. First parameter is service name we want to call.
In our case, we call '/gazebo/reset_world' to reset our Gazebo world. Second parameter is for srv which usually contain a request message and a response message.
For resetting Gazebo world, we do not need any request message nor response message. For resetting Gazebo world, second parameter should be 'Empty'
