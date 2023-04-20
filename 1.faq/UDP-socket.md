# UDP- Sockets 

## Author: Lucian Fairbrother

Do you need to give information to your roscore that you can't transport with rosnodes? 

You may have trouble running certain libraries or code in your vnc environment, a UDP connection could allow you to run it somewhere else and broadcast it into your vnc.
There are many reasons this could happen and UDP sockets are the solution. In our project we used multiple roscores to broadcast the locations of our robots. We send the robot coordinates over a UDP socket that the other roscore can then pickup and use.

![image](https://user-images.githubusercontent.com/92168798/166608732-3deb84bd-4af4-41ad-aadc-36fb91352dec.png)
# Simple Sender

Here is an example of the most basic sender that you could use for your project. In this example the sender sends out a string to be picked up:


```
import socket
host = <enter host IP here>
port = 5000
s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.bind(('', port))
s.listen(1)
c, addr = s.accept()
print("CONNECTION FROM:", str(addr))
c.send(b"HELLO, How are you ? Welcome to Akash hacking World")
msg = "Bye.............."
c.send(msg.encode())
c.close()

```

# Simple Receiver

You need to run a receiver to pickup the information that your sender put out

```
import socket
host = <Same IP>
port = 5000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('127.0.0.1', port))
msg = s.recv(1024)
while msg:
	print('Received:' + msg.decode())
	msg = s.recv(1024)
s.close()
```

# My Sender

Often times you may want to create a sender node that will take information from your ROS environment and publish it to the outside world. Here is an example of how I went about doing this.


```
#!/usr/bin/env python
import os
from socket import *
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
# Because of transformations
import tf_conversions 
import tf2_ros
import geometry_msgs.msg
import math 
from geometry_msgs.msg import PoseWithCovarianceStamped 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node("sender")
#100.71.173.127
#100.74.41.103
host = "100.71.173.127" # set to IP address of target computer
port = 13000
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)



roll = 0.0
pitch = 0.0
yaw = 0.0

def get_rotation (msg):
    if msg is not None:
        global roll, pitch, yaw 
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        print('X =',msg.pose.pose.position.x, 'Y =',msg.pose.pose.position.y, 'Yaw =',math.degrees(yaw))
        mess=str(msg.pose.pose.position.x)+" "+str(msg.pose.pose.position.y)
        data = bytes(str(mess), 'utf-8')
        UDPSock.sendto(data, addr)
    

sub = rospy.Subscriber ('/amcl_pose', PoseWithCovarianceStamped, get_rotation) # geometry_msgs/PoseWithCovariance pose

while not rospy.is_shutdown():
    hi = "r"
```

# My Receiver

And here is the receiver we created to handle our sender

```
#!/usr/bin/env python
import os
from socket import *
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import json
from std_msgs.msg import Float64MultiArray




rospy.init_node("receiver")
mypub = rospy.Publisher('/other_odom', Float64MultiArray,queue_size = 10)


host = "100.74.41.103"
port = 13000
buf = 1024
addr = (host, port)
UDPSock = socket(AF_INET, SOCK_DGRAM)
UDPSock.bind(addr)
while not rospy.is_shutdown():
    (data, addr) = UDPSock.recvfrom(buf)
    data=data.decode('utf-8')
    data=data.split()
    x=data[0]
    y=data[1]
    x=float(x)
    y=float(y)
    my_msg = Float64MultiArray()
    d=[x, y, 67.654236]
    my_msg.data = d
    mypub.publish(my_msg)
    if data == "exit":
        break
UDPSock.close()
os._exit(0)
```

Overall UDP-sockets aren't very difficult to make. Ours simplified the complexity of our project and helped build modularity. Overall this receiver and sender acts as another Ros publisher and subscriber. It has the same function it instead builds a metaphorical over-arching roscore for both roscores the sender and receiver live in. 
