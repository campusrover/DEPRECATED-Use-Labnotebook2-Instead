---
title: How to Define and Use Your Own Message Types?
author: Liulu Yue
description: This FAQ documents specific instructions to define a new Message, as well as solutions to some issues you might encounter in your message defining process
date: May-2023
---
## How to create a new message type?
After created a ROS package, our package is constructed by a src folder, a CMakeLists.txt file, and a package.xml file. We need to create a msg folder to hold all of our new msg file. Then in your msg folder, create a new file <new_message>.msg, which contains fields you needed for this message type. <br />
For each fields in your msg file, define the name of the field and the type of the field (usually use std_msgs/<Type> or geometry_msgs/<Type>). <br \>
  For example, if you want your message to have a list of string and an integer, you msg file should look like:
  <pre><code>std_msgs/String[] list
std_msgs/Int16 int</code></pre>

## How to let the new message type recognized by ROS?
There are some modifications you need to make to CMakeLists.txt and package.xml in order to let the new message type recognized by ROS. <br \>
For CMakeLists.txt:
1. Make sure message_generation is in find_package().
2. Uncomment add_message_files() and add your .msg file name to add_message_files().
3. Uncomment generate_messages()
4. Modify catkin_package() to
<pre><code>catkin_package(
  CATKIN_DEPENDS message_runtime
)</code></pre>
5. Uncomment include in include_directories()
  
For package.xml:
  1. Uncomment <build_depend>message_generation</build_depend> on line 40
  2. Uncomment <exec_depend>message_runtime</exec_depend> on line 46

## How to use the newly created message type?
1. How to import the message?
  <pre><code>from package_name.msg import message_name as message_name</code></pre>
  If your message type contains a list of buildin message type, also make sure to import that buildin message type:
  <pre><code>from std_msgs.msg import String</code></pre>
2. How to use the message? <br \>
  The publisher and subscriber's syntax are the same. However, we want to create a new topic name and make sure the new topic message type is specified. For example in my project I have a message type called see_intruder:
  <pre><code>self.detect_intruder_pub = rospy.Publisher('/see_intruder', see_intruder, queue_size=1)
self.detect_intruder_sub=rospy.Subscriber('/see_intruder', see_intruder,self.see_intruder_callback)
  </code></pre>
  Since our new message depends on some build in message type, when we try to access the feild of our msg, we need to do msg.<field_name>.data given that the build in message type has a field named data. So in the first example I had, if we want to access the string stored in index int of list, I need to do
  <pre><code>msg.list[msg.int.data].data</code></pre>
  
## How to check if the new message type is recognized by ROS?
Do a cm in your vnc. if the message is successfully recognized by ROS, you will see the msg file being successfully generated. 

## Having an error "No module named <>.msg"?
In your vnc terminal, type the command
<pre><code>source ~/catkin_ws/devel/setup.bash</code></pre>
This should solve the error.


