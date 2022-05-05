# **The Litter Picker Robot**
* Team: Peter Zhao (zhaoy17@brandeis.edu) and Veronika Belkina (vbelkina@brandeis.edu)
* Date: May 2022
* Github repo: https://github.com/campusrover/litter-picker

## **Introduction**

The litter picker was born from a wish to help the environment, one piece of trash at a time. We decided on creating a robot that would wander around a specified area and look for trash using a depth camera and computer vision. Once it had located a piece of trash, it would move towards it and then push it towards a designated collection site. 

### **Problem Statement including original objectives**
* Be able to localize and move to different waypoints within a given area
* Capable of recognizing object to determine whether the object is trash
* Predict the trash's location using depth camera
* Collect the trash and bring to a collection site

### **Relevant literature**
* [Joseph Redmon, Santosh Divvala, Ross Girshick, Ali Farhadi: You Only Look Once: Unified, Real-Time Object Detection](https://arxiv.org/abs/1506.02640)
* [M. Bjelonic: YOLO ROS: Real-Time Object Detection for ROS](https://github.com/leggedrobotics/darknet_ros)
* [Eitan Marder-Eppstein: ROS Navigation Stack](http://wiki.ros.org/navigation)

## **What was created**

### **Technical Description, illustrations**

In the summary of Nodes, Modules, and External Packages we show the corresponding tables with the names and descriptions of each file. Afterwards, we go into a more detailed description of some of the more important files within the program. 

### **Summary of Nodes, Modules, and External Packages**
We created a variety of nodes, modules, and used some external packages while trying to determine the best architecture for the litter picker. The following tables (Table 1, 2, 3) show descriptions of the various files that are separated into their respective categories. Table 4 shows the parameters we changed so that the robot moves more slowly and smoothly while navigating using move_base.  

Table 1: Description of the ROS Nodes 
| Node | Purpose |
| ------ | ------ |
| </span>master.py</span> | Coordinates the various tasks based on the current task's state |
| </span>vision.py</span> | Processes and publishes computer vision related messages. It subscribes to the bounding box topic, the depth camera, and rgb camera and publishes a Trash custom message that will be used by the Task classes to determine what to do. |

Table 2: Description of our python modules used by the above nodes
| Modules | Purpose |
| ------ | ------ |
| <span>task.py</span> | Contains the interface which will be implemented by the other classes. |
| <span>navigation.py</span> | Contains the navigation class, which uses the waypoints state to navigate using the AMCL navigation package and updates the waypoints. |
| <span>rotation.py</span> | Includes the rotation class that publishes cmd_vel for the robot to rotate and look for trash. |
| </span>trash_localizer.py</span> | Has the trash localizer class, which publishes cmd_vel to the robot depending on information received from the vision node. |
| </span>state.py</span> | Holds the current state of the robot that is being passed around between the classes.|
| </span>utils.py</span> | Provides some helper functions for the rest of the modules. |
| </span>topics.py</span> | A single place to store all the subscribed topics name as constants|
| </span>trash_class</span> | Contain the yolo classes that are considered trash|

Table 3: External packages
| Package | Purpose|
| ----- | ----- |
| darknet_ros | Uses computer vision to produce bounding boxes around the trash. |
| navigation stack | Navigation stack for navigation and obstacle avoidance.  |
| usb_cam | astra rgb camera driver |

Table 4: Parameters changed within the navigation stack file: dwa_local_planner_params_waffle.yaml
| Parameter | Value |
| ------ | ------ |
| max_vel_theta |  0.5 |
| min_vel_theta | 0.2 |
| yaw_goal_tolerance | 0.5 |

### **Trash Custom Message**

To simplify what information is being published and subscribed to, we created a custom message called Trash which includes three variables. A boolean called has_trash which indicated whether or not there is trash present. A float called err_to_center which returns a number that indicates how far from the center to the bounding_box center is. And a boolean called close_enough which will be true if we believe that the trash is at a reasonable distance to be collected, otherwise it will be false. These are used in the Rotation and Trash_localizer classes to publish the appropriate cmd_vel commands. It is under the message topic **litter_picker/vision**.  

```
bool has_trash
float32 err_to_center
bool close_enough
```
### **<span>vision.py</span>**

The vision node has several methods and callback functions to process the messages from the various topics it subscribes to and then publish that information as a Trash custom message. 

### **darknet_ros**

![image info](darknet_ros.png)
Image 1: Display of the darknet_ros putting a bounded box around the identified object

darknet_ros is the computer vision package that looks for the types of trash specified in the trash_class.py. We want to be able to find things like bottles, cups, forks, and spoons that are lying around. This package integrates darknet with ros for easier use. We then subscribe to the ```darknet_ros/bounding_boxes``` topic that it publishes to and use that to to determine where the trash is. 

### **Task interface and its children**

![image info](task_class.png)

Image 2: A diagram to show the setup of the Task interface and its children classes

*The Task interface* contains its constructor and two other methods, start() and next(). When a Task instance is initiated, it will take in a state in the form of the LitterPickerState class which contains a list of waypoints and the current waypoint index. The start() method is where the task is performed and the next() method is where the next Task is set. We created three Task children classes called Navigation, Rotation, and TrashLocalizer which implemented the Task interface. 

*The Navigation class* communicates with the navigation stack. In start(), it chooses the current waypoint and sends it to move_base using the move base action server. If the navigation stack returns that it has successfully finished, the has_succeed variable of the Navigation class will be set to true. If the navigation stack returns that it has failed, has_succeed will be set to false. Then when the next() method is called and if has_succeed is true,  it will update the waypoint index to the next one and return a Rotation class instance with the updated state. If has_succeed is false, it will return an instance of the Navigation class and attempt to go to the same waypoint again. 

*The Rotation class* publishes a cmd_vel command for a certain amount of time so that the robot will rotate slowly. During this rotation, the camera and darknet_ros are constantly looking for trash. The Rotation class subscribes to the Trash custom message. In the start() method, it looks at the has_trash variable to check if there is any trash present while it is rotating. If there is, then it will stop. In the next() method, it sets has_trash is true, then it will set the next task instance to be the TrashLocalizer task. Otherwise, it will set it to a Navigation task. 

*The TrashLocalizer class* also subscribes to the Trash custom message. In the start() method, it publishes cmd_vel commands to move towards the trash and collect it in the plow. It uses the boolean close_enough to determine how far it has to go before before stopping. Then it sees if it sees that there is still a bounding box, it will use the trap_trash() method to move forward for a specified amount of time. 

### **<span>master.py</span>**

The master node has a class called LitterPicker which coordinates the Tasks as the robot executes them. There is a constructor, an execute() method, and a main method within it. It also includes the LitterPickerState class which contains a list of all the waypoints and an index for the current waypoint. The constructor takes in a Task and a LitterPickerState and two variables are initialized based on those two: current_task and state. 

In the execute() method, there are two lines, as you can see below. The first line starts the execution of the current_task and the second line calls the next function of the current_task so that it can decide which Task should be executed next. 
```
def execute(self):
        self.current_task.start()
        self.current_task = self.current_task.next()
``` 

The main function initializes the node, creates an instance of the LitterPickerState using the waypoints file, creates an instance of the LitterPicker class using that state and a NavigationTask class. Then in the while not rospy.is_shutdown loop, it runs the execute() function. 

### **Guide on how to use the code written**

If you wish to change the map file to better suit your location, then you can edit the map file (in <arg name = "map_file"...>) inside the litter_picker.launch, as well as change the initial position of the robot. 

```
<include file="$(find litter_picker)/launch/navigation.launch">
      <arg name="initial_pose_x" default="-1.1083"/>
      <arg name="initial_pose_y" default="-1.6427"/>
      <arg name="initial_pose_a" default="-0.6"/>
      <arg name="map_file" default="$(find litter_picker)/maps/new_map.yaml"/>
</include>
```

You can also edit the new_map_waypoints.txt file in the waypoints folder to match the coordinates that you would like for the robot to patrol around that should be in the following format: 

```
-4.211, 1.945, 0 
-4.385, 4.937, 0
-4.38, 4.937, 0
-7.215, 1.342, 0
```

When you ssh onto the robot, you will need to run ```bringup```.

You will also need to run ```rosrun usb_cam usb_cam_node``` onboard the robot as well to initialize the depth camera's rgb stream. 

Then enter ```roslaunch litter-picker litter_picker.launch```. The program should launch with a few windows: rviz and YOLO camera view.  

## **Story of the project**

There were many challenges that we faced with this project, but let’s start from the beginning. At the start, we had tried to divide the work more so that we can create functionality more efficiently. Peter worked on finding a computer vision package that could create and creating the corresponding logic within the rotation node as well as creating the master node and implementing action lib when we had been using it. Veronika worked on creating the map and integrating the navigation stack into the program, installing the depth camera, and using it to localize the trash. Once we created most of the logic and were trying to get things to work, we became masters of pair programming and debugging together and for the rest of the project, we mainly worked together and constantly bounced ideas off each other to try and solve problems.  

### **master node and navigation stack**

In the beginning, we had started with a simple state system using the topic publisher and subscriber way of passing information between nodes, however, that quickly became very complicated and we thought we needed a better way to keep track of the current state. We chose to use the actionlib server method which would send goals to nodes and then wait for results. Depending on the results, the master node would send a goal to another node. We also created a map of the area that we wanted to use for our project. Then we incorporated the AMCL navigation system into our state system. We decided that moving from waypoint to waypoint would be the best way to obtain the benefits of AMCL such as path planning and obstacle avoidance. This was a system that we kept throughout the many changes that we underwent throughout the project. 

However, when we tested it with the ROS navigation stack, things did not work as expected. The navigation would go out of control as if something was interfering with it. After some testing, we concluded that the most likely cause of this was the actionlib servers, however we were not completely sure why this was happening. Perhaps the waiting for results by the servers was causing some blocking effect to the navigation stack. 

We ended up completely changing the architecture of our program. Instead of making each task as a separate node, we made them into python classes where the master node can initiate different actions. The reason for this change is that we felt that the tasks does not have to be running in parallel, so a more linear flow would be more appropriate and resource-efficient. This change greatly simplified our logic and also worked much better with the navigation stack. We also edited some parameters which has been previously described within the navigation stack that allowed for the robot to move more smoothly and slowly. 

### **computer vision**
Initially, we had wanted to train our computer vision model using the TACO dataset. However, there was not enough data and it took too long to train so we ended up choosing the pretrained yolo model that has some object classes such as bottle, cup, fork, and spoon that could be identified as trash. The YOLO algorithm and integrated with ros through the darknet_ros package as mentioned above. We had to make some changes to the darknet_ros files so that it could take in CompressedImage messages. 

### **depth camera**
Close to the beginning of the project, we had set up the [Astra depth camera](https://campus-rover.gitbook.io/lab-notebook/faq/astra_pro_depth_camera_setup). The story of the depth camera is long and arduous. We had begun the trash_localizer node by attempting to calculate a waypoint that the navigation stack could move to by determining the distance to the identified object and then calculating the angle. From there, we used some basic trigonometry to calculate the amount that needed to be traveled in the x and y directions. However, we found that the angle determined from the depth camera did not correspond with reality and therefore the waypoint calculated was not reliable. The next method we tried was to use the rotation node and stop the rotation when the bounding box was in the center and then use a pid controller to move towards the trash using the distance returned from the depth camera. There were several problems with that such as the bounding box being lost, but in general, this method seemed to work. Since we did most of this testing in Gazebo due to the LiDAR on Alien being broken, we didn't realize that we actually had an even larger problem at hand... 

When we started to test the depth camera on the actual robot in a more in depth manner, we realized that it was not returning reliable data with the method that we had chosen. We started to explore other options related to the depth stream but found that we did not have enough time to fully research those functionalities and integrate it in our program. This is when we decided pivot once again and not use the depth part of the depth camera. Instead we used the position of the bounded box in the image to determine whether the object was close enough and then move forward. 

### **plow system**
There was a robotic arm in the original plan, however, due to the physical limitations of both the arm, the robot, and time constraints, we ended up deciding to use a snow plow system instead. We had several ideas of having a movable one, but they were too small for the objects that we wanted to collect so we created a stationary one out of cardboard and attached it to the front of the robot. It works quite well with holding the trash within its embrace. 

![image info](alien_with_plow.jpg)
Image 3: Alien with the Astra depth camera on top and the plow setup below posing for a photo

### **personal assessment**
This project was much more challenging than we had anticipated. Training our own data and localizing an object based on an image and bounded box turned out to be very difficult. Despite the many problems we encountered along the way, we never gave up and persisted to present something that we could be proud of. We learned many lessons and skills along the way both from the project and from each other. One lesson we learned deeply is that a simulator is very different from the real world. Though for a portion of the time, we could only test simulator due to a missing LiDAR on our robot, it still showed the importance of real world testing to us. We also learned the importance of reasoning through the project structure and logic before actually writing it. Through the many frustrations of debugging, we experienced exciting moments of when something worked properly for the first time and then continued to work properly the second and third times. 

In conclusion, we were good teammates who consistently showed up to meetings, communicated with each other, spent many hours working side by side, and did our best to complete the work that we had assigned for ourselves. We are proud of what we have learned and accomplished even if it does not exactly match our original idea for this project. 