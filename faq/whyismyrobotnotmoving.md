---
title: Why is my robot not moving?
author: Jeremy Huey
date: may-2023
status: new
type: FAQ
---

# Basic Start Guide 
Created by Jeremy Huey 05/04/2023

### Basic Trouble Shooting:
Are you connected to roscore? 
Ensure that roscore is running either in simulation (create a terminal window running '$ roscore') or have run '$ bringup' on an real robot. 

If you are onboard a real robot, the window you ran `bringup` in should say [onboard]. 
To check that you have the connection, type `$ rostopic list`. 
This should output a list of topics the system can see. 

Is the battery dead or the hardware not working? 
Check to see if the battery has died/beeping or if any cables have come loose.

To check to see if the robot is getting commands, you can type in a single cmd_vel message. 
```bash
$ rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```
You must ensure there is a space after things like `x: `.
Info here: https://answers.ros.org/question/218818/how-to-publish-a-ros-msg-on-linux-terminal/

Let's say you make a new project and now you cannot find that package, or you try to run your launch file and it gives you the error that it cannot find that package.
To check or find out if you have the package you want you can do this: 
`$ roscd package_name`
This should auto complete. If it does not, you may not have the right package_name as designated in the package.xml. 
Other packages you may find will send you to opt noetic something. You can likely download these packages to your local somehow. 

### Getting onboard a robot
Follow the link here to get onboard a robot: https://campus-rover.gitbook.io/lab-notebook/faq/02_connect_to_robot

### How to understand and make a simple launch file: 
https://campus-rover.gitbook.io/labnotebook/faq/launch_file_create.md

### how to turn in a zip submission for class
TAR AND UNTAR ====
How to turn it in: 
On the VM/VNC, go to System Tools > File Manager. 
In a terminal window,  use to TAR the file:
```$ tar -czvf foldername.tar.gz foldername```

Then inside of the CODE window, you'll see the tar file appear in the vscode file manager. If you right click on a tar file, only this kind of file will seem to give the option to "Download". Do this to download to your computer. 

Then, go to your own PC's powershell/terminal. The UNTAR command is = -x.
```bash
$ cd .\Downloads\
$ tar -xvkf .\pa2.tar.gz 
```
Optionally also instead of downloading, you can also On the VM, go to the bottom menu, Internet, use Firefox or Chrome to log into your email or github or whatever to send that way.

### How to simple Git (and create package)
go to github.com and get a username and personal key. This key is important, save it somewhere safe where you can get access to it. When asked for a password, this is what you'll place in (as of 2023). 

On github.com, you should be able to create a repository do this. 
Then on your local computer or your CODE window go to the folder where you want to place your files. (In this class, as of 2023, you're recommended to place them in `$ cd ~/catkin_ws/src`).
Then copy the link to the github repo you just made, and run: `git clone your_link.git`.  Make sure you add .git to the end of it. You should now see a copy of it appear on your local computer. In the CODE window, you will see a circular arrow "Refresh Explorer" at the top of the File Explorer side tab. 

(Create Package)
To create a package run this: `$ catkin_create_pkg name_of_my_package std_msgs rospy roscpp`
replace name_of_my_package with the name you want for your package, eg. dance_pa.

The easiest thing to do when working on these group projects is to split the work up into separate areas/files/commented_segments so that you do not incur merge conflicts. Eg. Abby works on fileA.py, while you work on fileB.py. Or you both work on fileA.py but you comment out a segment line 1-50 for Abby to work, while you work on line51-100. 

Once you've made some changes, do this: 
(Be in the folder you want to make changes to, eg. your package folder.)
```bash
git add . //Alternatively, you can specify a folder or file here.
git commit -m "some commit message"
git pull // updates your local to take any new changes. 
git push //pushes your changes to origin/github. 
```

### Using the Python Object/Class structure
If you use a Python class and get an undefined error, you most likely forgot to make that variable a self variable. eg: 
linear_speed is undefined. change to: self.linear_speed. 
The same applies to functions and callback cb functions, reminder to place these inside the Class classname: section and to write them as: 
```python
import rospy
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Bool, String # if you get errors for creating your own publisher, see if you have the right type here.

Class Classname:
    def __init__(self):
        rospy.init_node('main')
        self.linear_speed = 0.0
        self.rate = rospy.Rate(30) # reminder: this must be self. too. 
        self.twist = Twist()
        # CORE pub subs ====
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
        self.state_sub = rospy.Subscriber("/state", String, self.state_cb) #self.state_cb

    def state_cb(self, msg):
        self.state = msg.data 
        # when accessing the information, you most likely will need to access .data

    def funcname(self):
        self.linear_speed = 0.3 # This is where you might see the error if you forgot self.
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = 0.0
        self.cmd_pub.publish(self.twist)

    def run(self):
        while not rospy.is_shutdown():
            self.funcname()
            self.rate.sleep()

if __name__ == '__main__':
c = Classname()
c.run()
# EOF
```

### More commands for running gazebo simulations: 
https://campus-rover.gitbook.io/labnotebook/faq/handy-commands.md