---
title: How do I create a ROS UI with TkInter?
order: 10
status: tophit
category: python
description: How to use the TKInter package for Ros Tools
authors: Brendon Lu and Benjamin Blinder
---
# Creating a Tkinter GUI and communicating w/ ROS nodes

Brendon Lu and Benjamin Blinder

Make sure you have the following packages imported: `tkinter` and `rospy`. The tkinter module is a basic and simple, yet effective way of implementing a usable GUI.

```
import tkinter as tk
import rospy
```

Because tkinter has a hard time recognizing functions created at strange times, you should next create any functions you want to use for your node. For this example, I recommend standard functions to publish very simple movement commands.

```
def turn_function():
    t=Twist()
    t.angular.z=0.5
    cmd_vel.publish(t)

def stop_function():
    t=Twist()
    t.angular.z=0
    cmd_vel.publish(t)
```

You still need to initialize the ros node, as well as any publishers and subscribers, which should be the next step in your code.

```
rospy.init_node("tkinter_example")
cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
```

Now it is time to create a basic window for your GUI. First, write `[window name] = tk.Tk()` to create a window, then set the title and size of the window. Not declaring a window size will create a window that adapts automatically to the size of whatever widgets you create on the window.

```
root=tk.Tk()
root.wm_title("Test TKinter Window")
root.geometry("250x250") #set size of window
```

The step is to populate your window with the actual GUI elements, which tkinter calls "Widgets". Here we will be making two basic buttons, but there are other common widget types such as the canvas, entry, label, and frame widgets.
```
turn_button=tk.Button(
    root,
    text="turn",
    bg="grey",
    command=turn_function
)
stop_button=tk.Button(
    root,
    text="stop",
    bg="grey",
    command=stop_function
)
```

And now that you have created widgets, you will notice that if you run your code, it is still blank. This is because the widgets need to be added to the window. You can use "grid", "place", or "pack" to put the widget on the screen, each of which have their own strengths and weaknesses. For this example, I will be using "pack".

```
turn_button.pack()
stop_button.pack()
```

And now finally, you are going to run the tkinter mainloop. Please note that you cannot run a tkinter loop and the rospy loop in the same node, as they will conflict with each other.

```
root.mainloop()
```

To run the node we have created here, you should have your robot already running either in the simulator or in real life, and then simply use rosrun to run your node. 
Here is the code for the example tkinter node I created, with some more notes on what different parts of the code does

```
#!/usr/bin/env python
import tkinter as tk #import tkinter
from geometry_msgs.msg import Twist
import rospy

#Put callbacks here

#Put all standard functions here

#Put functions that are called by pressing GUI buttons here
def turn_function():
    t=Twist() #creates a twist object
    t.angular.z=0.5 #sets the angular velocity of that object so the robot will turn
    cmd_vel.publish(t) #publishes the twist

def stop_function():
    t=Twist()
    t.angular.z=0
    cmd_vel.publish(t)


#initialize rospy node, publishers, and subscribers
rospy.init_node("tkinter")
cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1) #standard publisher to control movement


#initialize tkinter window
root=tk.Tk() #initialize window
root.wm_title("Test TKinter Window") #set window name
root.geometry("250x250") #set size of window

#create widgets
turn_button=tk.Button( #creates a button
    root, #sets the button to be on the root, but this could also be a frame or canvas if you want
    text="turn", #The text on the button
    bg="green", #The background of the button, tkinter lets you write out color names for many standard colors, but you can also use hex colors or rgb values
    command=turn_function #command will tie a previously defined function to your button. You must define this function earlier in the code.
)
stop_button=tk.Button(
    root,
    text="stop",
    bg="red",
    command=stop_function
)

#adding the buttons to the window
turn_button.pack() #pack will simply stack each widget on the screen in the order they were packed, but that can be changed with various arguments in the pack method. Please check the tkinter documentation to see more options.
stop_button.pack()

#Tkinter Mainloop
root.mainloop()
```
Although this code does technically move the robot and with some serious work it could run a much more advanced node, I do not recommend doing this. I would recommend that you create two nodes: A GUI node and a robot node. In the GUI node, create a custom publisher such as `command_pub=rospy.Publisher('command', Twist, queue_size=1)` and use this to send messages for movement to the robot node. This way, the robot node can handle things like LiDAR or odometry without issues, since the tkinter update loop will not handle those kinds of messages very efficiently.


Overall, tkinter is an industry staple for creating simple GUIs in Python, being fast, easy to implement, versatile, and flexible, all with an intuitive syntax. For more information, check out the links below.

[basic tkinter tutorial](https://www.geeksforgeeks.org/python-gui-tkinter/)

[basic tkinter video tutorial](https://www.youtube.com/watch?v=itRLRfuL_PQ)

[documentation](https://docs.python.org/3/library/tk.html)

[variable text](https://stackoverflow.com/questions/2603169/update-tkinter-label-from-variable)

[images using photoimage](https://www.pythontutorial.net/tkinter/tkinter-photoimage/)
