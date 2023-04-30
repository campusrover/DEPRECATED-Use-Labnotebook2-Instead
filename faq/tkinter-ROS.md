---
title: ROS and TkInter
order: 10
status: top
category: python
description: How to use the TK Inter package for Ros Tools
author: Brendon Lu
---
# Creating a Tkinter GUI and communicating w/ ROS nodes

Brendon Lu

Make sure you have the following packages imported: ==tkinter==, ==rospy==. The tkinter module is a basic and simple, yet effective way of implementing a usable GUI.

```
import tkinter as tk
import rospy
```

Immediately after importing the necessary packages, initialize a rospy node and publishers/subscribers, which can be used to communicate and pass userinput data to other nodes through respective topics. 

```
rospy.init_node("tkinter")
speed_pub = rospy.Publisher('[topic]', [datatype], queue_size = [queue size])
heat_sub = rospy.Subscriber('[topic]', [datatype], [callback fxn])
```

To create a basic gui, first run ```[root name] = tk.Tk()``` in order to create a root window. In order to add elements to your gui/window, create a canvas and grid using 

```
[canvas name] == tk.Canvas([root name], width = [width], height = [height])
[canvas name].grid(columnspan = [# of columns]
```

Some other basic functions to get you started include: ```tk.Label([root name] ... )``` for showing text or variables, and ```tk.PhotoImage(file = [img PATH])```. Note that to add an image, first you need to pass it into tk using PhotoImage, then run Label in order to show it. Overall, tkinter is an industry staple for creating simple GUIs in Python, being fast, easy to implement, versatile, and flexible, all with an intuitive syntax. For more information, check out the links below.

[basic tkinter tutorial](https://www.youtube.com/watch?v=itRLRfuL_PQ)

[documentation](https://docs.python.org/3/library/tk.html)

[variable text](https://stackoverflow.com/questions/2603169/update-tkinter-label-from-variable)

[images using photoimage](https://www.pythontutorial.net/tkinter/tkinter-photoimage/)
