### Using Rviz Markers

### Author: Jonah Jakab

Rviz markers are a useful tool for displaying custom information to the user of a program. This tutorial teaches you how to use them.

### What are Rviz markers?

Markers are visual features that can be added to an Rviz display. They can take many forms - cubes, spheres, lines, points, etc.

Rviz markers are published as a message of the Marker type. First, you want to create a publisher. It can publish on any topic, but the message type should be Marker. Next, you should fill out the fields of the Marker message type. Notably, you need to define the frame_id. You can either use the default "map" frame, or construct a new one that fits your requirements.

Now, you can publish the Marker message. Run your project, then open a new terminal tab and open Rviz. You should see an 'Add' option at the bottom left of the window. Click it, then scroll down and select Marker and select 'Ok'. Your marker should appear on the display. If you used a new frame as the frame_id, you need to select it under the Marker you added before it will show up. 

### Helpful Links

http://docs.ros.org/en/melodic/api/visualization_msgs/html/msg/Marker.html
