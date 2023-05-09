---
author: Aiden Dumas
description: 
date: may-2023
---
# Simplifying Lidar
## Author: Aiden Dumas

Using Lidar is fundamental for a robot’s understanding of its environment. It is the basis of many critical mapping and navigation tools such as SLAM and AMCL, but when not using a premade algorithm or just using Lidar for more simple tasks, some preprocessing can make Lidar readings much more understandable for us as programmers. Here I share a preprocessing simplification I use to make Lidar intuitive to work with for my own algorithms.

It essentially boils down to bundling subranges of the Lidar readings into regions. The idea for this can be found from: 
[Article](https://github.com/ssscassio/ros-wall-follower-2-wheeled-robot/blob/master/report/Wall-following-algorithm-for-reactive%20autonomous-mobile-robot-with-laser-scanner-sensor.pdf “On Github”)
	The simple preprocessing takes the 360 values of Lidar’s range method (360 distance measurements for 360 degrees around the robot) and gives you a much smaller number of values representing more intuitive concepts such as “forward”, “left”, “behind”. The processing is done by creating your subscriber for the Lidar message:
scan_sub = rospy.Subscriber(‘scan’, LaserScan, scan_cb)

Our callback function (named scan_cb as specified above), will receive the message as such:
def scan_cb(msg):

And depending on how often we want to preprocess a message, we can pass the 360 degree values to our preprocessing function:
ranges = preprocessor(msg.ranges)

In my practice, calling this preprocessing step as often as the message was received by the subscriber didn’t have any slowdown effect on my programs. The processor function itself is defined as follows:
```Python
def preprocessor(all_ranges):
	ranges = [float[(‘inf’)] * 20
	ranges _index = 0
	index = -9
	sum = 0
	batch = 0
	actual = 0
	for i in range(360):
		curr = all_ranges[index]
		if curr != float(‘inf” and not isnan(curr):
			sum += curr
			actual += 1
		batch += 1
		index += 1
		if batch == 18:
			if actual != 0:
				ranges[ranges_index] = sum/actual
			ranges_index += 1
			sum = 0
			batch = 0
			actual = 0
	return ranges
```
Essentially all we do here is take the average of valid values in a region of the 360 degree scan and put it into our corresponding spot in our new abbreviated/averaged ranges array that will serve as our substitute for the more lofty 360 degree value msg.ranges (passed to all_ranges). Something implicit to note here is that in the first line of this function we choose how many regions to divide our Lidar data into. Here 20 is chosen. I found that the size of these regions work well for balancing the focus of their region while not compromising being too blind to what lies in a region’s peripheral. This 20 then computes 360/20 = 18 which is our batch size condition (number of data points per region) and where we start our index 0 - 18/2 = -9 which is the offset we use to start to get a range for the “front” region (makes sense looking at the figure in the pdf linked). 
With this base code, any modifications can be made to suit more specific purposes. The averaging of the regions could be replaced by finding a max or min value for example. In one of my projects, I was focused on navigation in a maze where intersections were always at right angles, so I only needed 4 of these regions (forward, backward, left, right). For this I used the following modification of the code with a helper function listed first again using 18 as the number of values sampled per region:
(Note here for reference that in the original msg.ranges Front is at index 0, Back at 180, Left at 90, and Right at 270 (implying the Lidar data is read counterclockwise))
```Python
def averager(direction_ranges):
	real = 0
	sum = 0
	for i in ranges:
		if i != float(‘inf’) and not isnan(i):
			real += 1
			sum += 1
	return float(‘inf’) if real == 0 else sum/real
```
```Python
def cardinal_directions(ranges):
	directions = {“Front”: float(‘inf’), “Back”: float(‘inf’), “Left”: float(‘inf’), “Right”: float(‘inf’)}
	plus_minus = 9
	Front = ranges[-plus_minus:] + ranges[:plus_minus]
	Backward = ranges[180 - plus_minus:180+plus_minus]
	Left = ranges[90 - plus_minus:90+plus_minus]
	Right = ranges[270-plus_minus:270 + plus_minus]
	directions_before_processed = {“”Front”: Front, “Back”: Back, “Left”: Left, “Right”: Right}
	for direction, data in directions_before_processed:
		directions[direction] = averager(data)
	return directions
```
Hopefully this code or at least this idea of regional division helps simplify your coding with Lidar.
