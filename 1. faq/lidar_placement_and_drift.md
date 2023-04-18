# Issues with Lidar placement and callibration

## Source

A long thread on a Robotics list which yielded a lot of sophisticated and important insights.

### Question

When it comes to mounting a lidar on a robot, what are some of the considerations? My robot runs ROS and I've mounted the Lidar and set up the various transforms ("tf"s) correctly  --  I believe. 

When I display the Lidar data while moving the robot forward and backwards, it is fairly stable. In other words, I think that the odometry data reporting the motion of the robot, correctly "compensates" for the movement so that the lidar data as displayed stays more or less in the same place.

However when I turn the robot in place, the Lidar data drifts a little and then compensates somewhat. I also was able to create a decent Slam map with this setup. Although not as reliable as I would like.

It turns out that the place where the lidar is mounted is near the casters, and as a result, in an in place turn, the lidar doesn't simply rotate in place, but instead moves a lot. Because, it is not over the center of rotation, in fact its as far away from it as it could be.

My question is: does the math which is used to compute the lidar data during an in place turn compensate for the placement. Does it use the various transforms (which reflect the relative placement of the lidar) or does it just use the odometry of the robot as a whole?

(Hard to explain, but I hope you follow)

Yes, I could rebuild my robot to do the more intuitive thing and place the lidar over the center of turn. But I would want to avoid all that work if it it's not really needed.

### Answer 1

TF should negate any physical offsets - but it really depends on the SLAM package using TF correctly. The widely used ones (karto, cartographer, slam_toolbox) all should do this.

That said, you might also have timing issues - which TF won't handle (since the timing reported to it will be wrong!). If your odometry or laser are lagging/etc relative to one another, that could cause some issues.

### Answer 2

Note that the effect is visible without slam. In rviz as the robot moves forward, the "image" of the wall or obstacle stays more or less in place relative to the odom. However if I rotate or turn, then the image moves around and when the rotation stops it settles back down.

Is the timing problem exacerbated by having the lidar offset from the center of rotation? I could imagine that if the lidar is over the center of rotation then the timing is less critical... but I'm just going by gut feel. 

When you watch the physical robot rotate in place, the lidar follows a pretty wide circular arc around the center of rotation. You can easily imagine that this causes havoc with the calculations that produce the /scan topic.

My big question is, is it worth rearranging things to bring the lidar back to the center of rotation. I think my answer is yes.

### Answer 3

Almost every real robot out there has the laser offset from the center of the robot, so it's pretty much a solved problem (with TF and proper drivers giving correct timestamps).

If the timing is wrong, the change in offset really doesn't help much, since small angular errors result in LARGE x/y offsets when a scan point is several meters away from a robot (and this will be the majority of your error, far larger than the small offset from your non-centered laser).

Your comment that "when the rotation stops it settles back down", really makes me think it is a timing related issue. One way to get a better idea is to go into RVIZ, and set the "decay time" of the laser scanner display to something like 60 seconds. Then do your driving around - this will layer all the scans on top of each other and give you a better idea of how accurate the odometry/laser relationship is. In particular - if you just do a bit of rotation, does the final scan when you stop rotating line up with the first scan before you started rotating? If so, it's almost certainly timing related.

### Answer 4

There are a few parts to the answer. And I’ll mostly deal with what happens in a two wheel, differential drive robot.

It’s not clear what you mean by LIDAR data drifting. If, as I suspect, you mean visualizing the LIDAR with rviz, then what you may be seeing is the averaging effect. In rviz, the display component for LIDAR has an option to include the last “N” points, as I recall, showing up as the Decay Time parameter for the plugin. So, the LIDAR data is likely just fine, and you’re seeing the effect of history disappearing over time. Crank down the Decay Time parameter to, say, 1 and see if things look better.

Otherwise, the problem with LIDAR usually only shows up in something like a SLAM algorithm, and then it’s because the odometry and LIDAR disagree with each other. And this usually has nothing to do with LIDAR, which is typically pretty truthful.

In SLAM, the algorithm gets two competing truths (typically), odometry and LIDAR. And, besides the normal problems with wheel odometry, which have been bitched about repeatedly in this group, there is also the geometry problem that typically shows up in the motor driver.

Whether using gazebo (simulation) or real motor drivers, both typically rely on knowing the distance between the two wheels, and the wheel circumference. With those two constants and the wheel encoder counts (and some constants such has how many encoder ticks there are per revolution), simple math computes how far the left and right wheels moved during some small change in time, which allows computing the angle of revolution for that same time period. 

You can really see when the circumference is wrong by moving the robot back and forth with SLAM happening and the resulting map showing. If you find that the map shows a wall while the robot is moving and the LIDAR shows that what it thinks is the wall changes as the robot moves, then you have the wheel circumference constant wrong. 

That is, with a wall ahead of the robot, move the robot in a straight line forward. The map wall will stay in place in the rviz map display, but if the LIDAR dots corresponding to the wall move closer or farther than the map wall while movement is happeningl, the wheel circumference constant is wrong. Just adjust your wheel circumference until moving forwards and backwards shows the LIDAR points corresponding to the wall ahead of the robot staying atop the map’s wall.

An error in the distance between the two wheels shows up as an error when the robot rotates. For example, if you have a wall ahead of the robot again and you rotate in place, if you see the wall stays stable in the rviz map display but the LIDAR points corresponding to the wall change the angle of the wall as the robot rotates in place, then you have the wheel circumference wrong. Change the circumference value, usually in the URDF and often repeated in other YAML files as well, and try the experiment again.

My saying that the wall in the map stays stable also implies that in rviz you are setting your global frame to the map frame.

When you get both the circumference and inter wheel distance correct, the SLAM map will closely match the LIDAR points even while the robot moves. It won’t be perfect while the robot moves, partly which I’ll explain next, but it does mean that when the robot slows down or stops, SLAM will fix the error very quickly.

I’m sure people will talk about the time stamp of data as well. In ROS, sensor data is given a time stamp. This should faithfully record when the sensor reading took place. If you have multiple computers in your robot, the sense of time on all computers must match within, at most, say a couple of milliseconds. Less than a millisecond discrepancy between all the computers is better. 

Note that if you are getting sensor data from an Arduino over, say, ROS serial, you will have to deal with the getting the time stamp adjusted before the sensor data gets posted as a ROS message. Since the Arduino didn’t tag the time, and the Arduino is unlikely to know the time to high accuracy, you have to figure out the latency in reading the data over a serial port, and the serialization, deserialization and message conversion delays.

When various components work with sensor data, they often predict what the sensor values will be in some future after the readings actually took place. That’s because the sensor values are coming in with some delay (lag), and the algorithms, especially SLAM, want to predict the current state of the robot. SLAM is monitoring the commanded movement of the robot (the cmd_vel sent to the motors) and when odometry and LIDAR data comes in, it needs to predict where the odometry and LIDAR points would be NOW, not when they were last observed. If the timestamp of the sensor data is wrong, the guess is wrong and SLAM gets very sloppy.

To cope with bad time and unexpectant latencies, especially the notorious big scheduling delays in a Linux system (think of what preemptive time sharing really does to your threads when they think they know what the current time is), one simple filter ROS usually provides is to ignore any data that is too “old”. There are configuration parameters, for instance, for SLAM that say to just ignore data that is older than some relatively small time from now (on order of, say 10 milliseconds might by typical).

This is especially a problem with multiple computers in a robot, but even with a single computer. Remember that Linux is trying to run on order of 100 threads at the same time. On a Raspberry Pi, it’s tying to give each of those 100 threads the illusion that they are running in real time by giving them a tiny slice of time to run before going on to the next thread. A thread can ask for “what time is it now” and as soon as it gets the answer, before the very next instruction in that thread executes, tens or hundreds of milliseconds may have gone by.

Finally, as for positioning of the LIDAR, ROS provides a marvelous mathematical modeling package in the TF system. If you have pretty good modeling of fixed components, like LIDARs mounted solidly to the robot and you get the offsets correct to within a millimeter or two from 0,0,0 in the base_link frame of reference, you needn’t worry about where you put the LIDAR. Make sure you correctly account for all the rotations as well, though. For instance, with the casters on my robot, the plate holding the LIDAR isn’t exactly level with the floor, and my URDF needs to include the rotation of the LIDAR.

My LIDAR is NOT mounted at the center of rotation and rviz shows things just fine. And I’m about to replace my single LIDAR with 4 LIDARS mounted at the 4 corners of my robot’s frame at different heights. It will be no problem for rviz and SLAM to deal with this.

Finally there is an issue with computation speed and robot speed. The faster you robot moves, the faster it needs to get sensor data. SLAM works best when it gets, say, 20 to 100 sensor readings a second for a robot moving on order of a couple of meters per second. SLAM wants to robot to not have moved very far before it does its thing between sensor frame readings. If your odometry and LIDAR readings are coming in at, say, 5 frames per second, and your computer is slow and loaded (such as trying to do everything on a single Raspberry Pi), and you robot is moving at the equivalent of a couple of miles per hour, all bets are off.

## Answer 5

Actually, This is a very good question because 

Some robots use multiple LIDAR units and obviously only one of them can be the center point of a turn. and

If the robot used four wheel "Ackerman steering" (as almost every car) the center of rotation is not within the footprint of the robot. but on the ground some distance to the side of the robot.
So mounting the Lidar in the center of rotation is physically impossible in the above cases.   I hope the software "works".

It could be that you have wheel slip.  In fact I'd guess this is the problem and you should not be using raw odometry but rather the ROS Robot_Localization package.    This package will "fuse" odometry, IMUs, GPS, Visual Odometry and other sources to get the robot's position and orientation and account for wheel slip and other sensor errors. 
http://docs.ros.org/en/noetic/api/robot_localization/html/index.html

## Answer 6

The "drift" you describe is caused by the fact that, as the robot rotates, the lidar is not sampled at exactly the same spot.    This causes flat surfaces to appear to bend, but only while the robot is rotating.

Chris Netter has documented this extensively and written code that corrects for this effect for his ROS robot,  which I can't seem to locate at the moment.    

But it won't be fixed by moving the lidar to the center of rotation.  It is a result of the fact that both the robot and the lidar are rotating while sampling.  It's not seen when the robot is going straight, hence the "setteling down" which you mention.

## Answer 7

But Pito says he is using software to compensate for the movement of the LIDAR during a turn.   In theory, what he did should work.  The ROS tf2 package should be calculating the LIDAR real-world location as it swings around the center of rotation and interpolating the position at the time the LIDAR scan was processed.

He said he was using this already. For more on tf2 see http://wiki.ros.org/tf2

My guess is that the odometry data that drives the transforms that tf2 uses not accurate.   Odometry assumes a perford differential drive system and none are perfect.   

One way to test my guess would be to run the robot in a simulation where odometry is in fact "perfect" and see if the problem goes away
