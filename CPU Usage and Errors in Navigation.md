###### Huaigu Lin 11/13/2018

---

# CPU Usage and Errors in Navigation:

During navigation I've run into a lot different errors and warnings. I copied some of the frequent ones here:

    [ WARN] [1542144731.816164220]: Costmap2DROS transform timeout. Current time: 1542144731.8160, global_pose stamp: 1542144730.2532, tolerance: 1.5000
    [ WARN] [1542144731.816266907]: Could not get robot pose, cancelling reconfiguration
    [ WARN] [1542144732.472418469]: Unable to get starting pose of robot, unable to create global plan


    [ WARN] [1542144726.841978275]: Map update loop missed its desired rate of 1.0000Hz... the loop actually took 1.2625 seconds
    [ WARN] [1542144737.244639028]: Map update loop missed its desired rate of 5.0000Hz... the loop actually took 0.2093 seconds
    [ WARN] [1542144737.244865990]: Control loop missed its desired rate of 3.0000Hz... the loop actually took 0.8109 seconds

    [ERROR] [1542146023.217567014]: Extrapolation Error: Lookup would require extrapolation into the future.  Requested time 1542146022.709356378 but the latest data is at time 1542146021.995298140, when looking up transform from frame [odom] to frame [map]
    [ERROR] [1542146023.217646860]: Global Frame: odom Plan Frame size 1889: map
    [ WARN] [1542146023.217688426]: Could not transform the global plan to the frame of the controller
    [ERROR] [1542146023.217720997]: Could not get local plan

What I found in common from these problems is that they all have something to do with information loss in the update cycle of different parts of navigation. And this could be caused by computer not having enough processing power for the desired update frequency, which is actually not high at all, like the 1.0000Hz and 5.0000Hz in the warning messages above.

Then I found that the laptop's both CPUs' usage is nearly at 100% during navigation. I checked each node one by one. Rviz is very CPU-hungry, when running navigation and rviz together, the CPUs will have nearly 100% usage. But we can avoid using rviz once we have fiducial working. Besides Rviz, several custom nodes made by us are also very CPU-hungry.

For now, we are using Dell 11 inch laptop as the onboard computer for Turtlebot. The situation might not be the same if more powerful devices are used. However, generally speaking, in our custom nodes we should avoid pulishing/updating with a frequency that's too high.

Also please remember to check CPU usage if you find these errors and warnings again during navigation.
