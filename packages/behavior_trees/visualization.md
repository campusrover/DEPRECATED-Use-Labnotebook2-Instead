# Visualization of the Behavior Tree

Each tree that you build will be able to be visualized as a ROS Image topic under the topic name `/btree`

Once your behavior tree is executed with `roslaunch mr_bt btree.launch tree:=my_tree` the `/btree` topic will begin to be published and updated according to the state of your tree in real time.

The nodes which have not been run will appear white, those which have been run and returned `"failure"` will appear red, those which have been run and returned `"success"` will appear green, and those which returned `"running"` will appear yellow.

Since the image is updated in real time, you will be able to get feedback on which state your tree is in at any given moment, and also debug any issues.

## Using RVIZ to visualize the Behavior Tree

You can run `rviz` to open up the program and add the `/btree` topic as an image to visualize the tree.

<img src="images/tree_visual.png" width=100%>

## Logging the Blackboard

You can log the values in the blackboard by using the `log` argument when running the mr_bt launch file:
`roslaunch mr_bt btree.launch tree:=my_tree log:=true`

The blackboard will then be printed in the terminal where you run the launch file in the ROS log.

Because some blackboard variables will be two large to print, not all of the variables will show up in the output.
Strings, floats, ints, and booleans will be printed, as well as the first 5 elements of any lists or arrays.
If the full value of a variable is not printed, its data type will be printed instead.

