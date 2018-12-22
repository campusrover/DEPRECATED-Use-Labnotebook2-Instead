# *talk.py*

## Overview

This week, we built a ROS node that subscribes to a new topic, `/things_to_say`, and uses a text-to-speech service to make the computer speak the messages of type `std_msgs/String`.

**Prerequisite**

If you receive an error when you try to run this node, the computer likely does not have `pyttsx` installed. To install it, simply run `pip install pyttsx --user`.

If that command fails, the computer likely does not have pip installed. To install it, run `sudo apt-get install python-pip` and then attempt the `pyttsx` install again.

**Running the node**

To run the node, run `rosrun cr_ros talk.py`

**Making it talk**

Once the node is launched, publish a String to the `/things_to_say` topic from any device connected to the same ROS core as the node.

Sample standalone command: `rostopic pub /things_to_say std_msgs/String 'Kill all humans'`

Sample Python script:

	import rospy
	from std_msgs import String
	
	pub = rospy.Publisher('/things_to_say', String, queue_size=1)
	rospy.init_node('say_something')
	pub.publish('Kill all humans')
	
###### _Ari Carr and Ben Albert 10/31/2018_
