##### Evalyn Berleant, Kelly Duan

### TF prefixes
The tf prefix is a parameter that determines the prefix that will go before the name of a tf frame from broadcasters such as gazebo and the robot state publisher, similar to namespacing with nodes that publish to topics. It is important to include

``` xml
<param name="tf_prefix" value="$(arg ns)" />
```

inside the namespace spawning the robots.

However, not everything uses tf_prefix as it is deprecated in tf2. In most cases, the tf prefix will likely remain the same as the namespace in order to be compatible with most packages that rely on topics and tf trees, such as gmapping, which disregard the use of tf prefixes and use namespace for tf frames.

### Getting the namespace

To retrieve the current namespace with rospy, use `rospy.get_namespace()`. This will return the namespace with `/` before and after the name. To remove the `/`s, use `rospy.get_namespace()[1:-1]`.

### References
- [ROS wiki official tf2 migration and decline of the tf prefix](http://wiki.ros.org/tf2/Migration)
- [spawning robots with namespaces in gazebo and rviz - read bottom answer](https://answers.gazebosim.org//question/16497/spawning-multiple-robots-each-with-a-controller-in-the-same-namespace-as-the-controller_manager/)
- [robot_state_publisher and tf prefixes](https://answers.ros.org/question/195846/how-to-send-tf-data-from-multiple-namespaces-to-rviz/)