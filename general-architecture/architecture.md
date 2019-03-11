# Overview

[Architecture diagram here](https://docs.google.com/drawings/d/1K8Bq4vd7oYqD6yXStrwqmTUtPrRfLSoVaKmgR9WnArc/edit?usp=sharing) \(Outdated. Sorry (No worry, GEN3 will update a new one😉)\)

On the first step, we need to bring the laptop inside the Mark-I out and turn it on. Afterwards we need to SSH into the Mark-I laptop with our own device by `ssh turtlebot@129.64.243.64`. Then we attach the Mark-I laptop to her mothership and do a bringup for her by entering `roslaunch cr_ros campus_rover.launch` in the ssh terminal (Note this bringup will automatically star roscore so we don't need to do roscore seperately). By our past experience, we need to wait for a 'bep bep bep' sound effect made by the Kabuki machine and check for "Odom received" message on the ssh terminal. 

Then on the side of our own device, we want to open another terminal and enter `roslaunch cr_ros offboard_launch.launch`. This will do the off_board bringup and boot the Rviz for Mark-I to navigating herself. Once Mark-I has correctly localized herself, then we can minimize the Rviz window or even close it without any harm.

For the final step of booting Mark-I, here comes to the web server. First I assume our device has already had Flask installed and cr_web repo cloned. Then we want to open another terminal window and cd into the cr_web directory. Then entering command `flask run --no-reload --host=0.0.0.0 --with-threads`. This will start the localhost web server and we can manually open a browser and go to `localhost:5000/` and we shall see that Mark-I on the web server is ready for you to do some rock! 🦄

The navigation algorithm we used is the most complex software controlling the rover. It must efficiently process sensor data and plan routes over long range maps and avoid the short distance obstacles. A full-fledged campus rover must also handle complex obstacles like doors, elevators, and road crossings which would each require special navigation decision making. A fully functioning rover would also incorporate a unique localization algorithm to combine sensor data from fiducials, GPS, wifi signals, and camera/lidar inputs, etc.

Currently, reliability is a more pressing concern in building a rover to meet the mark 1 requirements. While a solution which provides more control in navigation would allow us to better expand our solution beyond Volen 1, it is not feasible at this stage. The turtlebot navigation package has been well tested and is proven to carefully plan routes and avoid obstacles. It also incorporates a powerful adaptive Monte Carlo localization algorithm which draws on data points from a single-beam lidar or depth camera.

A better solution for the mark 1 rover is to make use of the robust turtlebot navigation code and supplement the AMCL localization with a separate fiducial localization system which will supply poses to the existing algorithm by publishing pose estimates. A navigation\_controller will also wrap the standard navigation and localization nodes to provide additional control and feedback where possible.

It is also beneficial to wrap the cmd\_vel outputs of the navigation will allow for distinct wandering to refine an initial pose and for teleop commands. We use the `/cmd_vel_mux/inputs` topics. This allows teleop commands to block navigation directions which in turn will block localization commands.

At the top level, a central control node will manage core tasks like interacting with the tablet controls, publishing a map stored on disk, and deciding when the robot must return to recharge. Keeping these simple high level tasks combined will allow for rapid development but could be spun off into independent ROS nodes if the functionality required grows in complexity.

`process_fiducial_transforms` also publishes acts primarily as a transform from the camera-relative pose from the built-in aruco\_detect node, to a map relative pose based on it’s knowledge of the locations of fiducials in the map. The node is always running, but only when it sees a fiducial will it publish a cur\_pose message to assist the AMCL localization.

## Teleop:

Takes button presses from the UI and sends `cmd_vels`.
(However, there is a bug in this application. After the user teleop the rover, it can’t respond to new commands, such as back to charging station or go to xxx office. In gen3, we will fix this bug.)

## Rover controller:

Uses a `move_base` action to navigate. Subscribes to `/web/destination` which parses JSON input and `/destination` which takes a PoseStamped.

### @Alexander Feldman, feldmanay@gmail.com 10/26/2018\_
### @Sibo Zhu, siboz@brandeis.edu 3/11/2019\_
### @Yuchen Zhang, yzhang71@brandeis.edu 3/11/2019\_

