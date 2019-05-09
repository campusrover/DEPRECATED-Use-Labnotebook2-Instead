To launch Mutant, follow these steps:
1. Ensure that mutant has the most recent version of `cr_ros_2`. This can be accomplished by running `roscd cr_ros_2` and then `gp`.
1. SSH into the mutant and run `bu-mutant`. This will launch the mutant onboard bringup.
1. On your local machine (again after making sure that you have the most recent version of `cr_ros_2`, run `roslaunch cr_ros_2 mtnt_onb_rpicam.launch`. This command will start the web app, and you can proceed from there.

**Troubleshooting:**
1. Shut down and restart roscore.
1. Make sure everything is namespaced correctly (on your local machine).
1. 
