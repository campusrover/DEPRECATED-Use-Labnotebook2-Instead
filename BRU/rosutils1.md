# ROSUTILS - Standard directory

## Intro

All the key scripts for bru are in a standard github repo, [rosutils](https://github.com/campusrover/rosutils). You will find the following scripts:

* `bru.py` - the main script as described
* `common_alias.bash` - the standard collection of aliases for our robots andVMs.
* `pi_connect.sh` - the script for setting up a robot or VM with tailscale
* `bashrc_template.bash` - A template for the bashrc script for all Robots or VMs that are running BRU

## Process for setting up a new robot or VM

Note that in general we set up a new robot by copying the MicroSD from a similar robot. This is here for completeness. Here are the steps:

* Follow the Robot vendor instructions to set it up and set up ROS. All libraries, environment variables and so on should be set up in a standard way
* Clone [rosutils](https://github.com/campusrover/rosutils) to ~ on the robot
* Clone [gpg_bran4](https://github.com/campusrover/gpg_bran4) to ~/catkin_ws/src
* Create a symbolic link from ~/rosutils/bru.py to ~/bin and chmod +x that file (details may be different.)


