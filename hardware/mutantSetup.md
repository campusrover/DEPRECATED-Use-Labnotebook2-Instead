# Setting up Mutant
In the event that a new mutant ever has to be set up, or software problems require mutant to be reset, here are the necessary tips to installing everything that is needed to operate using the most recent campus rover ros package.

## Installing ros
[The Robotis Emanual](http://emanual.robotis.com/docs/en/platform/turtlebot3/raspberry_pi_3_setup/#install-linux-ubuntu-mate) serves as an adequate guide to getting 95% of the way to setting up ROS on a mutant turtlebot. There are a few divergences from their instructions, though:
* In part 3 of section 6.2.1.1, you could easily miss the note about installing what is needed to use a raspi camera. Either do not miss it (it is right below the first block of terminal commands) or check out [our labnotebook page on configuring the raspberry pi camera](RaspiCam.md).
* just below the camera hint, the emanual instructs to use these commands to remove unneeded packages:
```
cd ~/catkin_ws/src/turtlebot3
sudo rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
```
However, slam and navigation are actually useful to us, so use these commands instead:
```
cd ~/catkin_ws/src/turtlebot3
sudo rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_example/
```
* Once you have finished the emanual SBC setup, you still need to install a few dependencies that Robotis assumes you will only use on a remote pc. For your convenience, run this command:
```
sudo apt-get install ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
```
If you are curious as to where this came from, it is an edited version of the first command of emanual section 6.1.3, under PC setup.


## On the remote PC
you will need:
* the fiducial package `sudo apt-get install ros-kinetic-fiducials`
* cr_ros_2 github repo on branch mutant_transfer
* cr_web github repo on branch mutant
* the turtlebot3_navigation package, which should come with turtlebot3.
* google chrome (for the web app - you can use another browser, but that would require editing a shell script in the cr_web package)
* flask module for python
Google how to get Chrome and flask. cr_ros_2 and cr_web are repos in the campus rover github community. 

### Using Git and Github
If this class is your first time using github - don't worry! Though it may seem mildly confusing and daunting at first, it will eventually become your best friend. There's a pretty good guide called [git - the simple guide - no deep shit!](http://rogerdudler.github.io/git-guide/) which can walk you through the basics of using git in the terminal. Here's a bit of a short guide to the commands we have used most in gen3:
* `git clone` is how you will initially pull a repository off of Github
* in a repository's main directory in your terminal, `gs` (or `git status`) is a super useful command that will show you all the files you have edited.
* `git add` --> `git commit -m` --> `git push` is how you will be updating your repos on github. **Pro Tip:** if you've edited multiple files in a subdirectory, for example in src, then you can type `git add src` to add all modified files in src, rather than typing each file individually.
* always do a pull before a push if you think someone else has made edits to your repo.
* if you've made changes locally that you don't want to keep, `git reset --hard` will revert back to your last pull or clone. 
