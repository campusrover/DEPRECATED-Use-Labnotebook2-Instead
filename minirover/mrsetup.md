## Setup of MiniRover

* See the important information about turning the robot on and off here: [Setting Up the Robot](minirover/mrsetup.md)

### MicroSD Card

You are going to load software onto the microSD Card - which will wipe out what you loaded onto it according the the instructions before. We provide you a disk image on which to store it. It changes from time to time. [This link](https://drive.google.com/drive/folders/1rmt9I9YtlrG3B5IyqSFD_oM0xei-HdNa?usp=sharing) is to a google folder that will contain the versions. You should use the latest one in that folder. I recommend you use the app "Balena Etcher" to copy the image onto your microSD card.

To create a new MicroSD from the old one, see [Backup Raspi Card on MacOS](https://medium.com/@ccarnino/backup-raspberry-pi-sd-card-on-macos-the-2019-simple-way-to-clone-1517af972ca5)
### Connecting to the network

Now we are facing a dilemma. We need to get the robot on your network. There are several ways of doing this. Here is one.

1. Locate a wired network connection (on your router for example) and use a network cable to connect your robot to the network

1. Now turn the power on the robot and it should boot up into linux. But you won't know this because there's no keyboard or screen!

1. Using your own computer or development environment that is attached to the network check that you see the robot. This includes a linux computer where you program, or a browser web development environment. Broadly speaking, it's the computer "other than" to raspberry pi on the robot (we will refer to this as the *remote* computer from now on). 

`ping gopigo3.local`

Once this works you know that you have access to the robot from your remote computer. Make note of the robot' ip address. It will likely look like 192.168.1.xx but not necessarily.

1. Now use `ssh` (secure shell) to get to the robot from the remote:

`ssh pi@gopigo3.local`

It will ask you for the password for account `pi`. It is `raspberry`. Once you get in you are 1/2 way there!

1. Now we want to get your robot onto your local wifi network. You need to know the wifi network's name and password. On the robot command line type:

`sudo nmcli d wifi connect <SSID> password <password>`

Where <SSID> is the network's name. You can surround it in quotes if there are spaces or other funny characters. <password> is the password.

1. Next shutdown the robot nicely (see above), disconnect the network cable, and start the robot up nicely again (see above.)

1. Once it's back, follow the same steps to `ssh pi@gopigo3.local` and enter the password `raspberry` and you should have a wireless enabled robot.

### VPN

You can run everything on the robot itself but it doesn't have a screen. You could plug an external screen, keyboard and mouse and have a complete ROS enabled computer (the robot) at your finger-tips. But it would be real slow. So instead we are going to add the robot to a "VPN" - virtual private network which will allow you to work with it remotely.

#### Key Environment Variables

`ROS_MASTER_URI=http:/100.94.206.80:11311` (example!) should always be set to the computer where roscore is running. In our world, roscore runs on the robot itself. So we need to know the ip address of the robot. You think you know it, but wait.

`ROS_IP=100.94.206.80` (example) should be equal to the local ip address, a computer's own ip address. You think you know it, but wait.

As you know ROS requires that all nodes can talk to roscore. These two environment variables need to be defined on each computer that is involved, that is, both the robot and the remote computer. So you would have the following:

##### Robot

ROS_MASTER_URI = robot's own ip address
ROS_IP = robot's own ip address

##### Remote Computer 

ROS_MASTER_URI = robots ip address
ROS_IP = remote computer's own IP address

These IP addresses are on different networks and cannot access each other. So instead we've created what is called a "virtual private network" that connects them together. Both your robot and your cloud desktop have an *alternate* ip address which they can both see.

## Conclusion

This is the end of these instructions. From here you should have a working ROS environment on your robot and your "remote" computer.

## FAQ

* miniRover
  * hostname `gopigo3`
  * default account `pi`
  * default password raspberry
* Cloud or Docker Desktop
  * default password `dev@ros`
  * url: <unetid>.ros.campusrover.org:6080/vnc.html (desktop)
  * url: <unetid>.ros.campusrover.org:8080 (cloud vscode)