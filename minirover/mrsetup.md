## Setup of MiniRover

This section will help you get your robot set up to go. Note that if you received this from the Autonomous Robotics Lab then part of these steps may already be done.

See the important information about turning the robot on and off here: [Using the Robot](mruse.md). 

### MicroSD Card

You are going to load software onto the microSD Card - which will wipe out what you loaded onto it according the the instructions before. We provide you a disk image on which to store it. It changes from time to time. [This link](https://drive.google.com/drive/folders/1rmt9I9YtlrG3B5IyqSFD_oM0xei-HdNa?usp=sharing) is to a google folder that will contain the versions. You should use the latest one in that folder. I recommend you use the app "Balena Etcher" to copy the image onto your microSD card.

To create a new MicroSD from the old one, see [Backup Raspi Card on MacOS](https://medium.com/@ccarnino/backup-raspberry-pi-sd-card-on-macos-the-2019-simple-way-to-clone-1517af972ca5)

### Connecting to the network

Now we are facing a dilemma. We need to get the robot on your network. There are several ways of doing this. Below are two specific scenarios that we support:

1. A usb keyboard, usb mouse **and** a HDMI display **and** eduroam wifi access
1. *OR* A network cable **and** non-eduroam wifi access

#### Scenario 1

1. Locate a wired network connection (on your router for example) and use a network cable to connect your robot to the network

1. Now turn the power on (see [Using MiniRover](mruse.md)for instructions. It should boot up into linux. But you won't know this because there's no keyboard or screen!

1. Using your own computer or development environment that is attached to the network check that you see the robot. This includes a linux computer where you program, or a browser web development environment. Broadly speaking, it's the computer "other than" to raspberry pi on the robot (we will refer to this as the *remote* computer from now on). 

````
ping gopigo3.local
````

Once this works you know that you have access to the robot from your remote computer. Make note of the robot' ip address. It will likely look like 192.168.1.xx but not necessarily.

1. Now use `ssh` (secure shell) to get to the robot from the remote:

````
ssh pi@gopigo3.local
````

It will ask you for the password for account `pi`. It is `raspberry`. Once you get in you are 1/2 way there!

1. Now we want to get your robot onto your local wifi network. You need to know the wifi network's name and password. On the robot command line type:

````
sudo nmcli d wifi connect <SSID> password <password>
````

Where <SSID> is the network's name. You can surround it in quotes if there are spaces or other funny characters. <password> is the password.

1. Next shutdown the robot nicely (see above), disconnect the network cable, and start the robot up nicely again (see above.)

1. Once it's back, follow the same steps to `ssh pi@gopigo3.local` and enter the password `raspberry` and you should have a wireless enabled robot.

#### Scenario 2

1. Connect your mouse, keyboard and screen to the Raspberry pi. You will find several free USB ports and an HDMI port. Look closely they are all there.

1. Boot up the Rasberry Pi and wait until it is running and you see the desktop
1. Locate the network settings dialog box by clicking the network icon on the top right
1. Add eduroam as a network, and fill it in as follows:

![Button to reboot](networksettings.png)

1. Finally shutdown the robot, unplug the keyboard, mouse and monitor and reboot
1. Once it's back `ssh pi@gopigo3.local` and enter the password `raspberry` and you should have a wireless enabled robot

### Troubleshooting ssh gopigo3.local

Under certain circumstances gopigo3.local will be found. If so you need to find out the ip address of your robot when it is on wifi (not wired). If then this should work:

```
ssh pi@<ip address>
```

### Updating the hostname of your Robot

Next come up with a name for your robot. It comes to you called "gopigo3" but they are all called that and this can cause confusion. Let's say you want to call your robot `myrobot`. Oddly you have to change it in two places. Here's how"

```
# In the shell of the robot:
sudo hostname myrobot

# In the hostname file:
sudo nano /etc/hostname
```
Now the robot is called myrobot and at least it will be different from other miniRovers.

#### Eduroam

On eduroam it is often automatically known as myrobot.dyn.brandeis.edu. So after rebooting it check if this works. It's not guaranteed but often it works just fine.

### VPN

You can run everything on the robot itself but it doesn't have a screen. You could plug an external screen, keyboard and mouse and have a complete ROS enabled computer (the robot) at your finger-tips. But it would be real slow. So instead we are going to add the robot to a "VPN" - virtual private network which will allow you to work with it remotely.

1. Prepare the vpn configuration by:

````
sudo apt-get remove -y tailscale
sudo rm -rf /var/lib/tailscale/tailscaled.state

# Reboot right after!
sudo reboot
````

1. Setup vpn

````
# Get the tskey from Pito
cd ~/rosutils
chmod +x pi_connect.sh

# Run the script with the tailscale authkey
sudo ./pi_connect.sh <tskey-123abc456>

# On successful connect, you should see this
# Connected. IP address: 100.xx.xxx.xxx
````

1. `myvpnip` should now return that same IP address.

### Update a few things

We have installed some Brandeis specific software which should be updated:

````bash
cd ~/rosutils
git pull
cp ~/rosutils/bashrc_template.bash ~/.bashrc
cd ~/catkin_ws/src/gpg_bran4
git pull
````

#### And then

1. Edit the new ~/.bashrc according to the instructions in the file.

### Updates to your Cloud Desktop

1. Check that you have ~/rosutils directory on your cloud desktop. If not:
    1. `cd`
    1. `git clone https://github.com/campusrover/rosutils.git`
    1. `cp rosutils/bashrc_template.bash .bashrc`
    1. Edit .bashrc according to the instructions in it
1. Check that you have the following directories:
    1. ~/catklin_ws/src/Chapter9_GoPiGo3_Slam and ~/catklin_ws/src/Chapter8_Virtual_Slam. If not:
        1. `cd ~`
        1. `git clone https://github.com/PacktPublishing/Hands-On-ROS-for-Robotics-Programming.git`
        1. `cp -R Hands-On-ROS-for-Robotics-Programming/Chapter9_GoPiGo3_SLAM ~/catkin_ws/src`
        1. `cp -R Hands-On-ROS-for-Robotics-Programming/Chapter8_Virtual_SLAM ~/catkin_ws/src`
        1. `cd catkin_Ws`
        1. `catkin_make`

        