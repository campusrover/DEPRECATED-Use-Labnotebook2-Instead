---
title: Speech Recognition Report
date: feb-2023
author: Veronika Belkina
status: new
desc: 
---
# **Speech Recognition**
### [Github](https://github.com/campusrover/speech_recognition)
### Veronika Belkina (vbelkina@brandeis.edu)
### Spring 2023 

## **Description**

This project aims to utilize Python's speech recognition library to listen to user commands through a microphone. The recognized speech is then sent to another node where it is translated into dynamic messages using json, which are sent to a robot via roslibpy. The robot will execute the incoming commands based on the received messages. This program can be run locally on the robot, if you wish. It can also be run remotely from your computer. 

## **Links**

- [Bluetooth](https://github.com/campusrover/labnotebook/blob/master/faq/bluetooth.md)
- [Rosbridge](https://github.com/campusrover/labnotebook/blob/master/faq/rosbridge.md)

## **Installation**

These instructions assume that you have **ROS NOETIC** installed. This has not been tested on any other distro. To install them, first git clone this package into your catkin_ws and then run: 

```
git clone https://github.com/vbelkina/whisper_4.git
pip install -r requirements.txt
sudo apt install ros-noetic-rosbridge-server
```

*core:*

- **listen.py**
    - Using Python's speech-to-text package, you can listen to commands from the user through a microphone and then send the message to the designated topic (/whisper/command).
- **execute.py**
    - By listening to the specified topic (/whisper/command), incoming commands can be executed. This process involves the use of roslibpy and json, which allow for the dynamic publishing of messages to the robot.
- **commands.json**
    - a json file with the format shown below where 

            "command_1": {
                "command_2": { 
                    "receiver": "/cmd_vel",
                    "type": "geometry_msgs/Twist",
                    "msg" : {
                        "linear": {
                            "x": 0.2,
                            "y": 0.0,
                            "z": 0.0
                        },
                        "angular": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0
                        }
                    }
                }
    - *command_1* and *command_2* are the commands for certain actions for the robot to execute- for example, in "go back", command_1 = "go" and command_2 = "back"
    - *receiver* is the Topic to be published to 
    - *type* is the Message type for the topic
    - *msg* is the message to be sent, formatted in the same structure as the message is seen in ROS documention. 

*misc:* 

- **find_mic.py**
    - determine the device index of the microphone you want to use and see if there are any errors. 

## **Run**

To run this project in one terminal (could be on robot or remote computer, but robot works fine)

    roslaunch whisper_4 command.launch device_index:=0 

Or to run the files in separate terminals (same comment)

    roslaunch rosbridge_server rosbridge_websocket.launch
    rosrun whisper_4 execute.py
    rosrun whisper_4 listen.py

## **Connecting to a microphone**

Any microphone should work, even the built in microphone on your laptop. If you are running it on a linux machine, then it shouldn't be a problem as you can access the device indexes and see what they are using the find_mic.py file which should list all available microphone devices. 
I have not tested what happens on a Mac or Windows, however, my guess is that if you leave the device index to be 0, then it should choose the default microphone and speaker. 


## **Known Errors**

- `malloc(): mismatching next->prev_size (unsorted)` 
    - This error can occur when you select a device index for your microphone that is recognized as a microphone, but is not functioning properly. If you attempt to record sound using this microphone, you will encounter this error. I'm not sure how to catch this because it occurs after the microphone is already connected and has something to do with memory allocation. 

- small pause between publishing of messages
    - This 'error' occurs when publishing the same message over and over and there will be a slight pause between each message so the robot will pause for a second. I'm unable to fix this in a way that I am happy with at the moment. 

- `ALSA library warnings` 
    - I added some error supressors, however, it still shows up sometimes after initializing the program and then it shouldn't show up again. It doesn't affect the functionality but it looks a little ugly. 

## **Story**

### *Speech recognition option exploration*

When starting this project, I explored several options for speech recoginition. The goal was to have something that can run locally on the robot so it had to be lightweight. It also had to be accurate and fast. Initially, I looked at Pocketsphinx based speech recognition which was recommended in several different places as a lightweight speech recognition program that worked well. However, after some exploration, I struggled to get accurate results from pocketsphinx, even after a local dictionary. 

After doing some more research, I found that python has its own speech recognition module which let's you have access to Google Speech to Text, OpenAI's Whisper, and so on. These worked much better. I ended up deciding on Google's Speech to Text as I found it to be the best and fastest at interpreting data and returning results.  

### *Bluetooth struggles*
### 

Initially, I wanted to use a bluetooth headset to send commands. However, using bluetooth on linux and raspberry pi is not so simple. There was a lot of struggle to get a headset to connect. For example, different bluetooth headphones with microphones might have different profiles. Originally, I was trying to use the PulseAudio library which was not very simple to use with bluetooth and I played around a lot with changing a variety of settings. I ended up finding a set of instructions and switched to using PipeWire which is a server for handling audio, video streams, and hardware on Linux. This seemed to work better. In the end though, I switched to a USB headset which was much simpler to use as the headset is interpreted as hardware instead of a bluetooth device. 

### *Rosbridge and roslibpy*

Throughout the project, I wanted to have a dynamic way to send messages to the robot. It took a while to find a way that worked and it came in the form of using the rosbridge-server and roslibpy. The Rosbridge server created a websocket connection which allows you to pass JSON messages from the websocket to the rosbridge library which then converts the JSON to ROS calls. Roslibpy lets Python interact with ROS without having to have ROS installed. It uses WebSockets to connect to rosbridge 2.0 and provides publishing, subscribing, and other essential ROS functionalities. 

### *Hopes for the future*

This version of the project is a base with minimal commands and room for improvement. Some ideas to further improve this project is to add more interesting commands. For example, it would be interesting if a command such as "solve this maze" was said and that could start a node or send a series of tasks that would connect with algorithms that would then solve a maze. 

Another idea is to add parameters at the end of commands. For example, when the command "go forward" is given, it would good to have some numerical value to specify the speed at which the robot would go. Perhaps, "go forward speed 20" would indicate to move forward at 0.2 m/s or "go foward distance 5" would indicate to move foward for 5 meters, with some default values set if these specifiers are not mentioned. 
