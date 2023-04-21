# BRU commands

BRU is a simple command line tool (currently implemented as a bash script.) All it really does is help you properly set up the required ROS environment variables.

Here are the commands:

```
env     Display all relevant Environment variables
export  Display bash commands to export state
mode    Set running modes
name    Set name of Robot
robot   Control the attached robot remotely
status  Display current Bru sttings
```

## .bashrc set up

The most important part of BRU is the set up in the .bashrc script on computers, robots and vms. They all get it.

```
# Support for bru mode setter
export BRU_MY_IP=$(myip)
export BRU_VPN_IP=$(myvpnip)

# Setting for simulation mode
# $(bru mode sim)
# $(bru name roba -m $(myvpnip))

# Settings for a physical robot 
# $(bru mode real)
# $(bru name platform1 -m 100.107.118.103)
```
