## Setting up an eduroam connection via Command Line Interface on Linux

Often in robotics, you will find that it is necessary to configure wifi through a terminal interface rather than a GUI. This is often due to the fact that many on-board operating
systems for robots lack a desktop environment, and the only way to interface with them is through a terminal.

The task of connecting to a wireless network can be especially difficult when configuring "eduroam" which requires a more involved setup process. The following method for connecting
to "eduroam" has been tested on a barebones version of Ubuntu 20.04.
  
## Using NMCLI
 
`nmcli` is a command line tool used for networking on Linux machines. It is typically installed by default on Ubuntu Linux, so you should not need to connect your machine through 
  a wired connection and install it before using it.
    

