## Setting up an Eduroam connection via Command Line Interface on Linux

Often in robotics, you will find that it is necessary to configure wifi through a terminal interface rather than a GUI. This is often due to the fact that many on-board operating
systems for robots lack a desktop environment, and the only way to interface with them is through a terminal.

The task of connecting to a wireless network can be especially difficult when configuring Eduroam which requires a more involved setup process. The following method for 
connecting to Eduroam has been tested on a barebones version of Ubuntu 20.04.
  
## Using NMCLI
 
`nmcli` is a command line tool used for networking on Linux machines. It is typically installed by default on Ubuntu Linux, so you should not need to connect your machine through 
 a wired connection and install it before using it. However, if `nmcli` is not installed, follow the installation instructions.
 
 ### Installation (If needed)
 
 Connect your machine through a wired connection (e.g. To a router/wall connection through an ethernet port) and the following commands to install `nmcli` and configure it to
 start upon bootup of your machine.

 Note: If it doesn't work even with a connection, and you are on campus, send an email to help@brandeis.edu noting the mac address of the `eth0` device (using `ip link show`) and the fact that you are trying to connect. Ask them if they see any activity at that particular mac address.
 
 - `sudo apt-get update`
 - `sudo apt install network-manager`
 - `systemctl start NetworkManager.service`
 - `systemctl enable NetworkManager.service`
 
### Connection to Eduroam    

Run the following command to get the names of your wireless devices.

- `ip link show`

Running this command will list all of your networking devices. You will want to note the name of your wireless networking device, for this tutorial I will assume the wireless
device's name will be `wlan0` as it is named on the Raspberry Pi 3b+, however you will want to substitute this for the name of your wireless device if your's differs.

Next you will run the following commands to connect your machine to eduroam.

- ```
    nmcli con add type wifi con-name "eduroam" ifname wlan0 ssid "eduroam" wifi-sec.key-mgmt wpa-eap 802-1x.identity "exampleemail@brandeis.edu" 802-1x.password "examplepassword123" 802-1x.system-ca-certs yes 802-1x.eap "peap" 802-1x.phase2-auth mschapv2
  ```
- `nmcli connection up eduroam --ask`

You may then be prompted to enter in the wifi username and password, however the fields should already be filled in and you will just need to press enter.

### Troubleshooting

1. If you are trying to connect a robot and it is failing, sometimes it's good to try it on the wired network. It should just work. Brandeis networking requires that the MAC address of special devices (like raspberry Pis) be recorded. This is only for the wired network. To do this use this link: : https://www.brandeis.edu/its/services/network-connectivity/netreg.html . It is extremely slow so be patient. When you click the netreg button it is very very slow (minutes) but eventually it allows you to add a mac adress to the list.

1. Sometimes despite checking everything, the wireless (eduroam) network refuses to connect. One important detail that has caused problems is the system time on the Rasberry Pi. It has to be correct or close to it. Apparently that's part of the authentication. I think the command is `date -s "19 APR 2012 11:14:00"` and that sets the UTC time. 
