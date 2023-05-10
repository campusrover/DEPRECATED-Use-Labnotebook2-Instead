---
title: How to connect to multiple robots
author: Jalon Kimes
status: new
date: may 2023
---

Step 1: switch the .bashrc to be running in sim mode. 
  Step 1.1: Go into .bashrc file and uncomment the simulation mode as shown below:

  # Setting for simulation mode
  # $(bru mode sim)
  # $(bru name roba -m $(myvpnip))

  Step 1.2: Comment out real mode/robot ip addresses For Example:
  # Settings for donatello
  # $(bru mode real)
  # $(bru name donatello -m 100.106.194.39)

Step 2: run roscore on vnc. To do this type "roscore" into the terminal

Step 3: Now in the terminal do these steps
  Step 3.1: get vpn ip address: In the terminal type "myvpnipaddress" 
  Step 3.2: Type "$(bru mode real)"
  Step 3.3: Type "$(bru name robotname -m "vpn ip address from step 4")"
  Step 3.4: type"multibringup" in each robot terminal

Step 4: Repeat step 3 in a second tab for the other robot(s)



