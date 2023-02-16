# connect to robot
1. plug in battery and turn on robot with the power switch, give it a moment and wait for the lidar to start spinning. 
2. run `tailscale status | grep <name>` to find the robot’s IP address. Replace <name> with the name of the robot you are trying to connect to. 
  
![image](https://user-images.githubusercontent.com/72238100/219488504-47b18571-2dff-46ac-bf9b-dd31cb17574a.png)

3. go to .bashrc in my_ros_data folder and get it to look like this with your robot’s name and IP address instead: 

![image](https://user-images.githubusercontent.com/72238100/219488620-0e1a2bb2-44fd-42cf-a5b3-89e5fe5c1540.png)

4. Open a new terminal and you should see: 

![image](https://user-images.githubusercontent.com/72238100/219488675-d943c463-9c37-4f65-a856-0acfc7a85d0c.png)


5. You can then ssh into the robot, `ssh ubuntu@100.117.252.97` (enter your robot’s IP) and enter the password that is in the lab. 
6. Once onboard the robot, enter the command `bringup` which starts roscore and the Turtlebot’s basic functionalities. 
7. After that, open a new terminal (you’ll be in real mode again) and run your program!
8. To go back to simulation mode, go back to .bashrc and uncomment the settings for simulation mode and comment out the settings for a physical robot. Or type the command `sim` in the terminal. You will need to do this in every terminal that you open then. To switch to real mode, type command `real`.
 
When you're done, run `sudo shutdown now` onboard the robot (where is ran bringup) and then turn off the robot with the power switch. 
