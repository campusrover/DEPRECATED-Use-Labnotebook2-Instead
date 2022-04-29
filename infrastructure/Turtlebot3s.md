

# Setting up a new Operating System on Turtlebot3


Download Image: [ROS Noetic Ubuntu 20.04](https://drive.google.com/file/d/1EGy9g5gE4kLXJk-oa_l-MDt5miUB33-r/view?usp=sharing)


## Instructions:

  1. Flash the image into a microsd card using an imager (dd for linux, or RPiImager).
  
  2. Connect the Turtlebot to a monitor and keyboard, insert the microsd card and turn on the bot.
  
  3. Log into the turtlebot with the username: *ubuntu* and the password: *ROSlab134*
  
  4. You will want to remove the tailscale information using the following commands:
    <pre>
    sudo apt-get remove tailscale
    sudo rm /var/lib/tailscale/tailscaled.state
    sudo nano /etc/hostname 
    <i># change the hostname in this file from "roba" to your robot's name</i>
    sudo reboot now
    </pre>

  5. Once the turtlebot is rebooted, change the hostname and reinstall tailscale:
    <pre>
    sudo apt-get install tailscale
    sudo tailscale up --authkey=<i>ask-pito-for-code</i>
    </pre>
  6. Now update the OpenCR board with the following commands:
    <pre>
    export OPENCR_PORT=/dev/ttyACM0
    export OPENCR_MODEL=burger_noetic <i># or waffle_noetic if you have a waffle tb3</i>
    rm -rf ./opencr_update.tar.bz2
    wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 
    tar -xvf opencr_update.tar.bz2 
    cd ./opencr_update
    ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
    </pre>
  7. Once the OpenCR board is updated, shut down your bot and turn it back on and you are done with the setup.
