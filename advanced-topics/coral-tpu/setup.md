## Requirements

- Linux Debian 10, or a derivative thereof (such as Ubuntu 18.04)
- A system architecture of either x86-64, Armv7 (32-bit), or Armv8 (64-bit), Raspberry Pi 3b+ or later
- One available USB port (for the best performance, use a USB 3.0 port)
- Python 3.6-3.9
- *Note:* For use with ROS, you will want to have ROS Noetic installed on Ubuntu 20.04.

For details on how to set up for Windows or Mac OS, see [Get started with the USB Accelerator](https://coral.ai/docs/accelerator/get-started/) on the Coral website.

## Installing Required Packages

Follow the following steps in order to get your environment configured for running models on the Coral TPU
1. Open up a terminal window and run the following commands:
    - `echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list`
    - `curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -`
    - `sudo apt-get update`
2. Plug in your Coral TPU USB Accelerator into a USB 3.0 port on your computer. If you already have it plugged in, unplug it and then plug it back in.
3. Install one of the following Edge TPU Runtime libraries:
  
    To install the reduced clock-speed library, run the following command:
    - `sudo apt-get install libedgetpu1-std` 
  
    Or run this command to install the maximum clock-speed library:
    - `sudo apt-get install libedgetpu1-max`
  
    *Note:* If you choose to install the maximum clock-speed library, an excessive heat warning message will display in your terminal. To close this window, simply use the down arrow to select `OK` and press enter and your installation will continue.
    
    In order to switch runtime libraries, just run the command corresponding to the runtime library that you wish to install. Your previous runtime installation will be deleted automatically.
4. Install the PyCoral Python library with the following command:
    - `sudo apt-get install python3-pycoral`


You are now ready to begin using the PyCoral TPU to run Tensorflow Lite models.
    
