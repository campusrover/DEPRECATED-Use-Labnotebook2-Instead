# Bootable SSD Instructions

## **0. Introduction**
* **SSD** Solid state disk. Either internal or external. In our case we are using external SSDs which connect to your computer with a USB connection. 
* **USB Stick** Also known as a thumb drive, flash drive, memory stick, a memory key. Its a small form factor, dongle like device which also plugs into your computer.
* **Bootable SSD** and **Bootable USB Stick** have been formatted and created in such a way that the computer can boot from them and thereby leave your default internal disks untouched.

**Overview of the procedure**
We are going to create a bootable SSD with these instructions. You will:

* Download the right version of Ubuntu onto your computer
* Plug in your USB stick (which will be erased)
* "Flash" the USB stick which will make it bootable
* Boot using that USB stick
* This brings up the Ubuntu installer. You will use it to just "try ubuntu"
* You will now plug in the SSD and format it with the right partitions
* Next you will install Ubuntu 18.04 onto the SSD and reboot.

## **1. Create Bootable Ubuntu USB stick**

In order to install Ubuntu onto an SSD, we need to first create a bootable USB Flash Drive. This drive will be used to boot Ubuntu and install it onto the SSD. The process can be completed as follows:

**1. Plug the USB SSD drive into your computer**

**2. Download the appropriate Ubuntu 18.04 desktop image for your machine**
  
   http://releases.ubuntu.com/18.04/

**3. Flash the Ubuntu image onto your USB stick.**

   How to flash Ubuntu with Mac:
   https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-macos#0

   How to flash Ubuntu with Windows:
   https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows#0

   How to flasg Ubuntu with Ubuntu:
   https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-ubuntu#2

With Ubuntu flashed onto your flash drive, it can now be booted and installed onto your SSD. If your SSD has anything on it, be sure to format it before moving on to the next step.

## **2. Boot Ubuntu**

Booting from a USB depends on your operating system. Instructions for Windows and Mac are below:

**MacOS**

   Restart your computer. While it is booting up, hold down the alt/option button. A menu should appear, select the "EFI Boot" option. If the MacOS login screen appears, you will need to restart and try again.

**Windows**

   Restart your computer. While it is booting up, press the boot menu key. This is often one of the F keys on most Windows machines, but sometimes ESC as well. With the boot menu button pressed, you will be shown the BIOS menu. From here you need to change the boot order so that your Ubuntu USB drive is prioritized above the drive where Windows is installed. Once this is done, reboot and Ubuntu should be booted.

**Continuing...**

With Ubuntu running on your machine, plug in your external SSD (while keeping the USB stick plugged in) and select **“Try ubuntu without installing”** to begin the installation process.

   **NOTE:** If you are unable to use your mouse and keyboard on the Ubuntu desktop, a USB keyboard and mouse is required from this point on.


## **3. Install Ubuntu on External SSD**

Now that you are on the Ubuntu desktop, it is time to install Ubuntu to the SSD. The following steps will take you through installing Ubutnu onto your SSD:

**1. Partitioning your SSD**

You will use a program called **GParted** to partition your SSD for installation. From the applications menu in Ubuntu (top left corner), open up **GParted** and in the top right corner, select your SSD.

Once the drive is selected, remove all the current partitions by selecting them and pressing the delete / backspace key. With these all removed, select the green check in the top menu to confirm the changes.

Now that all the original partitions have been removed, the SSD needs to be partitioned for Ubuntu. To create a new partition, select the add button in the top left corner. You will need to create 3 partitions with the following specifications:

**Partition 1:**

   New Size = 8000 MB
   File System = linux-swap

**Partition 2:**

   New Size = 20,000 MB
   File system = ext4

**Partition 3:**

   New Size = 80,000 MB
   File System = ext4

To complete the process, click the green checkmark from the top menu once again. With all the partitions created, your GParted should look like this:

![Gparted](./gparted.png)


**2. Installing Ubuntu onto the SSD**

With the SSD partitioned, we can now install Ubuntu. Click the **Install Ubuntu 18.04** icon from the side bar to begin the process. You will be prompted to go through a series of menus. Choose "minimal" install.       

    BE SURE TO CHECK THE BOX TO INSTALL THIRD PARTY SOFTWARE. Without this, you will be unable to connect to WiFi.

To set up wifi, you need to choose: and the eduroam account robotics@brandeis.edu with our traditional password. See the image below for how to fill in the Wifi dialog box.

You will eventually reach a screen which asks to you how to install, titled Installation Type.

    DO NOT SELECT THE OPTION TO ERASE DISK AND INSTALL UBUNTU. THIS WILL ERASE YOUR CURRENT OS AND MEMORY. BE SURE TO SELECT "SOMETHING ELSE"

The **“Something else”** option allows us to assign mount points to the previously partitioned SSD. In the list of partitions, find the SSD. It should be called something like SDx. **Be really careful that you are picking the right one.** You will see the 3 partitions that were made in GParted. To set the partitions up for Ubuntu, double click on and configure the partitions as follows:

**sdx2**

    Use as: ext4
    format: checked
    Mount point = /

**sdx3**
  
    Use as: ext4
    format: checked
    Mount point = /home

The partitions are now ready and Ubuntu can be installed. Select the SSD from the menu below and click install. Ubuntu will now install, and once done you will need to restart your system and remove the thumb drive.

## **4. Make The SSD Bootable** ##

Now that Ubuntu has been installed, we need to make it bootable. To do this, we will follow a tutorial online. The tutorial begins at the section **Create an ESP on the Ubuntu HDD**, and can be found at the following link:

   https://www.dionysopoulos.me/portable-ubuntu-on-usb-hdd/

   **NOTE: You should skip the `umount` command instruction**

To be sure that this process worked, the final command should return a message saying the installation completed with no errors.

## **5. Install ROS** ##

The final step in the process is to install ROS. In order to install ROS, you will need access to the internet. If you are using Eduroam, the following settings will need to be used:

![Eduroam settings](./wifi.png)

Once you are connected to the internet, follow these instructions to install ROS:

   http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/

With this completed, your SSD is configured!
