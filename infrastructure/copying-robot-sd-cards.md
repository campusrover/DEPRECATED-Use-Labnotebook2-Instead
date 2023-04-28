# How to copy robot OS from one sd card to another on Linux

Often in the lab the need arises to create several robots with identical operating systems and packages, however it is incredibly time consuming to flash the base operating system to a new sd card and download all the required packages to each new robot. 

This guide will explain how to create identical boot sd cards from one working operating system in order to scale the ever-growing fleet of terminator robots this lab hopes to one day create, so let's get started.

## Shrink the existing SD card partition

Say you have a working operating system on a robot with an SD card of 128GB, but most of the space on that SD card is unused. If you created an OS image from this SD card it too would be 128GB which is large and time consuming to flash, not to mention it won't fit on smaller SD cards. You'll first need to shrink your existing card to the minimum size to fit the operating system.

Plug the SD card into your linux computer and boot up the program GParted. Select the SD card from the menu in the top left. You'll want to unmount both of the partitions displayed by right clicking on them in the list menu, and you'll then see they key icon next to their names disappear.

[](large_orig_sd_card.png)

You're going to resize the larger of the two partitions by selecting it from the list menu pressing the resize button in the top menu. Resize the partition to be slightly larger than the amount of used space. In my case here, I'm resizing the partion to be 13000MB with 12612MB used.

[](resize.png)

Then you're going to apply the resizing operation by clicking the green checkmark on the top menu bar, it should take a little bit of time to complete. 


Now you are ready to create your image file. Open your terminal and type the command `sudo fdisk -l`. It should display a list of storage partitions, so find the partition in the list 

[](resized.png)

Now 