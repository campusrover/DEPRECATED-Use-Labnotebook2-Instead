---
title: Setting up SSH on a new robot
---
# Note
All our lab robots, as of now, have SSH set up. This is for new robots.
# setting up ssh on a new robot

By default, ubuntu MATE comes with ssh *installed* but *disabled*. So there are a few steps to setup ssh to your raspberry pi.

1. ensure that `openssh-server` is installed with `sudo apt install openssh-server`
2. check ssh status with `sudo systemctl status ssh`
3. if it is `inactive`, you can start it with `sudo systemctl start ssh`, and/or automatically start ssh on boot with `sudo systemctl enable ssh`
4. go to settings -> login window and set automatic login to your robot's username.

For more information, here is [a useful site that explains it more](https://linuxize.com/post/how-to-enable-ssh-on-ubuntu-18-04/)
