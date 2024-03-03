---
title: Using ROS2 with Docker 
author: James Lee & anonymous
description: How to use ROS2 via Docker 
status: new 
date: march-2024
---

# Question

I want to try out a distribution of ROS2, but it does not support my
computer's operating system. But I also want to avoid installing a
compatible version of Ubuntu or Windows on my computer just for running
ROS2.

I heard that Docker might allow me to do this. How would I go about it?

# Answer

You would run ROS2 on your existing OS as a Docker image in a Docker
container. To do this, you first need to install Docker.

## Installing Docker

### On Linux Distributions 

Docker's official manuals and guides push you to install Docker
Desktop, regardless of your operating system. But Docker Desktop is
just a GUI that adds layers of abstraction on top of Docker Engine,
which is the technology that drives Docker. And we can communicate with
Docker Engine directly after installing it via the Docker CLI.

So if you use a Linux distribution (like Ubuntu, Debian, Fedora, etc.),
just install the Docker Engine directly; we won't need the features
Docker Desktop offers. Go to [this
page](https://docs.docker.com/engine/install/ubuntu) to install Docker
Engine for Ubuntu. 

Even if you don't use Ubuntu, just go to the page mentioned, and find
your distribution on the Table of Contents you find at the left bar of
the website. Just don't expect to find help on the Docker Engine
Overview page: they'll just start pushing you towards Docker Desktop
again. 

### On MacOS

You'll have to install Docker Desktop. Go to [this
page](https://docs.docker.com/desktop/install/mac-install/) and follow
the instructions.

### After Installation 

After installing Docker Engine or Docker Desktop, open a terminal and
execute:

```bash
sudo docker --version
```

You should see an output like:

```bash
Docker version 25.0.3, build 4debf41
```

if you installed Docker correctly.

## Running ROS2 on Docker

It's possible to develop your ROS2 package entirely via the
[minimalistic Docker
image](https://docs.ros.org/en/iron/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)
the ROS2 community recommends. In fact, this seems to be a, if not the,
common practice in the ROS2 community.

**But** the learning curve for this is probably, for most software
engineers, very, very steep. From what I gather, you'll need an
excellent grasp of Docker, networking, and the Linux kernel to pull it
off successfully.

If you don't have an excellent grasp of these three technologies, it's
probably better to use a more robust Docker image that has enough
configured for you to go through ROS2's official tutorial. Afterwards,
if you want to use ROS2 for actual development, I recommend installing
the necessary OS on your computer and running ROS2 directly on it.

Such a repository has been provided by tiryoh on github. To use it,
just run:

```bash
docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m tiryoh/ros2-desktop-vnc:iron
```

It will take a while for the command to run. After it does, keep the
terminal window open, and visit the address `http://127.0.0.1:6080/` on
your browser, and click "Connect" on the VNC window. This should take
you to an Ubuntu Desktop, where you can use the Terminator application
to go through [ROS2's official
tutorial](https://docs.ros.org/en/iron/Tutorials.html).

If afterwards you want to remove the docker image (since it's quite
large) enter

```bash
docker ps -a
```

Locate the `NAME` of your image in the output, and execute:

```bash
docker rm NAME
```

E.g. `docker rm practical_mclaren`.

Tiryoh's repository of the image can be found
[here](https://github.com/Tiryoh/docker-ros2-desktop-vnc). But you
don't need to clone it to use his image.

