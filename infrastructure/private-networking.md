# Private Networking

```

desktop-1 \        _ _ _ _ _        / robot-1 100.89.2.122
           \      |         |      /
desktop-2 - - - - |  Relay  | - - - - robot-2 100.99.31.234
           /      |_ _ _ _ _|      \
desktop-3 /      100.64.10.101      \ robot-3 100.86.232.111

```

## Introduction

To facilitate lab@home, we have created a private network for connect robots and computers together using Tailscale. This private network allows anyone to connect to any robots on the network around the world.

## Setup robot for private networking

### Requirement

* Raspberry Pi
* Authkey (like this `tskey-123abc456`)
* Running either `Raspbian 10`, `Ubuntu 18.04`, or `Ubuntu 20.04`

*You can check your OS version by doing `lsb_release -a`*

*If you don't have an Authkey, please reach out to your TA*

### Installation

* Clone the repo: `https://github.com/pitosalas/tb3-ros.git`
* `cd tb3-ros/clients`

```bash
chmod +x pi_connect.sh

# Run the script with the authkey
sudo ./pi_connect.sh tskey-123abc456

# On successful connect, you should see this
Connected. IP address: 100.xx.xxx.xxx
```

Once the robot is successfully connected to the network, you can try to reach it via cloud desktop

```bash
ssh pi@100.xx.xxx.xxx
```

## Setup non cloud desktop for private networking

You can still connect to the private network without using cloud desktop. However, right now we have limited support for OS.

Officially supported

* `Ubuntu 18.04`
* `Ubuntu 20.04`

Unofficially supported

* `MacOS`
* Check this [list](https://pkgs.tailscale.com/stable/)

### Officially supported installation

* Clone the repo: `https://github.com/pitosalas/tb3-ros.git`
* `cd tb3-ros/clients`

```bash
chmod +x pi_connect.sh

# Run the script with the authkey
sudo ./pi_connect.sh tskey-123abc456

# On successful connect, you should see this
Connected. IP address: 100.xx.xxx.xxx
```

To connect to the network after every reboot, just do

```bash
sudo ./pi_connect.sh tskey-123abc456
```

### Unofficially supported installation

**Caution: all the instructions here have not been tested, you are on your own!!!**

#### MacOS

* Download and Install Tailscale from [App Store](https://apps.apple.com/ca/app/tailscale/id1475387142?mt=12)
* Open the App, **DO NOT login**

With your terminal do,

```bash
$(ps -xo comm | grep MacOS/Tailscale$) up --authkey=tskey-123abc456 --accept-routes
```

#### Everything else

Follow the instructions for installing for your OS [here](https://pkgs.tailscale.com/stable/).

Once you have Tailscale installed, you can start it with the following command

```bash
sudo tailscale up --authkey=tskey-123abc456 --accept-routes
```
