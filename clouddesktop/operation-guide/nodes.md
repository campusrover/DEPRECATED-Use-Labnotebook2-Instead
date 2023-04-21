# Nodes

## Overview

Each nodes in the cluster is setup in a similar way. The software stack on a node looks like this:

![Software Stack](../../images/clouddesktop/arch-general.png)

## Container runtime

The default runtime is `nvidia` for all Nvidia-enabled nodes. For details, see `/etc/docker/daemon.json`.

## Software versions

*As of May 2021*

| Name | OS | K3s Version | Docker Version |
| --- | --- | --- | --- |
| robotics-rover1 | Ubuntu 20.04.1 | v1.19.5+k3s2 `(746cf403)` | Docker version 20.10.1, `build 831ebea` |
| robotics-rover2 | Ubuntu 18.04.5 | v1.19.5+k3s2 `(746cf403)` | Docker version 20.10.1, `build 831ebea` |

## Node hardwares

| Name | IP | Hardware Configuration
| --- | --- | --- |
| robotics-rover1 | rover1.cs.brandeis.edu | 12C/24T, 32GB, 1TB, RTX2060S
| robotics-rover2 | rover2.cs.brandeis.edu | 12C/24T, 32GB, 1TB, RTX2060S
