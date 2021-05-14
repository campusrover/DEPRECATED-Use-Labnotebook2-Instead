# Configuration

As of May 2021, the cloud desktop cluster is consisted of 2 physical nodes.

## Hardware

| Name | IP | Hardware Configuration | Role |
| --- | --- | --- | --- |
| robotics-rover1 | rover1.cs.brandeis.edu | 12C/24T, 32GB, 1TB, RTX2060S | Master |
| robotics-rover2 | rover2.cs.brandeis.edu | 12C/24T, 32GB, 1TB, RTX2060S | Node |

## Software Versions

The software stack is simple, each node run a K3s server managed by systemd.

| Name | OS | K3s Version | Docker Version |
| --- | --- | --- | --- |
| robotics-rover1 | Ubuntu 20.04.1 | v1.19.5+k3s2 `(746cf403)` | Docker version 20.10.1, `build 831ebea` |
| robotics-rover2 | Ubuntu 18.04.5 | v1.19.5+k3s2 `(746cf403)` | Docker version 20.10.1, `build 831ebea` |
