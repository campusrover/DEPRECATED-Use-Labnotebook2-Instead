---
title: Tailscale VPN
order: 1
status: new
---
# Tailscale VPN

We use a product called tailscale to create a vpn that allows our robots (which are on wifi) to be connected to our Rover cluster (which are connected to the network.) 

## Problem: Two Robots have the same IP or have the same or wrong name

When this occurs you need to fully uninstall Tailscale from one of the robots. This is the sequence:

```
sudo systemctl stop tailscaled
sudo rm /var/lib/tailscale/tailscaled.state
sudo systemctl start tailscaled
tailscale up
```
## More to come