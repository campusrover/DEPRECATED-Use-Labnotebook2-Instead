---
title: "Advanced: Help me troubleshoot weird camera problems"
description: For advanced users who are trying to tix weird problems
author: Pito Salas
date: may-2023
status: new
type: faq
---
# Camera fails to initialize:

Ensure that the camera is enabled and there is enough GPU memory: 

1. Add the following lines to the `/boot/firmware/config.txt` file:

```
start_x=1
gpu_mem=128

```

And then reboot. Note that the config.txt file will say that you should not modify it and instead make the changes in a file called userconfig.txt. However I found that the userconfig.txt file is not invoked. So I added the lines above directly to config.txt.

1. Check the amount of gpu_memory

```
vcgencmd get_mem gpu
```
It should show 256 or whatever number you put there.

