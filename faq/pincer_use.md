---
title: Using the Pincer Attachment
author: Matthew Merovitz
description: How to use pincer attachment on platform2
status: new
---
## Using platform2 pincer attachment

In order to use the pincer attachment, you must have a way publish to the servo motor.

![image](https://user-images.githubusercontent.com/47371441/236552674-41450e72-5fad-4752-8a4d-e8debf7c2a85.png)

## Publisher

First, you must import Bool. True will be to open the attachment, and False will be to close the attachment. 

``` from std_msgs.msg import Bool ```

The following publisher needs to be published to:

```rospy.Publisher('/servo', Bool, queue_size=1)```

By publishing to the servo motor, you are telling it to either open or close. If you publish True, the pincer will open. If you publish False, the pincer will close.

```
def open():
    self.pub.publish(Bool(True))

def close():
    self.pub.publish(Bool(False))
```

This has use beyond this pincer itself. By having any custom attachment with a servo motor, this provides an easy way to publish to it.

## Full example of the pincer class

```
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool

class Pincer:
    """ Allows pincer to open and close """

    def __init__(self):
        self.pub = rospy.Publisher('/servo', Bool, queue_size=1)

    def open(self):
        self.pub.publish(Bool(True))

    def close(self):
        self.pub.publish(Bool(False))
```
