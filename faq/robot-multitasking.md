---
title: robot multitasking
author: James Lee
description: How can I make my robot do more than one thing at once, while being in the same state?
status: new
date: may-2023
---

## Question
How can I make my robot do more than one thing at once, while being in the same state?

## Background

It's common for the driving logic of a ROS program to be structured like this:

```
rate = rospy.Rate(20)

while not rospy.is_shutdown():
  # your code
  rate.sleep()
```
Here, `your code` runs at most 20 times per second (it can run less frequently if `your code` takes longer than 1/20<sup>th</sup> of a second to execute). This is a useful framework, especially if you want to have your robot continually check for its current state and execute code accordingly.

For example, in the code below, the robot executes different functions depending on whether its state is that of `follower` or `leader`.

```
rate = rospy.Rate(20)
while not rospy.is_shutdown():
  if robot_state == 'follower':
    follow()
  elif robot_state == 'leader':
    lead()
  rate.sleep()
```

But suppose your robot must do more than one thing at once, that conceptually falls under its responsibilities as a `leader`. For example, it might have to execute some `complex_function`, while at the same time publishing messages over a topic.

In my case, my robot had to publish messages to other robots, letting it know that it was the leader, while at the same time telling the robots where to go.

One solution to this would be to write a whole new ROS node that publishes the required messages (in its own `while not rospy.is_shutdown` loop), and have `complex_function` run on the current node. But this separates into two processes what belongs as a logical unit, and also carries with it the overhead of having to launch another ROS node.

## Answer

A simpler solution is to use multithreading. For example:

```
def lead():
  if send_messages_thread is None:
    send_messages_thread = Thread(target = send_messages, daemon = True)
    send_messages_thread.start()
  else:
    complex_function()

def send_messages():
  while True:
    your_publisher.publish('message')
```
===WARNING: This is pseudocode, since, among other reasons, we didn't define the variable `send_messages_thread`. In real code, `send_messages_thread` should probably be an attribute of your robot, which you should define as a python `Class` (along the same lines, `robot_state` above should also be an attribute).===

This code first checks if the variable `send_messages_thread` has been initialized. If it hasn't, it defines a thread that, when started in `send_messages_thread.start()`, executes the function `send_messages`. Note that the thread is defined as a `daemon` thread, which shuts down when your current thread (the one in which you're defining the `send_messages_thread` shuts down).

## Upshot

This is the basis of a design framework you can apply to make your robot multitask appropriately. It's only the basis, because by leveraging a couple more items in python's concurrency library you can make your robot multitask in more sophisticated ways.

For example, suppose you have a mechanism which allows for communication between threads. By this device, thread A can tell thread B that something has happened that should cause B to act differently, or to simply terminate.

This would open up a lot of doors. In our example above, for example, you would be able to shut down the `send_messages_thread` at will by having the while loop of `send_messages` check for the communication from thread A that an event has happened:

```
def send_messages():
  while not threadA_told_me_to_stop:
    your_publisher.publish('message')
```

But this mechanism that allows for threads to communicate with each other is just what is provided by python's `Event` object. Together with the `join` function that allows a thread to wait for another thread to finish, you can do a suprising variety of multitasking.