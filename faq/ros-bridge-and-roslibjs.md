---
title: ROSBridge and ROSLIBJS
author: Naimul Hasan
description: An overview of how ROSBridge and ROSLIBJS works/utilized for command control
---
# Overview of ROSBridge and ROSLIBJS
This FAQ is deigned to provide an overview of ROSBridge and ROSLIBJS, two important tools for integrating web applications, such as the command control dashboard, with the Robot Operating System (ROS).

## Table of contents
* [What are ROSBridge and ROSLIBJS?](#what-are-rosbridge-and-roslibjs)
* [How do ROSBridge and ROSLIBJS work together?](#how-do-rosbridge-and-roslibjs-work-together)
* [How do I install and use ROSBridge and ROSLIBJS?](#how-do-i-install-and-use-rosbridge-and-roslibjs)
* [What are some common use cases for ROSBridge and ROSLIBJS?](#what-are-some-common-use-cases-for-rosbridge-roslibjs)
* [Are there any limitations or challenges when using ROSBridge and ROSLIBJS?](#challenges-limitations-of-rosbridge-and-roslibjs)

<a name="what-are-rosbridge-and-roslibjs"></a>

## What are ROSBridge and ROSLIBJS?

ROSBridge is a package for the Robot Operating System (ROS) that provides a JSON-based interface for interacting with ROS through WebSocket protocol (usually through TCP). It allows external applications to communicate with ROS over the web without using the native ROS communication protocol, making it easier to create web-based interfaces for ROS-based robots.

ROSLIBJS is a JavaScript library that enables web applications to communicate with ROSBridge, providing a simple API for interacting with ROS. It allows developers to write web applications that can send and receive messages, subscribe to topics, and call ROS services over websockets.

<a name="how-do-rosbridge-and-roslibjs-work-together"></a>

## How do ROSBridge and ROSLIBJS work together?

ROSBridge acts as a bridge between the web application and the ROS system. It listens for incoming WebSocket connections and translates JSON messages to ROS messages, and the other way around. ROSLIBJS, on the other hand, provides an API for web applications to interact with ROSBridge, making it easy to send and receive ROS messages, subscribe to topics, and call services.

In a typical application utilizing ROSBridge and ROSLIBJS, it would have the following flow:

1. Web client uses ROSLIBJS to establish a WebSocket connection to the ROSBridge server, through specifying a specific IP address and port number.
2. The web client sends JSON messages to ROSBridge, which converts them to ROS messages and forwards them to the appropriate ROS nodes.
3. If the node has a reply, the ROS nodes send messages back to ROSBridge, which converts them to JSON and sends them over the WebSocket connection to the web client.
4. ROSLIBJS API processes the incoming JSON messages so it can be displayed/utilized to the web client

<a name="how-do-i-install-and-use-rosbridge-and-roslibjs"></a>

## How do I install and use ROSBridge and ROSLIBJS?

### ROSBridge Installation

The following assumes that you have ROS Noetic installed on your system.

To install ROSBridge you need to install the `rosbridge-server` package

```bash
sudo apt install ros-noetic-rosbridge-server
```
### ROSLIBJS Installation

You can either use the hosted version of ROSLIBJS or download it for use in your project. To use the hosted version, include the following script tag in your HTML file:
```html
<script src="https://static.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
```

To download ROSLIBJS, visit the [Github repository builds](https://github.com/RobotWebTools/roslibjs/tree/develop/build) and download and save the files. To use the local version, include the following script tag in your HTML file:
```html
<script src="PATH-TO-DOWNLOADED-SCRIPT"></script>
```

#### Simple Example 

The following example is for React Application 

First, download the `roslib.min.js` file from the ROSLIBJS GitHub repository and place it in your project's public directory.

Next, create a new React component called `RosConnect`:

**RosConnect.jsx**
```javascript

```