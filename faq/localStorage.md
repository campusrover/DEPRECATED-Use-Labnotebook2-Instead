---
title: Working with localStorage in React for web clients
date: may-2023
status: new
author: James Kong
---

# Working with localStorage in React for web clients

**Author:** James Kong  
**Emails:** jameskong@brandeis.edu, jameskong098@gmail.com  
**GitHub:** [jameskong098](https://github.com/jameskong098)  
**LinkedIn:** [James Deming Kong](https://www.linkedin.com/in/jamesdemingkong/)

## What is localStorage?

localStorage is a web storage object that enables developers to store key-value pairs in a web browser. It ensures that this data survives all page refreshes, even when a user closes or restarts a browser. It does not have an expiration date.

## Why is it useful?

localStorage can be used for settings pages, such as the one used in my team's project [Command & Control](https://github.com/campusrover/command-control). While storing setting configurations within an online database like MongoDB is an option, it is not always necessary, especially when a user account system is not required. Saving settings in localStorage allows the website to function properly regardless of where it is accessed.

## Approach for using localStorage for a settings page

It is recommended to create a default configuration file that lists a set of default values in case no setting configurations have been stored in localStorage. Below is an example of a config file used in our project:

```javascript
let ros_config = {
    ROSBRIDGE_SERVER_IP: "127.0.0.1",
    ROSBRIDGE_SERVER_PORT: "9090",
    RECONNECTION_TIMEOUT: 3000,
    CHECK_IMAGE_CONFIG: 3000,
    ROSBRIDGE_BATTERY_STATE_THROTTLE: 5000,
    ROSBRIDGE_CMD_VEL: "/cmd_vel",
    ROSBRIDGE_ODOM: "/odom",
    ROSBRIDGE_CAMERA_TOPIC: "/camera/rgb/image_raw/compressed",
    ROSBRIDGE_RASPICAM_TOPIC: "/raspicam_node/image_res/compressed",
    ROSBRIDGE_IMAGE_CONFIGS: "/image_configs",
    ROSBRIDGE_ROSTOPICS_LIST: "/rostopic_list",
    ROSBRIDGE_IMAGE_WIDTH: "426",
    ROSBRIDGE_IMAGE_HEIGHT: "240",
    ROSBRIDGE_FRAME_WIDTH: 426,
    ROSBRIDGE_FRAME_HEIGHT: 240,
    ROSBRIDGE_BATTERY_TOPIC: "/battery_state",
    ROSBRIDGE_MANUAL_TELEOP: false,
    ROSBRIDGE_BATTERY_STATUS: true,
}

export default ros_config
```

In the example below, we use a default value that is defined beforehand from the ros_config file. The logic states that if no value exists within local storage,
then use the value from the ros_config value. You can just name the local storage value any name just make sure it stays consistent when you try to update it later on.

```javascript
this.state = {
    ...
    rosbridgeServerIP: localStorage.getItem('rosbridgeServerIP') || ros_config.ROSBRIDGE_SERVER_IP,
    rosbridgeServerPort: localStorage.getItem('rosbridgeServerPort') || ros_config.ROSBRIDGE_SERVER_PORT,
    imageWidth: localStorage.getItem('imageWidth') || ros_config.ROSBRIDGE_IMAGE_WIDTH,
    imageHeight: localStorage.getItem('imageHeight') || ros_config.ROSBRIDGE_IMAGE_HEIGHT,
    frameWidth: localStorage.getItem('frameWidth') || ros_config.ROSBRIDGE_FRAME_WIDTH,
    frameHeight: localStorage.getItem('frameHeight') || ros_config.ROSBRIDGE_FRAME_HEIGHT,
    batteryStatus: localStorage.getItem('batteryStatus') !== null ? localStorage.getItem('batteryStatus') === "true" : ros_config.ROSBRIDGE_BATTERY_STATUS,
    manualTeleop: localStorage.getItem('manualTeleop') !== null ? localStorage.getItem('manualTeleop') === "true" : ros_config.ROSBRIDGE_MANUAL_TELEOP,
    ...
}
```

After you define the logic for determining what value to use upon boot, you will want to also know how to change and/or clear the values from local storage for maybe
implementing a save button or a clear button.

To update the local storage value, you will want to use the following function (with the first parameter being the name of the local storage value and the second being the value you want to be used for replacing the local storage value):

```
localStorage.setItem('rosbridgeServerIP', this.state.rosbridgeServerIP);
```

To clear the values within local storage, you will want to use the following function:
```javascript
localStorage.clear();
```