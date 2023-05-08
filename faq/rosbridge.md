# **send dynamic JSON messages to ROS**

## **rosbridge_server**

"Rosbridge server creates a WebSocket connection and passes any JSON messages from the WebSocket to rosbridge_library, so rosbridge library can convert the JSON strings into ROS calls..." (to read more, see the [rosbridge_server documentation](http://wiki.ros.org/rosbridge_server))

## **roslibpy**

"Python ROS Bridge library allows to use Python and IronPython to interact with ROS, the open-source robotic middleware. It uses WebSockets to connect to rosbridge 2.0 and provides publishing, subscribing, service calls, actionlib, TF, and other essential ROS functionality..." (to read more, see the [roslibpy documentation](https://roslibpy.readthedocs.io/en/latest/))

## **installation**

to install rosbridge_server and roslibpy: 

`sudo apt install ros-noetic-rosbridge-server`

`pip install roslibpy`

## **the code**

    import roslibpy

    client = roslibpy.Ros(host='localhost', port=9090)
    client.run()

    pub = roslibpy.Topic(client, topic_name, message_type)

    pub.publish(roslibpy.Message(message_data))
    
    pub.unadvertise()
    client.terminate()

## **json format**

a json file with the format shown below where 

    "command": { 
        "receiver": "/cmd_vel",
        "type": "geometry_msgs/Twist",
        "msg" : {
            "linear": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            },
            "angular": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0
            }
        }
    }

## **to run**

    roslaunch rosbridge_server rosbridge_websocket.launch

    rosrun package_name node_name.py
