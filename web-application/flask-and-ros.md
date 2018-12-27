# Integrating with ROS

Our objective is to initialize ROS nodes _within_ a Flask app, enabling publishing and subscribing to ROS topics via API calls or UI interactions.

For example, when run/served, the Flask app would:

1. Initialize a node and a publisher to a teleop topic.
2. Define a Flask route as an endpoint for receiving POST requests containing data representing teleop commands \(e.g. 'forward', 'back', 'left', 'right', 'stop'\).
3. Handle POST requests to the endpoint by executing a Python function in which the ROS publisher publishes the appropriate cmd\_vel message to a teleop topic.

It's important to note that this differs from a more common approach to Web-ROS interfacing, which involves the following:

1. Establish a websocket connection between the web client and the machine running ROS, often via a 3rd party Python package called "rosbridge."
2. Within the web client's JavaScript, import a 3rd party library called "roslibjs," which provides ROS-like classes and actions for subscribing and publishing to ROS topics.
3. Unlike publishers and subscribers implemented in ROS, roslibjs sends _JSON_ to rosbridge which, in turn, publishes and subscribes to actual ROS messages. In short, rosbridge is required as an intermediary between a web client and a machine running ROS.

This has the advantage of providing a standard way for any web client to interface with ROS via JSON. However, this not only makes running rosbridge a necessity, but it also requires ROS developers to implement ROS-like programming _in JavaScript_. Flask, on the other hand, seems to offer a way to **implement ROS patterns purely in Python on** _**both**_ **client** _**and**_ **server,** _**without**_ **rosbridge and roslibjs as dependencies.**

There is an apparent obstacle to implementing ROS within Flask, though. It seems to involve the way Flask serves an app and the way ROS nodes need to be initialized. More specifically, the issue might arise from initializing a ROS node in a thread _other than_ the main thread, which seems to be the case for some of the ways Flask apps can be run/served. Others in the ROS community seem to have encountered this issue:

* [Example 1](https://answers.ros.org/question/234418/easiest-way-to-implement-http-server-that-can-send-ros-messages/)
* [Example 2](https://amp.reddit.com/r/ROS/comments/42w04t/running_a_web_server_in_ros/)
* [Example 3](http://ros-users.122217.n3.nabble.com/Discourse-ros-org-ROS-Projects-Flask-ask-ros-a-ROS-node-inside-an-Amazon-Alexa-web-service-td4027381.html)

Note that the 3rd example proposes a solution; their Flask app's main thread initializes and starts a new thread in which a ROS node is initialized:

```text
    # ROS node, publisher, and parameter.
    # The node is started in a separate thread to avoid conflicts with Flask.
    # The parameter *disable_signals* must be set if node is not initialized in the main thread.

    threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
    pub = rospy.Publisher('test_pub', String, queue_size=1)
```

However, to actually serve the app, they call `Flask.run()`:

```text
if __name__ == '__main__':
    if NGROK:
        print 'NGROK mode'
        app.run(host=os.environ['ROS_IP'], port=5000)
    else:
        print 'Manual tunneling mode'
        dirpath = os.path.dirname(__file__)
        cert_file = os.path.join(dirpath, '../config/ssl_keys/certificate.pem')
        pkey_file = os.path.join(dirpath, '../config/ssl_keys/private-key.pem')
        app.run(host=os.environ['ROS_IP'], port=5000,
                ssl_context=(cert_file, pkey_file))
```

Flask's documentation on [**Flask.run\(\)**](http://flask.pocoo.org/docs/0.12/api/) advises against using it in a production environment:

> "Do not use run\(\) in a production setting. It is not intended to meet security and performance requirements for a production server. Instead, see Deployment Options for WSGI server recommendations."
>
> "It is not recommended to use this function for development with automatic reloading as this is badly supported. Instead you should be using the flask command line script’s run support."
>
> "The alternative way to start the application is through the Flask.run\(\) method. This will immediately launch a local server exactly the same way the flask script does. This works well for the common case but it does not work well for development which is why from Flask 0.11 onwards the flask method is recommended. The reason for this is that due to how the reload mechanism works there are some bizarre side-effects \(like executing certain code twice, sometimes crashing without message or dying when a syntax or import error happens\). It is however still a perfectly valid method for invoking a non automatic reloading application."

**Solution**

Instead of using `Flask.run()` within a Flask app's main method/script, we've had success with using the following via Flask's command line interface:

```text
    flask run --no-reload
```

Without the `--no-reload` argument, the lines in which your ROS node is initialized will be executed _twice_, resulting in a ROS error stating that the node was shut down because another with the same name was initialized.

## _Brad Nesbitt & Huaigu Lin 10/31/2018_

