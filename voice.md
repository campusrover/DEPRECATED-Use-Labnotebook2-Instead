# Voice

The campusrover robot relies on an on-board Amazon Echo to verbally communicate with the user. The robot, via the echo, is expected to understand a varity of tasks such as motion, rotation, halting, item delivery, and navigation to specific locations.

Architecturally, spoken dialog is handled by Amazon Alexa in the cloud and by a Robotic Operating System \(ROS\) node called voice\_webhook running on the main computer alongside roscore1. The Alexa skill may be accessed and edited by logging into the [Alexa Developers Console](https://developer.amazon.com/alexa/console/ask) with the campusrover email \(deis.campusrover@gmail.com\) and the standard password \(ask Pito if you don't know\). The voice\_webook node may be found in the webhooks package on the main computer in the catkin workspace \('catkin\_ws'\) source code \('src'\).

## Alexa

Assuming you successfully made it to the [Alexa Developers Console](https://developer.amazon.com/alexa/console/ask) and logged in \(see above\), you should see a heading titled _Alexa Skills_, under which you should see only one skill, _Campusrover_. The term 'skill' is a bit misleading — you may prefer to think of it as an agent rather than as a command \(each particular command is an 'intent' within the skill\). This is the skill we use to communicate with the robot. Click on the _Campusrover_ skill to access it.

On the left sidebar, you should first see a button titled 'Invocation'. This basically determines what will trigger your skill. It is currently set to 'campusrover', and so may be triggered by saying to the echo, "echo tell campusrover to ...".

Next you should see a heading titled 'Intents', under which the various intents are listed. Each intent consists of sample phrases for the natural language processor to understand. Notice that many of these have embedded variables, termed 'slot types', so that the parser learns to recognize, for instance, both "echo tell campusrover to move forward 5 feet" and "echo tell campusrover to move back 7 meters".

These slot types may appropriately be found under the 'Slot Types' heading below 'Intents' on the left sidebar. You may want to look over the intents and slot types to understand how they work, how they interact, and how they have been set up for campusrover.

Under this on the left sidebar, you should eventually see a button titled 'Endpoint'. This is where the skill connects to the ROS node. On this page, the 'HTTPS' radio button should be selected \(as opposed to 'AWS Lambda ARN'\). Next to the 'Default Region' label, you should see a url of the format '[https://\[Random](https://[Random) String\].ngrok.io/alexa\_webhook'. This is the ngrok webhook Alexa uses to communicate with the ROS node \(more on this later\). If you ever have to change this URL, you must rebuild your entire model by clicking on the 'Build' tab at the top of the page and then clicking on the '3. Build Model' button on the right side of the page under the 'Skill builder checklist' heading. Furthermore, if you decide to use an ngrok URL from another computer \(which I won't recomend\), you must also upload a new certificate in the endpoint \(you may download this by typing your ngrok url \(more on this later\) into chrome, clicking on the little lock icon to the left of the URL text box, and downloading the certificate.

## Ngrok

Ngrok is basically a service which creates a public URL for your computer's localhost. In our case, it allows Alexa to communicate with a node. To run ngrok, first ensure it is downloaded onto the appropriate computer \(it should be for the main computer with roscore1, where I recommend you run it and the node\). Go to the terminal, change directory \(cd\) to wherever ngrok is located \(home in the case of the main computer\), and run the command `./ngrok http 5000`. You should a url of the format '[https://\[Random](https://[Random) String\].ngrok.io/' \(make sure to choose the one that begins with https\). This URL, with "\alexa\_webhook" appended to it, is the URL Alexa will use to connect to the webhook node \(more on this later\). I would recomend having this constantly running alongside roscore1 and the voice webhook node on the main computer to avoid having to keep updating the URL in the Alexa skill endpoint \(closing and reopening ngrok will change the URL\).

## Voice Webhook Node

The voice\_webhook node is a flask app \(and thus will require that flask is installed on the computer it is running on, as it should be on the main computer\). Furthermore, as it relied on ngrok to make itself available to Alexa, it must be running on the same computer as ngrok. Look over the node python file to get an understanding of how it works. Alexa will basically send over a rather large and convoluted JSON string representing a user intent every time the user commands the skill to do something. This node will basically parse the JSON string, produce a new, simplified JSON string, publish it to the voice\_intents topic \(in most cases\) for other nodes to execute, and return a reply for the echo to verbally say to the user.

