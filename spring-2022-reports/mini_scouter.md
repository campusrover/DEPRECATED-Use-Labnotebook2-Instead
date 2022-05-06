# Final Deliverable

# MiniScout **Project**

Team: Nazari Tuyo (nazarituyo@brandeis.edu) and Helen Lin (helenlin@brandeis.edu)

COSI 119a Fall 2022, Brandeis University

Date: May 4, 2022

Github repo: [Mini Scouter](https://github.com/campusrover/mini_scouter)
---

# **Introduction**

## **Problem Statement (including original objectives)**

If humans can’t enter an area because of unforeseen danger, what could be used instead? We created MiniScouter to combat this problem. The goal of our project was to create a robot that can be used to navigate or “scout” out spaces, with directions coming from the Leap Gesture Controller or voice commands supported by Alexa. The turtlebot robot takes in commands through hand gestures or voice, and interprets them. Once interpreted, the robot preforms the action requested.****

## **Relevant Literature**

We referred to several documentations for the software used in this project:

[Boto3 Python Documentation](https://aws.amazon.com/sdk-for-python/)

[Boto3 Simple Queue Service Documentation](https://boto3.amazonaws.com/v1/documentation/api/latest/reference/services/sqs.html)

[Leap Motion Python SDK Documentation](https://developer-archive.leapmotion.com/documentation/python/index.html)

[Alexa Skill Developer Design Guide](https://developer.amazon.com/en-US/alexa/alexa-haus)

[Python Lambda Documentation](https://docs.aws.amazon.com/lambda/latest/dg/lambda-python.html)

[ROS Documentation](http://wiki.ros.org)

---

# **What was created**

We designed and created a voice and gesture controlled tele-operated robot. Voice control utilizes Alexa and Lambda integration, while gesture control is supported with the Leap Motion Controller. Communication between Alexa, the controller, and the robot is all supported by AWS Simple Queue Service integration as well as boto3.

## **Technical Description, illustrations**

### Leap Motion

The Leap Motion Controller

![Imgur](https://i.imgur.com/EFEI5aM.png)

The Leap Motion Controller is an optical hand tracking module that captures hand movements and motions. It can track hands and fingers with a 3D interactive zone and identify up to 27 different components in a hand. The controller can be used for desktop based applications (as this project does) or in virtual reality (VR) applications.

The controller use an infrared light based stereoscopic camera. It illuminates the space near it and captures the user’s hands and fingers. The controller then uses a tracking algorithm to estimate the position and orientation of the hand and fingers. The range of detection is about 150° by 120° wide and has a preferred depth of between 10cm and 60cm, but can go up to about 80cm maximum, with the accuracy dropping as the distance from the controller increases. 

![Imgur](https://i.imgur.com/vjjx8yf.jpg)

The Leap Motion Controller maps the position and orientation of the hand and fingers onto a virtual skeletal model. The user can access data on all of the fingers and its bones. Some examples of bone data that can be accessed is shown below.

![00308_psisdg9946_99460p_page_7_1.jpg](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/00308_psisdg9946_99460p_page_7_1.jpg)

Alongside hand and finger detection, the Leap Motion Controller can additionally track hand gestures. The Leap Software Development Kit (SDK) offers support for four basic gestures:

- Circle: a single finger tracing a circle
- Swipe: a long, linear movement of a finger
- Key tap: a tapping movement by a finger as if tapping a keyboard key
- Screen tap: a tapping movement by the finger as if tapping a vertical computer screen

![00308_psisdg9946_99460p_page_8_1.jpg](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/00308_psisdg9946_99460p_page_8_1.jpg)

The controller also offers other data that can be used to manipulate elements:

- Grab strength: the strength of a grab hand pose in the range [0..1]
    - An open hand has a grab strength of zero and a closed hand (fist) has a grab strength of one
- Pinch strength: the strength of a pinch pose between the thumb and the closet finger tip as a value in the range [0..1]
    - As the tip of the thumb approaches the tip of a finger, the pinch strength increases to one

The Leap Motion Controller offers much functionality and a plethora of guides online on how to use it. It’s recommended that the user keep the implementation of an application simple as having too many gestures doing too many things can quickly complicate an algorithm.

**Leap Motion Setup**

The Leap Motion Controller can be connected to the computer using a USB-A to Micro USB 3.0 cable. After it’s connected, you’ll need to get the UltraLeap hand tracking software, available [here](https://www.leapmotion.com/setup/). You may need to use older versions of the software since V4 and V5 only offer support and code in C. We found that V2 of the UltraLeap software best fit our needs as it offered Python support and still included a robust hand-tracking algorithm. We installed the necessary software and SDK for V2 from [here](https://developer-archive.leapmotion.com/get-started). Once we had all the necessary hardware and software set-up, we used the UltraLeap Developer documentation for Python at [this site](https://developer-archive.leapmotion.com/documentation/python/index.html?proglang=python) to begin creating the algorithm for our project.

To start using the Leap Motion Controller, you’ll need to connect the device to your computer via a USB-A to Micro USB 3.0 cable. You’ll need to install the Leap software from their website at [https://developer.leapmotion.com/](https://developer.leapmotion.com/). We found that we had to install older versions of the tracking software as the latest versions (V4 and V5) only supported C and that the Python version had been deprecated. For our needs, V2 worked best as it offered support for creating Python and came with a robust tracking system. After that, you’ll need to install the Leap SDK, with more details on how to set this up below. (See **Setup** for more details on how to set up Leap for this project specifically)

### Alexa Skills Kit

A widely known IOT home device, Amazon Alexa is Amazon’s cloud-based voice service, offering natural voice experiences for users as well as an advanced and expansive collection of tools and APIs for developer use. 

When a phrase is said to Alexa, it’s first processed through the Alexa service, which uses natural language processing to interpret the users “intent” and then the “skill logic” (as noted down below) handles any further steps once the phrase has been interpreted.

![Untitled](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Untitled%201.png)

We created an Alexa skill for this project as a bridge between the user and the robot for voice commands. The Alexa skill also utilizes AWS Lambda, a serverless, event-driven computing platform, for it’s intent handling. The Lambda function sends messages to the AWS SQS Queue that we used for Alexa-specific motion commands. 

For example, if a user makes a request, saying “**Alexa, ask mini scout to move forward”**, the Alexa service identifies the user intent as “MoveForward”. Once identified, the lambda function is activated, and it uses a “handler” specific to the command to send a dictionary message to the queue.

![Screen Shot 2022-05-04 at 5.13.26 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-05-04_at_5.13.26_PM.png)

### AWS Simple Queue Service

AWS Simple Queue Service is a managed message queuing service used to send, store and retrieve multiple messages for large and small scale services as well as distributed systems. 

The robot, Alexa, and the Leap motion controller all utilize the AWS Simple Queue Service to pass commands to each other. There are two options when creating queues, a standard queue, and a “FIFO” queue - first in first out, i.e. messages will only be returned in the order they were received. We created two FIFO queues, one for Leap, called “LeapMotionQueue”, and one for Alexa, called “AlexaMotionQueue”. 

In the Alexa lambda function and the Leap script, a boto3 client is created for SQS, which is used to connect them to the queue. 

On VNC, the robot makes the same connection to SQS, and since all of our work is carried out through one AWS account, it’s able to access the data pushed to both queues upon request.

### Boto3

Boto3 is the latest version of the AWS SDK for python. This SDK allows users to integrate AWS functionalities and products into their applications, libraries and scripts. As mentioned above, we used Boto3 to create a SQS ‘client’, or connection, so that our Leap script and our Alexa skill could access the queues. 

## **Discussion of interesting algorithms, modules, techniques**

### Working with AWS

Initially, we planned on having the Leap Motion Controller plugged directly into the robot, but after some lengthy troubleshooting, it was revealed that the Raspberry Pi the robot uses would not be able to handle the Leap motion software. Instead of giving up on using Leap, we thought of other ways that would allow communication between our controller and the robot. 

At the suggestion of a TA (thanks August!) to look into AWS, we looked for an AWS service that might be able to support this type of communication. The Simple Queue Service was up to the task - it allowed our controller to send information, and allowed the robot to interpret it directly. (Deeper dive into this issue in the **Problems/Pivots** section)

### Leap Motion Gestures

One of the first and most important design choices we had to consider on this project was which gestures we should use to complete certain actions. We had to make sure to use gestures that were different enough so that the Leap Motion Controller would not have difficulty distinguishing between two gestures, as well as finding gestures that could encompass all the actions we wanted to complete. We eventually settled on the following gestures that were all individual enough to be identified separately:

- Hand pitch forward (low pitch): move backward
- Hand pitch backward (high pitch): move forward
- Clockwise circle with fingertip: turn right
- Counter-clockwise circle with fingertip: turn left
- Grab strength 1 (hand is a fist): stop all movement

Another aspect we had to take into consideration when creating the algorithm was how much to tune the values we used to detect the gestures. For example, with the gesture to move forward and backward, we had to decide on a pitch value that was not so sensitive that other gestures would trigger the robot to move forward/backward but sensitive enough that the algorithm would know to move the robot forward/backwards. We ran the algorithm many times with a variety of different values to determine which would result in the greatest accuracy across all gestures. Even with these considerations, the algorithm sometimes was still not the most accurate, with the robot receiving the wrong command from time to time. 

![Screen Shot 2022-05-04 at 11.40.51 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-05-04_at_11.40.51_PM.png)

We implemented this algorithm on the local side. Within the code, we created a Leap Motion Listener object to listen to commands from the Leap Motion Controller. The Leap Listener takes in data such as number of hands, hand position in an x, y, z plane, hand direction, grab strength, etc. This data is then used to calculate what gestures are being made. This information then makes a request to the queue, pushing a message that looks like the following when invoked:

```jsx
sqs_message = {
							"Current Time":current_time.strftime("%H:%M:%S"),
							"Motion Input": "Leap", 
							"Motion": motion
							}
```

On the VNC side, the motion is received from the SQS queue and turned back into a dictionary from a string. The VNC makes use of a dictionary to map motions to a vector containing the linear x velocity and angular z velocity for twist. For example, if the motion is “Move forward,” the vector would be [1,0] to indicate a linear x velocity of 1 and an angular z velocity of 0. A command is published to cmd_vel to move the robot. The current time ensures that all of the messages are received by the queue as messages with the same data cannot be received more than once. 

### Alexa Skill and Lambda

For the Alexa skill, many factors had to be considered. We wanted to implement it in the simplest way possible, so after some troubleshooting with Lambda we decided on using the source-code editor available in the Alexa developer portal. 

In terms of implementing the skill, we had to consider the best way handle intents and slots that would not complicate the way messages are sent to the queue.  We settled on giving each motion an intent, which were the following:

- MoveForward
- MoveBackward
- RotateLeft
- RotateRight
- Stop

In order for these ‘intents’ to work with our robot, we also had to add what the user might say so that the Alexa service could properly understand the intent the user had. This meant adding in several different “utterances”, as well as variations of those utterances to ensure that the user would be understood. This was very important because each of our intents had similar utterances. MoveForward had “move forward”, MoveBackward has “move backwards”, and if the Alexa natural language processor only has a few utterances to learn from it could easily confuse the results meant for one intent for a different intent. 

![Screen Shot 2022-05-04 at 8.38.23 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-05-04_at_8.38.23_PM.png)

Once the intent was received, the Lambda function gets to work. Each handler is a class of it’s own. This means that if the intent the Alexa service derives is “RotateRight”, it invokes only the “RotateRightHandler” class. This class makes a request to the queue, pushing a message that looks like the following when invoked. 

```python
sqs_message = {
							"Motion Input": "Alexa", 
							"Intent": intent_name, 
							"Current Time":current_time.strftime("%H:%M:%S")
							}
```

Once this reaches VNC, the ‘intent_name’ - which would be a string like ‘MoveForward’ - is interpreted by converting the string into a dictionary, just like the Leap messages. We had to add the current time as well, because when queue messages are identical they are grouped together, making it difficult to pop them off the queue when the newest one arrives. If a user requested the robot to turn right twice, the second turn right message would not make it to the queue in the same way as the first one since those message are identical. Adding the time makes each message to the queue unique - ensuring it reaches the robot without issue.

### VNC

The way that motions would be handled from two different sources in VNC was a huge challenge at one point during this project. Once we decided we’d incorporation voice control into our robots motion, it was clear that switching back and forth between leap commands and voice commands would be an issue. At first, we thought we’d try simultaneous switching between the two, but that quickly proved to be difficult due to the rate in which messages are deleted from each queue. It was hard to ensure that each command would used while reading from both of the queues, as messages could arrive at the same time. So we made the executive decision that voice commands take priority over Leap commands.

This means that when an Alexa motion is pushed onto the queue, if the script that’s running on VNC is currently listening to the Leap queue, it will pause the robot and proceed to listen to commands from the Alexa queue until it receives a stop command from Alexa.

This simplified our algorithm, allowing for the use of conditionals and calls to the SQS class we set up to determine which queue was to be listened to. 

On the VNC, there are 3 classes in use that help the main function operate.

- `MiniScoutAlexaTeleop`
- `MiniScoutLeapTeleop`
- `SQS_Connection`

The `MiniScoutMain` file handles checking each queue for messages and uses conditionals to determine which queue is to be listened to as well as making calls to the two teleop classes for twist motion calculation.

The `SQS_Connection` class handles queue connection, and has a few methods that assist the main class with it. The method `has_queue_message()` returns whether the requested queue has a message or not. This method makes a request to the selected queue, asking for the attribute `ApproximateNumberOfMessages` . This was the only way we could verify how many messages were present in the queue, but it proved to be a challenge as this request only returns the **approximate** number of messages in the queue. At any time during the request could the number change. This meant that we had to set a couple of time delays in the main script as well as checking the queue more than once to accomodate for this possibility. The method `get_sqs_message()` makes a request to the queue for a message, but only is called in the main function if `has_queue_message()` returns `True` . This helps insure that the request does not error out and end our scripts execution. 

The `MiniScoutAlexaTeleop` class handles incoming messages from the Alexa queue and converts them into twist messages based on the intent received in the dictionary that are then published to `cmd_vel` once returned.

The `MiniScoutLeapTeleop` class takes the dictionary that is received from the SQS queue. It makes use of this dictionary to map motions to a vector containing the linear x velocity and angular z velocity for twist, and then returns it. 

---

## Setup (g**uide on how to use the code)**

1. **Clone the Mini Scout repository (Be sure to switch to the correct branch based on your system)**
    1. [https://github.com/campusrover/mini_scouter](https://github.com/campusrover/mini_scouter.git)
2. **Install the Leap Motion Tracking SDK**
    1. Mac (V2): [https://developer-archive.leapmotion.com/v2](https://developer-archive.leapmotion.com/v2)
    2. Windows (Orion, SDK included): [https://developer.leapmotion.com/tracking-software-download](https://developer.leapmotion.com/tracking-software-download)
    
    ### **Mac OS Installation**
    
    1. Note: The MacOS software does not currently work with macOS Monterey, but there is a hack included below that does allow it to work
        
        [https://developer.leapmotion.com/tracking-software-download](https://developer.leapmotion.com/tracking-software-download)
        
    2. Once installed, open the Leap Motion application to ensure correct installation. 
        
        ![Screen Shot 2022-04-08 at 2.31.07 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-04-08_at_2.31.07_PM.png)
        
        A window will pop up with controller settings, and make sure the leap motion icon is present at the top of your screen.
        
    3. Plug your leap motion controller into your controller via a USB port.
        
        ![Screen Shot 2022-04-08 at 2.32.57 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-04-08_at_2.32.57_PM.png)
        
    4. From the icon dropdown menu, select visualizer. 
        
        ![Screen Shot 2022-04-08 at 2.36.23 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-04-08_at_2.36.23_PM.png)
        
        The window below should appear.
        
        ![Screen Shot 2022-04-08 at 2.36.52 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-04-08_at_2.36.52_PM.png)
        
    5. Note: If your MacOS software is an older version than Monterey, skip this step. Your visualizer should display the controller’s cameras on it’s own. 
        
        Restart your computer, leaving your Leap controller plugged in. Do not quit the Leap Application. (so it will open once computer is restarted)
        
    6. Once restarted, the computer will try to configure your controller. After that is complete, the cameras and any identifiable hand movement you make over the controller should appear.
    
    ### Windows Installation
    
    1. Note: the Leap Motion Controller V2 is not compatible with a certain update of Windows 10 so you’ll have to use the Orion version of the Leap software
    2. With V2, you can go into Program Files and either replace some of the .exe files or manually change some of the .exe files with a Hex editor
    3. However, this method still caused some problems on Windows (Visualizer could see hands but code could not detect any hand data) so it is recommended that Orion is used
        1. Orion is slightly worse than V2 at detecting hand motions (based off of comparing the accuracy of hand detection through the visualizer
    4. Orion is fairly simple to set up; once you install the file, you just run through the installation steps after opening the installation file
        1. Make sure to check off “Install SDK” when running through the installation steps
    5. After the installation is completed, open “UltraLeap Tracking Visualizer” on your computer to open the visualizer and make sure the Leap Motion Controller is connected
        
        The window below should look like this when holding your hands over the Leap Motion Controller:
        
        ![Untitled](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Untitled%202.png)
        

1. **AWS Setup**
    1. Create a AWS Account (if you have one already, skip this step)
    2. Create an SQS Queue in AWS
        1. Search for “Simple Queue Service”
        2. Click “Create Queue”
            
            ![Screen Shot 2022-04-08 at 2.51.39 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-04-08_at_2.51.39_PM.png)
            
        3. Give your queue a name, and select ‘FIFO’ as the type.
            
            ![Screen Shot 2022-04-08 at 2.52.20 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-04-08_at_2.52.20_PM.png)
            
        4. Change “Content-based deduplication” so that its on. There is no need to change any of the other settings under “Configuration”
            
            ![Screen Shot 2022-05-04 at 9.14.14 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-05-04_at_9.14.14_PM.png)
            
        5. Click “Create Queue”. No further settings need to be changed for now.
            
            **This is the type of queue you will use for passing messages between your robot, the leap controller, alexa and any additional features you decide to incorporate from the project.**
            
        6.  **Creating your Access Policy**
            1. Click “Edit”
            2. Under “Access Policy”, select advanced, and then select “Policy Generator”
                
                ![Screen Shot 2022-05-04 at 9.17.25 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-05-04_at_9.17.25_PM.png)
                
            3.  This will take you to the AWS Policy Generator. The Access Policy you create will give your Leap software, Alexa Skill and robot access to the queue.
            4.  Add your account ID number, which can be found under your username back on the SQS page, and your queue ARN, which can also be found on the SQS page.
                
                ![Screen Shot 2022-05-04 at 9.25.41 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-05-04_at_9.25.41_PM.png)
                
                1. Select “Add Statement”, and then “Generate Policy” at the bottom. Copy this and paste it into the Access Policy box in your queue (should be in editing mode).
        7. Repeat step 1-5 once more, naming the queue separately for Alexa.
    3. Create an AWS access Key (Start at “****Managing access keys (console)”****)
        
        [https://docs.aws.amazon.com/IAM/latest/UserGuide/id_credentials_access-keys.html](https://docs.aws.amazon.com/IAM/latest/UserGuide/id_credentials_access-keys.html)
        
        Save the details of your AWS access key somewhere safe.
        
    4. Installing AWS CLI, boto3
        1. Follow the steps at the link below:
            
            [https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html)
            
        2. Once completed, downgrade your pip version with:
            
            `sudo easy_install pip==20.3.4`
            
            (this is needed since the LEAP portion of the project can only run on python2.7)
            
        3.  Run the following commands
            1. `pip install boto3`
            2. `pip freeze`
            3. check that `boto3` is actually there!
            4. `python -m pip install --user boto3`
2. **Adding your credentials to the project package**
    1. In your code editor, open the cloned gesture bot package
    2. navigate to the “credentials” folder
    3. using your saved **AWS Access Key** info, edit the fields 
        - Your “`OWNER_ID`” can be found in the top right hand corner of your aws console
        - “`lmq_name`” is the name of your queue + “`.fifo`”
            
             i.e. `leapmotionqueue.fifo`
            
    4. change the name of `Add_Credentials.py` to `Credentials.py`
    5. You will need to do this step with each credentials file in each package
        
        **IMPORTANT**
        
        UNDER NO CONDITION should this file be uploaded to github or anywhere else online, so after making your changes, run `nano .git/info/exclude` and add `Credentials.py`
        
        Please make sure that it is excluded by running `git status` and then making sure it’s listed under `Untracked files`
        
3. **Running the script**
    
    **Before running this script, please make sure you have Python2.7 installed and ready for use**
    
    1. navigate to the ‘`scripts`' folder and run ‘`python2.7 hello_robot_comp.py`'
    2. If everything is installed correctly you should see some output from the controller!
    3. to run the teleop script, run ‘`LeapGestureToSQS.py`’ while your controller is plugged in and set up!
4. **Using the Alexa Skill**
    1. This step is a bit more difficult. 
    2. Go to the Alexa Developer Console
        
        [https://developer.amazon.com/alexa/console/ask](https://developer.amazon.com/alexa/console/ask)
        
    3. Click “Create Skill”
        
        ![Screen Shot 2022-05-04 at 9.42.43 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-05-04_at_9.42.43_PM.png)
        
    4. Give your skill a **name**. Select **Custom Skill**. Select ‘Alexa-Hosted Python’ for the host.
    5. Once that builds, go to the ‘Code’ tab on the skill. Select **‘Import Code’,** and import the “VoiceControlAlexaSkill.zip” file from the repository, under the folder “**Alexa”**
        
        ![Screen Shot 2022-05-04 at 9.44.54 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-05-04_at_9.44.54_PM.png)
        
    6. Import all files.
    7. In Credentials.py, fill in your credentials, ensuring you have the correct name for your queue. 
        
        “`vcq_name`” is the name of your queue + “`.fifo`”
        
    8. Click ‘Deploy’
    9. Once Finished, navigate to the ‘Build’ tab. Open the `intents.txt` file from the repository’s ‘Skill’ folder (under the Alexa folder), and it’s time to build our intents. 
    10. Under invocation, give your skill a name. This is what you will say to activate it. We recommend **mini scout**! Be sure to save!
    11. Under intents, click ‘Add Intent’ and create the following 5 intents:
        1. Move
        2. MoveForward
        3. MoveBackward
        4. RotateLeft
        5. RotateRight
        
        ![Screen Shot 2022-05-04 at 9.58.13 PM.png](Final%20Deliverable%2024e6a76061d34cb68f3866852d07deb9/Screen_Shot_2022-05-04_at_9.58.13_PM.png)
        
    12. Edit each intent so that they have the same or similar “Utterances” as seen in the `intents.txt` file. Don’t forget to save!
    13. Your skill should be good to go! Test it in the Test tab with some of the commands, like “ask **mini scout** to move right”
5. **Using MiniScout in ROS**
    1. Clone the branch entitled vnc into your ROS environment
    2. install boto3 using similar steps as the ones above
    3. run `catkin_make`
    4. give execution permission to the file `MiniScoutMain.py`
    5. change each `Add_Credentials.py` file so that your AWS credentials are correct, and change the name of each file to `Credentials.py`
6. running hello robot!
    1. run the hello_robot_comp.py script on your computer
    2. run the hello_robot_vnc.py script on your robot

---

## **Clear description and tables of source files, nodes, messages, actions and so on**

```python
MacOS
path on computer/mini_scouter
│
├── Alexa
│	├── VoiceControlAlexaSkill.zip
│	├── VoiceControlAlexaSkill
│		├── interactionModels
│			├── custom / en-US.json
│		├── lambda
│			├── Credentials.py
│			├── lambda_function.py
│			├── requirements.txt
│			├── utils.py
│		└── skill.json
│	└── intents.txt
├── src
│	├── credentials
│		├── __init__.py
│		└──  Add_Credentials.py
│	├── MacOSLeap
│		├── Leap.py
│		├── LeapPython.so
│		└── libLeap.dylib
├── hello_robot_comp.py
├── LeapGestureToSQS.py
└── .gitignore

Windows
path on computer/mini_scouter
│
├── Alexa
│	├── VoiceControlAlexaSkill.zip
│	├── AlexaSkillUnzipped
│		├── interactionModels
│			├── custom / en-US.json
│		├── lambda
│			├── Credentials.py
│			├── lambda_function.py
│			├── requirements.txt
│			├── utils.py
│		└── skill.json
│	└── intents.txt
├── src
│	├── credentials
│		├── __init__.py
│		└──  Credentials.py
│	├── WindowsLeap
│		├── Leap.py
│		├── LeapPython.so
│		└── libLeap.dylib
├── LeapGestureToSQS.py
└── .gitignore

vnc
catkin_ws/src/mini_scout_vnc
│
├── src
│	├── credentials
│		└──  Credentials.py
│	├── hello_robot
│		└──  src
│			├── Credentials.py
│			└── hello_robot_vnc.py
│	├── sqs
│		└── sqs_connection.py
│	├── teleop
│		├── MiniScoutAlexaTeleop.py
│		└── MiniScoutLeapTeleop.py
│	└── MiniScoutMain.py
├── CMakeLists.txt
├── README.md
├── package.xml
└── .gitignore
```

# **Story of the project**

### Our initial idea

When we first started our project, we had the idea of creating a seeing-eye dog robot that would guide the user past obstacles. We decided we wanted to use the Leap Motion Controller in our project and we planned to have the user direct the robot where to go using Leap. We would then have the robot detect obstacles around it and be able to localize itself within a map. Our final project has definitely deviated a lot from our original project. At the beginning, we made our project more vague to just become a “gesture-controlled bot” since we felt that our project was taking a different direction than the seeing-eye dog bot we had planned on. After implementing features for the robot to be commanded to different positions through hand gestures and voice commands, we settled on creating a mini scouter robot that can “scout” ahead and help the user detect obstacles or dangerous objects around it.

### Problems, Pivots and Concluding Remarks

Our first main challenge with the Leap Motion Controller was figuring out how to get it connected. We first attempted to connect the controller directly to the robot via USB. However, the computer on the turtlebot3 ended up not being strong enough to handle Leap, so we had to consider other methods. August recommended using AWS IOT and we began looking into AWS applications we could use to connect between the local computer and VNC. We settled on using AWS SQS to send messages from the local computer to the VNC and the robot and we used boto3 to add SQS to our scripts.

Our next challenge was with getting the Leap Motion Controller setup. The software we needed for Leap ended up not being compatible with Monterey (the OS Nazari’s laptop was on) and it didn’t seem to work when attempting to set it up on Windows (Helen’s laptop). For Mac, we found a workaround (listed above under setup) to get V2 of the Leap software working. For Windows, we had to try a couple different versions before we found one that worked (Orion). 

There were also challenges in finding the best way to represent the gestures within the algorithm for moving the robot. Our gestures were not clear at times and having the robot constantly detect gestures was difficult. We had to make changes to our code so that the robot would see a gesture and be able to follow that command until it saw another gesture. This way, Leap would only have to pick up on gestures every few seconds or so - rather than every second - making the commands more accurate.

When we started implementing a voice control system into the robot, we had challenges with finding a system that wasn’t too complex but worked well with our needs. We first explored the possibility of using pocketsphinx package but this ended up not being the most accurate. We decided to use Amazon Alexa and AWS lambda to implement voice recognition.

Overall, we are very satisfied with the progress on this project. It was quite tricky at times, but working with AWS and finding a new way for the controller to communicate with the robot was an unexpected “pivot” we took that worked out well in the long run. The entire project was truly a learning experience, and it’s exciting to watch our little Mini Scouter move as it uses the code we created over the last three months.
