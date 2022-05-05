# Building a skill with Alexa Flask-ASK for ROS
#### Ben Soli

Before you use this tutorial, consult with the Campus Rover Packages which outline setting up ngrok with flask and getting to the Alexa Developer Console. 

The Flask Alexa Skills Kit module allows you to define advanced voice functionality with your robotic application.

In your VNC terminal 
```
pip3 install flask-ask
```
Then in your rospy node file 
```python
from flask_ask import Ask, statement, question, elicit_slot, confirm_slot
```
declare your Flask application as 
```python
app = Flask(__name__)
```

and connect it to the Flask ASK
```python

ask = Ask(app, '[endpoint]')
```

So, if your ngrok subdomain is campusrover and your Alexa endpoint is /commands

your code would be 
```python
ask = Ask(app, '/commands')
```
On the Alexa Developer Consoles define your intents and determine which slots your intents will use. Think of intents as functions, and slots as parameters you need for that function. If a slot can have multiple items in a slot mark that in the developer console.
You do not need to provide the Alexa responses on the Developer Console because you will be writing them with Flask-ASK. The advantage of doing this is you can write responses to the user that take into account the robot's state and publish information to other nodes 
as you receive input with Alexa.

Let's assume we have an intent called 'bring_items' that can contain multiple items to fetch. Assume that the robot needs at least one item to fulfill this intent and that the robot has a weight capacity for how much it can carry.
Let's also assume we have some kind of lookup table for these items which tells us their weight. 
With Flask-ASK we can quickly build this. 

You are required to have a response for launching the intent marked as 
```python
@ask.launch
def start_skill():
  return question('hello, what you need?)
```
This intent is not called every single time you use the skill, but its a good way to tell the user what the bot can do or tell the user what information the bot needs from them.
There are a few different response types, 

```statement(str: response)``` which returns a voice response and closes the session. </br>
```question(str: response)``` which you can use to ask the user a question and keep the session open. </br>
```elicit_slot(str: slot_name, str: response)``` which you can use to prompt the user for a specific type of information needed to fulfill the intent. </br>
```confirm_slot(str: slot_name, str: response)``` which you can use to confirm with the user that Alexa heard the slot correctly. </br>

It is important to note, that to use ```elicit_slot``` and ```confirm_slot``` you must have a Dialog Model enabled on the Alexa Developer Console.
The easiest way I have to found to enable this is to create an intent on the console that requires confirmation. To avoid this intent being activated,
set its activation phrase to gibberish like 'asdlkfjaslfh'. You can check a switch marking that this intent requires confirmation.

Now let's build out an intent in our flask application.


Start with a decorator for your skill response marking which intent you are programming a response for. 
```python
@ask.intent('bring_items')
def bring_items():
```
First, let's assure that the user has provided the robot with some items. When you talk to Alexa, it essentially just publishes a JSON dictionary to your endpoints which your flask app can read. 
To get a list of the slots for this intent use the line:
```python
  slots = request.get_json()['request']['intent']['slots']
```
Let's call our items slots 'items'. To check that the user has provided at least one item, write. 
```python
if 'value' not in slots['items'].keys() 
  return elicit_slot('items', 'what do you want me to bring?)
``` 
This will check that there is an item, and if there is not, prompt the user for some items.
The string ```python 'value' ``` is a key in the dictionary if and only if the user has provided the information for this slot, so this is the best way to check.

Let's assume that our robot has a carrying capacity of 15 pounds and has been asked to carry some items that weigh more than 15 pounds. 
Once we've checked that the items are too heavy for the robot, you can elicit the slot again with a different response 
like 
```python
elicit_slot('items','those are too heavy, give me something else to bring')
```
and the user can update the order. 
Returning ```elicit_slots``` keeps the Alexa session open with that intent, so even though each call is a return statement, you are essentially making a single function call and updating a single JSON dictionary. 

Once the robot is given a list of items that it can carry, you can use a rospy publisher to send a message to another node to execute whatever robotic behavior you've implemented.


You should also include 
 
```python
@ask.intent('AMAZON.FallbackIntent')
```
This is an intent you can use for any phrase that you have not assigned to an intent that your robot will use. If you do not implement a response for this, 
you will very likely get error messages from Alexa. 

Skills come prebuilt with some intents such as these. If a user activates one of these intents, and you don't define a response, you will get a skills response error.
You should thoroughly test possible utterances a user might use to avoid errors. Skills response errors do not crash the application, but it makes for bad user-experience.

Another default intent you will likely need is
```python 
@ask.intent('AMAZON.CancelIntent')
```

This handles negative responses from the user. An example of this is:

<b>User:</b> <i> Alexa launch campus rover</i> </br>
<b>Alexa:</b> <i> hello, what do you need </i> </br>
<b>User:</b> <i> nothing </i> </br>
<b>Alexa:</b> <i> AMAZON.CancelIntent response </i> </br>



I hope this tutorial has made clear the advantages of using the flask-ASK for your Alexa integration. It is a great way to rapidly develop different voice responses for your robot and quickly integrate those with robotic actions while avoiding the hassle of constantly rebuilding your Alexa skill in the Developer Console.
