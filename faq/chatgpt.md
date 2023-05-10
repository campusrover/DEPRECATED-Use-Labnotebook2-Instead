---
title: Basic Chatgpt ROS interface
desc: This is a quick guide to connecting to ChatGPT and getting it to generate a message that you can publish to a topic for use in other nodes. <br/>
date: feb-2023
status: new
---

# Basic ChatGPT Connection and Publishing the Output to a Topic
by Kirsten Tapalla - Spring 2023 <br/><br/>
This is a quick guide to connecting to ChatGPT and getting it to generate a message that you can publish to a topic for use in other nodes. <br/>

## Necessary Imports:
- <code>import requests</code>: used to connect to the internet and post the url to connect to ChatGPT
- <code>import rospy</code>: used to publish the response from ChatGPT to a topic
- <code>from std_msgs.msg import String</code>: used to publish the response as a ros String message 

## Node Initialization and Variables/Parameters: 
- Since you want to publish the message into a topic, you will have to initialize a rospy node withing your file using <code>rospy.init_node('ENTER-NODE-NAME-HERE')</code>. 
- You will also want to initialze a rospy Publisher with the name of the topic you would like to publish the data to, for example: <code>text_pub = rospy.Publisher('/chatgpt_text', String, queue_size=10)</code>. 
- If you would like to be able to change the prompt you are passing it when running the node, you should add a line allowing you to do this by initializing an input string to get the parameter with the name of what you would like to use to specify the input message you would like to pass it. 
    - For example, do this by writing <code>input_string = rospy.get_param('~chatgpt_prompt')</code>
    - When setting up your launch file later on, you will want to include a line to handle this argument. This can be done by including <code>arg name="chatgpt_prompt" default="ENTER-YOUR-DEFAULT-PROMPT-HERE"</code> into your launch file. You can set the default to whatever default prompt you would like to be passed if you are not giving it a specific one. 
- You will also want to add this line into your code to to specify the URL going that will be used to connect to ChatGPT: <code>url = 'https://api.openai.com/v1/completions'</code>. Since the chat/text completions model is what we are using the get the output responses, the URL specifies 'completions' at the end.

## ChatGPT Information: 
### Headers: 
To be able to access ChatGPT, you will need to include the following information in your 'header' dictionary: Content-Type and Authorization. Below is an example of what yours might look like:<br/>
<code>headers = {<br/>
&nbsp;&nbsp;&nbsp;&nbsp;'Content-Type': 'application/json',<br/>
&nbsp;&nbsp;&nbsp;&nbsp;'Authorization': 'Bearer INSERT-YOUR-OPENAI-API-KEY-HERE',<br/>
}</code><br/>

### Data: 
This will include the information you will want to pass into ChatGPT. The only required field will be specifying the model you want to use, but since you are passing in a prompt, you will also want to include that as well. You will be able to specify the maximum amount of tokens you want ChatGPT to generate, but the best way to get the full output messeage from ChatGPT is to enter the maximum amount for the model you are using. For example, as you can see below I am using the 'text-davinci-003' model, and the maximum tokens that this model can generate is 2048. Furthermore, you can adjust the sampling temperature, which will determine how creative the output of ChatGPT will be. The range goes between 0-2, and higher values will cause it to be more random, while lower values will cause it to be more focused and deterministic. An example of a request body you can make is shown below:<br/>
<code>data = {<br/>
&nbsp;&nbsp;&nbsp;&nbsp;'model': 'text-davinci-003',<br/>
&nbsp;&nbsp;&nbsp;&nbsp;'prompt': input_string,<br/>
&nbsp;&nbsp;&nbsp;&nbsp;'max_tokens': 2048,<br/>
&nbsp;&nbsp;&nbsp;&nbsp;'temperature': 0.5,<br/>
}</code><br/>

## Getting and Publishing the Response
To get the response for your input message from ChatGPT, include the following line in your code <code>response = requests.post(url, headers=headers, json=data)</code>. Note that if you are using different names for your variables, you will want to pass in those names in place of 'headers' and 'data'. <br/><br/>
To publish your output, you will want to make sure that your request went through. If it did, you will be able to get the output from the json file that was returned in the response variable. An example of how to do this is shown below: <br/>
<code><br/>if response.status_code == 200:<br/>
&nbsp;&nbsp;&nbsp;&nbsp;generated_text = response.json()['choices'][0]['text']<br/>
&nbsp;&nbsp;&nbsp;&nbsp;text_pub.publish(generated_text)</code><br/><br/>
By doing all of the steps above, you will be able to connect to ChatGPT, pass it a prompt, and publish its response to that prompt to a topic in ros. 