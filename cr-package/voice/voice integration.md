# Voice Integration

## Overview

### Goals

Control robot through robust voice commands. This includes simple motion as well as global and local planning.

### Ngrok and webhook setup

Ngrok is installed on the roscore1 computer is ran with './ngrok http 5000' which opens up an ngrok window which should have a link that looks like 'https://96ba716c.ngrok.io'. From there open up the Alexa Skills developer console (get the login from Pito) and click on the endpoints section of the console in Build. From there paste the ngrok link in the endpoint followed by '/alexa_webhook' there should be a SSL certificate set up if not you must aquire one by going to the ngrok url and downloading a certificate. After the endpoint is set up is done run the webhook node on roscore1 with 'rosrun webhooks voice_webhook.py'. This is now set up to publish intents based on Alexa input.

## _@Arjun Albert arjunalbert@brandeis.edu_
