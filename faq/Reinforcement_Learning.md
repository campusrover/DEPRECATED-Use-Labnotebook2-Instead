---
title: Reinforcement Learning and its Applications
---
# Reinforcement Learning and its Applications
**Authors: Rachel Lese, Tiff Lyman**

A lot of people are intimidated by the idea of machine learning, which is fair. However, machine learning doesn't have to be some complicated multi-layered network. In fact, you can implement a machine learning algorithm with just a dictionary in ROS. Specifically, you can run a reinforcement learning algorithm that replaces PID to steer a robot and have it follow a line.


# What is Reinforcement Learning?
Reinforcement learning is exactly what it sounds like, learning by reinforcing behavior. Desired outcomes are rewarded in a way that makes them more likely to occur down the road (no pun intended). There are a few key components to a reinforcement learning algorithm and they are as follows:
  - A set of possible behaviors in response to the reinforcement
  - A quantifiable observation/state that can be evaluated repeatedly
  - A reward function that scores behaviors based on the evaluation

With this, the algorithm takes the highest scoring behavior and applies it.
  

# Application: Line Follower
Lets look at what those required traits would be in a Line Follower. 
### Behaviors
In this case would be our possible linear and angular movements. At the start we want everything to be equally likely, so we get the following dictionaries.
```sh
angular_states_prob =  { -.5: .5, -.45: .5, -.4: .5, -.35: .5, -.3: .5, -.25: .5, -.2: .5, -.15: .5, -.1: .5, -.05: .5, 0 : .5, .05 : .5, .1 : .5, .15 : .5, .2: .5, .25: .5, .3: .5, .35: .5, .4: .5, .45: .5, .5: .5 }
linear_states_prob = { 0.1 : .5, 0.25 : .5, 0.4 : .5 }
```
Here the key is the possible behavior (in radians/sec or m/sec) and the value is the associated weight. 

### Observation to Evaluate
As for the observation that we evaluate at regular intervals, we have camera data in CV callback. Just like with the original line follower, we create a mask for the camera data so that we see just the line and have everything else black. With this we can get the "center" of the line and see how far it is from being at the center of the camera. We do this by getting the center x and y coordinates as shown below.
```sh
moments = cv2.moments(mask_yellow)
        if moments['m00'] > 0:
            cx = int(moments['m10']/moments['m00'])
            cy = int(moments['m01']/moments['m00'])
```
Here cx and cy represent pixel indecies for the center, so if we know the resolution of the camera we can use this to establish preferable behaviors. To do this, we want our angular movement to be proportional to how far we deviate from the camera's center. For the Waffle model in ROS, we determined that it's roughly 1570x1000, meaning that cx has a range of (0,1570) and cy has a range of (0,1000).

### Reward Function
This next part might also be intimidating, only because it involves some math. What we want to do is convert our pixel range to the range of possible outcomes and reward velocities based on how close they are to the converted index. For example, right turns are negative and further right means greater cx, so the line of code below does the transformation (0,1570) => (0.5, -0.5) for cx:
```sh
angle = (float(cx)/1570.0 - 0.5) * -1.0
```
With this, we check which keys are close to our value angle and add weight to those values as follows:
```sh
for value in angular_states_prob.keys():
            angleSum += angular_states_prob[value]
            if abs(value - angle) < 0.05:
                angular_states_prob[value] = angular_states_prob[value]*5
            elif abs(value - angle) < 0.1:
                angular_states_prob[value] = angular_states_prob[value]*2
```
So if the key is closest to our angle we multiply the weight by 5, and if it's somewhat close we multiply it by 2. To make sure weights don't go off to infinity, we have the value angleSum which keeps track of the sum of the weights. If it exceeds a certain limit, we scale all of the weights down. Similarly, we dont scale a weight down if it is less than 0.05 so that they don't become infinitesimally small.

### Conclusion
This is just one example of reinforcement learning, but it's remarkably ubiquitous. Hopefully this demonstrates that reinforcement learning is a straightforward introduction to machine learning. 






