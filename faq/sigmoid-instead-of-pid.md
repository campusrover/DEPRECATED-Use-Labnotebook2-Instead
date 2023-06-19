---
title: How do I use a sigmoid function instead of a PID
description: An alternative function that is sometimes suggested as an alternative to pid
status: new
type: faq
date: may-2022
author: ChatGPT
---
To use a sigmoid function instead of a PID controller, you need to replace the PID control algorithm with a sigmoid-based control algorithm. Here is a step-by-step guide to do this:

1. Understand the sigmoid function: A sigmoid function is an S-shaped curve, mathematically defined as:
   f(x) = 1 / (1 + exp(-k * x))

   where x is the input, k is the steepness factor, and exp() is the exponential function. The sigmoid function maps any input value to a range between 0 and 1.

2. Determine the error: Just like in a PID controller, you need to calculate the error between the desired setpoint and the current value (process variable). The error can be calculated as:
   error = setpoint - process_variable

3. Apply the sigmoid function: Use the sigmoid function to map the error to a value between 0 and 1. You can adjust the steepness factor (k) to control the responsiveness of the system:
   sigmoid_output = 1 / (1 + exp(-k * error))

4. Scale the output: Since the sigmoid function maps the error to a range between 0 and 1, you need to scale the output to match the actual range of your control signal (e.g., motor speed or actuator position). You can do this by multiplying the sigmoid_output by the maximum control signal value:
   control_signal = sigmoid_output * max_control_signal

5. Apply the control signal: Send the control_signal to your system (e.g., motor or actuator) to adjust its behavior based on the error.

Note that a sigmoid function-based controller may not provide the same level of performance as a well-tuned PID controller, especially in terms of overshoot and settling time. However, it can be useful in certain applications where a smooth, non-linear control response is desired.