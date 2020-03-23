# PID for beginner roboticists

PID is a common and powerful programming tool that can be used to maintain stability and accuracy in many tasks that give feedback. This guide will walk through how to de-mystify PID and teach you to become a confident PID user.

PID stands for *P*roportional, *I*ntegral and *D*erivative. All three mathematical techniques are used in a PID controller.

First, let's think of a PID application. There is a wheel on a motor with a quadrature shaft encoder that you desire to spin at a certain RPM. In this example, imagine you have already written the code to get the speed of the motor by reading the encoder data.

The goal is for the wheel to spin at 60 RPM. so 60 is our **Setpoint Variable**. the actual speed that the motor encoders report is the **Process Variable**. By subtracting the Process Variable *PV* from the Setpoint Variable *SP*, we get the **error**

``` c++
double error = target_speed - get_wheel_speed(); // assume this function has been written and returns motor speed in RPM
```

We will use this error three ways, and combine them using weights (called *Gains*) to acheive PID control. The three components answer three questions:

1. How big is my error?
2. How much error have I accumulated?
3. Is my error getting bigger or smaller?

## Proportional

Proportional control could not be simpler: multiply the error by some constant **Kp**

``` c++
double p = error * Ki;
```

## Integral

If you know about Integrals then you know that they represent the area under a curve, which is a sum of all the points on said curve. Therefore, we can calculate the Integral component by summating the total error over time, like this:

```C++
double i;
...
i += Ki * error * change_in_time();
```

Of course, **Ki** is the Integral Gain constant, similar to **Kp**. Error is multiplied by the change in time because remember, the integral is the area under the curve. So if we plot error on the Y axis, time would likely be plotted on the x axis, thus area is error * time.

## Derivative

Derivatives are the rate at which things change, so naturally a simple way to get the Derivative component is to compare the current error to the pervious error, and then account for time, like this:

``` C++
double d = Kd * (error - pervious_error) / change_in_time();
previous_error = error;
```

Again, **Kd** is the derivative gain constant, and we divide by the change in time because if the change is 4 units in half a second, then the rate of change is 8 units/second (4 / 0.5)

## What to do with P, I and D

Add them up. This is an adjusted representation of your control system's error. Therefore, you can apply it as a change to your previous command to try to get closer to the **Setpoint Variable**

``` c++
double pid = p + i + d;
command = previous_command + pid;
previous_command = command;
```

Then your command can be sent to your actuator. Please note that some additional arithmatic may be required, this is a bare-bones simple example and does by no means serve as a copy+paste solution to all PID applications.

## Tuning

A PID system must be tuned with the proper values of **Kp**, **Ki** and **Kd** in order to acheive smooth control. The main goals of tuning are to minimize error and over shooting the Setpoint Goal. There are plenty of guides and theories as to how to tune, so [here is the Google Search to get you started](https://www.google.com/search?sxsrf=ALeKk03hzusA-DgWl--WkfRhsvZIQFNCXg%3A1584985643367&ei=K_Z4Xr7oFerB_QaC8JWoCA&q=how+to+tune+a+PID+loop&oq=how+to+tune+a+PID+loop&gs_l=psy-ab.3..0j0i22i30l8.357490.364912..365211...2.3..2.117.2696.31j1......0....1..gws-wiz.....10..0i71j0i67j0i10j35i362i39j35i39j0i273j0i131.R09JRmXVhGI&ved=0ahUKEwj-qfzRk7HoAhXqYN8KHQJ4BYUQ4dUDCAs&uact=5)
