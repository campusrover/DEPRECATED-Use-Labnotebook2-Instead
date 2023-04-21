# Troubleshooting

## linobase.h Pins

The most common problem with a new build is that the pin numbers are incorrect. You can find the hardware configuration in `linorobot/teensy/firmware/lib/config/lino_base.h`. In that same directory you will find other versions of that file for our different models. If you are qualifying a new model, then you should add a copy there.

There are numberous variables in this file. The key ones for now are:
```
#define MOTOR1_IN_A 20
#define MOTOR1_IN_B 1
#define MOTOR1_PWM 22

#define MOTOR2_IN_A 6
#define MOTOR2_IN_B 8
#define MOTOR2_ PWM 5
```

and

```
#define MOTOR1_ENCODER_A 14
#define MOTOR1_ENCODER_B 15

#define MOTOR2_ENCODER_A 12
#define MOTOR2_ENCODER_B 11
```

MOTOR1 is front left, and MOTOR2 is front right. There are several misconfigurations possible, basically all the permutations of left/right and forward/backward, on both the motors and encoders. 

### Symptoms

If all wheels are spinning but the robot spins in circles, goes backward, is unresponsive to `cmd_vel` commands or in general acts crazy, your first hypothesis should be that one or more of the pins above are incorrect or swapped.

If one of the wheels doesn't spin at all then it's probably an electrical connection to the motor. If both wheels don't spin then it's probably a power/battery issue.
## Erratic Travel on a newly built robot

### Check Encoders

To check the encoders, disconnect the power from the motors, leaving just the encoders. Then when you roslaunch linorobot minimal.launch it will print four numbers over and over again which are the readings of the potential for encoders (two in front and two in the back.) 

As I had only two wheels I just got two non-zero numbers. Put the robot on the floor and push it gently forward. Those two numbers will change. They should both go up approximately as fast, even though they are not the same number. In my case, they went in opposite directions. Note the one that is going down. The left wheel is encoder1 and the right wheel is encoder2. In the setup file I switched the pins for the corresponding motor and that solved it.

(Note: This is very obvious but I got it wrong: The caster is on the BACK of the robot not the front. You need to know what is the front of the robot to know which wheel is left or right.)

### Check Motors

For me, this didn't quite solve it. So the next thing to check was the motor itself. I changed the Arduino code (.ino) so that instead of sending the calculated numbers to the motors, I sent 100 to the left motor and 500 to the right motor. This is so that I could tell if the left motor turned slower than the right. If not, I had to switch those pins. Next I had to tell that both motors turned so that the effect was forward motion of the robot. That was incorrect for me too. That too is fixed by switching PIN numbers.

### Check PID

Next came the PID settings. The instructions are good there in terms of getting monitoring the result of the pid calculations but not as far as what numbers are right. There are an infinite number of references on the web on tuning pid and they are all vague and different. 

Here again I made a small change to the Arduino code. I had it print out the error between the desired and actual rate of the left and right wheels. If things are working like they should that error starts at zero and when you give the robot a command it temporarily goes away from zero and then returns nicely to zero. I don' know yet what "technique" I used, nor whether I have the right parameters yet. But at least the robot is behaving better.



