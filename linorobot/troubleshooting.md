# Troubleshooting

## linobase.h

The most common problem with a new build is that the pin numbers are incorrect. You can find the hardware configuration in `linorobot/teensy/firmware/lib/config/lino_base.h`. In that same directory you will find other versions of that file for our different models. If you are qualifying a new model, then you should add a copy there.

There are numberous variables in this file. The key ones for now are:
```
#define MOTOR1_IN_A 20
#define MOTOR1_IN_B 1
#define MOTOR1_PWM 22

#define MOTOR2_IN_A 6
#define MOTOR2_IN_B 8
#define MOTOR2_PWM 5
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



