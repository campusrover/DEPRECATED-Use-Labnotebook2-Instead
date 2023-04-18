# Controls
The `controller.py` provides an API to control the robot by emulating a PS4 controller, similar to [this](https://github.com/stanfordroboticsclub/PupperKeyboardController). Main difference is some keybinds are different, but it can also be called from code.

## The UDP message
This is a breakdown of the dictionary's fields that is sent through UDPComms to the robot.

| Field | Value | Comments |
| :---- | :---- | :------- |
| lx    | [-1,1]| Walk left or right |
| ly    | [-1,1]| Walk front or backwards |
| rx    | [-1,1]| Turn left or right (yaw)|
| ry    | [-1,1]| Tilt up or down (pitch) |
| L2    | 0     | Nothing |
| R2    | 0     | Nothing |
| R1    | 1 or 0| Toggle trot/rest |
| L1    | 1 or 0| Toggle activation/deactivation |
| dpadx | [-1,1]| Move body up or down |
| dpady | [-1,1]| Roll left/right (roll) |
| square | 0 | Nothing |
| circle | 0 | Nothing |
| triangle | 0 | Nothing |
| x | 0 | Nothing |
| message_rate | 20 | Rate of which messages are sent |

**Note:** values `[-1,1]` means any values between -1 and 1 and values `1 or 0` are a toggle. This means that the first time `1` is sent, it will cause the value on the pupper to change. A `0` needs to be sent before another `1` will cause a toggle.

It is a good idea to use some sort of smoothing for `lx`, `ly` and `rx` to avoid abrupt stops.

## Keybinds
If running the controller manually, these are the controls:

| Keybind | Function |
| :------ | :------- |
| space | Toggle trot/rest |
| v | Toggle activation |
| w/s | Move forward/back |
| a/d | Turn left/right |
| q/e | Move left/right |
| x | Stop any sort of movement |

## Control sequence
Robot must first be activated, this will also trigger calibration if the pupper software was run with the `--zero` flag. Then it must be in trotting mode and only then can it be controlled with other functions.