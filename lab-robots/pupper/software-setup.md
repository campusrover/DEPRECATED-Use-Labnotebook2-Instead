# Software Setup

## Pupper Base Software Setup
Instructions for the base software setup can be found in the [doc](https://docs.google.com/document/d/1ztO-zyF31r9wYJvHZEMbAYyhtuHloNoHOm8AScbr3wc/edit?pli=1). See notes section for project-specific instructions.

### Notes on the setup doc
* **3.c:** `en1` might not exist on the Pi. To find out which network interface to use run `sudo ifconfig` and look at the first interface it lists. For this project, it was `eth0`.
* **4.d.ii:** Although it says `Mac/linux`, this is not always the case. If `ls /dev | grep tty.usbmodem` shows no results try `ls /dev | grep ttyACM`. For this project, it was `ttyACM0`.

### Running base software
Follow instructions [here](https://github.com/stanfordroboticsclub/StanfordQuadruped/blob/dji/README.md). The keyboard program mentioned can be found [here](https://github.com/stanfordroboticsclub/PupperKeyboardController).

## Setting up this project
Install the dependencies using pip:

### Dependencies
* `picamera`
* `pupil_apriltags`
* `tqdm`
* `scipy`
* `UDPComms`
* `pyyaml`
* `opencv` or `cv2`
* `argparse`
* `pickle`
* `matplotlib`

### Running
The full software can be run using `python main.py`, or the controller can be run separately using `python3 src/controller.py`.