# typinator2023

### Running the code

Before running the project, connect the arm and a usb webcam to a computer with ROS and the software for the Interbotix arm installed (this project will not work in the vnc). [More details on the arm software are provided here.](https://campus-rover.gitbook.io/lab-notebook/faq/interbotixpincherx100)

Launch the arm: <code>roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=px100 use_sim:=false</code>

<b>Calibration</b><br>

Follow these instructions if the keyboard you are using has not already been calibrated.

Position the usb camera in the center of the keyboard, facing directly down. The whole keyboard should be in frame. Ideally the keyboard will have minimal glare and it will be on a matte, uniform surface. The width of the image must be known, in meters. Alternatively, an image of the keyboard can be uploaded, cropped exactly to known dimensions (i.e. the dimensions of the keyboard).

Run <code>roslaunch typinator image_setup.launch</code> to calibrate the image filters for optimized key detection. A variety of preset filters will be applied to the image. Apply different combinations of the following parameters and determin which filter preset/parameter combination is the best. An optimal filter will show keys with large boxes around them, not have a lot of noise, and most importantly detect all of the keys. If there is a lot of noise, calibration will take a long time.

| Parameter | Default | Use |
|:---|:---|:---|
| image_file | False | Input the path to the image file you would like to use if not using the camera |
| reduce_glare | False | Set to True to apply glare-reduction to the image |
| blur | 1 | Apply blur to the image before finding contours. 1 results in no blur. |
| kernel | 1 | Apply a dilation kernel to the image. 1 has no effect (1 pixel kernel) |


Once the tranformation preset, reduce_glare, blur, and kernel have been selected, the arm can be calibrated and run. Run <code>roslaunch typinator typinator.launch</code> with the appropriate parameters:

| Parameter | Default | Use |
|---|---|---|
| calibrate | True | Set to false if loading the keyboard from a preset |
| keyboard_preset | "temp_preset.json" | In calibration mode, save the keyboard to this file. Load this keyboard file if calibrate=False. |
| chatgpt | "Generate random text, but not lorem ipsum" | Text input to chatgpt |
| img_width | 0 | MUST SET THIS VALUE DURING CALIBRATION: Set to the the real world width of the image in meters |
| img_height | img_width*aspect ratio | Set this to the real world height of the image if necessary |
| arm_offset | 0 | Set this to the distance from the bottom edge of the image to the middle of the base of the arm |
| image_file | False | Input the path to the image file you would like to use if not using the camera |
| reduce_glare | False | Set to True to apply glare-reduction to the image |
| blur | 1 | Apply blur to the image before finding contours. 1 results in no blur. |
| kernel | 1 | Apply a dilation kernel to the image. 1 has no effect (1 pixel kernel) |
| thresh_preset | 1 | Apply this threshold preset to the image. |

The arm will press each key it detects and save its position! It will then type the output from ChatGPT! You can open a file you want it to type into, even while calibrating.

<b>Running from a preset</b></br>

If a specific keyboard has already been calibrated, position the arm appropriately and run <code>roslaunch typinator typinator.launch calibrate:=False keyboard_preset:="filename.json"</code>
Also set <code>image_file</code> if you want boxes to display current keys, and <code>chatgpt</code> if you want a custom input to chatgpt.
