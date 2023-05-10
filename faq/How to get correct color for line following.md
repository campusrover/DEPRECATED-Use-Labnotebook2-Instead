How to get correct color for line following in the lab by Rongzi Xie.
Line follower may work well and easy to be done in gazebo because the color is preset and you don't need to consider real life effect. However, if you ever try this in the lab, you'll find that many factors will influence the result of your color.

Real Life Influencer:
1. Light: the color of the tage can reflect in different angles and rate at different time of a day depend on the weather condition at that day. 
2. Shadow: The shadow on the tape can cause error of color recognization
3. type of the tape: The paper tage is very unstable for line following, the color of such tape will not be recognized correctly. The duct tape can solve most problem since the color that be recognized by the camera will not be influenced much by the light and weather that day. Also it is tougher and easy to clean compare to other tape.
4. color in other object: In the real life, there are not only the lines you put on the floor but also other objects. Sometimes robots will love the color on the floor since it is kind of a bright white color and is easy to be included in the range. The size of range of color is a trade off. If the range is too small, then the color recognization will not be that stable, but if it is too big, robot will recognize other color too.
if you are running multiple robots, it might be a good idea to use electric tape to cover the red wire in the battery and robot to avoid recognizing robot as red line.

OpenCV and HSV color:
Opencv use hsv to recognize color, but it use different scale than normal.
Here is a comparison of scale:
normal use  H: 0-360, S: 0-100, V: 0-100
Opencv use  H: 0-179, S: 0-255, V: 0-255

So if we use color pick we find online, we may need to rescale it to opencv's scale.
