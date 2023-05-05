By Long Yi

## How to Approach Computer Vision

Using machine learning models for object detection/ classification) might be harder than you expect, from our own groups’ experience, as machine learning models are usually uninterpretable and yield stochastic results. 
Therefore, if you don’t have a solid understanding of how to build/train a model or if you’re not confident that you know what to do when the model JUST doesn’t work as expected, I recommend you to start from more interpretable and stable computer vision techniques to avoid confusion/frustration, and also try to simplify the object you want to detect/classify – try to make them easily differentiable with other background objects from color and shape. 
For examples, several handy functions from OpenCV that I found useful and convenient-to-use include color masking (cv2.inRange), contour detection (cv2.findContours), indices extraction (cv2.convexHul) etc.. 
These functions suffice for easy object detection (like colored circles, colored balls, arrows, cans, etc.); and you can use cv2.imshow to easily see the transformation from each step -- this would help you debug faster and have something functional built first. 

For one example (this is what our team used for traffic sign classification):
Using this piece of code:
![Screenshot 2023-05-05 at 7 38 05 PM](https://user-images.githubusercontent.com/59838570/236585107-8e69b398-36c1-4977-8116-ad4ff33f6aba.jpg)
You can detect the contour of the arrow
![Screenshot 2023-05-05 at 7 38 59 PM](https://user-images.githubusercontent.com/59838570/236585145-c92799a8-d5ae-4da9-9db8-935fe0263750.jpg)
After which you can find the tip of the arrow, and then determine the direction of the arrow
![Screenshot 2023-05-05 at 7 40 25 PM](https://user-images.githubusercontent.com/59838570/236585224-9bb3d2d1-0daf-4bfa-b313-0c4551323ece.jpg)

The algorithm of finding the tip might be a little complicated to understand (it uses the convexity defects to do it). But the point I'm making here is that with these few lines of pretty simple code, you can achieve probably more than you expected. So do start with these "seamingly easy" techniques first before you use something more powerful but confusing.
