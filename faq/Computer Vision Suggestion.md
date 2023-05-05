How to Approach Computer Vision

Using machine learning models for object detection/ classification) might be harder than you expect, from our own groups’ experience, as machine learning models are usually uninterpretable and yield stochastic results. 
Therefore, if you don’t have a solid understanding of how to build/train a model or if you’re not confident that you know what to do when the model JUST doesn’t work as expected, I recommend you to start from more interpretable and stable computer vision techniques to avoid confusion/frustration, and also try to simplify the object you want to detect/classify – try to make them easily differentiable with other background objects from color and shape. 
For examples, several handy functions from OpenCV that I found useful and convenient-to-use include color masking (cv2.inRange), contour detection (cv2.findContours), indices extraction (cv2.convexHul) etc.. 
These functions suffice for easy object detection (like colored circles, colored balls, arrows, cans, etc.); and you can use cv2.imshow to easily see the transformation from each step -- this would help you debug faster and have something functional built first. 

