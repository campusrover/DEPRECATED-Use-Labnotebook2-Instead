# SSD model \(deep learning\)

## Deep learning

The final approach I took was deep learning. I used a [pre-trained SSD](https://github.com/victordibia/handtracking) \(Single Shot multibox Detector\) model to recongnize hands. The model was trained on the Egohands Dataset, which contains 4800 hand-labeled JPEG files \(720x1280px\). Code can be found [here](ssd.md)

Raw images from the robot’s camera are sent to another local device where the SSD model can be applied to recognize hands within those images. A filtering method was also applied to only recognize hands that are close enough to the camera.

After processing raw images from the robot, a message \(hands recognized or not\) will be sent to the State Manager. The robot will take corresponding actions based on messages received.

## References

Wei Liu, Dragomir Anguelov, Dumitru Erhan, Christian Szegedy,Scott E. Reed, Cheng-Yang Fu, and Alexander C. Berg. SSD: single shot multibox detector. CoRR, abs/1512.02325, 2015. Victor Dibia, Real-time Hand-Detection using Neural Networks \(SSD\) on Tensorflow, \(2017\), GitHub repository, [https://github.com/victordibia/handtracking](https://github.com/victordibia/handtracking)

