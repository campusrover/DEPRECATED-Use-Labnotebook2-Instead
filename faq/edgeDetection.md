---
title: Edge Detection
author: Harris Ripp
---
# Detection of Edges 
### By Harris Ripp

The detection of edges in image processing is very important when needing to find straight lines in pictures using a camera. One of the most popular ways to do so is using an algorithm called a Canny Edge Detector. This algorithm was developed by John F. Canny in 1986 and there are 5 main steps to using it. Below are examples of the algorithm in use:

![Pre Canny](https://upload.wikimedia.org/wikipedia/commons/f/f0/Valve_original_%281%29.PNG)
![Post Canny](https://upload.wikimedia.org/wikipedia/commons/9/93/Valve_monochrome_canny_%286%29.PNG)

The second image displays the result of using canny edge detection on the first image.

## Steps to Edge Detection:
* Apply [Gaussian filter](https://en.wikipedia.org/wiki/Gaussian_filter) to smooth image
* Find intensity gradients of image
* Apply gradient magnitude thresholding or lower bound cut-off suppression to remove false results
* Track edge by surprisessing weak edges so only strong ones appear

## Application of Canny Edge Detection

The below function demonstrates how to use this algorithm:

```python
def img_callback(self, msg):

        # Canny Edge Detection
        # Blurs image and find intensity gradients
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        kernelSize = 31
```

First, the image is converting into something usable by cv. It is then grayed and the intensity gradient for the kernel is found

```python
grayBlur = cv2.GaussianBlur(gray, (kernelSize, kernelSize), 0)
```

The image is then blurred for canny preparation. 

```python
lowEnd = 30
        highEnd = 100
        edges = cv2.Canny(grayBlur, lowEnd, highEnd)

        # Publish for use in finding centroid 
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(edges))
```

The lower and upper bounds are decided and the Canny algorithm is run on the image. In the case of this function, the new image is then published to a topic called "canny mask" for use by another node. 

The above code was created for use in a project completed by myself and fellow student Adam Ring
