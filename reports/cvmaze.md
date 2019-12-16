# Computer Vision Maze Solver

## Author: Zekai Wang & Zhidong Liu

### Introduction

Solving a maze is an interesting sub-task for developing real-world path schedule solutions on robots. Typically, developers prefer to use depth sensors such as lidar or depth camera to walk around in a maze, as there a lots of obstacle walls that the robot need to follow and avoid crashing. Depth sensors have their advantage to directly provide the distance data, which naturally supports the robot to navigate against the obstacles.  

However, we have come up with another idea to solve the maze using the computer vision approach. By reading articles of previous experts working on this area, we know that the RGB image cameras (CMOS or CCD) could also be used to get the depth information when they have multiple frames captured at different points of view at the same time, as long as we have the knowledge of the relative position where the frames are taken. This is basically the intuition of how human eyes work and developers have made a lot of attempts on this approach. Though it could calculate the depth data by using cameras no less than 2, it is not as accurate as directly using the depth sensors to measure the environment. Also, the depth calculation and 3D reconstruction requires a lot of computational resource with multiple frames to be processed at the same time. Our hardware resource might not be able to support a real-time response for this computation. So we decide not to use multiple cameras or calculate the depth, but to extract features from a single image that allows the robot to recognize the obstacles.  

The feature extraction from a single frame also requires several image processing steps. The features we chose to track are the lines in the image frames, as when the robot is in a maze boundary lines will could be detected as because walls and the floor could have different colors. We find the lines detected from the frames, which slopes are in certain ranges, are exactly the boundaries between the floor and walls. As a result, we tried serval approaches to extract these lines as the most important feature in the navigation and worked on optimization to let the robot perform more reliable.  

So the general idea is: when the robot detects a corner in the maze by analyzing the lines, its main node will call an action to handle the case until the robot is has pass that corner. During the other time, the robot will use the pid algorithm to try to keep at the middle of the left and right walls in the maze, going forward until reaches the next corner. 

### Relevant literature

To detect the proper lines stably, we used some traditional algorithms in the world of computer vision to process the frames step by step, including:  

1. Gaussian blur to smooth the image, providing more chance to detect the correct lines.  
https://web.njit.edu/~akansu/PAPERS/Haddad-AkansuFastGaussianBinomialFiltersIEEE-TSP-March1991.pdf  
https://en.wikipedia.org/wiki/Gaussian_blur  

2. converting image from RGB color space to HSV color space  
https://en.wikipedia.org/wiki/HSL_and_HSV  

3. convert the hsv image to gray and then binary image  

4. do Canny edge detection on the binary image  
http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.420.3300&rep=rep1&type=pdf
https://en.wikipedia.org/wiki/Canny_edge_detector

5. do Hough line detection on the edges result  
http://www.ai.sri.com/pubs/files/tn036-duda71.pdf
https://en.wikipedia.org/wiki/Hough_transform

6. apply an slope filter on the lines to reliably find the boundary of the walls and the floor

### Technical descriptions, illustrations

