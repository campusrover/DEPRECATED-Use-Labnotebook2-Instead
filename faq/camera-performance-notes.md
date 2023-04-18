# Notes on Camera Performance
*Author: Pito Salas*, *Date: April 2023*

I posted a question to the world about improving camera performance. For posterity I am saving the rought answers here. Over time I will edit these intomore specific, tested instructions.

## Original Question

My robot has:

* ROS1 Notetic 
* Rasberry Pi 4 Raspberry
* Pi Camera V2 Running raspicam node (https://github.com/UbiquityRobotics/r...)
* My "remote" computer is running
* Ubuntu 20.04 on a cluster which is located in my lab
* They are communicating via Wifi.
* roscore is running on the robot
* The raspicam node is publishing images to it's variety of topics.

I have two nodes on my remote computer, each is processing the images in a different way. One of them is looking for fiducially and one of them is doing some simple image processing with opencv.

Performance is not good. Specifically it seems like the robot itself is not moving smoothly, and there is too much of a delay before the image gets to the remote computer. I have not measured this so this is just a subjective impression.

I hypothesize that the problem is that the image data is big and causing one of a number of problems (or all).

a. Transmitting it iis too much for the Pi 
b. The wifi is not fast enough 
c. Because there are several nodes on the remote computer that are subscribing to the images, the images are being sent redunantly to the remote computer

I have some ideas on how to try to fix this: 

a.  Reduce the image size at the raspicam node 
b.  Do some of the imate processing onboard the Pi 
c. Change the image to black and white
d. Turn off any displays of the image on the remote computer (i.e. rviz and debugging windows)

## Answer

If you have more nodes receiving from 1 raspberry ROS node, image is sent many times over wifi. One clear improvement would be having one node that communicates to raspberry over wifi and publishes to subscribers over ethernet and offloads raspberry. Maybe use something else than ROS node to async copy image towards listeners so you dont need to first wait for whole picture before you can send data forward to reduce latency.

Not sure what is the exact problem, but there are different speeds that different wifi versions support. Base raspberry has 1 pcb antenna that is not so great. Compute node can support better antenna, some embedded support m.2 slot where you can plug better stuff. Mimo wlan with 2 antennas and 40mhz channel can provide much more bandwidth. But also AP need to support similar modes. Faster transmission is also reduced latency.

If in urban area, wifi channels can have more traffic. There are tools like iptraf and more to show you the traffic. And analyzers for wifi, too.

And raspberry might not be the most powerful platform, see that you minimize image format changes on raspberry side or use hw acceleration for video to do that (if available).

## Answer

I'd suggest experimenting with drone FPV camera, 5.8 GHz USB receiver connected to desktop, which would process video. No need to use Wifi for video stream. I posted here before on my use of it, and the optimal setup.

## Answer

I have a similar setup on a recent robot, except that I'm using a USB camera instead of the Raspberry Pi camera. I was initially using OpenCV to capture the images and then send over WiFi. The first limitation I found was that Image messages are too large for a decent frame rate over WiFi, so I initially had the code on the robot compress to JPEG and send ImageCompressed. That improved things but there was still a considerable delay in receiving the images (> 1.5 sec delay to the node on the remote computer doing object detection). I was only able to get about 2 frames/sec transferred using this method, too, and suspected that the JPEG compression in OpenCV was taking too much time.

So I changed the capture to use V4L2 instead of OpenCV, in order to get JPEG images directly from an MJPEG stream from the camera. With this change in place I can get over 10 frames/sec (as long as lighting is good enough - the camera drops the frame rate if the lighting is low), and the delay is only a fraction of a second, good enough for my further processing. This is with a 1280x720 image from a fisheye camera. If I drop the resolution to 640x360 I believe I can get 20 frames/sec, but that's likely overkill for my application, and I'd rather keep the full resolution.

(Another difference from your setup that probably doesn't matter: My robot is running on a Beaglebone Blue rather than Raspberry Pi, and does not run ROS. Instead, I use ZeroMQ to serve the images to a ROS2 node on my remote computer.)

What Sampsa said about ROS messages to multiple subscribers is also salient: unless the underlying DDS implementation is configured to use UDP multicasting you'll end up with multiple copies of the image being transferred. With the setup above, using a bridging node on the remote computer as the only recipient, only one copy of the image is transferred over WiFi.

## Answer

Sergei's idea to use analog video removes the delay completely but them you need an analog to digital conversion at the server.   The video quality is as good as you are willing to pay for.     Analog video seems to many people a radical solution as they are used to computers and networks.

An even more radical solution is to move the camera off the robots and place them on the walls.  Navigation is easier but of course the robots is confined to where you have cameras.  Then with fixed cameras you can use wires to transmit the signals and have zero delay.  You can us standed security cameras that use "POE".

But if the camera must be digital and must be on the robot them it is best to process as much as you cam on the robot, compress it and send it to one node on the server, that node then redistributes the data.

With ROS2 the best configuration is many CPU cores and large memory rather then a networked cluster.  Because ROS2 can do "zero copy" message passing where the data states on the same RAM location and pointers are passed to nodes.  The data never moves and it is very fast.

## Answer

Quick comment: as with everything wireless: make sure to first base benchmark wireless throughput/performance (using something like iperf). Compare result to desired transfer rate (ie: image_raw bw). If achieved_bw < desired_bw (or very close to), things will not work (smoothly). Note that desired_bw could be multiple times the bw of a single image_raw subscription, as you write you have multiple subscribers.

In all cases though: transmitting image_raw (or their rectified variants) topics over a limited bw link is not going to work. I'd suggest looking into image_transport and configure a compressed transport. There are lossless plugins available, which would be important if you're looking to use the images for image processing tasks). One example would be [swri-robotics/imagezero_transport](https://github.com/swri-robotics/imagezero_transport). It takes some CPU, but reduces bw significantly.

See also [#q413068](http://answers.ros.org/question/413068/) for a recent discussion about a similar topic.





