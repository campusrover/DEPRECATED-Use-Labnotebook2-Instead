# setting up apriltags

Setting up apriltags to work with your program is pretty simple. 

You can run the following two lines to install apriltags and its ros version onto your ubuntu: 

`sudo apt install ros-noetic-apriltag`

`sudo apt install ros-noetic-apriltag-ros`

Afterwards, connect your camera

`roslaunch usb_cam usb_cam-test.launch`

- changed the video_device to /dev/video2 (or whichever video device you figure out it is) to take from usbcam
- created head_camera.yaml file from this [example](https://github.com/olinrobotics/usb_cam/blob/master/head_camera.yaml) and added the [calibration](https://www.notion.so/camera-calibration-c7449147443a411f8045b1a434372f3c) information:

```java
cam0:
  camera_model: pinhole
  intrinsics: [684.9223702799867, 686.2362451089259, 307.24291147440965, 257.2964284210188]
  distortion_model: plumb_bob
  distortion_coeffs: [9.39260444e-03, 4.90535732e-01, 1.48962564e-02, 4.68503188e-04, -1.77954077e+00]
  resolution: [640, 480]
  tagtopic: /cam0
```

`roslaunch apriltag_ros continuous_detection.launch publish_tag_detections_image:=true`

- can remap input topics to what you need:
    - image_rect:=usb_cam/image_raw
    - camera_info:=usb_cam/camera_info
