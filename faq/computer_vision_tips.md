---
title: What are some Computer Vision Tips
description:
author:
date: may-2023
status: needs review
type: faq
---
# Computer Vision

## Attention

For new contributors of Percetion\_CV team, please first create **your own branch** and make sure all your work is done within your branch. Do **PR \(pull request\)** only if your team leader asks you to do so.   


For new team leaders of Perception\_CV, the `master branch` should **only** contain stable code that has been confirmed working. Master branch will be the source we use for integration with other teams when the time is ready.

## Introduction

Full CV Repo here: [https://github.com/campusrover/Robotics\_Computer\_Vision](https://github.com/campusrover/Robotics_Computer_Vision)

This repo is originally forked from [https://github.com/ultralytics/yolov3](https://github.com/ultralytics/yolov3) but heavily modified for our own use. The purpose of this repo is to achieve custom object detection for Brandeis Autonomous Robotics Course. Changes were made based on our object of deploying CV on ROS.   
 To download our most recent best trained weights, please go to [https://drive.google.com/file/d/1DquRwpNDaXgkON2gj9Oks8N2ZWgN2Z9q/view?usp=sharing](https://drive.google.com/file/d/1DquRwpNDaXgkON2gj9Oks8N2ZWgN2Z9q/view?usp=sharing)   
 Then unzip the file and copy `coco` and `weights` directory in this repo and replace everything.

**Notes**:  
 I've put a low of useful tools inside the `./utils` directory, please feel free to use them whenever you need it.   


* `./utils/vid_2_frm.py` : The python script that extracts all frames out of a video, you can control the extracting rate by reading the comment and do small modification. This script will also tell you the fps of the source video which will be useful for later converting frames back to video. 
* `./utils/frm_2_vid.py` : The python script that is converting frames by its name into a video, you better know the original/target video's fps to get the optimal output. 
* `./utils/xml_2_txt` : The repo that converts .xml format annotation into our desired .txt format \(Yolo format\), read and follow the README file inside. 
* `./utils/labelimg` : The repo that we use for labelling images, great tool! Detailed README inside. 
* `./utils/check_missing_label.py` : The python script that can be used for checking if there's any missing label in the annotation/image mixed directory. 
* `./utils/rename_dataset.py` : The python script doing mass rename in case different datasets' images names and annotations are the same and need to be distinguished. 
* `./list_img_path.py` : The python script that splits the datasets \(images with its corresponding annotations\) into training set and validation set in the ratio of 6:1 \(you can modify the ratio\). 
* `./utils/img_2_gif.py` : The python script that converts images to gif. 
* `./coco/dataset_clean.py` : The python script that cleans the uneven images and labels that is going to be trained and make sure they are perfectly parallel. 
* `./utils/video_recorder_pi.py` : The python script that records videos on pi camera. This script should be located in the robot and run under SSH 

Here are links to download our **datasets** \(images and annotations\) by certain class:

**Doorplate Recognition**:  


* custom\_volen\(provided by Haofan\): [https://drive.google.com/file/d/1A9yI5PdLeAlKEVQww2NJgQLRDxq9tcOJ/view?usp=sharing](https://drive.google.com/file/d/1A9yI5PdLeAlKEVQww2NJgQLRDxq9tcOJ/view?usp=sharing)  
* custom\_doorplate\(provided by Haofan\): [https://drive.google.com/file/d/1jITWceHYYFXjUyaJ1bp4Wdb\_tKhtylQJ/view?usp=sharing](https://drive.google.com/file/d/1jITWceHYYFXjUyaJ1bp4Wdb_tKhtylQJ/view?usp=sharing)  

**Facial Recognition**:  


* Abhishek: [https://drive.google.com/file/d/1Z3ICrLEVt50ia1C07ZCxE\_Na105aRjsE/view?usp=sharing](https://drive.google.com/file/d/1Z3ICrLEVt50ia1C07ZCxE_Na105aRjsE/view?usp=sharing)  
* Haofan: [https://drive.google.com/file/d/1nDcGb0QGSzLJaQL1ewMWVXtvXXjSwWC0/view?usp=sharing](https://drive.google.com/file/d/1nDcGb0QGSzLJaQL1ewMWVXtvXXjSwWC0/view?usp=sharing)  
* Yuchen: [https://drive.google.com/file/d/1PomjuCvcJ25\_d\_EaQwqE9l1wuZ5z5Zz3/view?usp=sharing](https://drive.google.com/file/d/1PomjuCvcJ25_d_EaQwqE9l1wuZ5z5Zz3/view?usp=sharing)  
* Huaigu: [https://drive.google.com/file/d/1QNKtvanc58PoQZCg6htQpcImd00toYby/view?usp=sharing](https://drive.google.com/file/d/1QNKtvanc58PoQZCg6htQpcImd00toYby/view?usp=sharing)  
* Eli: [https://drive.google.com/file/d/14qII9t4tyDsYqj\_bxxCdxSyIip0CRwQT/view?usp=sharing](https://drive.google.com/file/d/14qII9t4tyDsYqj_bxxCdxSyIip0CRwQT/view?usp=sharing)  
* Nate: [https://drive.google.com/file/d/1KE0UVu7dalip4mDVoVGpBgr1uyyhoHQB/view?usp=sharing](https://drive.google.com/file/d/1KE0UVu7dalip4mDVoVGpBgr1uyyhoHQB/view?usp=sharing)  
* Cody: [https://drive.google.com/file/d/1Yb4RmYWXWCBO3nb\_Di--3tRh0LdiRZBn/view?usp=sharing](https://drive.google.com/file/d/1Yb4RmYWXWCBO3nb_Di--3tRh0LdiRZBn/view?usp=sharing)  
* Pito: [https://drive.google.com/file/d/1NZ4SBfv1Y5zuGpRQLebOlK-duG\_p\_0pg/view?usp=sharing](https://drive.google.com/file/d/1NZ4SBfv1Y5zuGpRQLebOlK-duG_p_0pg/view?usp=sharing)  
* Sibo: [https://drive.google.com/file/d/1c7ZcMN-LcMAjmO62oS\_C3y2hpA6IUgvP/view?usp=sharing](https://drive.google.com/file/d/1c7ZcMN-LcMAjmO62oS_C3y2hpA6IUgvP/view?usp=sharing)  
* Arjun: [https://drive.google.com/file/d/10NnfTU150Pis5ugOWLzxVvwcesi873LY/view?usp=sharing](https://drive.google.com/file/d/10NnfTU150Pis5ugOWLzxVvwcesi873LY/view?usp=sharing)  
* Charlie: [https://drive.google.com/file/d/1UmCUl-uLPwwOub2ZsTNpQHK9Q\_rdVScI/view?usp=sharing](https://drive.google.com/file/d/1UmCUl-uLPwwOub2ZsTNpQHK9Q_rdVScI/view?usp=sharing)  

## CV Subscriber & Publisher

All the CV subscriber and publisher are located at `./utils/` directory, they are:

* `./utils/image_subscriber.py` : The python script that subscribe image from `raspicam_node/image` rostopic. 
* `./utils/string_publisher.py` : The python script that publishes a string on rostopic of `/mutant/face_detection` which is generated from `detect.py`, the format is explained below:  

CV Publisher example: "\['sibo', -4.34, 1.63\]"   


\[   
 &lt;"class name"&gt;,   
 &lt;"angle of target to front in degree \(negative -&gt; left, positive -&gt; right"\)&gt;,   
 &lt;"rough distance in meter"&gt;   
 \]   


## Cheat Sheet For Raspberry Pi Camera

Detailed official user guide here: [http://emanual.robotis.com/docs/en/platform/turtlebot3/appendix\_raspi\_cam/](http://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/)   


**Some useful commands**:   


* `raspivid -vf -hf -t 30000 -w 640 -h 480 -fps 25 -b 1200000 -p 0,0,640,480 -o pivideo.h264`   recording 30 seconds video on 25 fps.  
* `MP4Box -add pivideo.h264 pivideo.mp4`   converting .h264 video to .mp4  
* `scp donatello@129.64.243.61:~/pivideo.mp4 ~/Downloads/`   downloading video from ssh to local machine  
* `rqt_image_view`   getting vision from camera, requires bringup which is conflict to the video recording function  
* `rosrun rqt_reconfigure rqt_reconfigure`   edit camera configuration  

**Pipeline of recording video on** `DONATELLO`:

* ssh `donatello@129.64.243.61`
* If you want to see preview images, `roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch`, then on remote computer, do `rqt_image_view`
* when you recording video, shut down the `rpicamera bringup` in advance
* Do `raspivid -vf -hf -t 30000 -w 640 -h 480 -fps 25 -b 1200000 -p 0,0,640,480 -o pivideo.h264` on `DONATELLO` to record video

## Cheat Sheet For USB Web-Camera

**Get image\_view**

* ssh &lt;Robot\_name\_space&gt;@  
* plug in the USB camera  
* On slave, do `lsusb` and `ls /dev |grep video` to check if camera was recognized by system  
* On slave, install usb\_cam ROS node `sudo apt install ros-kinetic-usb-cam`  
* On slave, check the usb camera launch file `cat /opt/ros/kinetic/share/usb_cam/launch/usb_cam-test.launch`  
* \(Optional\) On local client machine \(master machine\), run `roscore` \(Usually it's constantly running on the desktop of Robotics Lab so you won't need to do this line\)  
* On slave, start usb\_cam node `roslaunch usb_cam usb_cam-test.launch`  
* \(Optional\) On slave, bring running process to background with `CTRL+Z` and execute `bg` command to continue execution it in background  
* \(Optional\) On slave, check the topic of usb camera `rostopic list`  
* \(Optional\) On master, check the topics in GUI `rqt_graph`  
* On master, read camera data with image\_view `rosrun image_view image_view image:=/<name_space>/usb_cam/image_raw`  
* On slave, to bring background task to foreground `fg`  

**Web Streaming**

* On slave, install web-video-server ROS node `sudo apt install ros-kinetic-web-video-server`  
* On slave, to make it right, create catkin workspace for our custom launch file `mkdir -p ~/rosvid_ws/src`  
* On slave,`cd ~/rosvid_ws`  
* On slave, `catkin_make`  
* On slave, `source devel/setup.bash`  
* On slave, create ROS package `cd src` then `catkin_create_pkg vidsrv std_msgs rospy roscpp`  
* On slave, create launch file using nano, vim, etc `mkdir -p vidsrv/launch` then `nano vidsrv/launch/vidsrv.launch`. Then copy and paste the code below  

[https://github.com/campusrover/Perception\_CV/blob/master/utils/vidsrv.launch](https://github.com/campusrover/Perception_CV/blob/master/utils/vidsrv.launch)   


* On slave, build package `cd..` then `catkin_make`  
* On master, Make sure `roscore` is running  
* On slave, run created launch file `roslaunch vidsrv vidsrv.launch`  
* On your client machine, open web browser and go to `<Robot IP address>:8080` . Under `/usb_cam/` categoryand and click `image_raw` .  
* Enjoy the web streaming  

## Description

The [https://github.com/ultralytics/yolov3](https://github.com/ultralytics/yolov3) repo contains inference and training code for YOLOv3 in PyTorch. The code works on Linux, MacOS and Windows. Training is done on the COCO dataset by default: [https://cocodataset.org/\#home](https://cocodataset.org/#home). **Credit to Joseph Redmon for YOLO:** [https://pjreddie.com/darknet/yolo/](https://pjreddie.com/darknet/yolo/).

## Requirements

Python 3.7 or later with the following `pip3 install -U -r requirements.txt` packages:

* `numpy`
* `torch >= 1.0.0`
* `opencv-python`
* `tqdm`

## Tutorials

* [GCP Quickstart](https://github.com/ultralytics/yolov3/wiki/GCP-Quickstart)
* [Transfer Learning](https://github.com/ultralytics/yolov3/wiki/Example:-Transfer-Learning)
* [Train Single Image](https://github.com/ultralytics/yolov3/wiki/Example:-Train-Single-Image)
* [Train Single Class](https://github.com/ultralytics/yolov3/wiki/Example:-Train-Single-Class)
* [Train Custom Data](https://github.com/ultralytics/yolov3/wiki/Train-Custom-Data)

## Training

**Start Training:** Run `train.py` to begin training after downloading COCO data with `data/get_coco_dataset.sh`.

**Resume Training:** Run `train.py --resume` resumes training from the latest checkpoint `weights/latest.pt`.

Each epoch trains on 117,263 images from the train and validate COCO sets, and tests on 5000 images from the COCO validate set. Default training settings produce loss plots below, with **training speed of 0.6 s/batch on a 1080 Ti \(18 epochs/day\)** or 0.45 s/batch on a 2080 Ti.

Here we see training results from `coco_1img.data`, `coco_10img.data` and `coco_100img.data`, 3 example files available in the `data/` folder, which train and test on the first 1, 10 and 100 images of the coco2014 trainval dataset.

`from utils import utils; utils.plot_results()` ![results](https://user-images.githubusercontent.com/26833433/55669383-df76c980-5876-11e9-9806-691bd507ee17.jpg)

### Image Augmentation

`datasets.py` applies random OpenCV-powered \([https://opencv.org/](https://opencv.org/)\) augmentation to the input images in accordance with the following specifications. Augmentation is applied **only** during training, not during inference. Bounding boxes are automatically tracked and updated with the images. 416 x 416 examples pictured below.

| Augmentation | Description |
| :--- | :--- |
| Translation | +/- 10% \(vertical and horizontal\) |
| Rotation | +/- 5 degrees |
| Shear | +/- 2 degrees \(vertical and horizontal\) |
| Scale | +/- 10% |
| Reflection | 50% probability \(horizontal-only\) |
| H**S**V Saturation | +/- 50% |
| HS**V** Intensity | +/- 50% |

![](https://user-images.githubusercontent.com/26833433/50525037-6cbcbc00-0ad9-11e9-8c38-9fd51af530e0.jpg)

### Speed

[https://cloud.google.com/deep-learning-vm/](https://cloud.google.com/deep-learning-vm/)  
**Machine type:** n1-standard-8 \(8 vCPUs, 30 GB memory\)  
**CPU platform:** Intel Skylake  
**GPUs:** K80 \($0.198/hr\), P4 \($0.279/hr\), T4 \($0.353/hr\), P100 \($0.493/hr\), V100 \($0.803/hr\)  
**HDD:** 100 GB SSD  
**Dataset:** COCO train 2014

| GPUs | `batch_size` | batch time | epoch time | epoch cost |
| :--- | :--- | :--- | :--- | :--- |
|  | \(images\) | \(s/batch\) |  |  |
| 1 K80 | 16 | 1.43s | 175min | $0.58 |
| 1 P4 | 8 | 0.51s | 125min | $0.58 |
| 1 T4 | 16 | 0.78s | 94min | $0.55 |
| 1 P100 | 16 | 0.39s | 48min | $0.39 |
| 2 P100 | 32 | 0.48s | 29min | $0.47 |
| 4 P100 | 64 | 0.65s | 20min | $0.65 |
| 1 V100 | 16 | 0.25s | 31min | $0.41 |
| 2 V100 | 32 | 0.29s | 18min | $0.48 |
| 4 V100 | 64 | 0.41s | 13min | $0.70 |
| 8 V100 | 128 | 0.49s | 7min | $0.80 |

## Inference

Run `detect.py` to apply trained weights to an image, such as `zidane.jpg` from the `data/samples` folder:

**YOLOv3:** `python3 detect.py --cfg cfg/yolov3.cfg --weights weights/yolov3.weights` ![](https://user-images.githubusercontent.com/26833433/50524393-b0adc200-0ad5-11e9-9335-4774a1e52374.jpg)

**YOLOv3-tiny:** `python3 detect.py --cfg cfg/yolov3-tiny.cfg --weights weights/yolov3-tiny.weights` ![](https://user-images.githubusercontent.com/26833433/50374155-21427380-05ea-11e9-8d24-f1a4b2bac1ad.jpg)

**YOLOv3-SPP:** `python3 detect.py --cfg cfg/yolov3-spp.cfg --weights weights/yolov3-spp.weights` ![](https://user-images.githubusercontent.com/26833433/54747926-e051ff00-4bd8-11e9-8b5d-93a41d871ec7.jpg)

### Webcam

Run `detect.py` with `webcam=True` to show a live webcam feed.

## Pretrained Weights

* Darknet `*.weights` format: [https://pjreddie.com/media/files/yolov3.weights](https://pjreddie.com/media/files/yolov3.weights)
* PyTorch `*.pt` format: [https://drive.google.com/drive/folders/1uxgUBemJVw9wZsdpboYbzUN4bcRhsuAI](https://drive.google.com/drive/folders/1uxgUBemJVw9wZsdpboYbzUN4bcRhsuAI)

## mAP

* Use `test.py --weights weights/yolov3.weights` to test the official YOLOv3 weights.
* Use `test.py --weights weights/latest.pt` to test the latest training results.
* Compare to darknet published results [https://arxiv.org/abs/1804.02767](https://arxiv.org/abs/1804.02767).

|  | [ultralytics/yolov3](https://github.com/ultralytics/yolov3) | [darknet](https://arxiv.org/abs/1804.02767) |
| :--- | :--- | :--- |
| `YOLOv3 320` | 51.8 | 51.5 |
| `YOLOv3 416` | 55.4 | 55.3 |
| `YOLOv3 608` | 58.2 | 57.9 |
| `YOLOv3-spp 320` | 52.4 | - |
| `YOLOv3-spp 416` | 56.5 | - |
| `YOLOv3-spp 608` | 60.7 | 60.6 |

```bash
git clone https://github.com/ultralytics/yolov3
# bash yolov3/data/get_coco_dataset.sh
git clone https://github.com/cocodataset/cocoapi && cd cocoapi/PythonAPI && make && cd ../.. && cp -r cocoapi/PythonAPI/pycocotools yolov3
cd yolov3

python3 test.py --save-json --img-size 416
Namespace(batch_size=32, cfg='cfg/yolov3-spp.cfg', conf_thres=0.001, data_cfg='data/coco.data', img_size=416, iou_thres=0.5, nms_thres=0.5, save_json=True, weights='weights/yolov3-spp.weights')
Using CUDA device0 _CudaDeviceProperties(name='Tesla V100-SXM2-16GB', total_memory=16130MB)
               Class    Images   Targets         P         R       mAP        F1
Calculating mAP: 100%|█████████████████████████████████████████| 157/157 [05:59<00:00,  1.71s/it]
                 all     5e+03  3.58e+04     0.109     0.773      0.57     0.186
 Average Precision  (AP) @[ IoU=0.50:0.95 | area=   all | maxDets=100 ] = 0.335
 Average Precision  (AP) @[ IoU=0.50      | area=   all | maxDets=100 ] = 0.565
 Average Precision  (AP) @[ IoU=0.75      | area=   all | maxDets=100 ] = 0.349
 Average Precision  (AP) @[ IoU=0.50:0.95 | area= small | maxDets=100 ] = 0.151
 Average Precision  (AP) @[ IoU=0.50:0.95 | area=medium | maxDets=100 ] = 0.360
 Average Precision  (AP) @[ IoU=0.50:0.95 | area= large | maxDets=100 ] = 0.493
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=   all | maxDets=  1 ] = 0.280
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=   all | maxDets= 10 ] = 0.432
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=   all | maxDets=100 ] = 0.458
 Average Recall     (AR) @[ IoU=0.50:0.95 | area= small | maxDets=100 ] = 0.255
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=medium | maxDets=100 ] = 0.494
 Average Recall     (AR) @[ IoU=0.50:0.95 | area= large | maxDets=100 ] = 0.620

python3 test.py --save-json --img-size 608 --batch-size 16
Namespace(batch_size=16, cfg='cfg/yolov3-spp.cfg', conf_thres=0.001, data_cfg='data/coco.data', img_size=608, iou_thres=0.5, nms_thres=0.5, save_json=True, weights='weights/yolov3-spp.weights')
Using CUDA device0 _CudaDeviceProperties(name='Tesla V100-SXM2-16GB', total_memory=16130MB)
               Class    Images   Targets         P         R       mAP        F1
Computing mAP: 100%|█████████████████████████████████████████| 313/313 [06:11<00:00,  1.01it/s]
                 all     5e+03  3.58e+04      0.12      0.81     0.611     0.203
 Average Precision  (AP) @[ IoU=0.50:0.95 | area=   all | maxDets=100 ] = 0.366
 Average Precision  (AP) @[ IoU=0.50      | area=   all | maxDets=100 ] = 0.607
 Average Precision  (AP) @[ IoU=0.75      | area=   all | maxDets=100 ] = 0.386
 Average Precision  (AP) @[ IoU=0.50:0.95 | area= small | maxDets=100 ] = 0.207
 Average Precision  (AP) @[ IoU=0.50:0.95 | area=medium | maxDets=100 ] = 0.391
 Average Precision  (AP) @[ IoU=0.50:0.95 | area= large | maxDets=100 ] = 0.485
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=   all | maxDets=  1 ] = 0.296
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=   all | maxDets= 10 ] = 0.464
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=   all | maxDets=100 ] = 0.494
 Average Recall     (AR) @[ IoU=0.50:0.95 | area= small | maxDets=100 ] = 0.331
 Average Recall     (AR) @[ IoU=0.50:0.95 | area=medium | maxDets=100 ] = 0.517
 Average Recall     (AR) @[ IoU=0.50:0.95 | area= large | maxDets=100 ] = 0.618
```

## Citation

[![DOI](https://zenodo.org/badge/146165888.svg)](https://zenodo.org/badge/latestdoi/146165888)

## Contact

Issues should be raised directly in the repository. For additional questions or comments please contact your CV Team Leader or Sibo Zhu at siboz1995@gmail.com

