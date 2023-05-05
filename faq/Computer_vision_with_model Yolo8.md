# Computer vision with pretrained model Yolo8

Yolo8 is a cutting-edge, state-of-the-art (SOTA) model that builds upon the success of previous YOLO versions and introduces new features and improvements to further boost performance and flexibility. YOLOv8 is designed to be fast, accurate, and easy to use, making it an excellent choice for a wide range of object detection and tracking, instance segmentation, image classification and pose estimation tasks. It is a powerful model that can be used to detect multiple objects in an image. 

It has been wrapped into a user-friendly python package Ultralytics (https://docs.ultralytics.com/). To detect objects of interest, the pre-trained model yolo8 can be used. Or one can customize the yolo8 model by training it with provided train image data. Here the website Roboflow (https://roboflow.com/) has a variety of object datasets, e.g. traffic sign dataset (https://universe.roboflow.com/usmanchaudhry622-gmail-com/traffic-and-road-signs/model/1). Once the dataset is downloaded and the Ultralytics package is installed, the yolo8 model can be trained easily:
```
yolo detect train model=yolov8n.pt data=traffic_sign.yaml
```
Where the traffic_sign.yaml is the path to the yaml file inside your downloaded dataset from roboflow. You have the options to use various yolo8 models. See [Detection Docs](https://docs.ultralytics.com/tasks/detect/) for usage examples with these models.

| Model                                                                                | size<br><sup>(pixels) | mAP<sup>val<br>50-95 | Speed<br><sup>CPU ONNX<br>(ms) | Speed<br><sup>A100 TensorRT<br>(ms) | params<br><sup>(M) | FLOPs<br><sup>(B) |
| ------------------------------------------------------------------------------------ | --------------------- | -------------------- | ------------------------------ | ----------------------------------- | ------------------ | ----------------- |
| [YOLOv8n](https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt) | 640                   | 37.3                 | 80.4                           | 0.99                                | 3.2                | 8.7               |
| [YOLOv8s](https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt) | 640                   | 44.9                 | 128.4                          | 1.20                                | 11.2               | 28.6              |
| [YOLOv8m](https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8m.pt) | 640                   | 50.2                 | 234.7                          | 1.83                                | 25.9               | 78.9              |
| [YOLOv8l](https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8l.pt) | 640                   | 52.9                 | 375.2                          | 2.39                                | 43.7               | 165.2             |
| [YOLOv8x](https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8x.pt) | 640                   | 53.9                 | 479.1                          | 3.53                                | 68.2               | 257.8             |

The larger the model is, the higher the latency will present in the ros node that holds the model. Therefore, the smaller model should be used as long as the model works for the objects of interest.

This is a quick ann simply way to train a customized model that is powerful in objects detection of robot vision.


