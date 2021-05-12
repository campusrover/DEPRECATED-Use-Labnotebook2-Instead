# Software Overview

The code can be found at [Pupper GitHub Repository](https://github.com/campusrover/pupper2/tree/master)

## Directory 

```bash
├───models
│   └───agent.py
│   └───goal.py
│   └───obstacle.py
├───plots
│   └───map.png
├───src
│   └───boundary_detection.py
│   └───controller.py
│   └───fiducial_vision.py
│   └───geometry.py
│   └───main.py
│   └───node.py
│   └───path_finder.py
│   └───boundary_profiler.py
│   └───transforms.py
│   └───path_test.py
│   └───viz.py
├───params.yaml
```

## Components Overview 

### Models
* The models can be found at `/models/`
* The obstacle, agent, and goal models are of type Shape which can be found within  `src/boundary_detection.py`
* The paramters of the models can be found within `params.yaml` 
* The goal model is a shape containing only a single point 
* The obstacle and agent models are defined by 4 corner points which are then interpolated to create a series of points defining the boundary of the model
* Each model also has a center as well which would be it's relative location to the pupper 

### Computer Vision
* The computer vision module can be found at `/src/fiducial_vision.py`
* The fiducial and computer vision package requires numerous parameters to work correctly, these include fiducial tag size, lens size, and center pixel of the image
* The module itself contains the `Vision` class which  contains a method `capture_continuous` which returns a generator which yields the results of the fiducial detection module on frames of the RaspberryPi camera

### Boundary Generation and Navigation
* The modules relevant to boundary generation and planning are `src/boundary_detection.py`, `src/path_finder.py`, `/src/path_profiler.py`, and `path_test.py`
* Boundary generation works by taking in the detected postions and rotations of fidcials and then creating `obstacle` classes to represent each fiducial. Then each `obstacle.points` of type `Point[]` are transformed to it's corresponding fiducials location by the `src/transform.py` class. Then The pupper robot model `models/agent.py` which is at `(0, 0)` and each `obstacle` are used to calculate the configuration space for robot using the [Minkowski Sum](https en.wikipedia.org/wiki/Minkowski_addition)
* Also, the `models/goal.py`class is used to represent the goal of the pupper which corresponds to the fiducial with `id = 0` 
* Each point in the resulting configuration space is used to generate a graph of the area where vertices of the graph close to the points in the configuration space are removed so that when a shortest path search is performed the resulting path only includes valid positions from the robot's current position to the goal 
* Finally, the path is interpolated and converted into an array of distances to travel and at what angle it should travel at, which is then converted into command interfaces commands based on the velocity of the robot 

### Vizualization
* The vizialization class in `src/viz.py` uses `matplotlib` to generate a plot of the agent model, obstacles, obstacle boundaries, position of the goal, graph nodes, and the path of the robot

### Main
* The program can be run simply by navigating to the root of the repository and then running `python3 main.py` 
