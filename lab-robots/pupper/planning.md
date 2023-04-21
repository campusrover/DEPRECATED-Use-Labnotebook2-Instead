# Planning

![Example Map](motion_planning_map.png)

## Discretization
* The discretization of the environment is done in `/src/path_finder.py` in the `PathFinder` class 
* The parameters for discretization are in the `params.yaml` file and affect the inflation of obstacles in the graph, step size of the robot (distance from one vertex to it's immediate neighbors), the length and width of the graph to generate, as well as if the graph search should explore nodes diagonally
* The discretization happens in the `explore` method which uses two serial doubly nested lambda expressions to create a matrix of `Point` objects which gets converted into a matrix of `Node` objects containing `point`, `distance`, `previous`, and `explored` which are the required fields to perform a graph search on this data 

## Graph Search
* The graph search happens in the `PathFinder` class in the `solve` method which performs a Dijkstra shortest path search from the node corresponding to the agent location to the node nearest to the goal location
* The algorithm operates the same as a standard shortest path search but has several optimizations built in to account for the limited hardware of the RaspberryPi

## Path Profiling 
* Path profiling is the process of converting the path, a list of `Point` objects to a series of distances, and headings for the controller to follow
* The math for it is in `src/geometry.py` which finds the angle and distance between two points

## Potential Improvements
Many potential improvements exist to boos the performance, accuracy, and resolution of the planning module. Some ideas are: 
* Use dynamic programming to eliminate redundant `in_range_of_boundary_quick` checks for the same node
* Implement a gradient based approach to converting configuration space into edge weights
* Use a better shortest path search algorithm or a sampling-based approach so discretization is not necessary 