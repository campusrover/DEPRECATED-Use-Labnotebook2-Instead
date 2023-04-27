# Conditional Nodes

## Basic Conditionals

- BoolVar
    ```
    Returns "success" if boolean variable in blackboard is true, otherwise returns "failure".

    Params:
        var_name (string): name of boolean variable in blackboard
    ```
- BoolVarNot
    ```
    Returns "failure" if boolean variable in blackboard is true, otherwise returns "success".

    Params:
        var_name (string): name of boolean variable in blackboard
    ```

## Odometry Conditionals

- ReachedPosition
    ```
    Returns "success" if the position of the robot given by the /odom topic is close within error to the provided goal position variable

    Params:
        goal_pos_var_name (string): Name of the blackboard variable holding an array of the x,y goal coordinates.
        error (float): Tolerated error for reaching the goal position
    ```

## LaserScan Conditionals

- ClearAhead


