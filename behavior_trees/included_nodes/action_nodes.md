# Action Nodes

## Basic Movement

- LinearAngularStatic
    ```
    Publishes a Twist message on the topic /cmd_vel with a linear and angular velocity defined in the node JSON

    Params:
        lin_vel (float): Linear x velocity for Twist msg
        ang_vel (float): Angular z velocity for Twist msg
    ```

- LinearAngularDynamic
    ```
    Publishes a twist message on the topic /cmd_vel with a linear and angular velocity provided by variables in the blackboard

    Params:
        linear_var_name (string): Name of the blackboard variable holding the linear x velocity to add to Twist msg
        angular_var_name (string): Name of the blackboard variable holding the angular z velocity to add to Twist msg
    ```
- Stop
    ```
    Publishes a Twist msg on /cmd_vel with all velocity values set to 0
    ```


    
