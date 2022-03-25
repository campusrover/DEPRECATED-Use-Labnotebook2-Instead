# Complex Movement

These nodes classified as "basic movement" nodes are used to implement complex movements without the use of basic movement nodes.


## RotateToAngleDynamic

This node rotates the robot to a particular angle (in degrees). That angle is specified in the blackboard by the variable name specified upon instantiation.

*Example:*
```
{
    "name":"my_node",
    "type":"RotateToAngleDynamic",
    "angule_var_name":"my_goal_angle",
    "curr_angle_var_name": "my_curr_angle",
    "blackboard":{
        "my_goal_angle":null,
        "my_curr_angle":null
    }
}
```
