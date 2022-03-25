# Complex Movement

These nodes classified as "basic movement" nodes are used to implement complex movements without the use of basic movement nodes.


## RotateToAngleDynamic

This node is similar to LinearAngularStatic how,ever instead of providing velocities during construction, you provide names of blackboard topics
which correspond to the linear and angular velocities that the node will publish in cmd_vel when activated. This allows for velocities to be updated
by update nodes and then have one LinearAngularDynamic node providing dynamic movements.

*Example:*
```
{
    "name":"my_node",
    "type":"RotateToAngleDynamic",
    "angular_var_name":"my_angular_vel",
    "curr_angle_var_name": "my_curr_angle",
    "blackboard":{
        "my_angular_vel":null,
        "my_curr_angle":null
    }
}
```
