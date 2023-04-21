# Basic Movement

The 3 nodes classified as "basic movement" nodes are the fundamental building blocks needed in order to make a
robotics behavior tree application.

## LinearAngularStatic

When this node is activated, it will publish a cmd_vel message including the linear and angular velocities specified during its construction:

*Example:*
```
{
    "name":"my_node",
    "type":"LinearAngularStatic",
    "lin_vel":0.5,
    "ang_vel":1
}
```

## Stop

A special case of LinearAngularStatic; upon activation the Stop node will publish a cmd_vel message with all velocity values set to 0.

*Example:*
```
{
    "name":"my_node",
    "type":"Stop"
}
```


## LinearAngularDynamic

This node is similar to LinearAngularStatic how,ever instead of providing velocities during construction, you provide names of blackboard topics
which correspond to the linear and angular velocities that the node will publish in cmd_vel when activated. This allows for velocities to be updated
by update nodes and then have one LinearAngularDynamic node providing dynamic movements.

*Example:*
```
{
    "name":"my_node",
    "type":"LinearAngularDynamic",
    "linear_var_name":"my_linear_vel",
    "angular_var_name":"my_angular_vel",
    "blackboard":{
        "my_angular_vel":null,
        "my_linear_vel":null
    }
}
```
