
# Blackboard

The blackboard is the main data storage component of the behavior tree in mr_bt which contains ROS messages, and other variables vital to the function of your program. The variables in the blackboard will be defined in your tree JSON or throughout multiple tree JSONs in your project.

The blackboard is, at its core, just a referece to a python dictionary that gets passed down through each node recursively in your tree. All nodes called in your tree have access to the same python dictionary so they can share information.

## Populating variables in the Blackboard

Let's start off with defining a blackboard that has two input variables: `max_speed` and `goal_position`. You want to define these two variables yourself at the root of your tree, so your tree JSON `root.json` should look something like this:
```
{
    "name":"move_to_position",
    "type":"Selector",
    "children":[
        ...
    ],
    "blackboard":{
        "max_speed":0.5,
        "goal_position":[1, 1]
    }
}
```
You can see that the blackboard is defined within the node with the keyword `"blackboard"` and the value being another JSON dictionary containing the key, value pairs of the blackboard variables.

## Initializing variables in the blackboard

Let's say you want a few variables in your blackboard to keep track of where your robot is, and whether it has reached a certain position. You want your variables to be called `current_position` and `reached_position`.

The various nodes within your tree will update these variables with the correct information when the tree is executed, however you don't want to populate them yourself. In this case, you can simply define the blackboard as so:
```
{
	...
	"blackboard":{
		"current_position":null,
		"reached_position":null
	}
}
```
Since your other nodes will populate these values upon execution of the tree, you can initialize them as `null`.

## ROS messages in the Blackboard

You'll likely need to subscribe to ROS messages coming from either your robot, or other ros programs in your behavior tree. In order to program your behavior tree to subscribe to these messages, you must define them in the blackboard as a special case.

For example, let's say you need want to use the `GetPosition` node to populate the `current_position` variable from the previous example. The `GetPosition` node uses an `Odometry` message to get the current positionof the robot, so your blackboard should look like this:
```
{
	"name":"Get my robot's position",
	"type":"GetPosition",
	"position_var_name":"current_position",
	"blackboard":{
		"current_position":null,
		"/odom":"Odometry"
	}
}
```
You'll notice that in the special case of subscribing to a ROS topic in the blackboard, the name of the variable is preceded with a `/`, and the value is the name of the ROS message type. This will be the case for any ROS topic subscription in your behavior tree.

## Defining the blackboard in multiple JSONs

Likely you will split your project up into multiple different files, and it will be difficult to consolidate all of the required blackboard variables and ROS topic subscriptions into one blackboard definition. Luckily, you don't need to do that.

You can define different sections of the blackboard in different JSON files, and all of the different blackboard definitions will be combined into one upon compilation of your tree.

For example, if you have two files in your tree each with a different blackboard definition such as:

```
{
	...
	"blackboard":{
		"current_position":null,
		"/odom":"Odometry"
	}
}

```
And then
```
{
	...
	"blackboard":{
		"max_speed":0.5,
		"goal_position":[1, 1]
	}
}
```
The final functional blackboard will be 

```
"blackboard":{
	"current_position":null,
	"/odom":"Odometry",
	"max_speed":0.5,
	"goal_position":[1, 1]
}
```
