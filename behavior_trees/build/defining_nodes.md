# Working with Leaf Nodes in Tree JSON

The usage of the included nodes can be generalized to a few rules.

- All of the included nodes exist within python files inside the folder `mr_bt/src/nodes/`. The files themselves are using the snake naming convention, for example: `my_bt_node`. The classes inside each file uses the CapWords naming convention conversion of the file name, for example: `class MyBtNode:`. When calling the node in a JSON tree, use reference the `"type"` with the CapWords name of the node class, i.e.
```
{
	"name":"node name",
	"type": "MyBtNode",
	...
}
```

- The arguments passed into the node definition in the tree JSON should exactly match the names of the arguments defined in the python class `__init__` function, for example if the class definition looks like this: 
```
class MyBtNode:
	def __init__(self, arg1: str, arg2: int, arg3: float):
		...
```
* Your tree JSON should look like this: 
```
{
	"name":"node name",
	"type": "MyBtNode",
	"arg1": "hello world",
	"arg2": 4,
	"arg3": 10.8
}
```

# Working with Parent Nodes in Tree JSON

The usage of parent nodes follows the same rules as the usage of the leaf nodes, however all parent nodes require their children to be defined in the tree JSON as well. The children are defined as a list of nodes within the `"children"` agument of the parent node. Here is an example parent node with two children: 
```
{
    "name":"reached_goal",
    "type":"Sequencer",
    "children":[
        {
            "name":"reached_position",
            "type":"ReachedPosition",
            "goal_pos_var_name":"goal_pos",
            "error":0.05
        },
        {
            "name":"stop",
            "type":"Stop"
        }
    ]
}
```
