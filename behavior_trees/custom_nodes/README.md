# Writing custom Behavior Tree nodes

You may find yourself wanting to extend the functionality provided by the default nodes in this package by adding your own parent, action, conditional, or update nodes.

## Directory structure

In order to create your own custom nodes, the nodes can exist anywhere in the directory `mr_bt/src/nodes/` as a `.py` file.

## Naming

All behavior tree nodes are created as a python class in a python file. The name of your `.py` file should correspond with the name of your class within the file. 

For example, if you want to create a node called `MyNewNode`, the file that it exists in should be called `my_new_node.py` and the class definition should be `class MyNewNode(SomeNodeType):`.

## Required function

Each different type of node will have a required "tick" function, but the naming is different depending on the type of node you are creating. Regardless of what the "main"  of function is called, it always returns one of three string type values: `"success"`, `"failure"`, or `"running"`.


For further details on how to create custom nodes of each type, see the following sections.