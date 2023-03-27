# Creating a custom action node

To create a custom action node, you must define your node class by inheriting from the superclass that defines a generic action node.

You also must include the `execute` function as the "main" function that performs the actions of the node.

Here is an example of a custom action node:

```
from ...nodes.action import Action

class MyActionNode(Action):
    def __init__(self):
        super().__init__()
    
    def execute(self, blackboard: dict) -> str:
        ...
```

This type of node will return one of the following string values: `"success"`, `"failure"`, or `"running"`