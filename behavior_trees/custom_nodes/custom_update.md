# Creating a custom Update node

To create a custom update node, you must define your node class by inheriting from the superclass that defines a generic update node.

You also must include the `update_blackboard` function as the "tick" function that updates the blackboard with relevant information.

Here is an example of a custom update node:

```
from ...nodes.update import Update

class MyUpdateNode(Update):
    def __init__(self):
        super().__init__()
    
    def update_blackboard(self, blackboard: dict) -> str:
        ...
```

This type of node will return one of the following string values: `"success"`, `"failure"`, or `"running"`